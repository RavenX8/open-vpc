#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

// hide console window on Windows in release
extern crate hidapi;

use std::ffi::CString;
use std::rc::Rc;
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::Duration;

use clap::Parser;
use eframe::{egui, glow};
use eframe::egui::{Context, Ui};
use hidapi::HidApi;
#[cfg(feature = "logging")]
use log::{debug, error, info, trace};

mod about;

const PROGRAM_TITLE: &str = "OpenVPC - Shift Tool";
const INITIAL_WIDTH: f32 = 720.0;
const INITIAL_HEIGHT: f32 = 260.0;

#[derive(Copy, Clone, PartialEq)]
enum State {
    About,
    Initialising,
    Running,
}

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Allows all firmware versions
    #[arg(short, long, default_value_t = false)]
    skip_firmware: bool,
}

fn main() -> eframe::Result<()> {
    env_logger::init(); // Log to stderr (if you run with `RUST_LOG=debug`).
    let _ = Args::parse();

    #[cfg(feature = "logging")] {
        info!("Creating main window...");
    }

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([INITIAL_WIDTH, INITIAL_HEIGHT]),
        ..Default::default()
    };

    eframe::run_native(
        PROGRAM_TITLE,
        options,
        Box::new(|_cc| Box::new(ShiftTool::default())),
    )
}

fn read_bit(value: u16, position: u8) -> bool {
    (value & (1 << position)) != 0
}

fn merge_u8_into_u16(high: u8, low: u8) -> u16 {
    // Shift `high` to the left by 8 bits and combine it with `low`
    let merged = (high as u16) << 8 | (low as u16);
    merged
}

fn is_supported(input: String) -> bool {
    let args = Args::parse();
    if args.skip_firmware {
        return true;
    }

    let fixed_list = vec![
        String::from("VIRPIL Controls 20230328")
    ];

    fixed_list.contains(&input)
}


#[derive(Debug, PartialEq, PartialOrd, Ord, Eq, Hash, Clone)]
struct VpcDevice {
    path: String,
    name: Rc<String>,
    firmware: Rc<String>,
}

impl std::fmt::Display for VpcDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{} {}", self.firmware, self.name)
    }
}

struct ShiftTool {
    state: State,
    device_list: Vec<VpcDevice>,
    source_path: String,
    shift_state: Arc<(Mutex<u16>, Condvar)>,
    receiver_list: Vec<usize>,
    thread_state: Arc<(Mutex<bool>, Condvar)>,
}

impl Default for ShiftTool {
    fn default() -> Self {
        Self {
            state: State::Initialising,
            device_list: vec![],
            shift_state: Arc::new((Mutex::new(0), Condvar::new())),
            source_path: "".to_string(),
            receiver_list: vec![],
            thread_state: Arc::new((Mutex::new(false), Condvar::new())),
        }
    }
}

impl ShiftTool {
    fn about_screen(&mut self, ui: &mut Ui) {
        let loading = self.state != State::About;
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.centered_and_justified(|ui| {
                    let mut about = "";
                    if !loading {
                        about = "About ";
                    }
                    ui.heading(format!("{}{}", about, PROGRAM_TITLE, ));
                });
            });
            if loading {
                ui.horizontal(|ui| {
                    ui.centered_and_justified(|ui| {
                        ui.add(egui::Label::new("-- Loading, please wait --"));
                    });
                });
            }
            for line in about::about() {
                ui.horizontal(|ui| {
                    ui.centered_and_justified(|ui| {
                        ui.add(egui::Label::new(line));
                    });
                });
            }

            if !loading {
                ui.horizontal(|ui| {
                    let blank: String = "         ".to_string();
                    ui.centered_and_justified(|ui| {
                        ui.label(&blank);
                        if ui.button("OK")
                            .clicked() {
                            self.state = State::Running;
                        }
                        ui.label(&blank);
                    });
                });
            }
        });
    }

    fn init(&mut self) {
        // Do some init stuff here
        let hidapi = HidApi::new_without_enumerate().expect("Was unable to open hid instance");
        self.refresh_devices(hidapi);
        self.state = State::Running;
    }

    fn refresh_devices(&mut self, mut hidapi: HidApi) {
        match hidapi.reset_devices() {
            Ok(_api) => {}
            Err(e) => {
                error!("Error: {}", e);
            }
        };
        match hidapi.add_devices(0x3344, 0) {
            Ok(_api) => {}
            Err(e) => {
                error!("Error: {}", e);
            }
        }

        // if hidapi.device_list().count() != self.device_list.len() {
        //     self.device_list.clear();
        // }

        let mut new_devices: Vec<VpcDevice> = vec![];
        for device in hidapi.device_list() {
            let firmware_string = device.manufacturer_string().unwrap_or_else(|| "");
            let device_name = device.product_string().unwrap_or_else(|| "");
            let device_path = device.path().to_str().expect("Invalid UTF-8").to_string();
            let mut found = false;
            if !self.device_list.is_empty() {
                found = self.device_list.iter().any(|r| r.path == device_path.clone());
            }
            if !found {
                if is_supported(firmware_string.to_owned().into()) {
                    new_devices.push(VpcDevice { path: device_path.into(), name: device_name.to_owned().into(), firmware: firmware_string.to_owned().into() });
                }
            }
        }

        if !new_devices.is_empty() {
            if !self.device_list.is_empty() {
                self.device_list.remove(0);
            }

            let mut merged_list = self.device_list.clone();
            merged_list.extend(new_devices.into_iter());

            merged_list.sort_by(|a, b| a.path.cmp(&b.path));
            merged_list.dedup_by(|a, b| a.path.eq(&b.path));

            merged_list.insert(0, VpcDevice { path: String::from("").into(), name: String::from("-NO CONNECTION (Select device from the list)-").into(), firmware: String::from("").into() });
            self.device_list = merged_list.clone();
        }

        if self.device_list.is_empty() {
            self.device_list.insert(0, VpcDevice { path: String::from("").into(), name: String::from("-NO CONNECTION (Select device from the list)-").into(), firmware: String::from("").into() });
        }
    }

    fn spawn_worker(&mut self) -> bool {
        let reference_to_self = self;
        let shared_shift_state = reference_to_self.shift_state.clone();
        let shared_run_state = reference_to_self.thread_state.clone();
        let source_device = CString::new(&*reference_to_self.source_path.clone()).expect("Failed to create CString");
        let mut hidapi = HidApi::new_without_enumerate().expect("Was unable to open hid instance");

        // Clear out our device list
        match hidapi.reset_devices() {
            Ok(_api) => {}
            Err(e) => {
                #[cfg(feature = "logging")] {
                    error!("Error: {}", e);
                }
            }
        };
        // Grab all virpil devices
        match hidapi.add_devices(0x3344, 0) {
            Ok(_api) => {
                #[cfg(feature = "logging")] {
                    debug!("Got devices");
                }
            }
            Err(e) => {
                #[cfg(feature = "logging")] {
                    error!("Error: {}", e);
                }
            }
        }

        // Attempt to open the source device
        let hid_source_device = match hidapi.open_path(source_device.as_ref()) {
            Ok(device) => device,
            Err(_) => {
                #[cfg(feature = "logging")] {
                    error!("Was unable to open the source device {:?}", source_device);
                }

                return false;
            }
        };
        hid_source_device.set_blocking_mode(false).expect("unable to set source blocking mode");

        // List of device handles for the thread to write to
        let mut receiver_devices = vec![];
        // Loop for all receiver devices and open a handle to them
        for i in 0..reference_to_self.receiver_list.len() {
            let receiver_path = match CString::new(&*reference_to_self.device_list[reference_to_self.receiver_list[i]].path.clone()) {
                Ok(str) => str,
                Err(e) => {
                    #[cfg(feature = "logging")] {
                        error!("Error: {}", e);
                    }
                    continue;
                }
            };
            println!("{:?}", receiver_path);
            let hid_device = match hidapi.open_path(&receiver_path) {
                Ok(device) => device,
                Err(_) => continue,
            };
            hid_device.set_blocking_mode(false).expect("unable to set receiver blocking mode");
            receiver_devices.push(hid_device);
        }


        #[cfg(feature = "logging")] {
            trace!("Launching Thread...");
        }
        thread::spawn(move || {
            // Shift modes only return 2 bytes.
            // byte[0] = report id
            // byte[1..2]: u16 = shift mode. each bit rep each mode
            // It might be possible to have multiple modes set at the same time, but I'm not sure
            let mut buf: [u8; 3] = [0; 3];
            let &(ref lock, ref _cvar) = &*shared_run_state;

            loop {
                // Make sure we grab the lock in this small context here
                {
                    // Attempt to acquire the lock
                    let started = match lock.try_lock() {
                        Ok(guard) => guard,
                        Err(_) => continue, // Retry if the lock couldn't be acquired immediately
                    };

                    // Check if the thread should exit
                    if !*started {
                        break; // Exit the loop if the variable is false
                    }
                }
                // Now that we unlocked the lock we can continue working

                // Do some work
                buf[0] = 0x04;
                match hid_source_device.get_feature_report(&mut buf) {
                    Ok(bytes_written) => bytes_written,
                    Err(e) => {
                        error!("{}", e);

                        let mut started = match lock.try_lock() {
                            Ok(guard) => guard,
                            Err(_) => continue, // Retry if the lock couldn't be acquired immediately
                        };
                        *started = false;
                        break;
                    }
                };

                {
                    // notify the main gui of the current shift state.
                    // This is only to show the current state on the GUI.
                    let &(ref lock2, ref _cvar2) = &*shared_shift_state;
                    // Attempt to acquire the lock
                    let mut shift = match lock2.try_lock() {
                        Ok(guard) => guard,
                        Err(_) => continue, // Retry if the lock couldn't be acquired immediately
                    };
                    *shift = merge_u8_into_u16(buf[2], buf[1]);
                }

                // println!("Sending data...");
                // This sends the shift data read from the source device directly to the receivers
                for device in &receiver_devices {
                    match device.send_feature_report(&mut buf) {
                        Ok(bytes_written) => bytes_written,
                        Err(e) => {
                            eprintln!("{}", e);

                            // Since we got an error sending the data, lets stop the thread
                            //TODO: Maybe we don't want to do this without letting the user know?
                            let mut started = match lock.try_lock() {
                                Ok(guard) => guard,
                                Err(_) => continue,
                            };
                            *started = false;
                            break;
                        }
                    };
                }

                // Sleep for 200 milliseconds
                thread::sleep(Duration::from_millis(200));
            }
            #[cfg(feature = "logging")] {
                trace!("Exiting Thread...");
            }
        });
        return true;
    }

    fn run(&mut self, ui: &mut Ui, ctx: &Context) {
        let thread_running: bool;
        {
            {
                // Check to see if the worker thread is running
                let &(ref lock, ref _cvar) = &*self.thread_state;
                thread_running = *(lock.lock().unwrap());
            }
        }
        let hidapi = HidApi::new_without_enumerate().expect("Was unable to open hid instance");
        self.refresh_devices(hidapi);

        ui.columns(
            2,
            |columns| {
                columns[0].horizontal(|ui| {
                    let _ = ui.label("Source");
                    egui::ComboBox::from_id_source("Source")
                        .selected_text(format!("{} {}", match self.device_list.iter().find(|&device| device.path == self.source_path) {
                            None => "",
                            Some(item) => &item.firmware,
                        }, match self.device_list.iter().find(|&device| device.path == self.source_path) {
                            None => "",
                            Some(item) => &item.name,
                        }))
                        .show_ui(ui, |ui| {
                            for i in 0..self.device_list.len() {
                                let value = ui.selectable_value(&mut &self.device_list[i].path, &self.source_path, format!("{} {}", self.device_list[i].firmware, self.device_list[i].name));
                                if value.clicked() && !thread_running {
                                    self.source_path = self.device_list[i].path.clone();
                                }
                            }
                        });
                });

                // This is to output the data that we got from the source
                columns[0].horizontal(|ui| {
                    if self.source_path != "" {
                        let state;
                        {
                            let &(ref lock2, ref _cvar) = &*self.shift_state;
                            state = *(lock2.lock().unwrap());
                        }
                        let _ = ui.label("Shift:");
                        for i in 0..5 {
                            let shift = read_bit(state, i);
                            // println!("{:02x} {}", state, shift);
                            let _ = ui.selectable_label(shift, format!("{}", i + 1));
                        }

                        // I'm not sure which of these 3 bits rep which mode
                        let dtnt = read_bit(state, 6);
                        let zoom = read_bit(state, 7);
                        let trim = read_bit(state, 8);

                        let _ = ui.selectable_label(dtnt, "DTNT");
                        let _ = ui.selectable_label(zoom, "ZOOM");
                        let _ = ui.selectable_label(trim, "TRIM");
                    }
                });

                for i in 0..self.receiver_list.len() {
                    columns[0].horizontal(|ui| {
                        egui::ComboBox::from_id_source(i)
                            .selected_text(format!("{}", &self.device_list[self.receiver_list[i]]))
                            .show_ui(ui, |ui| {
                                for j in 0..self.device_list.len() {
                                    let value = ui.selectable_value(&mut &self.device_list[j].path, &self.device_list[self.receiver_list[i]].path, format!("{} {}", self.device_list[j].firmware, self.device_list[j].name));
                                    if value.clicked() && !thread_running {
                                        self.receiver_list[i] = j;
                                    }
                                }
                            });
                    });
                }


                let mut start_stop_button_text = "Start";
                if thread_running {
                    start_stop_button_text = "Stop";
                }
                if columns[1].button(start_stop_button_text).clicked() {
                    // Don't do anything if we didn't select a source and receiver
                    if self.source_path == "" || self.receiver_list.len() == 0 {
                        return;
                    }

                    #[cfg(feature = "logging")] {
                        trace!("Toggling run thread...");
                    }
                    let is_started;
                    {
                        let &(ref lock, ref cvar) = &*self.thread_state;
                        let mut started = lock.lock().unwrap();
                        is_started = *started;
                        *started = !*started;
                        cvar.notify_all();
                    }

                    if !is_started {
                        if !self.spawn_worker() {
                            {
                                let &(ref lock, ref cvar) = &*self.thread_state;
                                let mut started = lock.lock().unwrap();
                                *started = false;
                                cvar.notify_all();
                            }
                        }
                    }
                    #[cfg(feature = "logging")] {
                        trace!("Done");
                    }
                }

                if columns[1].button("Add Receiver").clicked() {
                    if !thread_running {
                        self.receiver_list.push(0);
                    }
                }
                if columns[1].button("Remove Receiver").clicked() {
                    if !thread_running {
                        self.receiver_list.pop();
                    }
                }
                // if columns[1].button("Save").clicked() {}
                if columns[1].button("About").clicked() {
                    self.state = State::About;
                }
                if columns[1].button("Exit").clicked() {
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                }
            }
        );
    }
}


impl eframe::App for ShiftTool {
    fn update(&mut self, ctx: &Context, _frame: &mut eframe::Frame) {
        ctx.request_repaint_after(Duration::ZERO);

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::Resize::default()
                .default_width(INITIAL_WIDTH)
                .default_height(INITIAL_HEIGHT)
                .auto_sized()
                .show(ui, |ui| {
                    match self.state {
                        State::Initialising => self.init(),
                        State::About => self.about_screen(ui),
                        State::Running => self.run(ui, ctx),
                    }
                });
        });
    }

    fn on_exit(&mut self, _gl: Option<&glow::Context>) {
        #[cfg(feature = "logging")] {
            info!("Shutting down...");
        }
        // Make sure we clean up our thread
        {
            let &(ref lock, ref cvar) = &*self.thread_state;
            let mut started = lock.lock().unwrap();
            *started = false;
            cvar.notify_all();
        }
        // Give the thread some time to get the shutdown event
        thread::sleep(Duration::from_millis(200));
        #[cfg(feature = "logging")] {
            info!("Done");
        }
    }
}