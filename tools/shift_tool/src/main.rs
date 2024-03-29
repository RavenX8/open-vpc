#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
// hide console window on Windows in release
extern crate hidapi;

use std::ffi::CString;
use std::rc::Rc;
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::Duration;

use clap::Parser;
use eframe::egui;
use hidapi::HidApi;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Allows all firmware versions
    #[arg(short, long, default_value_t = false)]
    skip_firmware: bool,
}

fn main() -> eframe::Result<()>{
    env_logger::init(); // Log to stderr (if you run with `RUST_LOG=debug`).
    let _ = Args::parse();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([700.0, 240.0]),
        ..Default::default()
    };

    eframe::run_native(
        "OpenVPC - Shift Tool",
        options,
        Box::new(|_cc| Box::new(MyApp::default())),
    )
}

fn read_bit(value: u16, position: u8) -> bool {
    (value & (1 << position)) != 0
}

// fn set_bit(value: u16, position: u8) -> u16 {
//     value | (1 << position)
// }

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
    shift_mode: [bool; 8],
    firmware: Rc<String>,
}

impl std::fmt::Display for VpcDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{} {}", self.firmware, self.name)
    }
}

struct MyApp {
    device_list: Vec<VpcDevice>,
    source_path: String,
    shift_state: Arc<(Mutex<u16>, Condvar)>,
    receiver_list: Vec<usize>,
    run_state: Arc<(Mutex<bool>, Condvar)>,
    hidapi: HidApi,
}

impl Default for MyApp {
    fn default() -> Self {
        Self {
            device_list: vec![],
            shift_state: Arc::new((Mutex::new(0), Condvar::new())),
            source_path: "".to_string(),
            receiver_list: vec![],
            run_state: Arc::new((Mutex::new(false), Condvar::new())),
            hidapi: HidApi::new_without_enumerate().expect("Was unable to open hid instance"),
        }
    }
}

impl MyApp {
    fn spawn_worker(&mut self) {
        let reference_to_self = self;
        let shared_shift_state = reference_to_self.shift_state.clone();
        let shared_run_state = reference_to_self.run_state.clone();
        let source_device = CString::new(&*reference_to_self.source_path.clone()).expect("Failed to create CString");
        let mut hidapi = HidApi::new_without_enumerate().expect("Was unable to open hid instance");
        match hidapi.reset_devices() {
            Ok(_api) => {}
            Err(e) => {
                eprintln!("Error: {}", e);
            }
        };
        match hidapi.add_devices(0x3344, 0) {
            Ok(_api) => {
                println!("Got devices");
            }
            Err(e) => {
                eprintln!("Error: {}", e);
            }
        }

        let hid_source_device = match hidapi.open_path(source_device.as_ref()) {
            Ok(device) => device,
            Err(_) => {
                eprintln!("Was unable to open the source device {:?}", source_device);
                return;
            }
        };
        hid_source_device.set_blocking_mode(false).expect("unable to set source blocking mode");

        let mut receiver_devices = vec![];
        for i in 0..reference_to_self.receiver_list.len() {
            let receiver_path = match CString::new(&*reference_to_self.device_list[reference_to_self.receiver_list[i]].path.clone()) {
                Ok(str) => str,
                Err(e) => {
                    eprintln!("Error: {}", e);
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


        println!("Launching Thread...");
        thread::spawn(move || {
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
                // println!("Working...");
                buf[0] = 0x04;
                match hid_source_device.get_feature_report(&mut buf) {
                    Ok(bytes_written) => bytes_written,
                    Err(e) => {
                        eprintln!("{}", e);

                        let mut started = match lock.try_lock() {
                            Ok(guard) => guard,
                            Err(_) => continue, // Retry if the lock couldn't be acquired immediately
                        };
                        *started = false;
                        break;
                    }
                };

                {
                    let &(ref lock2, ref _cvar2) = &*shared_shift_state;
                    // Attempt to acquire the lock
                    let mut shift = match lock2.try_lock() {
                        Ok(guard) => guard,
                        Err(_) => continue, // Retry if the lock couldn't be acquired immediately
                    };
                    *shift = merge_u8_into_u16(buf[2], buf[1]);
                }

                // println!("Sending data...");
                for device in &receiver_devices {
                    match device.send_feature_report(&mut buf) {
                        Ok(bytes_written) => bytes_written,
                        Err(e) => {
                            eprintln!("{}", e);

                            let mut started = match lock.try_lock() {
                                Ok(guard) => guard,
                                Err(_) => continue, // Retry if the lock couldn't be acquired immediately
                            };
                            *started = false;
                            break;
                        }
                    };
                }

                // Sleep for 200 milliseconds
                thread::sleep(Duration::from_millis(200));
            }

            println!("Exiting Thread...");
        });
    }
}


impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let thread_running;
        {
            {
                // Check to see if the worker thread is running
                let &(ref lock, ref _cvar) = &*self.run_state;
                thread_running = *(lock.lock().unwrap());
            }
        }

        match self.hidapi.reset_devices() {
            Ok(_api) => {}
            Err(e) => {
                eprintln!("Error: {}", e);
            }
        };
        match self.hidapi.add_devices(0x3344, 0) {
            Ok(_api) => {}
            Err(e) => {
                eprintln!("Error: {}", e);
            }
        }

        if self.hidapi.device_list().count() != self.device_list.len() {
            self.device_list.clear();
        }

        let mut new_devices: Vec<VpcDevice> = vec![];
        for device in self.hidapi.device_list() {
            let firmware_string = device.manufacturer_string().unwrap_or_else(|| "");
            let device_name = device.product_string().unwrap_or_else(|| "");
            let device_path = device.path().to_str().expect("Invalid UTF-8").to_string();
            let mut found = false;
            if self.device_list.is_empty() {
                found = self.device_list.iter().any(|r| r.path == device_path.clone());
            }
            if !found {
                if is_supported(firmware_string.to_owned().into()) {
                    new_devices.push(VpcDevice { path: device_path.into(), name: device_name.to_owned().into(), shift_mode: [false; 8], firmware: firmware_string.to_owned().into() });
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

            merged_list.insert(0, VpcDevice { path: String::from("").into(), name: String::from("-NO CONNECTION (Select device from the list)-").into(), shift_mode: [false; 8], firmware: String::from("").into() });
            self.device_list = merged_list.clone();
        }

        if self.device_list.is_empty() {
            self.device_list.insert(0, VpcDevice { path: String::from("").into(), name: String::from("-NO CONNECTION (Select device from the list)-").into(), shift_mode: [false; 8], firmware: String::from("").into() });
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("VPC Shift Tool");
            ui.columns(2, |columns| {
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
                        let _ = ui.selectable_label(false, "DTNT");
                        let _ = ui.selectable_label(false, "ZOOM");
                        let _ = ui.selectable_label(false, "TRIM");
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

                    println!("Toggling run thread...");
                    let is_started;
                    {
                        let &(ref lock, ref cvar) = &*self.run_state;
                        let mut started = lock.lock().unwrap();
                        is_started = *started;
                        *started = !*started;
                        cvar.notify_all();
                    }

                    if !is_started {
                        self.spawn_worker();
                    }
                    println!("Done");
                }

                if columns[1].button("Add Receiver").clicked() {
                    self.receiver_list.push(0);
                }
            });
        });
        // thread::sleep(Duration::from_millis(10));
    }
}