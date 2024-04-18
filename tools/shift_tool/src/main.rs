#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

// hide console window on Windows in release
extern crate hidapi;

use std::ops::{Index, IndexMut};
use std::rc::Rc;
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::Duration;

use clap::Parser;
use eframe::{egui, glow};
use eframe::egui::{Color32, Context, Ui};
use hidapi::{DeviceInfo, HidApi};
#[cfg(feature = "logging")]
use log::{debug, error, info, trace};

mod about;

const PROGRAM_TITLE: &str = "OpenVPC - Shift Tool";
const INITIAL_WIDTH: f32 = 720.0;
const INITIAL_HEIGHT: f32 = 260.0;

const DISABLED_COLOR: Color32 = Color32::from_rgb(255, 0, 0);

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

fn set_bit(value: u8, bit_position: u8, bit_value: bool) -> u8 {
    if bit_value {
        value | (1 << bit_position) // Set the bit to 1
    } else {
        value & !(1 << bit_position) // Set the bit to 0
    }
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
        String::from("VIRPIL Controls 20220720"),
        String::from("VIRPIL Controls 20230328")
    ];

    fixed_list.contains(&input)
}

fn calculate_full_device_name(device_info: &DeviceInfo) -> String {
    let full_name = format!("{:04x}:{:04x}:{}:{}", device_info.vendor_id(), device_info.product_id(), device_info.serial_number().unwrap_or_default(), device_info.usage());
    full_name
}

#[derive(Debug, PartialEq, PartialOrd, Ord, Eq, Hash, Clone)]
struct VpcDevice {
    path: String,
    full_name: String,
    name: Rc<String>,
    firmware: Rc<String>,
    vendor_id: u16,
    product_id: u16,
    serial_number: String,
    usage: u16,
    active: bool,
}

impl Default for VpcDevice {
    fn default() -> Self {
        Self {
            path: String::from("").into(),
            full_name: String::from("").into(),
            name: String::from("-NO CONNECTION (Select device from the list)-").into(),
            firmware: String::from("").into(),
            vendor_id: 0,
            product_id: 0,
            serial_number: String::from(""),
            usage: 0,
            active: false,
        }
    }
}

impl std::fmt::Display for VpcDevice {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        if self.vendor_id == 0 && self.product_id == 0 {
            write!(f, "{}", self.name)
        } else {
            write!(f, "VID: {:04x} PID: {:04x} {} ({})", self.vendor_id, self.product_id, self.name, self.firmware)
        }
    }
}

#[derive(Debug, Copy, Clone)]
enum ShiftModifiers {
    OR = 0,
    AND = 1,
    XOR = 2,
}

impl std::fmt::Display for ShiftModifiers {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let text = match self {
            ShiftModifiers::OR => "OR",
            ShiftModifiers::AND => "AND",
            ShiftModifiers::XOR => "XOR",
        };
        write!(f, "{}", text)
    }
}

#[derive(Debug, Copy, Clone)]
struct ModifiersArray {
    data: [ShiftModifiers; 8],
}

impl Index<usize> for ModifiersArray {
    type Output = ShiftModifiers;

    fn index(&self, index: usize) -> &ShiftModifiers {
        &self.data[index]
    }
}

impl IndexMut<usize> for ModifiersArray {
    fn index_mut(&mut self, index: usize) -> &mut ShiftModifiers {
        &mut self.data[index]
    }
}

struct ShiftTool {
    state: State,
    device_list: Vec<VpcDevice>,
    source_list: Vec<usize>,
    receiver_list: Vec<usize>,
    source_states: Vec<Arc<(Mutex<u16>, Condvar)>>,
    shift_state: Arc<(Mutex<u16>, Condvar)>,
    thread_state: Arc<(Mutex<bool>, Condvar)>,
    shift_modifiers: ModifiersArray,
    source_state_enabled: Vec<[bool; 8]>,
}

impl Default for ShiftTool {
    fn default() -> Self {
        Self {
            state: State::Initialising,
            device_list: vec![],
            source_list: vec![],
            receiver_list: vec![],
            source_states: vec![],
            shift_state: Arc::new((Mutex::new(0), Condvar::new())),
            thread_state: Arc::new((Mutex::new(false), Condvar::new())),
            shift_modifiers: ModifiersArray{data: [ShiftModifiers::OR; 8]},
            source_state_enabled: vec![]
        }
    }
}

impl ShiftTool {
    fn about_screen(&mut self, ui: &mut Ui) {
        ui.set_width(INITIAL_WIDTH);
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.centered_and_justified(|ui| {
                    let about = "About ";
                    ui.heading(format!("{}{}", about, PROGRAM_TITLE));
                });
            });
            for line in about::about() {
                ui.horizontal(|ui| {
                    ui.centered_and_justified(|ui| {
                        ui.add(egui::Label::new(line));
                    });
                });
            }

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
            let full_name = calculate_full_device_name(device);
            #[cfg(feature = "logging")] {
                info!("{}", full_name );
            }
            let firmware_string = device.manufacturer_string().unwrap_or_else(|| "");
            let device_name = device.product_string().unwrap_or_else(|| "");
            let device_path = device.path().to_str().expect("Invalid UTF-8").to_string();
            let serial = device.serial_number().unwrap_or_default();
            let mut found = false;
            if !self.device_list.is_empty() {
                found = self.device_list.iter().any(|r| r.path == device_path.clone());
            }
            if !found {
                if is_supported(firmware_string.to_owned().into()) {
                    new_devices.push(VpcDevice { path: device_path.into(), full_name: full_name.to_string(), name: device_name.to_owned().into(), firmware: firmware_string.to_owned().into(), vendor_id: device.vendor_id(), product_id: device.product_id(), serial_number: serial.to_string(), usage: device.usage(), active: false });
                }
            }
        }

        if !new_devices.is_empty() {
            if !self.device_list.is_empty() {
                self.device_list.remove(0);
            }

            let mut merged_list = self.device_list.clone();
            merged_list.extend(new_devices.into_iter());

            // product_id is the only unique value from the device
            merged_list.sort_by(|a, b| a.product_id.cmp(&b.product_id));
            merged_list.dedup_by(|a, b| a.product_id.eq(&b.product_id));

            merged_list.insert(0, VpcDevice::default());
            self.device_list = merged_list.clone();
        }

        if self.device_list.is_empty() {
            self.device_list.insert(0, VpcDevice::default());
        }
    }

    fn spawn_worker(&mut self) -> bool {
        let reference_to_self = self;
        let shared_shift_state = reference_to_self.source_states.clone();
        let shared_shift_modifiers = reference_to_self.shift_modifiers.clone();
        let shared_source_state_enabled = reference_to_self.source_state_enabled.clone();
        let shared_final_shift_state = reference_to_self.shift_state.clone();
        let shared_run_state = reference_to_self.thread_state.clone();
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
        let mut source_devices = vec![];
        for i in 0..reference_to_self.source_list.len() {
            // product_id is the only unique value from the device
            let hid_device = match hidapi.open_serial(reference_to_self.device_list[reference_to_self.source_list[i]].vendor_id, reference_to_self.device_list[reference_to_self.source_list[i]].product_id, &*reference_to_self.device_list[reference_to_self.source_list[i]].serial_number) {
                Ok(device) => device,
                Err(_) => {
                    reference_to_self.device_list[reference_to_self.source_list[i]].active = false;
                    continue
                },
            };
            reference_to_self.device_list[reference_to_self.source_list[i]].active = true;
            hid_device.set_blocking_mode(false).expect("unable to set receiver blocking mode");
            source_devices.push(hid_device);
        }

        // List of device handles for the thread to write to
        let mut receiver_devices = vec![];
        // Loop for all receiver devices and open a handle to them
        for i in 0..reference_to_self.receiver_list.len() {
            // product_id is the only unique value from the device
            let hid_device = match hidapi.open_serial(reference_to_self.device_list[reference_to_self.receiver_list[i]].vendor_id, reference_to_self.device_list[reference_to_self.receiver_list[i]].product_id, &*reference_to_self.device_list[reference_to_self.receiver_list[i]].serial_number) {
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
            let mut finalbuf: [u8; 3] = [0; 3];
            let &(ref lock, ref _cvar) = &*shared_run_state;

            loop {
                let mut shift_states: Vec<u16> = vec![];
                shift_states.resize(source_devices.len(), 0);
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
                finalbuf[0] = 0x04;
                let mut index = 0;
                for device in &source_devices {
                    match device.get_feature_report(&mut buf) {
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
                        let &(ref lock2, ref _cvar2) = &*shared_shift_state[index];
                        // Attempt to acquire the lock
                        let mut shift = match lock2.try_lock() {
                            Ok(guard) => guard,
                            Err(_) => continue, // Retry if the lock couldn't be acquired immediately
                        };
                        shift_states[index] = merge_u8_into_u16(buf[2], buf[1]);
                        *shift = shift_states[index];
                    }
                    index = index + 1;
                }

                for i in 0..8 {
                    let mut values: Vec<bool> = vec![];
                    for j in 0..shift_states.len() {
                        if (&shared_source_state_enabled[j])[<u8 as Into<usize>>::into(i)] == true {
                            values.push(read_bit(shift_states[j], i));
                        }
                    }

                    let modifier = shared_shift_modifiers[i.into()];
                    let final_value = match modifier {
                        ShiftModifiers::OR => values.iter().fold(false, |acc, &x| acc | x),
                        ShiftModifiers::AND => values.iter().fold(true, |acc, &x| acc & x),
                        ShiftModifiers::XOR => values.iter().fold(false, |acc, &x| acc ^ x),
                    };
                    finalbuf[1] = set_bit(finalbuf[1], i, final_value);
                }

                {
                    // notify the main gui of the current shift state.
                    // This is only to show the current state on the GUI.
                    let &(ref lock2, ref _cvar2) = &*shared_final_shift_state;
                    // Attempt to acquire the lock
                    let mut shift = match lock2.try_lock() {
                        Ok(guard) => guard,
                        Err(_) => continue, // Retry if the lock couldn't be acquired immediately
                    };
                    *shift = merge_u8_into_u16(finalbuf[2], finalbuf[1]);
                }

                // println!("Sending data...");
                // This sends the shift data read from the source device directly to the receivers
                for device in &receiver_devices {
                    match device.send_feature_report(&mut finalbuf) {
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

                // Make sure we clear the buffer
                finalbuf[1] = 0;
                finalbuf[2] = 0;

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

        if self.source_list.len() == 0 {
            self.source_list.push(0);
            self.source_states.push(Arc::new((Mutex::new(0), Condvar::new())));
            self.source_state_enabled.push([true; 8]);
        }

        ui.columns(
            2,
            |columns| {
                // This is to output the data that we got from the source
                for i in 0..self.source_list.len() {
                    columns[0].horizontal(|ui| {
                        let _ = ui.label(format!("Source {}", i+1));
                        egui::ComboBox::from_id_source(format!("Source {}", i+1))
                            .width(500.0)
                            .selected_text(format!("{}", &self.device_list[self.source_list[i]]))
                            .show_ui(ui, |ui| {
                                for j in 0..self.device_list.len() {
                                    let value = ui.selectable_value(&mut &self.device_list[j].full_name, &self.device_list[self.source_list[i]].full_name, format!("{}", self.device_list[j]));
                                    if value.clicked() && !thread_running {
                                        self.source_list[i] = j;
                                    }
                                }
                            });
                    });
                    columns[0].horizontal(|ui| {
                        let state;
                        {
                            let &(ref lock2, ref _cvar) = &*self.source_states[i];
                            state = *(lock2.lock().unwrap());
                        }

                        let _ = ui.label("Shift:   ");
                        let mut color;
                        for j in 0..5 {
                            let shift = read_bit(state, j);

                            color = egui::Color32::default();
                            if self.source_state_enabled[i][<u8 as Into<usize>>::into(j)] == false {
                                color = DISABLED_COLOR;
                            }
                            if ui.selectable_label(shift, egui::RichText::new(format!("{}", j + 1)).background_color(color)).clicked() && !thread_running {
                                self.source_state_enabled[i][<u8 as Into<usize>>::into(j)] = !self.source_state_enabled[i][<u8 as Into<usize>>::into(j)];
                            };
                        }

                        // I'm not sure which of these 3 bits rep which mode
                        let dtnt = read_bit(state, 5);
                        let zoom = read_bit(state, 6);
                        let trim = read_bit(state, 7);

                        color = egui::Color32::default();
                        if self.source_state_enabled[i][5] == false {
                            color = DISABLED_COLOR;
                        }
                        if ui.selectable_label(dtnt, egui::RichText::new("DTNT").background_color(color)).clicked() && !thread_running {
                            self.source_state_enabled[i][5] = !self.source_state_enabled[i][5];
                        }

                        color = egui::Color32::default();
                        if self.source_state_enabled[i][6] == false {
                            color = DISABLED_COLOR;
                        }
                        if ui.selectable_label(zoom, egui::RichText::new("ZOOM").background_color(color)).clicked() && !thread_running {
                            self.source_state_enabled[i][6] = !self.source_state_enabled[i][6];
                        }

                        color = egui::Color32::default();
                        if self.source_state_enabled[i][7] == false {
                            color = DISABLED_COLOR;
                        }
                        if ui.selectable_label(trim, egui::RichText::new("TRIM").background_color(color)).clicked() && !thread_running {
                            self.source_state_enabled[i][7] = !self.source_state_enabled[i][7];
                        }
                        let active = self.device_list[self.source_list[i]].active;
                        let mut active_text = "OFFLINE";
                        if active == true {
                            active_text = "ONLINE";
                        }
                        let _ = ui.selectable_label(active, active_text);
                    });
                    columns[0].separator();
                }
                columns[0].horizontal(|ui| {
                    let _ = ui.label("Rules:");
                    for j in 0..8 {
                        let rule = self.shift_modifiers[j];
                        if ui.selectable_label(false, format!("{}", rule)).clicked() && !thread_running {
                            self.shift_modifiers[j] = match self.shift_modifiers[j] {
                                ShiftModifiers::OR => ShiftModifiers::AND,
                                ShiftModifiers::AND => ShiftModifiers::XOR,
                                ShiftModifiers::XOR => ShiftModifiers::OR,
                            }
                        }
                    }
                });
                columns[0].horizontal(|ui| {
                    let state;
                    {
                        let &(ref lock2, ref _cvar) = &*self.shift_state;
                        state = *(lock2.lock().unwrap());
                    }

                    let _ = ui.label("Result:");
                    for j in 0..5 {
                        let shift = read_bit(state, j);
                        let _ = ui.selectable_label(shift, format!("{}", j + 1));
                    }

                    // I'm not sure which of these 3 bits rep which mode
                    let dtnt = read_bit(state, 5);
                    let zoom = read_bit(state, 6);
                    let trim = read_bit(state, 7);

                    let _ = ui.selectable_label(dtnt, "DTNT");
                    let _ = ui.selectable_label(zoom, "ZOOM");
                    let _ = ui.selectable_label(trim, "TRIM");
                });
                columns[0].separator();
                for i in 0..self.receiver_list.len() {
                    columns[0].horizontal(|ui| {
                        egui::ComboBox::from_id_source(i)
                            .width(500.0)
                            .selected_text(format!("{}", &self.device_list[self.receiver_list[i]]))
                            .show_ui(ui, |ui| {
                                for j in 0..self.device_list.len() {
                                    let value = ui.selectable_value(&mut &self.device_list[j].full_name, &self.device_list[self.receiver_list[i]].full_name, format!("{}", self.device_list[j]));
                                    if value.clicked() && !thread_running {
                                        self.receiver_list[i] = j;
                                    }
                                }
                            });
                    });
                }

                columns[1].vertical(|ui| {
                    let mut color = egui::Color32::default();
                    let mut start_stop_button_text = "Start";
                    if thread_running {
                        start_stop_button_text = "Stop";
                        color = DISABLED_COLOR;
                    }
                    if ui.button(egui::RichText::new(start_stop_button_text).background_color(color))
                        .clicked() {
                        // Don't do anything if we didn't select a source and receiver
                        if self.source_list.len() == 0 || self.receiver_list.len() == 0 {
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
                        } else {
                            for source_state in self.source_states.clone() {
                                {
                                    // reset each source state
                                    let &(ref lock, ref cvar) = &*source_state;
                                    let mut state = lock.lock().unwrap();
                                    *state = 0;
                                    cvar.notify_all();
                                }
                            }
                            {
                                // reset result state
                                let &(ref lock, ref cvar) = &*self.shift_state;
                                let mut state = lock.lock().unwrap();
                                *state = 0;
                                cvar.notify_all();
                            }
                            for i in 0..self.device_list.len() {
                                self.device_list[i].active = false;
                            }
                        }
                        #[cfg(feature = "logging")] {
                            trace!("Done");
                        }
                    }

                    if ui.add_enabled(!thread_running, egui::Button::new("Add Source")).clicked() {
                        self.source_list.push(0);
                        self.source_states.push(Arc::new((Mutex::new(0), Condvar::new())));
                        self.source_state_enabled.push([true; 8]);
                    }
                    if self.source_list.len() > 1 && ui.add_enabled(!thread_running, egui::Button::new("Remove Source")).clicked() {

                        self.source_list.pop();
                        self.source_states.pop();
                        self.source_state_enabled.pop();

                    }

                    if ui.add_enabled(!thread_running, egui::Button::new("Add Receiver")).clicked() {
                        self.receiver_list.push(0);
                    }

                    if self.receiver_list.len() > 0 && ui.add_enabled(!thread_running, egui::Button::new("Remove Receiver")).clicked() {
                        self.receiver_list.pop();
                    }
                    // if columns[1].button("Save").clicked() {}
                    if ui.button("About").clicked() {
                        self.state = State::About;
                    }
                    if ui.button("Exit").clicked() {
                        ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });
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