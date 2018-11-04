extern crate rand;
extern crate libusb;

use std::env;
use std::str;
use std::io::Read;
use std::io::Seek;
use std::io::SeekFrom;
use std::fs;
use std::fs::File;
use std::time::Duration;
use std::path::PathBuf;
use std::u8;
use rand::Rng;


/*
 * sets and reads fan and pump speeds
 */
struct UsbController<'a> {
    handle: libusb::DeviceHandle<'a>,
    interface: u8,
    read_address: u8,
    write_address: u8
}


impl<'a> UsbController<'a> {
    fn open(device: &'a libusb::Device) -> UsbController<'a> {

        let mut selected_interface = 0x00;
        let mut selected_read_address = 0x82;
        let mut selected_write_address = 0x00;

        let config = device.active_config_descriptor().unwrap();
        for interface in config.interfaces() {
            selected_interface = interface.number();

            for descriptor in interface.descriptors() {

                for endpoint in descriptor.endpoint_descriptors() {
                    if endpoint.direction() == libusb::Direction::In {
                        selected_read_address = endpoint.address();
                    }
                    else {
                        selected_write_address = endpoint.address();
                    }
                }
            }
        }

        println!("Opening interface 0x{:02x}", selected_interface);
        println!("Read address 0x{:02x}", selected_read_address);
        println!("Write address 0x{:02x}", selected_write_address);

        return UsbController {
            handle: device.open().unwrap(),
            interface: selected_interface,
            read_address: selected_read_address,
            write_address: selected_write_address,
        }
    }

    fn claim(&mut self) {
        println!("Claiming device");
        self.handle.detach_kernel_driver(self.interface);
        self.handle.claim_interface(self.interface);
    }

    fn release(&mut self) {
        self.handle.release_interface(self.interface);
    }


    fn set_color(&mut self, mode: u8, r: u8, g: u8, b: u8) {
        self.write_init_commands();
    }


    fn read_response(&mut self, out: &mut [u8]) -> usize {
        let result = self.handle.read_interrupt(self.read_address, out, Duration::from_secs(3)).unwrap_or(0);
        print!("read {:03} bytes: ", result);
        for b in out {
            print!("{:x}, ", b);
        }
        println!("");
        return result;
    }


    /**
     * Writes 7 byte packets to 0x0210
     */
    fn send_ctl_packet_7(&mut self, msg: &[u8]) {
        println!("write ctl {:03} bytes", msg.len());
        let result = self.handle.write_control(0x21, 9, 0x0210, 1, msg, Duration::from_secs(10)).unwrap();
    }

    /**
     * Writes 20 byte packets to 0x0211
     */
    fn send_ctl_packet_20(&mut self, msg: &[u8]) {
        println!("write ctl {:03} bytes", msg.len());
        let result = self.handle.write_control(0x21, 9, 0x0211, 1, msg, Duration::from_secs(10)).unwrap();
    }

    fn send_0_root(&mut self, di: u8, fi: u8, swid: u8, param_a: u8, param_b: u8, param_c: u8) {
        let byte_3 = 0x00 + swid;
        let buf: [u8; 7] = [
            0x10, di, fi, byte_3, param_a, param_b, param_c
        ];
        self.send_ctl_packet_7(&buf);

        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
    }

    /**
     * get feature data
     */
    fn send_1_get_features(&mut self, di: u8, fi: u8, swid: u8, param_a: u8, param_b: u8, param_c: u8) {
        let byte_3 = 0x10 + swid;
        let buf: [u8; 7] = [
            0x10, di, fi, byte_3, param_a, param_b, param_c
        ];
        self.send_ctl_packet_7(&buf);

        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
    }

    /**
     * checks a feature is available?
     */
    fn send_2_connected(&mut self, di: u8, fi: u8, swid: u8, param_a: u8, param_b: u8, param_c: u8) {
        let byte_3 = 0x20 + swid;
        let buf: [u8; 7] = [
            0x10, di, fi, byte_3, param_a, param_b, param_c
        ];
        self.send_ctl_packet_7(&buf);

        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
    }

    /**
     * Returns an error code and current profile
     */
    fn send_4_status(&mut self, di: u8, fi: u8, swid: u8) {
        let byte_3 = 0x40 + swid;
        let buf: [u8; 7] = [
            0x10, di, fi, byte_3, 0x00, 0x00, 0x00
        ];
        self.send_ctl_packet_7(&buf);

        // returns err code and current profile
        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
    }

    /**
     * Reads the record 10 bytes at a time, starting at offset
     * a and profile = 1, 1 : 0, {1-5}
     */
    fn send_5_read_record(&mut self, di: u8, fi: u8, swid: u8, id_a: u8, profile: u8, offset: u8) {
        let byte_3 = 0x50 + swid;
        let buf: [u8; 20] = [
            0x11, di, fi, byte_3, id_a, profile, 0x00, offset,
            0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        ];
        self.send_ctl_packet_20(&buf);

        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
    }

    /**
     * Profile is 0x01 for profile 1
     */
    fn send_6_start_record(&mut self, di: u8, fi: u8, swid: u8, profile: u8) {
        let byte_3 = 0x60 + swid;
        let buf20: [u8; 20] = [
            0x11, di, fi, byte_3, 0x00, profile, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        ];
        self.send_ctl_packet_20(&buf20);

        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
    }


    fn send_7_record(&mut self, di: u8, fi: u8, swid: u8, params: [u8; 16]) {
        let command_7 = 0x70 + swid;
        let buf: [u8; 20] = [
            0x11, di, fi, command_7, params[0], params[1], params[2], params[3],
            params[4], params[5], params[6], params[7], params[8], params[9], params[10], params[11],
            params[12], params[13], params[14], params[15],
        ];
        self.send_ctl_packet_20(&buf);

        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
    }

    /**
     * A big set of data send over multiple packets
     */
    fn send_group_7_record(&mut self, di: u8, fi: u8, swid: u8) {
        println!("Set DPI");

        // 0x01 = 1000hz
        // 0x02 = 500hz
        let poll_rate = 0x01;

        // seems to link to the last 2 bytes somehow
        // 1a -> 31 fe
        // 1c -> 8e e5
        // 2f -> e1 b6
        // 30 -> 4c 75
        // 33 -> 70 86
        // 35 -> e5 73
        // 2f -> e1 b6
        // 37 -> 96 20

        // 500hz
        // 30 -> 4c 75
        // 31 -> 03 d5
        // 32 -> 3f 26
        // 34 -> aa d3
        // 36 -> d9 80
        let cmd_param = 0x1a;
        let checksum_a = 0x31;
        let checksum_b = 0xfe;

        // send a 256 byte set of data
        // Line 0 is the dpi settings
        // should be ordered low to high
        // 1st: 0x0190
        // 2st: 0x0320

        let packets: [[u8; 16]; 16] = [
            [poll_rate, 0x01, 0x00, 0x90, 0x01, 0x20, 0x03, 0x40, 0x06, 0x80, 0x0c, 0x00, 0x00, 0xff, 0xff, 0xff],
            [0xff, 0x00, cmd_param, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0x80, 0x01, 0x00, 0x01, 0x80, 0x01, 0x00, 0x02, 0x80, 0x01, 0x00, 0x04, 0x80, 0x01, 0x00, 0x08],
            [0x80, 0x01, 0x00, 0x10, 0x90, 0x05, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0x80, 0x01, 0x00, 0x01, 0x80, 0x01, 0x00, 0x02, 0x80, 0x01, 0x00, 0x04, 0x80, 0x01, 0x00, 0x08],
            [0x80, 0x01, 0x00, 0x01, 0x80, 0x01, 0x00, 0x02, 0x80, 0x01, 0x00, 0x04, 0x80, 0x01, 0x00, 0x08],
            [0x80, 0x01, 0x00, 0x10, 0x90, 0x05, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00, 0x00],
            [0x50, 0x00, 0x72, 0x00, 0x6f, 0x00, 0x66, 0x00, 0x69, 0x00, 0x6c, 0x00, 0x65, 0x00, 0x20, 0x00], // "Profile"
            [0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], // "1" -- Profile 1 is modified
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00, 0x00],
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, checksum_a, checksum_b],
        ];

        for &packet in &packets {
            self.send_7_record(di, fi, swid, packet);
        }
    }

    fn write_init_commands(&mut self) {

        let device_index = 0xff;
        let feature_index = 0x0f; // 0x0f is dpi, 0x0e is leds
        let swid = 0xc;

        // 0x7c for color panel
        // 0x7d for dpi panel
        // either end in 'c' or 'd' will work...
        let command_type_0 = 0x0c; // reset?
        let command_type_1 = 0x1c; // c
        let command_type_2 = 0x2c; // check connected
        let command_type_3 = 0x3c; // color setting
        let command_type_4 = 0x4c; // get status, returns err and current profile
        let command_type_6 = 0x6c;
        let command_type_7 = 0x7c;
        let command_type_8 = 0x8c;
        let command_type_b = 0xbc; // error handling
        let command_type_c = 0xcc; // error handling

        println!("Sending commands");

        // reset?
        self.send_1_get_features(device_index, 0x00, swid, 0x00, 0x03, 0x00);
        self.send_1_get_features(device_index, 0x00, swid, 0x00, 0x03, 0x39);

        self.send_0_root(device_index, 0x00, swid, 0x00, 0x03, 0x00);
        self.send_0_root(device_index, 0x02, swid, 0x00, 0x00, 0x00);

        self.send_1_get_features(device_index, 0x02, swid, 0x00, 0x00, 0x00);

        self.send_0_root(device_index, 0x00, swid, 0x00, 0x05, 0x00);

        self.send_2_connected(device_index, 0x03, swid, 0x00, 0x00, 0x00);

        self.send_0_root(device_index, 0x02, swid, 0x00, 0x00, 0x00);
        self.send_0_root(device_index, 0x00, swid, 0x10, 0x01, 0x00);
        self.send_0_root(device_index, 0x00, swid, 0x1f, 0x20, 0x00);
        self.send_0_root(device_index, 0x00, swid, 0x10, 0x00, 0x00);

        self.send_1_get_features(device_index, 0x02, swid, 0x00, 0x00, 0x00);

        self.send_0_root(device_index, 0x03, swid, 0x00, 0x00, 0x00);

        self.send_1_get_features(device_index, 0x03, swid, 0x00, 0x00, 0x00);
        self.send_1_get_features(device_index, 0x03, swid, 0x10, 0x00, 0x00);


        self.send_0_root(device_index, 0x00, swid, 0x81, 0x10, 0x00);
        self.send_0_root(device_index, 0x10, swid, 0x00, 0x00, 0x00);
        self.send_0_root(device_index, 0x00, swid, 0x80, 0x90, 0x00);
        self.send_0_root(device_index, 0x00, swid, 0x13, 0x90, 0x00);
        self.send_0_root(device_index, 0x00, swid, 0x81, 0x00, 0x00);

        self.send_2_connected(device_index, 0x0f, swid, 0x00, 0x00, 0x00);

        self.send_0_root(device_index, 0x00, swid, 0x80, 0x70, 0x00);
        self.send_0_root(device_index, 0x0e, swid, 0x00, 0x00, 0x00);

        self.send_1_get_features(device_index, 0x0e, swid, 0x00, 0x00, 0x00);

        // ...

        let reset_w: [u8; 7] = [
            0x10, 0xff, 0x0e, command_type_8, 0x01, 0x01, 0x00
        ];
        self.send_ctl_7(&reset_w);

        self.send_4_status(device_index, feature_index, swid);


        // the fix for err 1, x
        let fix: [u8; 7] = [
            0x10, 0xff, 0x0f, command_type_3, 0x00, 0x01, 0x00
        ];
        self.send_ctl_7(&fix);

        self.send_4_status(device_index, feature_index, swid);

        // these 7 byte packets seem to reset state
        // used on setting dpi
        let buf7_8: [u8; 7] = [
            0x10, 0xff, 0x0f, command_type_b, 0x00, 0x00, 0x00
        ];
        self.send_ctl_7(&buf7_8);

        // used on setting dpi
        let buf7_7: [u8; 7] = [
            0x10, 0xff, 0x0f, command_type_c, 0x03, 0x00, 0x00
        ];
        self.send_ctl_7(&buf7_7);
    }

    /**
     * Writes 7 byte packets to 0x0210
     */
    fn send_ctl_7(&mut self, msg: &[u8]) {
        println!("write ctl {:03} bytes", msg.len());
        let result = self.handle.write_control(0x21, 9, 0x0210, 1, msg, Duration::from_secs(10)).unwrap();
        self.print_status();
    }

    /**
     * Writes 20 byte packets to 0x0211
     */
    fn send_ctl_20(&mut self, msg: &[u8]) {
        println!("write ctl {:03} bytes", msg.len());
        let result = self.handle.write_control(0x21, 9, 0x0211, 1, msg, Duration::from_secs(10)).unwrap();
        self.print_status();
    }


    fn send_msg(&mut self, msg: &[u8]) {
        let result = self.handle.write_interrupt(self.write_address, msg, Duration::from_secs(10)).unwrap();
        self.print_status();
    }

    fn clear_reads(&mut self) {
        println!("Clearing messages");
        let mut state = 1;
        let mut resp: [u8; 20] = [0; 20];
        while state > 0 {
            state = self.read_response(&mut resp);
        }
    }


    /**
     * Normally begins with 0x11, 0xff, 0x0f, 0x7c ...
     */
    fn print_status(&mut self) {
        let mut buf: [u8; 20] = [0; 20];
        let mut bit_3 = 0x00;
        let mut bit_5 = 0xff;

        // the packets with byte 3 == 0, need to be skipped
        while bit_3 == 0 {
            self.read_response(&mut buf);
            bit_3 = buf[3];
            bit_5 = buf[5];
        }
    }


    fn apply_color(&mut self) {
        let device_index = 0xff;
        let feature_index = 0x0f; // 0x0f is dpi, 0x0e is leds
        let swid = 0xc;

        self.send_4_status(device_index, feature_index, swid);

        // only for the LED control
        self.send_0_root(device_index, 0x0e, swid, 0x00, 0x00, 0x00);

        let led_mode = 0x00; // 0x00 = off, 0x01 = static, 0x02 = cycle
        let led_r = 0x00;
        let led_g = 0xff;
        let led_b = 0x00;

        // cycle mode params (are 0x00 for other modes)
        let cycle_a = 0x2a; // speed major
        let cycle_b = 0xf8; // speed minor
        let cycle_c = 0x64; // brightness

        // final msg sets led colors
        let command_3 = 0x30 + swid;
        let buf20_final: [u8; 20] = [
            0x11, 0xff, 0x0e, command_3, 0x00, led_mode, led_r, led_g,
            led_b, 0x01, 0x00, cycle_a, cycle_b, cycle_c, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        ];
        self.send_ctl_20(&buf20_final);
    }


    fn apply_settings(&mut self) {
        let device_index = 0xff;
        let feature_index = 0x0f; // 0x0f is dpi, 0x0e is leds
        let swid = 0xd;
        let profile = 0x01;

        println!("Existing settings:");
        self.send_5_read_record(device_index, feature_index, swid, 0x00, profile, 0x10);
        self.send_5_read_record(device_index, feature_index, swid, 0x00, profile, 0xe0);

        // change profile
        println!("set profile");
        let command_8 = 0x80 + swid;
        let command_8_end: [u8; 7] = [
            0x10, device_index, feature_index, command_8, 0x01, 0x01, 0x00
        ];
        self.send_ctl_packet_7(&command_8_end);
        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);

        // or does this change profile? -- when 0x00 it will read
        self.send_2_connected(device_index, feature_index, swid, 0x01, 0x00, 0x00);

        self.send_4_status(device_index, feature_index, swid);

        // this seems to be the beginning of altering the settings
        // all of the following command type 7 following are connected

        self.send_6_start_record(device_index, feature_index, swid, profile);

        self.send_group_7_record(device_index, feature_index, swid);

        // the end of the command type 7 sequence
        // seems to get 3 response lines
        println!("Applying settings");
        let command_8 = 0x80 + swid;
        let command_8_end: [u8; 7] = [
            0x10, device_index, feature_index, command_8, 0x00, 0x00, 0x00
        ];
        self.send_ctl_packet_7(&command_8_end);

        // extra output after command 8
        // a previous command triggers the extra outputs
        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
        self.read_response(&mut resp);
        self.read_response(&mut resp);
    }
}


fn print_endpoint(endpoint: libusb::EndpointDescriptor) {
    println!("Endpoint address {:02x}", endpoint.address());
    println!("Endpoint number {:02x}", endpoint.number());
    println!("Endpoint direction {:?}", endpoint.direction());
    println!("Endpoint transfer {:?}", endpoint.transfer_type());
    println!("Endpoint sync {:?}", endpoint.sync_type());
    println!("Endpoint usage {:?}", endpoint.usage_type());
    println!("Endpoint packet size {}", endpoint.max_packet_size());
}


fn print_device(device: &libusb::Device) {
    let device_desc = device.device_descriptor().unwrap();
    println!("Bus {:03} Device {:03} ID {:04x}:{:04x}",
        device.bus_number(),
        device.address(),
        device_desc.vendor_id(),
        device_desc.product_id());

    let config = device.active_config_descriptor().unwrap();
    println!("Number {}, Interfaces {}", config.number(), config.num_interfaces());

    for interface in config.interfaces() {
        println!("Interface {:04x}", interface.number());
        for descriptor in interface.descriptors() {
            println!("Endpoints {}", descriptor.num_endpoints());
            for endpoint in descriptor.endpoint_descriptors() {
                print_endpoint(endpoint);
            }
        }
    }
}


/**
 * Set selected device to some mode.
 */
fn select_device(device: libusb::Device, mode: u8) {

    // print all device information
    print_device(&device);

    let mut controller = UsbController::open(&device);
    controller.claim();
    controller.clear_reads();
    //controller.set_color(mode, 255, 0, 0);
    controller.apply_settings();
    controller.release();
}


fn main() {
    let mut mode = 4;

    let args: Vec<String> = env::args().collect();
    let mut i = 1;
    while i < args.len() {
        if args[i] == "--mode" || args[i] == "-m" {
            i += 1;
            mode = args[i].parse::<u8>().unwrap();
        }
        i += 1;
    }

    // usb id
    let vendor_id = 0x046d;
    let product_id = 0xc08c;
    let mut context = libusb::Context::new().unwrap();

    // device selection
    for mut device in context.devices().unwrap().iter() {
        let device_desc = device.device_descriptor().unwrap();
        if device_desc.vendor_id() == vendor_id && device_desc.product_id() == product_id {
            select_device(device, mode);
        }
    }
}
