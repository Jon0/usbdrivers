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

    fn write_init_commands(&mut self) {

        // 0x7c for color panel
        // 0x7d for dpi panel
        // either end in 'c' or 'd' will work...
        let command_type_0 = 0x0d;
        let command_type_1 = 0x1d;
        let command_type_3 = 0x3d; // color setting
        let command_type_4 = 0x4d;
        let command_type_6 = 0x6d;
        let command_type_7 = 0x7d;
        let command_type_8 = 0x8d;
        let command_type_b = 0xbd;
        let command_type_c = 0xcd;

        let buf: [u8; 7] = [
            0x10, 0xff, 0x0f, 0x4c, 0x00, 0x00, 0x00
        ];
        self.send_ctl_7(&buf);

        let buf20: [u8; 20] = [
            0x11, 0xff, 0x0f, 0x6c, 0x00, 0x01, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        ];
        self.send_ctl_20(&buf20);


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

        // used on setting dpi
        let buf7_9: [u8; 7] = [
            0x10, 0xff, 0x0f, command_type_6, 0x00, 0x01, 0x00
        ];
        self.send_ctl_7(&buf7_9);

        println!("Set DPI");
        // 0x7d or 0x7c ?
        // possibly the dpi settings
        // 1st: 0x0190
        // 2st: 0x0320
        let buf20_0: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0x01, 0x01, 0x00, 0x90,
            0x01, 0x20, 0x03, 0x40, 0x06, 0x80, 0x0c, 0x00,
            0x00, 0xff, 0xff, 0xff
        ];
        self.send_ctl_20(&buf20_0);

        // 0x10 for colors is sometimes 0x17 or 0x19 for dpi change
        let cmd_param = 0x19;
        let buf20_01: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0xff, 0x00, cmd_param, 0x00,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff
        ];
        self.send_ctl_20(&buf20_01);


        let buf20_1: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0x80, 0x01, 0x00, 0x01,
            0x80, 0x01, 0x00, 0x02, 0x80, 0x01, 0x00, 0x04,
            0x80, 0x01, 0x00, 0x08
        ];
        self.send_ctl_20(&buf20_1);

        // this section is played twice
        let buf20_2: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0x80, 0x01, 0x00, 0x10,
            0x90, 0x05, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff
        ];
        self.send_ctl_20(&buf20_2);

        // maybe repeated
        let buf20_21: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff
        ];
        self.send_ctl_20(&buf20_21);

        let buf20_22: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0x80, 0x01, 0x00, 0x01,
            0x80, 0x01, 0x00, 0x02, 0x80, 0x01, 0x00, 0x04,
            0x80, 0x01, 0x00, 0x08
        ];
        self.send_ctl_20(&buf20_22);


        // this section seems to apply settings
        // contains string 'profile'
        let buf20_23: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0x50, 0x00, 0x72, 0x00,
            0x6f, 0x00, 0x66, 0x00, 0x69, 0x00, 0x6c, 0x00,
            0x65, 0x00, 0x20, 0x00
        ];
        self.send_ctl_20(&buf20_23);

        // contains string '1' -- profile 1 is modified
        let buf20_24: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0x31, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        ];
        self.send_ctl_20(&buf20_24);

        let buf20_30: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        ];
        self.send_ctl_20(&buf20_30);

        let buf20_3: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0x01, 0xff, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
            0xff, 0x00, 0x00, 0x00
        ];
        self.send_ctl_20(&buf20_3);

        let buf20_31: [u8; 20] = [
            0x11, 0xff, 0x0f, command_type_7, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff
        ];
        self.send_ctl_20(&buf20_31);

        let buf7_4: [u8; 7] = [
            0x10, 0xff, 0x0f, command_type_8, 0x00, 0x00, 0x00
        ];
        self.send_ctl_7(&buf7_4);

        // repeat first msg
        let buf7_5: [u8; 7] = [
            0x10, 0xff, 0x0f, command_type_4, 0x00, 0x00, 0x00
        ];
        self.send_ctl_7(&buf7_5);

        let buf7_6: [u8; 7] = [
            0x10, 0xff, 0x0e, command_type_0, 0x00, 0x00, 0x00
        ];
        self.send_ctl_7(&buf7_6);

        let led_mode = 0x00; // 0x00 = off, 0x01 = static, 0x02 = cycle
        let led_r = 0x00;
        let led_g = 0xff;
        let led_b = 0x00;

        // cycle mode params (are 0x00 for other modes)
        let cycle_a = 0x2a; // speed major
        let cycle_b = 0xf8; // speed minor
        let cycle_c = 0x64; // brightness

        // final msg sets led colors
        let buf20_final: [u8; 20] = [
            0x11, 0xff, 0x0e, command_type_3, 0x00, led_mode, led_r, led_g,
            led_b, 0x01, 0x00, cycle_a, cycle_b, cycle_c, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        ];
        self.send_ctl_20(&buf20_final);

        // used on setting dpi
        let buf7_72: [u8; 7] = [
            0x10, 0xff, 0x0f, command_type_c, 0x03, 0x00, 0x00
        ];
        self.send_ctl_7(&buf7_72);

        // used on setting dpi
        let buf7_82: [u8; 7] = [
            0x10, 0xff, 0x0f, command_type_b, 0x00, 0x00, 0x00
        ];
        self.send_ctl_7(&buf7_82);
    }

    /**
     * offset: led index
     * modes: 0x01 = fade, 0x04 = static, 0x06 = sequence, 0x08 = blink
     */
    fn write_color_command(&mut self, r1: u8, g1: u8, b1: u8) {

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


    /**
     * Normally begins with 0x11, 0xff, 0x0f, 0x7c ...
     */
    fn print_status(&mut self) {
        let mut buf: [u8; 20] = [0; 20];
        let result = self.handle.read_interrupt(self.read_address, &mut buf, Duration::from_secs(10)).unwrap();
        println!("read {:03} bytes", result);
        for b in &buf {
            print!("{:x}, ", b);
        }
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
    controller.set_color(mode, 255, 0, 0);
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