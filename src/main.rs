extern crate rand;
extern crate libusb;
extern crate crc;

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
use crc::{crc16, Hasher16};

/*
 * sets and reads fan and pump speeds
 */
struct UsbController<'a> {
    handle: libusb::DeviceHandle<'a>,
    interface: u8,
    read_address: u8,
    write_address: u8,
    print_messages: bool
}


impl<'a> UsbController<'a> {
    fn open(device: &'a libusb::Device, print_messages: bool) -> UsbController<'a> {

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

        if print_messages {
            println!("Opening interface 0x{:02x}", selected_interface);
            println!("Read address 0x{:02x}", selected_read_address);
            println!("Write address 0x{:02x}", selected_write_address);
        }

        return UsbController {
            handle: device.open().unwrap(),
            interface: selected_interface,
            read_address: selected_read_address,
            write_address: selected_write_address,
            print_messages: print_messages
        }
    }

    fn claim(&mut self) {
        self.handle.detach_kernel_driver(self.interface);
        self.handle.claim_interface(self.interface);
    }

    fn release(&mut self) {
        self.handle.release_interface(self.interface);
    }

    fn read_response(&mut self, out: &mut [u8]) -> usize {
        let result = self.handle.read_interrupt(self.read_address, out, Duration::from_secs(1)).unwrap_or(0);
        if self.print_messages {
            print!("read {:03} bytes: [", result);
            for b in 0..result {
                print!("{:x}, ", out[b]);
            }
            println!("]");
        }
        return result;
    }


    /**
     * Writes 7 byte packets to 0x0210
     */
    fn send_ctl_packet_7(&mut self, msg: &[u8]) {
        //println!("write ctl {:03} bytes", msg.len());
        let result = self.handle.write_control(0x21, 9, 0x0210, 1, msg, Duration::from_secs(1)).unwrap();
    }

    /**
     * Writes 20 byte packets to 0x0211
     */
    fn send_ctl_packet_20(&mut self, msg: &[u8]) {
        //println!("write ctl {:03} bytes", msg.len());
        let result = self.handle.write_control(0x21, 9, 0x0211, 1, msg, Duration::from_secs(1)).unwrap();
    }

    fn send_0_root(&mut self, di: u8, fi: u8, swid: u8, offset: u8, param_b: u8, param_c: u8) {
        let byte_3 = 0x00 + swid;
        let buf: [u8; 7] = [
            0x10, di, fi, byte_3, offset, param_b, param_c
        ];
        self.send_ctl_packet_7(&buf);

        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
    }

    /**
     * get feature data
     * first param is an offset for large data
     */
    fn send_1_get_features(&mut self, di: u8, fi: u8, swid: u8, offset: u8, param_b: u8, param_c: u8) {
        let byte_3 = 0x10 + swid;
        let buf: [u8; 7] = [
            0x10, di, fi, byte_3, offset, param_b, param_c
        ];
        self.send_ctl_packet_7(&buf);

        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
    }

    /**
     * checks a feature is available?
     * -- when param_b 0x00 it will read
     * otherwise saves a key-value?
     */
    fn send_2_connected(&mut self, di: u8, fi: u8, swid: u8, offset: u8, param_b: u8, param_c: u8) {
        let byte_3 = 0x20 + swid;
        let buf: [u8; 7] = [
            0x10, di, fi, byte_3, offset, param_b, param_c
        ];
        self.send_ctl_packet_7(&buf);

        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
    }

    /**
     * Changes the Profile
     * reports info when profile = 0x00
     * changes led colors
     */
    fn send_3_profile(&mut self, di: u8, fi: u8, swid: u8, profile: u8) {
        let byte_3 = 0x30 + swid;
        let buf: [u8; 7] = [
            0x10, di, fi, byte_3, 0x00, profile, 0x00
        ];
        self.send_ctl_packet_7(&buf);

        // returns many packets
        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
        self.read_response(&mut resp);
        self.read_response(&mut resp);
    }

    /**
     * Returns an error code and current profile
     */
    fn send_4_status(&mut self, di: u8, fi: u8, swid: u8) -> (u8, u8) {
        let byte_3 = 0x40 + swid;
        let buf: [u8; 7] = [
            0x10, di, fi, byte_3, 0x00, 0x00, 0x00
        ];
        self.send_ctl_packet_7(&buf);

        // returns err code and current profile
        let mut resp: [u8; 20] = [0; 20];
        let len = self.read_response(&mut resp);

        if len == 20 {
            return (resp[4], resp[5]);
        }
        else {
            return (0, 0);
        }
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
     * Profile is 0x00 when switching profile
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

    fn send_8_end_record(&mut self, di: u8, fi: u8, swid: u8) {

        // the end of the command type 7 sequence
        // seems to get 3 response lines
        let command_8 = 0x80 + swid;
        let buf: [u8; 7] = [
            0x10, di, fi, command_8, 0x00, 0x00, 0x00
        ];
        self.send_ctl_packet_7(&buf);

        // extra output after command 8
        // a previous command triggers the extra outputs
        let mut resp: [u8; 20] = [0; 20];
        self.read_response(&mut resp);
        self.read_response(&mut resp);
        self.read_response(&mut resp);
    }


    fn get_magic_numbers(&self) -> (u8, u8) {
        // seems to link to the last 2 bytes somehow
        // affected by the profile but not the content
        //
        // 1a -> 31 fe
        // 1b -> d8 ae
        // 1c -> 8e e5
        // 1d -> 0f ce
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

        // reset profile 1:
        // 1d -> cd c4
        // 1e -> 0f 2e

        // profile 2:
        // 0a -> cc a0
        // 0c -> dd 58
        // 0d -> 6c e1
        // 0f -> 2f 20
        // 10 -> 4c 68
        // 11 -> cd 43
        // 12 -> 3f 3b
        // 13 -> be 10
        // 14 -> 9a 5c
        // 15 -> 1b 77
        // 16 -> e9 0f
        // 17 -> 68 24
        // 18 -> a1 97
        // 19 -> 20 bc
        // 1a -> d2 c4
        // 1b -> 53 ef

        let cmd_param = 0x1d;
        let checksum_a = 0xcd;
        let checksum_b = 0xc4;

        // Changing profile:
        // Profile 1: e5 da
        // profile 3: 00 -> 3c e2

        return (checksum_a, checksum_b);
    }


    /**
     * A big set of data send over multiple packets
     */
    fn send_group_7_record(&mut self, di: u8, fi: u8, swid: u8, write_id: u8, poll_rate: u16, dpi_array: &[u16]) {

        // 0x00 = change profile
        // 0x01 = 1000hz
        // 0x02 = 500hz
        let poll_rate = 0x01;

        // this must be incremented by 1 on each write
        let cmd_param = write_id; //0x1d;
        let checksum_a = 0xff;//0xcd; // bytes for 0x1d
        let checksum_b = 0xff;//0xc4;

        // send a 256 byte set of data
        // Line 0 is the dpi settings
        // should be ordered low to high
        // 1st: 0x0190
        // 2st: 0x0320

        let mut dpi_1_hi = 0x01;
        let mut dpi_1_lo = 0x90;
        let mut dpi_2_hi = 0x02;
        let mut dpi_2_lo = 0x8a;
        let mut dpi_3_hi = 0x03;
        let mut dpi_3_lo = 0x84;
        let mut dpi_4_hi = 0x04;
        let mut dpi_4_lo = 0x7e;
        let mut dpi_5_hi = 0x00; // 0 to disable
        let mut dpi_5_lo = 0x00;

        // if dpi_array.len() > 0 {
        //     dpi_1_hi = (dpi_array[0] >> 8) as u8;
        //     dpi_1_lo = (dpi_array[0] & 0xff) as u8;
        // }
        // if dpi_array.len() > 1 {
        //     dpi_2_hi = (dpi_array[1] >> 8) as u8;
        //     dpi_2_lo = (dpi_array[1] & 0xff) as u8;
        // }
        // if dpi_array.len() > 2 {
        //     dpi_3_hi = (dpi_array[2] >> 8) as u8;
        //     dpi_3_lo = (dpi_array[2] & 0xff) as u8;
        // }
        // if dpi_array.len() > 3 {
        //     dpi_4_hi = (dpi_array[3] >> 8) as u8;
        //     dpi_4_lo = (dpi_array[3] & 0xff) as u8;
        // }
        // if dpi_array.len() > 4 {
        //     dpi_5_hi = (dpi_array[4] >> 8) as u8;
        //     dpi_5_lo = (dpi_array[4] & 0xff) as u8;
        // }

        println!("DPI {:02x}, {:02x}", dpi_2_hi, dpi_2_lo);


        let mut packets: [[u8; 16]; 16] = [
            [poll_rate, 0x01, 0x00, dpi_1_lo, dpi_1_hi, dpi_2_lo, dpi_2_hi, dpi_3_lo, dpi_3_hi, dpi_4_lo, dpi_4_hi, dpi_5_lo, dpi_5_hi, 0xff, 0xff, 0xff],
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
            [0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00], // "1" -- Profile 1 is saved as the profile name
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            [0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00, 0x00, 0x00],
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, checksum_a, checksum_b], // sometimes has 0x00 in byte 4-6
        ];

        let mut digest = crc16::Digest::new_with_initial(crc16::USB, 0xffff);
        let mut crc_0 = 0x0000;
        for &packet in &packets {
            //let mut g = [0; 16];
            //g.clone_from_slice(&packet);
            //g.reverse();
            digest.write(&packet);

            for x in 0..packet.len() {
                //let data = ((packet[(x * 2) + 0] as u16) << 8) + (packet[(x * 2) + 1] as u16);
                crc_0 = UsbController::crc_update(crc_0, packet[x]);
            }
        }
        println!("CRC {:04x}", digest.sum16());
        println!("CRC {:04x}", crc_0);

        // this is the correct crc.
        // how is it calculated?
        packets[15][14] = 0xcd;
        packets[15][15] = 0xc4;

        for &packet in &packets {
            self.send_7_record(di, fi, swid, packet);
        }
    }

    fn crc_update(mut crc: u16, data: u8) -> u16 {
      crc ^= ((data as u16) << 8);
      for x in 0..8 {
          if (crc & 0x8000) == 0x8000 {
              crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
          }
          else {
             crc <<= 1;
          }
      }
      crc &= 0xFFFF;
      return crc;
  }


    fn send_group_7_enable_profile(&mut self, di: u8, fi: u8, swid: u8) {
        println!("Set Profile");

        let checksum_a = 0xe5;
        let checksum_b = 0xda;

        // Changing profile:
        // Profile 1: e5 da
        // profile 3: 00 -> 3c e2

        // profile on/off switches
        let p1 = 0x01;
        let p2 = 0x00;
        let p3 = 0x00;
        let p4 = 0x00;
        let p5 = 0x00;

        let packets: [[u8; 16]; 16] = [
            [0x00, 0x01, p1, 0x00, 0x00, 0x02, p2, 0x00, 0x00, 0x03, p3, 0x00, 0x00, 0x04, p4, 0x00],
            [0x00, 0x05, p5, 0x00, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff],
            [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, checksum_a, checksum_b],
        ];

        for &packet in &packets {
            self.send_7_record(di, fi, swid, packet);
        }
    }

    fn write_init_commands(&mut self) {

        let device_index = 0xff;
        let feature_index = 0x0f; // 0x0f is dpi, 0x0e is leds
        let swid = 0xa;

        let command_type_0 = 0x0c; // root
        let command_type_1 = 0x1c; //
        let command_type_2 = 0x2c; // check connected
        let command_type_3 = 0x3c; // color setting
        let command_type_4 = 0x4c; // get status, returns err and current profile
        let command_type_6 = 0x6c;
        let command_type_7 = 0x7c;
        let command_type_8 = 0x8c;
        let command_type_b = 0xbc; // error handling?
        let command_type_c = 0xcc; // error handling?

        // reset logic when device is first connected
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

        // This returns response of device name string over 2 responses
        // 0x10 is an offset
        self.send_1_get_features(device_index, 0x03, swid, 0x00, 0x00, 0x00);
        self.send_1_get_features(device_index, 0x03, swid, 0x10, 0x00, 0x00);

        self.send_0_root(device_index, 0x00, swid, 0x81, 0x10, 0x00);
        self.send_0_root(device_index, 0x10, swid, 0x00, 0x00, 0x00);
        self.send_0_root(device_index, 0x00, swid, 0x80, 0x90, 0x00);
        self.send_0_root(device_index, 0x00, swid, 0x13, 0x00, 0x00);
        self.send_0_root(device_index, 0x00, swid, 0x81, 0x00, 0x00);

        self.send_2_connected(device_index, 0x0f, swid, 0x00, 0x00, 0x00);

        self.send_0_root(device_index, 0x00, swid, 0x80, 0x70, 0x00);
        self.send_0_root(device_index, 0x0e, swid, 0x00, 0x00, 0x00);

        self.send_1_get_features(device_index, 0x0e, swid, 0x00, 0x00, 0x00);
        self.send_2_connected(device_index, 0x0e, swid, 0x00, 0x00, 0x00);

        // adds 0x01, 0x01 params
        let reset_w: [u8; 7] = [
            0x10, 0xff, 0x0e, command_type_8, 0x01, 0x01, 0x00
        ];
        self.send_ctl_7(&reset_w);

        self.send_4_status(device_index, feature_index, swid);

        // set profile
        self.switch_to_profile(0x01);

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


    /**
     * Make sure no messages are queued to be read
     */
    fn clear_reads(&mut self) {
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
        let device_index = 0xff;
        let feature_index = 0x0f;
        let swid = 0xa;

        let (err, profile) = self.send_4_status(device_index, feature_index, swid);

        println!("Error code: {:02x}, Profile {:02x}", err, profile);
    }

    /**
     * 0x01 to 0x05
     */
    fn switch_to_profile(&mut self, n: u8) {
        let device_index = 0xff;
        let feature_index = 0x0f;
        let swid = 0xa;

        self.send_4_status(device_index, feature_index, swid);

        self.send_3_profile(device_index, feature_index, swid, n);

        self.send_4_status(device_index, feature_index, swid);
    }

    fn enable_profile(&mut self, n: u8) {
        let device_index = 0xff;
        let feature_index = 0x0f; // 0x0f is dpi, 0x0e is leds
        let swid = 0xa;

        println!("Enabling profile");
        self.send_6_start_record(device_index, feature_index, swid, 0x00);

        self.send_group_7_enable_profile(device_index, feature_index, swid);

        // the end of the command type 7 sequence
        // seems to get 3 response lines

        let command_8 = 0x80 + swid;
        let command_8_end: [u8; 7] = [
            0x10, device_index, feature_index, command_8, 0x00, 0x00, 0x00
        ];
        self.send_ctl_packet_7(&command_8_end);
    }


    /**
     * Mode: 0x00 = off, 0x01 = static, 0x02 = cycle
     */
    fn apply_color(&mut self, mode: u8) {
        let device_index = 0xff;
        let feature_index = 0x0f; // 0x0f is dpi, 0x0e is leds
        let swid = 0xa;

        self.send_4_status(device_index, feature_index, swid);

        // only for the LED control
        self.send_0_root(device_index, 0x0e, swid, 0x00, 0x00, 0x00);

        let led_mode = mode; // 0x00 = off, 0x01 = static, 0x02 = cycle
        let led_r = 0x00;
        let led_g = 0xff;
        let led_b = 0x00;

        // cycle mode params (are 0x00 for other modes)
        let cycle_a = 0x2a; // speed major
        let cycle_b = 0xf8; // speed minor
        let cycle_c = 0x64; // brightness

        // final msg sets led colors
        let command_3 = 0x30 + swid;
        let buf: [u8; 20] = [
            0x11, 0xff, 0x0e, command_3, 0x00, led_mode, led_r, led_g,
            led_b, 0x01, 0x00, cycle_a, cycle_b, cycle_c, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00
        ];
        self.send_ctl_20(&buf);
    }


    fn before_apply(&mut self, profile: u8) {
        let device_index = 0xff;
        let feature_index = 0x0f; // 0x0f is dpi, 0x0e is leds
        let swid = 0xa;

        println!("Existing settings:");
        self.send_5_read_record(device_index, feature_index, swid, 0x00, profile, 0x10);
        self.send_5_read_record(device_index, feature_index, swid, 0x00, profile, 0xe0);

        self.switch_to_profile(profile);

        // try to fix err 0x01
        self.send_0_root(device_index, 0x00, swid, 0x80, 0x60, 0x00);
        self.send_0_root(device_index, 0x0d, swid, 0x00, 0x00, 0x00);

        // these 7 byte packets seem to reset state
        // used on setting dpi
        let command_b = 0xb0 + swid;
        let buf7_8: [u8; 7] = [
            0x10, 0xff, 0x0f, command_b, 0x00, 0x00, 0x00
        ];
        self.send_ctl_7(&buf7_8);

        // used on setting dpi
        let command_c = 0xc0 + swid;
        let buf7_9: [u8; 7] = [
            0x10, 0xff, 0x0f, command_c, 0x03, 0x00, 0x00
        ];
        self.send_ctl_7(&buf7_9);
    }

    fn apply_settings(&mut self, profile: u8, poll_rate: u16, dpi_array: &[u16]) {
        let device_index = 0xff;
        let feature_index = 0x0f; // 0x0f is dpi, 0x0e is leds
        let swid = 0xa;
        let mut use_profile = profile;

        // is a profile isn't given find the current profile
        while use_profile == 0 {
            let (err, p) = self.send_4_status(device_index, feature_index, swid);
            use_profile = p;
        }

        println!("Setting profile {:02x}", use_profile);

        // this first attempt fails from non-incrementing id (0x1c) but alters the current id
        // allows the check to be bypassed on the retry
        self.send_6_start_record(device_index, feature_index, swid, use_profile);
        self.send_group_7_record(device_index, feature_index, swid, 0x1c, poll_rate, dpi_array);
        self.send_8_end_record(device_index, feature_index, swid);

        // try again with incremented id 0x1c -> 0x1d
        // the beginning of actually altering the settings
        self.send_6_start_record(device_index, feature_index, swid, use_profile);
        self.send_group_7_record(device_index, feature_index, swid, 0x1d, poll_rate, dpi_array);

        // ends the record and applies settings
        self.send_8_end_record(device_index, feature_index, swid);

        // get final status
        self.send_4_status(device_index, feature_index, swid);
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


struct Config {
    vendor_id: u16,
    product_id: u16,
    print_endpoints: bool,
    print_status: bool,
    clear_queue: bool,
    switch_to_profile: u8
}


impl Config {
    pub fn default() -> Config {
        Config {
            vendor_id: 0x046d,
            product_id: 0xc08c,
            print_endpoints: true,
            print_status: true,
            clear_queue: true,
            switch_to_profile: 0x00
        }
    }
}


/**
 * Set selected device to some mode.
 */
fn select_device(device: libusb::Device, config: &Config) {
    let mut controller = UsbController::open(&device, config.print_endpoints);

    // print all device endpoint information
    if (config.print_endpoints) {
        print_device(&device);
    }

    controller.claim();

    // remove any queued usb responses
    if config.clear_queue {
        controller.clear_reads();
    }

    // change the current profile
    if config.switch_to_profile > 0 {
        controller.switch_to_profile(config.switch_to_profile);
    }

    controller.apply_settings(config.switch_to_profile, 1000, &[400, 650, 1600, 3200]);

    controller.apply_color(0x00);

    if config.print_status {
        controller.print_status();
    }
    controller.release();
}


fn main() {
    let mut config = Config::default();

    let args: Vec<String> = env::args().collect();

    // device selection
    let mut context = libusb::Context::new().unwrap();
    for mut device in context.devices().unwrap().iter() {
        let device_desc = device.device_descriptor().unwrap();
        if device_desc.vendor_id() == config.vendor_id && device_desc.product_id() == config.product_id {
            select_device(device, &config);
        }
    }
}
