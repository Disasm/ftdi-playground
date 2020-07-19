use std::io::{self, Read, Write};

mod ftdi;

struct FtdiProbe {
    device: ftdi::Device,
}

impl FtdiProbe {
    pub fn open(vid: u16, pid: u16) -> Result<Self, ftdi::Error> {
        let mut builder = ftdi::Builder::new();
        builder.set_interface(ftdi::Interface::A)?;
        let mut device = builder.usb_open(vid, pid)?;

        device.usb_reset()?;
        device.usb_purge_buffers()?;
        device.set_latency_timer(2)?;
        device.set_bitmode(0x0b, ftdi::BitMode::Mpsse).unwrap();

        Ok(Self {
            device
        })
    }

    fn shift_tms(&mut self, mut data: &[u8], mut bits: usize) -> io::Result<()> {
        assert!(bits > 0);
        assert!((bits + 7) / 8 <= data.len());

        let mut command = vec![];

        while bits > 0 {
            if bits >= 8 {
                command.extend_from_slice(&[0x4b, 0x07, data[0]]);
                data = &data[1..];
                bits -= 8;
            } else {
                command.extend_from_slice(&[0x4b, (bits - 1) as u8, data[0]]);
                bits = 0;
            }
        }
        self.device.write_all(&command)
    }

    fn shift_tdi(&mut self, mut data: &[u8], mut bits: usize) -> io::Result<()> {
        assert!(bits > 0);
        assert!((bits + 7) / 8 <= data.len());

        let mut command = vec![];

        let full_bytes = (bits - 1) / 8;
        if full_bytes > 0 {
            assert!(full_bytes <= 65536);

            command.extend_from_slice(&[0x18]);
            let n: u16 = (full_bytes - 1) as u16;
            command.extend_from_slice(&n.to_le_bytes());
            command.extend_from_slice(&data[..full_bytes]);

            bits -= full_bytes * 8;
            data = &data[full_bytes..];
        }
        assert!(bits <= 8);

        if bits > 0 {
            let byte = data[0];
            if bits > 1 {
                let n = (bits - 2) as u8;
                command.extend_from_slice(&[0x1a, n, byte]);
            }

            let last_bit = (byte >> (bits - 1)) & 0x01;
            let tms_byte = 0x01 | (last_bit << 7);
            command.extend_from_slice(&[0x4b, 0x00, tms_byte]);
        }

        self.device.write_all(&command)
    }

    /// Reset and go to RUN-TEST/IDLE
    pub fn reset(&mut self) -> io::Result<()> {
        self.shift_tms(&[0xff, 0xff, 0xff, 0xff, 0x7f], 40)
    }

    /// Execute RUN-TEST/IDLE for a number of cycles
    pub fn idle(&mut self, cycles: usize) -> io::Result<()> {
        let mut buf = vec![];
        buf.resize((cycles + 7) / 8, 0);
        self.shift_tms(&buf, cycles)
    }

    /// Shift to IR and return to IDLE
    pub fn shift_ir(&mut self, data: &[u8], bits: usize) -> io::Result<()> {
        self.shift_tms(&[0b0011], 4)?;
        self.shift_tdi(data, bits)?;
        self.shift_tms(&[0b01], 2)?;
        Ok(())
    }

    /// Shift to DR and return to IDLE
    pub fn shift_dr(&mut self, data: &[u8], bits: usize) -> io::Result<()> {
        self.shift_tms(&[0b001], 3)?;
        self.shift_tdi(data, bits)?;
        self.shift_tms(&[0b01], 2)?;
        Ok(())
    }
}

fn main() {
    println!("Opening probe...");

    let mut probe = match FtdiProbe::open(0x0403, 0x6010) {
        Ok(probe) => probe,
        Err(e) => {
            println!("Cannot find/open device: {:?}", e);
            return;
        }
    };

    probe.reset().unwrap();
    probe.shift_ir(&[0x10], 5).unwrap();
    probe.idle(42);
    probe.shift_dr(&[0xff;5], 40).unwrap();
}
