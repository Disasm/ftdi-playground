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

    fn set_tms(&mut self, mut data: &[u8], mut bits: usize) -> io::Result<()> {
        assert!((bits + 7) / 8 <= data.len());

        let mut buf = vec![];
        while bits > 0 {
            if bits >= 8 {
                buf.extend_from_slice(&[0x4b, 0x07, data[0]]);
                data = &data[1..];
                bits -= 8;
            } else {
                buf.extend_from_slice(&[0x4b, (bits - 1) as u8, data[0]]);
                bits = 0;
            }
        }
        self.device.write_all(&buf)
    }

    pub fn reset(&mut self) -> io::Result<()> {
        self.set_tms(&[0xff; 5], 40)
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
}
