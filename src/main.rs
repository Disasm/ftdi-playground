use std::io::{self, Read, Write};
use std::convert::TryInto;

mod ftdi;

struct JtagChainItem {
    idcode: u32,
    irlen: usize,
}

#[derive(Debug)]
struct ChainParams {
    irpre: usize,
    irpost: usize,
    drpre: usize,
    drpost: usize,
    irlen: usize,
}

struct FtdiProbe {
    device: ftdi::Device,
    chain_params: Option<ChainParams>,
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
            device,
            chain_params: None
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
            command.extend_from_slice(&[0x4a, 0x00, tms_byte]);
        }

        self.device.write_all(&command)
    }

    fn tranfer_tdi(&mut self, mut data: &[u8], mut bits: usize) -> io::Result<Vec<u8>> {
        assert!(bits > 0);
        assert!((bits + 7) / 8 <= data.len());

        let mut command = vec![];

        let full_bytes = (bits - 1) / 8;
        if full_bytes > 0 {
            assert!(full_bytes <= 65536);

            command.extend_from_slice(&[0x3c]);
            let n: u16 = (full_bytes - 1) as u16;
            command.extend_from_slice(&n.to_le_bytes());
            command.extend_from_slice(&data[..full_bytes]);

            bits -= full_bytes * 8;
            data = &data[full_bytes..];
        }
        assert!(0 < bits && bits <= 8);

        let byte = data[0];
        if bits > 1 {
            let n = (bits - 2) as u8;
            command.extend_from_slice(&[0x3e, n, byte]);
        }

        let last_bit = (byte >> (bits - 1)) & 0x01;
        let tms_byte = 0x01 | (last_bit << 7);
        command.extend_from_slice(&[0x6b, 0x00, tms_byte]);

        self.device.write_all(&command)?;

        let mut expect_bytes = full_bytes + 1;
        if bits > 1 {
            expect_bytes += 1;
        }
        let mut reply = vec![];
        reply.resize(expect_bytes, 0);
        self.device.read_exact(&mut reply)?;

        let mut last_byte = reply[reply.len() - 1] & 0x01;
        if bits > 1 {
            let byte = reply[reply.len() - 2];
            last_byte = byte | (last_byte << (bits - 1));
        }
        reply[full_bytes + 1] = last_byte;
        reply.truncate(full_bytes + 1);

        Ok(reply)
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

    /// Shift to IR and return to IDLE
    pub fn transfer_ir(&mut self, data: &[u8], bits: usize) -> io::Result<Vec<u8>> {
        self.shift_tms(&[0b0011], 4)?;
        let r = self.tranfer_tdi(data, bits)?;
        self.shift_tms(&[0b01], 2)?;
        Ok(r)
    }

    /// Shift to DR and return to IDLE
    pub fn shift_dr(&mut self, data: &[u8], bits: usize) -> io::Result<()> {
        self.shift_tms(&[0b001], 3)?;
        self.shift_tdi(data, bits)?;
        self.shift_tms(&[0b01], 2)?;
        Ok(())
    }

    /// Shift to DR and return to IDLE
    pub fn transfer_dr(&mut self, data: &[u8], bits: usize) -> io::Result<Vec<u8>> {
        self.shift_tms(&[0b001], 3)?;
        let r = self.tranfer_tdi(data, bits)?;
        self.shift_tms(&[0b01], 2)?;
        Ok(r)
    }

    fn scan(&mut self) -> io::Result<Vec<JtagChainItem>> {
        let max_device_count = 8;

        self.reset()?;

        let cmd = vec![0xff; max_device_count*4];
        let r = self.transfer_dr(&cmd, cmd.len()*8)?;
        let mut targets = vec![];
        for i in 0..max_device_count {
            let idcode = u32::from_le_bytes(r[i*4..(i+1)*4].try_into().unwrap());
            if idcode != 0xffffffff {
                println!("Device found: {:08x}", idcode);
                let target = JtagChainItem {
                    idcode,
                    irlen: 0
                };
                targets.push(target);
            } else {
                break;
            }
        }

        self.reset()?;
        let cmd = vec![0xff; max_device_count];
        let mut r = self.transfer_ir(&cmd, cmd.len()*8)?;

        let mut ir = 0;
        let mut irbits = 0;
        for (i, target) in targets.iter_mut().enumerate() {
            if r.len() > 0 && irbits < 8 {
                let byte = r[0];
                r.remove(0);
                ir |= (byte as u32) << irbits;
                irbits += 8;
            }
            if ir & 0b11 == 0b01 {
                ir &= !1;
                let irlen = ir.trailing_zeros();
                ir = ir >> irlen;
                irbits -= irlen;
                println!("dev {} irlen: {}", i, irlen);
                target.irlen = irlen as usize;
            } else {
                println!("invalid irlen for device {}", i);
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "Invalid IR sequence during the chain scan"
                ));
            }
        }

        Ok(targets)
    }

    pub fn select_target(&mut self, idcode: u32) -> io::Result<()> {
        let targets = self.scan()?;

        let mut found = false;
        let mut params = ChainParams {
            irpre: 0,
            irpost: 0,
            drpre: 0,
            drpost: 0,
            irlen: 0
        };
        for target in targets {
            if target.idcode == idcode {
                params.irlen = target.irlen;
                found = true;
            } else if found {
                params.irpost += target.irlen;
                params.drpost += 1;
            } else {
                params.irpre += target.irlen;
                params.drpre += 1;
            }
        }

        if found {
            println!("Target chain params: {:?}", params);
            self.chain_params = Some(params);
            Ok(())
        } else {
            Err(io::Error::new(io::ErrorKind::NotFound, "target not found"))
        }
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
    probe.select_target(0x790007a3).unwrap();
}
