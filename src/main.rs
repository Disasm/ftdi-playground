use bitvec::order::Lsb0;
use bitvec::vec::BitVec;
use std::convert::TryInto;
use std::io::{self, Read, Write};
use std::time::Duration;

mod ftdi;

#[derive(Debug)]
struct JtagChainItem {
    idcode: u32,
    irlen: usize,
}

#[derive(Clone, Debug)]
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
    idle_cycles: u8,
}

impl FtdiProbe {
    pub fn open(vid: u16, pid: u16) -> Result<Self, ftdi::Error> {
        let mut builder = ftdi::Builder::new();
        builder.set_interface(ftdi::Interface::A)?;
        let device = builder.usb_open(vid, pid)?;

        Ok(Self {
            device,
            chain_params: None,
            idle_cycles: 0,
        })
    }

    pub fn attach(&mut self) -> Result<(), ftdi::Error> {
        self.device.usb_reset()?;
        self.device.set_latency_timer(1)?;
        self.device.set_bitmode(0x0b, ftdi::BitMode::Mpsse)?;
        self.device.usb_purge_buffers()?;

        let mut junk = vec![];
        let _ = self.device.read_to_end(&mut junk);

        // Minimal values, may not work with all probes
        let output: u16 = 0x0008;
        let direction: u16 = 0x000b;
        self.device
            .write_all(&[0x80, output as u8, direction as u8])?;
        self.device
            .write_all(&[0x82, (output >> 8) as u8, (direction >> 8) as u8])?;

        // Disable loopback
        self.device.write_all(&[0x85])?;

        Ok(())
    }

    fn read_response(&mut self, size: usize) -> io::Result<Vec<u8>> {
        let timeout = Duration::from_millis(10);
        let mut result = Vec::new();

        let t0 = std::time::Instant::now();
        while result.len() < size {
            if t0.elapsed() > timeout {
                return Err(io::Error::from(io::ErrorKind::TimedOut));
            }

            self.device.read_to_end(&mut result)?;
        }

        if result.len() > size {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Read more data than expected",
            ));
        }

        Ok(result)
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

            command.extend_from_slice(&[0x19]);
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
                command.extend_from_slice(&[0x1b, n, byte]);
            }

            let last_bit = (byte >> (bits - 1)) & 0x01;
            let tms_byte = 0x01 | (last_bit << 7);
            command.extend_from_slice(&[0x4b, 0x00, tms_byte]);
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

            command.extend_from_slice(&[0x39]);
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
            command.extend_from_slice(&[0x3b, n, byte]);
        }

        let last_bit = (byte >> (bits - 1)) & 0x01;
        let tms_byte = 0x01 | (last_bit << 7);
        command.extend_from_slice(&[0x6b, 0x00, tms_byte]);

        self.device.write_all(&command)?;

        let mut expect_bytes = full_bytes + 1;
        if bits > 1 {
            expect_bytes += 1;
        }

        let mut reply = self.read_response(expect_bytes)?;

        let mut last_byte = reply[reply.len() - 1] & 0x01;
        if bits > 1 {
            let byte = reply[reply.len() - 2];
            last_byte = byte | (last_byte << (bits - 1));
        }
        reply[full_bytes] = last_byte;
        reply.truncate(full_bytes + 1);

        Ok(reply)
    }

    /// Reset and go to RUN-TEST/IDLE
    pub fn reset(&mut self) -> io::Result<()> {
        self.shift_tms(&[0xff, 0xff, 0xff, 0xff, 0x7f], 40)
    }

    /// Execute RUN-TEST/IDLE for a number of cycles
    pub fn idle(&mut self, cycles: usize) -> io::Result<()> {
        if cycles == 0 {
            return Ok(());
        }
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
    pub fn transfer_dr(&mut self, data: &[u8], bits: usize) -> io::Result<Vec<u8>> {
        self.shift_tms(&[0b001], 3)?;
        let r = self.tranfer_tdi(data, bits)?;
        self.shift_tms(&[0b01], 2)?;
        Ok(r)
    }

    fn scan(&mut self) -> io::Result<Vec<JtagChainItem>> {
        let max_device_count = 8;

        self.reset()?;

        let cmd = vec![0xff; max_device_count * 4];
        let r = self.transfer_dr(&cmd, cmd.len() * 8)?;
        let mut targets = vec![];
        for i in 0..max_device_count {
            let idcode = u32::from_le_bytes(r[i * 4..(i + 1) * 4].try_into().unwrap());
            if idcode != 0xffffffff {
                log::debug!("tap found: {:08x}", idcode);
                let target = JtagChainItem { idcode, irlen: 0 };
                targets.push(target);
            } else {
                break;
            }
        }

        self.reset()?;
        let cmd = vec![0xff; max_device_count];
        let mut r = self.transfer_ir(&cmd, cmd.len() * 8)?;

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
                log::debug!("tap {} irlen: {}", i, irlen);
                target.irlen = irlen as usize;
            } else {
                log::debug!("invalid irlen for tap {}", i);
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "Invalid IR sequence during the chain scan",
                ));
            }
        }

        Ok(targets)
    }

    pub fn select_target(&mut self, idcode: u32) -> io::Result<()> {
        let taps = self.scan()?;

        let mut found = false;
        let mut params = ChainParams {
            irpre: 0,
            irpost: 0,
            drpre: 0,
            drpost: 0,
            irlen: 0,
        };
        for tap in taps {
            if tap.idcode == idcode {
                params.irlen = tap.irlen;
                found = true;
            } else if found {
                params.irpost += tap.irlen;
                params.drpost += 1;
            } else {
                params.irpre += tap.irlen;
                params.drpre += 1;
            }
        }

        if found {
            log::debug!("Target chain params: {:?}", params);
            self.chain_params = Some(params);
            Ok(())
        } else {
            Err(io::Error::new(io::ErrorKind::NotFound, "target not found"))
        }
    }

    fn get_chain_params(&self) -> io::Result<ChainParams> {
        match &self.chain_params {
            Some(params) => Ok(params.clone()),
            None => Err(io::Error::new(
                io::ErrorKind::Other,
                "target is not selected",
            )),
        }
    }

    fn target_transfer(
        &mut self,
        address: u32,
        data: Option<&[u8]>,
        len_bits: usize,
    ) -> io::Result<Vec<u8>> {
        let params = self.get_chain_params()?;
        let max_address = (1 << params.irlen) - 1;
        if address > max_address {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid register address",
            ));
        }

        // Write IR register
        let irbits = params.irpre + params.irlen + params.irpost;
        assert!(irbits <= 32);
        let mut ir: u32 = (1 << params.irpre) - 1;
        ir |= address << params.irpre;
        ir |= ((1 << params.irpost) - 1) << (params.irpre + params.irlen);
        self.shift_ir(&ir.to_le_bytes(), irbits)?;

        let drbits = params.drpre + len_bits + params.drpost;
        let request = if let Some(data) = data {
            let mut data = BitVec::<Lsb0, u8>::from_slice(data);
            data.truncate(len_bits);

            let mut buf = BitVec::<Lsb0, u8>::new();
            buf.resize(params.drpre, false);
            buf.append(&mut data);
            buf.resize(buf.len() + params.drpost, false);

            buf.into_vec()
        } else {
            vec![0; (drbits + 7) / 8]
        };
        let reply = self.transfer_dr(&request, drbits)?;

        // Process the reply
        let mut reply = BitVec::<Lsb0, u8>::from_vec(reply);
        if params.drpre > 0 {
            reply = reply.split_off(params.drpre);
        }
        reply.truncate(len_bits);
        let reply = reply.into_vec();

        // Idle cycles
        self.idle(self.idle_cycles as usize)?;

        Ok(reply)
    }

    fn read_register(&mut self, address: u32, len: u32) -> io::Result<Vec<u8>> {
        self.target_transfer(address, None, len as usize)
    }

    fn read_register32(&mut self, address: u32) -> io::Result<u32> {
        let r = self.read_register(address, 32)?;
        Ok(u32::from_le_bytes(r[0..4].try_into().unwrap()))
    }

    fn write_register(&mut self, address: u32, data: &[u8], len: u32) -> io::Result<Vec<u8>> {
        self.target_transfer(address, Some(data), len as usize)
    }

    fn write_register32(&mut self, address: u32, value: u32) -> io::Result<u32> {
        let r = self.write_register(address, &value.to_le_bytes(), 32)?;
        Ok(u32::from_le_bytes(r[0..4].try_into().unwrap()))
    }

    fn set_idle_cycles(&mut self, idle_cycles: u8) {
        self.idle_cycles = idle_cycles;
    }
}

fn main() {
    env_logger::init();

    println!("Opening probe...");

    let mut probe = match FtdiProbe::open(0x0403, 0x6010) {
        Ok(probe) => probe,
        Err(e) => {
            println!("Cannot find/open device: {:?}", e);
            return;
        }
    };
    probe.attach().unwrap();

    probe.reset().unwrap();
    probe.shift_ir(&[0x10], 5).unwrap();
    probe.idle(42);
    //probe.shift_ir(&[0x1f, 0x02], 10).unwrap();
    probe.select_target(0x1000563d).unwrap();
    probe.set_idle_cycles(8);

    let r = probe.read_register32(0x01).unwrap();
    println!("idcode: {:08x}", r);

    let r = probe.read_register32(0x10).unwrap();
    println!("dtmcs: {:08x}", r);
    let r = probe.read_register32(0x11).unwrap();
    println!("dmi: {:08x}", r);
    probe.write_register32(0x10, 0b11 << 16).unwrap();
    let r = probe.read_register32(0x10).unwrap();
    println!("dtmcs: {:08x}", r);
}
