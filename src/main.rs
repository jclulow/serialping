use std::io;
use std::io::{Write, Read};
use std::os::unix::io::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::sync::mpsc;
use std::time::{Duration, Instant};
use std::collections::VecDeque;

use anyhow::{bail, Result};
use termios::*;
use rand::Rng;

const B3000000: u32 = 29; /* XXX libc/termios need help here */
const CRTSXOFF: u32 = 0o10000000000; /* XXX */

fn open_serial(dev: &str) -> Result<std::fs::File> {
    Ok(std::fs::OpenOptions::new()
        .custom_flags(libc::O_NOCTTY)
        .write(true)
        .read(true)
        .open(dev)?)
}

fn setup_port(f: &std::fs::File) -> Result<()> {
    let fd = f.as_raw_fd();
    let mut t = Termios::from_fd(fd)?;
    cfmakeraw(&mut t);
    cfsetspeed(&mut t, B3000000)?;
    //cfsetspeed(&mut t, libc::B115200)?;
    //cfsetspeed(&mut t, libc::B921600)?;

    /*
     * We want to block forever waiting for one byte of data to arrive.
     */
    t.c_cc[VMIN] = 1;
    t.c_cc[VTIME] = 0;

    /*
     * 8 bits, no parity, 1 stop bit:
     */
    t.c_cflag &= !(CSIZE | CSTOPB | PARENB);
    t.c_cflag |= CS8;

    /*
     * Inbound and outbound hardware flow control:
     */
    t.c_cflag |= CRTSXOFF | libc::CRTSCTS;

    tcflush(fd, TCIOFLUSH)?;
    tcsetattr(fd, TCSANOW, &t)?;
    Ok(())
}

#[derive(Debug)]
struct Recv {
    when: Instant,
    value: u8,
}

fn main() -> Result<()> {
    let mut rng = rand::thread_rng();

    let mut t0 = open_serial("/dev/term/0")?;
    let mut t1 = open_serial("/dev/term/1")?;

    setup_port(&t0)?;
    setup_port(&t1)?;

    let (tx, rx) = mpsc::channel();

    std::thread::spawn(move || {
        loop {
            let mut buf = [0u8; 1];

            match t0.read(&mut buf) {
                Ok(sz) => {
                    let when = Instant::now();
                    if sz == 0 {
                        println!("zero bytes!");
                    } else {
                        tx.send(Recv { when, value: buf[0] }).unwrap();
                    }
                }
                Err(e) => {
                    eprintln!("read error: {:?}", e);
                    std::thread::sleep(Duration::from_secs(1));
                }
            }
        }
    });

    let mut samples = VecDeque::new();

    let mut last_print = Instant::now();
    let mut count = 0;

    loop {
        let byt = rng.gen_range(u8::MIN..=u8::MAX);
        //println!("rand = {}", byt);
        let mut outbuf = [byt];
        let start = Instant::now();
        t1.write_all(&outbuf)?;

        let r = rx.recv()?;
        let delta = r.when.checked_duration_since(start).unwrap();
        let deltaus = delta.as_micros();

        samples.push_front(deltaus);
        while samples.len() > 128 {
            samples.pop_back();
        }

        // if let Some(max) = sample_max {
        //     if max < deltaus {
        //         sample_max = Some(deltaus);
        //     }
        // } else {
        //     sample_max = Some(deltaus);
        // }
        // if let Some(min) = sample_min {
        //     if min > deltaus {
        //         sample_min = Some(deltaus);
        //     }
        // } else {
        //     sample_min = Some(deltaus);
        // }

        count += 1;
        if Instant::now().duration_since(last_print).as_millis() > 1000 {
            last_print = Instant::now();

            let sample_min = *samples.iter().min().unwrap();
            let sample_max = *samples.iter().max().unwrap();
            let avg: f64 = (samples.iter().sum::<u128>() as f64) /
                (samples.iter().count() as f64);

            println!("ping/sec {:>4} avg {:>6.2}   max {:>6.2}   min {:>6.2}   \
                cur {:>6.2}",
                count,
                avg / 1000.0,
                (sample_max as f64) / 1000.0,
                (sample_min as f64) / 1000.0,
                (deltaus as f64) / 1000.0);

            count = 0;
        }

        if r.value != byt {
            //println!("r = {:?}, delta = {} us {:0.1} ms", r, deltaus, delta);
            bail!("out of sync? sent {} != recv {}", byt, r.value);
        }

        //std::thread::sleep(Duration::from_millis(3));
    }

    Ok(())
}
