extern crate tessel;
extern crate libc;

use std::result::Result;
use std::thread::{sleep, yield_now};
use std::time::{Duration, Instant};
use std::cmp;
use std::fs::File;

use std::io::{Write, Read};


use tessel::*;

// Maximum number of ticks before period completes
const PWM_MAX_PERIOD : usize = 0xFFFF;
// Actual lowest frequency is ~0.72Hz but 1Hz is easier to remember.
// 5000 is the max because any higher and the resolution drops
// below 7% (0xFFFF/5000 ~ 7.69) which is confusing
const PWM_MAX_FREQUENCY : usize = 5000;
const PWM_MIN_FREQUENCY : usize = 1;
const PWM_PRESCALARS : &'static [usize] = &[1, 2, 4, 8, 16, 64, 256, 1024];
// Maximum number of unscaled ticks in a second (48 MHz)
const SAMD21_TICKS_PER_SECOND : usize = 48000000;
// GPIO number of RESET pin
// const SAMD21_RESET_GPIO = 39;

#[derive(Debug, Clone, PartialEq)]
enum PwmFrequencyError {
    Range,
    Prescalar,
}

fn pwm_frequency(port: &mut protocol::PortSocket, frequency: usize) -> Result<(), PwmFrequencyError> {
    if frequency < PWM_MIN_FREQUENCY || frequency > PWM_MAX_FREQUENCY {
        return Err(PwmFrequencyError::Range);
    }

    let (period, prescalar_index) = try!(determine_duty_cycle_and_prescalar(frequency));

    unsafe {
      pwm_period = period;
      pwm_prescalar_index = prescalar_index;
    }

    let TCC_ID = 0;

    port.write_command(protocol::Command::PwmPeriod{
      prescalar: prescalar_index as u8,
      tcc_id: TCC_ID,
      period: period as u16,
    }).unwrap();

    Ok(())
}

static mut pwm_period : usize = 0;
static mut pwm_prescalar_index : usize = 0;

fn determine_duty_cycle_and_prescalar(frequency: usize) -> Result<(usize, usize), PwmFrequencyError> {
    let mut period = 0;
    let mut prescalar_index = 0;

    loop {
        period = (
          SAMD21_TICKS_PER_SECOND as f32 /
          PWM_PRESCALARS[prescalar_index] as f32 /
          frequency as f32
        ).floor() as usize;

        if period < PWM_MAX_PERIOD {
            break;
        }

        prescalar_index += 1;
        if prescalar_index == PWM_PRESCALARS.len() {
            return Err(PwmFrequencyError::Prescalar);
        }
    }

    Ok((period, prescalar_index))
}

#[derive(Debug, Clone, PartialEq)]
enum PwmDutyError {
    Range,
    NoFrequency,
}

fn pwm_duty_cycle(port: &mut protocol::PortSocket, pin: u8, duty_cycle: f32) -> Result<(), PwmDutyError> {
    if duty_cycle > 1.0 || duty_cycle < 0.0 {
        return Err(PwmDutyError::Range);
    }

    unsafe {
        if pwm_period == 0 {
            return Err(PwmDutyError::NoFrequency);
        }
    }

    let duty_period = unsafe { (duty_cycle * pwm_period as f32).floor() as u16 };
    port.write_command(protocol::Command::PwmDutyCycle {
      pin: pin,
      duty_cycle: duty_period,
    }).unwrap();

    Ok(())
}

fn gpio_set(port: &mut protocol::PortSocket, pin: u8, value: u8) {
  port.write_command(
    if value == 0 {
      protocol::Command::GpioLow(pin)
    }
    else {
      protocol::Command::GpioHigh(pin)
    }
  ).unwrap();
}

fn echo(port: &mut protocol::PortSocket, queue: &mut Vec<Box<FnMut()>>, cb: Box<FnMut()>) {
  queue.push(cb);
  port.write_command(
    protocol::Command::Echo(&[
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00,
    ]),
  ).unwrap();
  // port.socket.flush().unwrap();
}

fn main() {
    // unsafe {
    //     use libc;
    //     use std::ffi::CString;
    //     use std::mem::transmute;
    //     let fd = libc::fopen(CString::new("/dev/spidev32766.1").unwrap().into_raw(), CString::new("r").unwrap().into_raw());
    //     let mut speed = 0u32;
    //     libc::ioctl(transmute(fd), 0x80046b04, &mut speed as *mut u32);
    //     println!("speed {}", speed);
    //     libc::fclose(fd);
    // }

    let mut tessel = Tessel::new();
    tessel.led[2].on().unwrap();
    let (port_a, port_b) = Tessel::ports().unwrap();

    let mut socket_a = port_a.socket.lock().unwrap();
    let mut socket = port_b.socket.lock().unwrap();

    pwm_frequency(&mut socket_a, 5000);

    let mut queue = Vec::new();

    // let f = File::create("/tmp/echo_times").unwrap();

    let start = Instant::now();
    // let mut f_ = f.try_clone().unwrap();
    echo(&mut socket_a, &mut queue, Box::new(move || {
        // f_.write_fmt(format_args!("{}\n", Instant::now().duration_since(start).subsec_nanos()));
        println!("{:?}", Instant::now().duration_since(start));
    }));

    loop {
        if queue.len() == 0 {
            break;
        }
        let mut buffer = [0];
        socket_a.read_exact(&mut buffer).unwrap();
        if buffer[0] == 0x84 {
            (queue.remove(0))();
        }
    }

    loop {
        sleep(Duration::from_millis(900));

        // let start = Instant::now();
        // echo(&mut socket_a, &mut queue, Box::new(move || {
        //     // println!("{:?}", Instant::now().duration_since(start));
        // }));
        // let start = Instant::now();
        // let mut f_ = f.try_clone().unwrap();
        // // println!("{}", Instant::now().duration_since(start).subsec_nanos());
        // echo(&mut socket_a, &mut queue, Box::new(move || {
        //     f_.write_fmt(format_args!("{}\n", Instant::now().duration_since(start).subsec_nanos()));
        //     // println!("{}", Instant::now().duration_since(start).subsec_nanos());
        // }));
        // let start = Instant::now();
        // echo(&mut socket_a, &mut queue, Box::new(move || {
        //     // println!("{:?}", Instant::now().duration_since(start));
        // }));
        // let start = Instant::now();
        // echo(&mut socket_a, &mut queue, Box::new(move || {
        //     // println!("{:?}", Instant::now().duration_since(start));
        // }));
        let start = Instant::now();
        // let mut f = f.try_clone().unwrap();
        echo(&mut socket_a, &mut queue, Box::new(move || {
            // f.write_fmt(format_args!("{}\n", Instant::now().duration_since(start).subsec_nanos()));
            println!("{:?}", Instant::now().duration_since(start));
        }));

        for i in 0..40 {
            // gpio_set(&mut socket_a, 7, 1);
            // gpio_set(&mut socket_a, 7, 0);
            // pwm_duty_cycle(&mut socket_a, 5, 0.5);
        }

        loop {
            if queue.len() == 0 {
                break;
            }
            let mut buffer = [0];
            // yield_now();
            // sleep(Duration::from_millis(1));
            socket_a.read_exact(&mut buffer).unwrap();
            if buffer[0] == 0x84 {
                (queue.remove(0))();
                // socket_a.read_exact(&mut [0; 253]).unwrap();
            }
        }
        // loop {
        //     if queue.len() == 0 {
        //         break;
        //     }
        //     let mut buffer = [0];
        //     socket.read_exact(&mut buffer).unwrap();
        //     if buffer[0] == 0x84 {
        //         (queue.remove(0))();
        //         socket.read_exact(&mut [0]).unwrap();
        //     }
        // }
    }
}
