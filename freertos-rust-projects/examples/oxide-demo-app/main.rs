// Copyright 2022, Ivan Palijan <ivan95.603@gmail.com>
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/license/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT> or BSD, at your
// option.  This file may not be copied, modified, or distributed
// except according to those terms.

//!
//! This app is intended to work on STM32 Black pill.
//! It is no-std comliant and uses wrapped FreeRTOS OS.
//! It uses MLX90614 crate and BMP180_nostd crate that I have written.
//! 

#![no_std]
#![no_main]

#![allow(dead_code)]
#![deny(missing_docs)]
#![deny(warnings)]

// For allocator
#![feature(lang_items)]
#![feature(alloc_error_handler)]

use freertos_rust::*;
#[macro_use]
extern crate alloc;
use core::alloc::Layout;
use alloc::{boxed::Box, string::String};

use cortex_m::{asm};
use cortex_m_rt::{exception, entry, ExceptionFrame};
use embedded_hal::digital::v2::OutputPin;
use stm32f4::stm32f411::I2C1;

use bmp180_nostd as bmp180;
use mlx9061x::{Mlx9061x, SlaveAddr};

use stm32f4xx_hal as hal;
use crate::hal::{ 
    pac::{Peripherals},
    i2c::I2c,
    prelude::*, 
    serial::config::Config, 
    serial::Serial,
    gpio::{
        Alternate,
        *,
    },
};

use core::fmt::{Error, Write}; // for pretty formatting of the serial output
extern crate panic_halt; // panic handler

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

/// Holds pin for LED
pub struct MyDevice<D1: OutputPin> {
    d1: D1,
}

impl<D1: OutputPin> MyDevice<D1>
{
    /// Return wrapped LED port
    pub fn from_pins(d1: D1) -> MyDevice<D1> {
        MyDevice {
            d1
        }
    }
    /// Set LED output level
    pub fn set_led(&mut self,on:bool){
        if on {
            let _ = self.d1.set_high();
        } else {
            let _ = self.d1.set_low();
        }
    }
}

// Function for writing to the serial port.
fn serial_writer<W: Write>(f: &mut W, s: &str) -> Result<(), Error> {
    f.write_str(s)
}

// Type for passing I2C to the shared_bus crate for multi thread usage of I2C.
type I2C1Bus = I2c<I2C1, (stm32f4xx_hal::gpio::Pin<Alternate<OpenDrain, 4_u8>, 'B', 8_u8>, stm32f4xx_hal::gpio::Pin<Alternate<OpenDrain, 4_u8>, 'B', 9_u8>)>;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let _cp = cortex_m::Peripherals::take().unwrap();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let rcc = dp.RCC.constrain();
    let clocks = rcc
                        .cfgr
                        .use_hse(25.MHz())
                        .sysclk(100.MHz())
                        .pclk1(50.MHz())
                        .pclk2(100.MHz())
                        .freeze();  

    // // Create a delay abstraction based on SysTick
    // let mut delayObj = hal::delay::Delay::new(cp.SYST, &clocks);


    // // Create a delay abstraction based on general-pupose 32-bit timer TIM5
    let mut delay_obj  = dp.TIM5.delay_us(&clocks);

    let mut device = MyDevice::from_pins(gpioc.pc13.into_push_pull_output());
    device.set_led(false);

    // define RX/TX pins for UART
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();

    // configure serial
    let serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(115200.bps()),
        &clocks,
    )
    .unwrap()

    // Make this Serial object use u16s instead of u8s
    .with_u8_data();

    let (mut tx, _rx) = serial.split();

    let gpiob = dp.GPIOB.split();
    let scl = gpiob
        .pb8
        .into_alternate()
        .internal_pull_up(true)
        .set_open_drain();
    let sda = gpiob
        .pb9
        .into_alternate()
        .internal_pull_up(true)
        .set_open_drain();


    delay_obj.delay_ms(100 as u32);


    let i2c = I2c::new(dp.I2C1, (scl, sda), /*57600*/100.kHz(), &clocks); //100.khz()
    

    // The bus is a 'static reference -> it lives forever and references can be
    // shared with other threads.
    let bus: &'static _ = shared_bus::new_cortexm!(I2C1Bus = i2c).unwrap();
    // For passing dyn trait object to BMP180 library.
    let boxy: Box<dyn embedded_hal::blocking::delay::DelayMs<u32>+ core::marker::Send> = Box::new(delay_obj);
    
    // Create and initilize pressure sensor.
    let mut barometer = bmp180::BMP180BarometerThermometer::new(bus.acquire_i2c(), boxy , bmp180::BMP180PressureMode::BMP180Standard);
    // Create and initialize ir temperature sensor.
    let mut ir_sensor = Mlx9061x::new_mlx90614(bus.acquire_i2c(), SlaveAddr::Alternative(0x5A), 5).unwrap();


    // RTOS requires base task in order to work.
    let _main_task = Task::new().name("main").start(move |_| 
    {
        // Task for communicating with pressure and temperature sensors and writing to serial port.
        Task::new().name("temp").priority(TaskPriority(2)).stack_size(4096).start(move |_| {
            loop{
                let mut txt_buff: String;

                // Measure MLX object temperature.
                let t_obj = ir_sensor.object1_temperature().unwrap_or(-1.0);
                txt_buff = format!("OT:{:.2}\n", &t_obj);
                serial_writer(&mut tx, &txt_buff).unwrap();
                freertos_rust::CurrentTask::delay(Duration::ms(300));

                
                // Measure MLX ambient temperature.
                let t_a = ir_sensor.ambient_temperature().unwrap_or(-1.0);
                txt_buff = format!("AT:{:.2}\n",&t_a);
                serial_writer(&mut tx, &txt_buff).unwrap();
                freertos_rust::CurrentTask::delay(Duration::ms(300));

                // Measure BMP180 pressure in hectopascals.
                let pressure_in_hpa: f32 = barometer.pressure_hpa();
                txt_buff = format!("P:{:.2}\n",&pressure_in_hpa);
                serial_writer(&mut tx, &txt_buff).unwrap();
                freertos_rust::CurrentTask::delay(Duration::ms(300));

                // Measure BMP180 temperature in celsius.
                let pressure_temp_celsius: f32 = barometer.temperature_celsius();
                txt_buff = format!("PTC:{:.2}\n",&pressure_temp_celsius);
                serial_writer(&mut tx, &txt_buff).unwrap();
                freertos_rust::CurrentTask::delay(Duration::ms(300));
            }
        }).unwrap();

        // Blinky blinky
        Task::new().name("led").priority(TaskPriority(2)).start(move |_| {
            loop{
                freertos_rust::CurrentTask::delay(Duration::ms(500));
                device.set_led(true);

                freertos_rust::CurrentTask::delay(Duration::ms(500));
                device.set_led(false);
            }
        }).unwrap();

    }).unwrap();

    // Start RTOS scheduler. Nothing below this call will be reached.
    FreeRtosUtils::start_scheduler();
}

// Below are fault handlers.

#[exception]
unsafe fn DefaultHandler(_irqn: i16) {
// custom default handler
// irqn is negative for Cortex-M exceptions
// irqn is positive for device specific (line IRQ)
// set_led(true);(true);
// panic!("Exception: {}", irqn);
}

#[exception]
unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
    asm::bkpt();
    loop {}
}

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    // set_led(true);
    asm::bkpt();
    loop {}
}

#[allow(non_snake_case)]
#[no_mangle]
fn vApplicationStackOverflowHook(_pxTask: FreeRtosTaskHandle, _pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}
