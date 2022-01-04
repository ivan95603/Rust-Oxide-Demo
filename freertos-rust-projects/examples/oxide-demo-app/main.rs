#![no_std]
#![no_main]
// For allocator
#![feature(lang_items)]
#![feature(alloc_error_handler)]

use cortex_m::asm;
use cortex_m_rt::exception;
use cortex_m_rt::{entry, ExceptionFrame};
use embedded_hal::digital::v2::OutputPin;
use freertos_rust::*;
use core::alloc::Layout;
use stm32f4xx_hal::gpio::*;

use cortex_m;
use stm32f4xx_hal as hal;

use crate::hal::{
    pac, 
    pac::{Peripherals},
    block, 
    prelude::*, 
    serial::config::Config, 
    serial::Serial, serial::*
};

use core::fmt; // for pretty formatting of the serial output

extern crate panic_halt; // panic handler

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

fn delay() {
    let mut _i = 0;
    for _ in 0..2_00 {
        _i += 1;
    }
}

fn delay_n(n: i32) {
    for _ in 0..n {
        delay();
    }
}

pub struct MyDevice<D1: OutputPin> {
    d1: D1,
}

impl<D1: OutputPin> MyDevice<D1>
{
    pub fn from_pins(d1: D1) -> MyDevice<D1> {
        MyDevice {
            d1
        }
    }
    pub fn set_led(&mut self,on:bool){
        if on {
            self.d1.set_high();
        } else {
            self.d1.set_low();
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.mhz()).freeze();

    let mut device = MyDevice::from_pins(gpioc.pc13.into_push_pull_output());
    device.set_led(false);



    // define RX/TX pins
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

    let (mut tx, mut rx) = serial.split();


    let txt = "RTOS UART TEST\n";







    Task::new().name("hello").stack_size(128).priority(TaskPriority(2)).start(move |_| {
        loop{
            freertos_rust::CurrentTask::delay(Duration::ms(1000));
            device.set_led(true);



            use embedded_hal::serial::Write;
            txt.bytes()
                .try_for_each(|c| block!(tx.write(c)));


            freertos_rust::CurrentTask::delay(Duration::ms(1000));
            device.set_led(false);
        }
    }).unwrap();
    FreeRtosUtils::start_scheduler();
}

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
// Blink 3 times long when exception occures
    delay_n(10);
    for _ in 0..3 {
        // set_led(true);
        // delay_n(1000);
        // set_led(false);
        // delay_n(555);
    }
    loop {}
}

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    //set_led(true);
    asm::bkpt();
    loop {}
}

#[no_mangle]
fn vApplicationStackOverflowHook(pxTask: FreeRtosTaskHandle, pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}
