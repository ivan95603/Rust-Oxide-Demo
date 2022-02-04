#![no_std]
#![no_main]
// For allocator
#![feature(lang_items)]
#![feature(alloc_error_handler)]

use cortex_m::asm;
use cortex_m_rt::{exception, entry, ExceptionFrame};
use freertos_rust::*;
use hal::i2s::Pins;
use core::alloc::Layout;
use stm32f4xx_hal::gpio::*;

use cortex_m;


use stm32f4::stm32f411::I2C1;

use stm32f4xx_hal as hal;

use crate::hal::{
    pac, 
    pac::{Peripherals},
    i2c::I2c,
    block, 
    prelude::*, 
    gpio,
    gpio::{
        Alternate,
        gpiob,
    },
};

use mlx9061x::{Mlx9061x, SlaveAddr};


extern crate panic_halt; // panic handler

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;


// TODO:  Initial should be tested. Passes compilation. 
type I2C1Bus = I2c<I2C1, (stm32f4xx_hal::gpio::Pin<Alternate<OpenDrain, 4_u8>, 'B', 8_u8>, stm32f4xx_hal::gpio::Pin<Alternate<OpenDrain, 4_u8>, 'B', 9_u8>)>;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.mhz()).freeze();
  

/**
 * 
 * TODO: LOOKS OK SHOUD BE TESTED
 * 
*/


    let mut gpiob = dp.GPIOB.split();
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



    let i2c = I2c::new(dp.I2C1, (scl, sda), 400.khz(), &clocks);

    // The bus is a 'static reference -> it lives forever and references can be
    // shared with other threads.
    let bus: &'static _ = shared_bus::new_cortexm!(I2C1Bus = i2c).unwrap();

    let mut sensor = Mlx9061x::new_mlx90614(bus.acquire_i2c(), SlaveAddr::default(), 5).unwrap();


    Task::new().name("hello").stack_size(512).priority(TaskPriority(2)).start(move |_| {
        loop{
            freertos_rust::CurrentTask::delay(Duration::ms(1000));


            let t_obj = sensor.object1_temperature().unwrap_or(-1.0);
            

            freertos_rust::CurrentTask::delay(Duration::ms(1000));

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
    loop {}
}

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(_layout: Layout) -> ! {
    // set_led(true);
    asm::bkpt();
    loop {}
}

#[no_mangle]
fn vApplicationStackOverflowHook(pxTask: FreeRtosTaskHandle, pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}
























// #![no_std]
// #![no_main]
// // For allocator
// #![feature(lang_items)]
// #![feature(alloc_error_handler)]

// use cortex_m::asm;
// use cortex_m_rt::{exception, entry, ExceptionFrame};
// use embedded_hal::digital::v2::OutputPin;
// use freertos_rust::*;
// use core::alloc::Layout;
// use stm32f4xx_hal::gpio::*;

// use cortex_m;


// use stm32f4::stm32f411::I2C1;

// use stm32f4xx_hal as hal;

// use crate::hal::{
//     pac, 
//     pac::{Peripherals},
//     i2c::I2c,
//     block, 
//     prelude::*, 
//     serial::config::Config, 
//     serial::Serial, serial::*,
//     gpio,
//     gpio::{
//         Alternate,
//         gpiob,
//     },
// };

// use core::fmt::{Error, Write}; // for pretty formatting of the serial output

// use mlx9061x::{Mlx9061x, SlaveAddr};
// use rtt_target::{rprintln, rtt_init_print};

// extern crate panic_halt; // panic handler

// #[global_allocator]
// static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

// fn delay() {
//     let mut _i = 0;
//     for _ in 0..2_00 {
//         _i += 1;
//     }
// }

// fn delay_n(n: i32) {
//     for _ in 0..n {
//         delay();
//     }
// }

// pub struct MyDevice<D1: OutputPin> {
//     d1: D1,
// }

// impl<D1: OutputPin> MyDevice<D1>
// {
//     pub fn from_pins(d1: D1) -> MyDevice<D1> {
//         MyDevice {
//             d1
//         }
//     }
//     pub fn set_led(&mut self,on:bool){
//         if on {
//             self.d1.set_high();
//         } else {
//             self.d1.set_low();
//         }
//     }
// }


// fn serial_writer<W: Write>(f: &mut W, s: &str) -> Result<(), Error> {
//     f.write_str(s)
// }





// type I2C1Bus = I2c<I2C1, (gpiob::PB8<Alternate<gpio::AF1>>, gpiob::PB9<Alternate<gpio::AF1>>)>;

// #[entry]
// fn main() -> ! {
//     rtt_init_print!();
//     rprintln!("MLX90614 example");

//     let dp = Peripherals::take().unwrap();
//     let gpioa = dp.GPIOA.split();
//     let gpioc = dp.GPIOC.split();

//     let rcc = dp.RCC.constrain();
//     let clocks = rcc.cfgr.use_hse(25.mhz()).freeze();

//     let mut device = MyDevice::from_pins(gpioc.pc13.into_push_pull_output());
//     device.set_led(false);

//     // define RX/TX pins
//     let tx_pin = gpioa.pa2.into_alternate();
//     let rx_pin = gpioa.pa3.into_alternate();

//     // configure serial
//     let serial = Serial::new(
//         dp.USART2,
//         (tx_pin, rx_pin),
//         Config::default().baudrate(115200.bps()),
//         &clocks,
//     )
//     .unwrap()

//     // Make this Serial object use u16s instead of u8s
//     .with_u8_data();

//     let (mut tx, rx) = serial.split();










// /**
//  * 
//  * TODO: LOOKS OK SHOUD BE TESTED
//  * 
// */


//     let mut gpiob = dp.GPIOB.split();
//     let scl = gpiob
//         .pb8
//         .into_alternate()
//         .internal_pull_up(true)
//         .set_open_drain();
//     let sda = gpiob
//         .pb9
//         .into_alternate()
//         .internal_pull_up(true)
//         .set_open_drain();





//     let i2c = I2c::new(dp.I2C1, (scl, sda), 400.khz(), &clocks);


//     // The bus is a 'static reference -> it lives forever and references can be
// // shared with other threads.
// let bus: &'static _ = shared_bus::new_cortexm!(I2C1Bus = i2c).unwrap();






//     // let bus: &'static _ = shared_bus::new_cortexm!(SomeI2cBus = i2c).unwrap();

//     let mut sensor = Mlx9061x::new_mlx90614(bus.acquire_i2c(), SlaveAddr::default(), 5).unwrap();



// ////////////////////////////











//     let mut txt_buff = "RTOS UART TEST\n";

//     Task::new().name("hello").stack_size(512).priority(TaskPriority(2)).start(move |_| {
//         loop{
//             freertos_rust::CurrentTask::delay(Duration::ms(1000));
//             device.set_led(true);




//             let t_obj = sensor.object1_temperature().unwrap_or(-1.0);
//             // freertos_rust::CurrentTask::delay(Duration::ms(50));
//             // let t_a = sensor.ambient_temperature().unwrap_or(-1.0);


//     // t_a.to_string();
//             // txt_buff = "Object: " + str:: ::from(t_obj);
//             // serial_writer(&mut tx, &txt_buff).unwrap();
//             // txt_buff = "Ambient: " + String::from(t_a);
//             // serial_writer(&mut tx, &txt_buff).unwrap();






            

//             freertos_rust::CurrentTask::delay(Duration::ms(1000));
//             device.set_led(false);
//         }
//     }).unwrap();
//     FreeRtosUtils::start_scheduler();
// }

// #[exception]
// unsafe fn DefaultHandler(_irqn: i16) {
// // custom default handler
// // irqn is negative for Cortex-M exceptions
// // irqn is positive for device specific (line IRQ)
// // set_led(true);(true);
// // panic!("Exception: {}", irqn);
// }

// #[exception]
// unsafe fn HardFault(_ef: &ExceptionFrame) -> ! {
// // Blink 3 times long when exception occures
//     delay_n(10);
//     for _ in 0..3 {
//         // set_led(true);
//         // delay_n(1000);
//         // set_led(false);
//         // delay_n(555);
//     }
//     loop {}
// }

// // define what happens in an Out Of Memory (OOM) condition
// #[alloc_error_handler]
// fn alloc_error(_layout: Layout) -> ! {
//     // set_led(true);
//     asm::bkpt();
//     loop {}
// }

// #[no_mangle]
// fn vApplicationStackOverflowHook(pxTask: FreeRtosTaskHandle, pcTaskName: FreeRtosCharPtr) {
//     asm::bkpt();
// }
