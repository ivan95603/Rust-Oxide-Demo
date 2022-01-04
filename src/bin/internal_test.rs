#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use crate::hal::{block, pac, prelude::*, serial::config::Config, serial::Serial, serial::*};


use core::fmt; // for pretty formatting of the serial output



#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    let gpioa = dp.GPIOA.split();

    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.use_hse(25.mhz()).freeze();

    let mut delay = hal::delay::Delay::new(cp.SYST, &clocks);


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

    loop {

       let txt = "TEST\n";

       use embedded_hal::serial::Write;
       txt.bytes()
           .try_for_each(|c| block!(tx.write(c)))
           .map_err(|_| fmt::Error);





        //tx.write_str("s: &str");

        // write_str(&mut tx, "hola").unwrap();
      
       // writeln!(tx, "value: {:02}\r", value).unwrap();

        delay.delay_ms(500_u32);
    }
}










// //! blinky timer using interrupts on TIM2, adapted from blinky_timer_irq.rs example from
// //! stm32f1xx-hal
// //!
// //! This assumes that a LED is connected to pa5 (sck/d13) as is the case on most nucleo board.

// #![no_main]
// #![no_std]

// use panic_halt as _;

// use stm32f4xx_hal as hal;

// use crate::hal::{
//     gpio::{gpioc, Output, PushPull},
//     pac::{interrupt, Interrupt, Peripherals, TIM2},
//     prelude::*,
//     timer::{CounterUs, Event, Timer},
// };

// use core::cell::RefCell;
// use cortex_m::{asm::wfi, interrupt::Mutex};
// use cortex_m_rt::entry;

// // NOTE You can uncomment 'hprintln' here and in the code below for a bit more
// // verbosity at runtime, at the cost of throwing off the timing of the blink
// // (using 'semihosting' for printing debug info anywhere slows program
// // execution down)
// //use cortex_m_semihosting::hprintln;

// // A type definition for the GPIO pin to be used for our LED
// // For the onboard nucleo LED, use gpioa::PA5 or gpiob::PB13 depending your model
// type LedPin = gpioc::PC13<Output<PushPull>>;

// // Make LED pin globally available
// static G_LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));

// // Make timer interrupt registers globally available
// static G_TIM: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));

// // Define an interupt handler, i.e. function to call when interrupt occurs.
// // This specific interrupt will "trip" when the timer TIM2 times out
// #[interrupt]
// fn TIM2() {
//     static mut LED: Option<LedPin> = None;
//     static mut TIM: Option<CounterUs<TIM2>> = None;

//     let led = LED.get_or_insert_with(|| {
//         cortex_m::interrupt::free(|cs| {
//             // Move LED pin here, leaving a None in its place
//             G_LED.borrow(cs).replace(None).unwrap()
//         })
//     });

//     let tim = TIM.get_or_insert_with(|| {
//         cortex_m::interrupt::free(|cs| {
//             // Move LED pin here, leaving a None in its place
//             G_TIM.borrow(cs).replace(None).unwrap()
//         })
//     });

//     let _ = led.toggle();
//     let _ = tim.wait();
// }

// #[entry]
// fn main() -> ! {
//     let dp = Peripherals::take().unwrap();

//     let rcc = dp.RCC.constrain();
//     let clocks = rcc.cfgr.sysclk(25.mhz()).pclk1(/*(12.5).mhz()*/12500000).freeze();

//     // Configure PA5 pin to blink LED
//     let gpioc = dp.GPIOC.split();
//     let mut led = gpioc.pc13.into_push_pull_output();
//     let _ = led.set_high(); // Turn off

//     // Move the pin into our global storage
//     cortex_m::interrupt::free(|cs| *G_LED.borrow(cs).borrow_mut() = Some(led));

//     // Set up a timer expiring after 1s
//     let mut timer = Timer::new(dp.TIM2, &clocks).counter();
//     timer.start(1.secs()).unwrap();

//     // Generate an interrupt when the timer expires
//     timer.listen(Event::TimeOut);

//     // Move the timer into our global storage
//     cortex_m::interrupt::free(|cs| *G_TIM.borrow(cs).borrow_mut() = Some(timer));

//     //enable TIM2 interrupt
//     unsafe {
//         cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
//     }

//     loop {
//         wfi();
//     }
// }
