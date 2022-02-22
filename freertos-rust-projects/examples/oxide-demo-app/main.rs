#![no_std]
#![no_main]
// For allocator
#![feature(lang_items)]
#![feature(alloc_error_handler)]

use alloc::{string::ToString, boxed::Box};
use cortex_m::{asm, delay::Delay};
use cortex_m_rt::{exception, entry, ExceptionFrame};
use embedded_hal::digital::v2::OutputPin;
use freertos_rust::*;
use core::alloc::Layout;
use stm32f4xx_hal::gpio::*;

use cortex_m;


use stm32f4::stm32f411::I2C1;

use stm32f4xx_hal as hal;

use bmp180_nostd as bmp180;


use crate::hal::{
    pac, 
    pac::{Peripherals},
    i2c::I2c,
    block, 
    prelude::*, 
    serial::config::Config, 
    serial::Serial, serial::*,
    gpio,
    gpio::{
        Alternate,
        gpiob,
    },
};

use core::fmt::{Error, Write}; // for pretty formatting of the serial output

#[macro_use]
extern crate alloc;

use mlx9061x::{Mlx9061x, SlaveAddr};
use rtt_target::{rprintln, rtt_init_print};

extern crate panic_halt; // panic handler

#[global_allocator]
static GLOBAL: FreeRtosAllocator = FreeRtosAllocator;

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


fn serial_writer<W: Write>(f: &mut W, s: &str) -> Result<(), Error> {
    f.write_str(s)
}


type I2C1Bus = I2c<I2C1, (stm32f4xx_hal::gpio::Pin<Alternate<OpenDrain, 4_u8>, 'B', 8_u8>, stm32f4xx_hal::gpio::Pin<Alternate<OpenDrain, 4_u8>, 'B', 9_u8>)>;



#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("MLX90614 example");

    let dp = Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let rcc = dp.RCC.constrain();
    let clocks = rcc
                        .cfgr
                        .use_hse(25.mhz())
                        .sysclk(100.mhz())
                        .pclk1(50.mhz())
                        .pclk2(100.mhz())
                        .freeze();  

    // Create a delay abstraction based on SysTick
    let mut delayObj = hal::delay::Delay::new(cp.SYST, &clocks);


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

    let (mut tx, rx) = serial.split();

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


    delayObj.delay_ms(100 as u32);


    let i2c = I2c::new(dp.I2C1, (scl, sda), 57600, &clocks); //100.khz()
    

    // The bus is a 'static reference -> it lives forever and references can be
    // shared with other threads.
    let bus: &'static _ = shared_bus::new_cortexm!(I2C1Bus = i2c).unwrap();



    let mut boxy = Box::new(Box::new(&mut delayObj));






    

    // let mut barometer = bmp180::BMP180BarometerThermometer::new(bus.acquire_i2c(), &mut boxy, bmp180::BMP180PressureMode::BMP180Standard);





    let mut sensor = Mlx9061x::new_mlx90614(bus.acquire_i2c(), SlaveAddr::Alternative(0x5A), 5).unwrap();







    // delayObj.delay_ms(100 as u32);

////////////////////////////

    let mut txt_buff = "RTOS UART TEST\n";

    Task::new().name("hello").stack_size(1024).priority(TaskPriority(2)).start(move |_| {
        loop{
            freertos_rust::CurrentTask::delay(Duration::ms(100));
            device.set_led(true);

            // let pressure_in_pascals: f32 = barometer.pressure_pa();


            let t_obj = sensor.object1_temperature().unwrap_or(-1.0);
            freertos_rust::CurrentTask::delay(Duration::ms(10));
            let t_a = sensor.ambient_temperature().unwrap_or(-1.0);

            let text = format!("OT:{}\n", &t_obj);
            serial_writer(&mut tx, &text).unwrap();
            freertos_rust::CurrentTask::delay(Duration::ms(100));
            let text = format!("AT:{}\n",&t_a);
            serial_writer(&mut tx, &text).unwrap();

            // let text = format!("P:{}\n",&pressure_in_pascals);
            // serial_writer(&mut tx, &text).unwrap();

            

            freertos_rust::CurrentTask::delay(Duration::ms(100));
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

#[no_mangle]
fn vApplicationStackOverflowHook(pxTask: FreeRtosTaskHandle, pcTaskName: FreeRtosCharPtr) {
    asm::bkpt();
}
