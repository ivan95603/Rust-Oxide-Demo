## First time opening the project:
### Execute folowing commands:
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    
    rustup default nightly
    rustup toolchain install nightly-gnu
    rustup default nightly-gnu

    rustup target add thumbv7em-none-eabihf

    cargo install cargo-binutils
    rustup component add llvm-tools-preview

</br>
</br>

## Update compiler package

    rustup update

# Every time:

## Start OpenOCD instance in separate terminal first and connect [BlackPill Board (STM32F411CEU6)](https://www.adafruit.com/product/4877):
    openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

</br>
</br>

# How to:

## Build the demo app:
    cargo build --package freertos-rust-projects --example oxide-demo-app --target thumbv7em-none-eabihf

## Transfer and run on device:
    cargo run --package freertos-rust-projects --example oxide-demo-app --target thumbv7em-none-eabihf

## Create hex file to be flashed (If you need hex for production else use step above):

    cargo objcopy --package freertos-rust-projects  --example oxide-demo-app --target thumbv7em-none-eabihf -- -O ihex oxide-demo-app.hex




## Check nightly build for RLS component

    cargo install lorikeet

</br>

    lorikeet check_rls.yml 