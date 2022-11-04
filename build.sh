
set -x
set -e

cargo build --release
arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/stm32rsled  target/thumbv7m-none-eabi/release/stm32rsled.bin
cp target/thumbv7m-none-eabi/release/stm32rsled.bin /mnt/c/Users/moesa/Downloads/stm32flash-0.7-binaries/
