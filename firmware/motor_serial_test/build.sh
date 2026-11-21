#!/bin/sh
set -eu

# 中文注释：编译需要 USB 串口命令才会短时点动的电机测试固件。
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
OUT_DIR="$SCRIPT_DIR/build"
mkdir -p "$OUT_DIR"

arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 -Wall -Wextra -Werror \
    -ffreestanding -fdata-sections -ffunction-sections \
    -c "$SCRIPT_DIR/../uart_link_test/startup.S" -o "$OUT_DIR/startup.o"
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 -Wall -Wextra -Werror \
    -ffreestanding -fdata-sections -ffunction-sections \
    -c "$SCRIPT_DIR/main.c" -o "$OUT_DIR/main.o"
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -nostdlib \
    -Wl,--gc-sections -Wl,-Map="$OUT_DIR/nucleo_motor_serial_test.map" \
    -T "$SCRIPT_DIR/../uart_link_test/linker.ld" \
    "$OUT_DIR/startup.o" "$OUT_DIR/main.o" -o "$OUT_DIR/nucleo_motor_serial_test.elf"
arm-none-eabi-objcopy -O binary "$OUT_DIR/nucleo_motor_serial_test.elf" \
    "$OUT_DIR/nucleo_motor_serial_test.bin"
arm-none-eabi-size "$OUT_DIR/nucleo_motor_serial_test.elf"
