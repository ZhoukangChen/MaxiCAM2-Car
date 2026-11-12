#!/bin/sh
set -eu

# 中文注释：编译 NUCLEO-F446RE 的 UART 协议接收与回 ACK 固件。
cd "$(dirname "$0")"
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 -Wall -Wextra -ffreestanding -fdata-sections -ffunction-sections -c startup.S -o startup.o
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 -Wall -Wextra -ffreestanding -fdata-sections -ffunction-sections -c main.c -o main.o
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -nostdlib -Wl,--gc-sections -Wl,-Map=nucleo_uart_protocol_v1.map -T linker.ld startup.o main.o -o nucleo_uart_protocol_v1.elf
arm-none-eabi-objcopy -O binary nucleo_uart_protocol_v1.elf nucleo_uart_protocol_v1.bin
arm-none-eabi-size nucleo_uart_protocol_v1.elf
