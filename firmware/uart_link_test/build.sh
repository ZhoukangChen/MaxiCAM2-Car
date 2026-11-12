#!/bin/sh
set -eu

# 中文注释：在当前目录编译 NUCLEO-F446RE 的 USART1 回显测试固件。
cd "$(dirname "$0")"
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 -ffreestanding -fdata-sections -ffunction-sections -c startup.S -o startup.o
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -O2 -ffreestanding -fdata-sections -ffunction-sections -c main.c -o main.o
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -nostdlib -Wl,--gc-sections -Wl,-Map=nucleo_uart_echo.map -T linker.ld startup.o main.o -o nucleo_uart_echo.elf
arm-none-eabi-objcopy -O binary nucleo_uart_echo.elf nucleo_uart_echo.bin
arm-none-eabi-size nucleo_uart_echo.elf
