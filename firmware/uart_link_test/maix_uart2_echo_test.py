from maix import uart
import time

# 中文注释：通过 MaixCAM2 的 UART2 发送 PING，并等待 NUCLEO 原样回传。
serial = uart.UART("/dev/ttyS2", 115200)
message = b"PING:MAIXCAM2\r\n"

for attempt in range(10):
    serial.write(message)
    print("发送：", message)
    response = serial.read()
    if response:
        print("收到 NUCLEO 回传：", response)
        break
    print("尚未收到回传，重试次数：", attempt + 1)
    time.sleep(0.3)
else:
    print("未收到回传；检查 NUCLEO 是否已烧录 USART1 回显程序，以及 TX/RX/GND 接线。")
