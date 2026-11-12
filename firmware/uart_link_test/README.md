# MaixCAM2 ↔ NUCLEO-F446RE 串口连通测试

此目录包含一次性的 UART 连通测试固件。NUCLEO 固件在 USART1 上回显收到的字节；MaixCAM2 脚本发送 `PING` 并等待回传。

NUCLEO 使用 PA9 / USART1_TX、PA10 / USART1_RX，串口参数为 115200、8N1。MaixCAM2 使用 UART2：U2T / B0 连接 PA10，U2R / B1 连接 PA9，并连接两板 GND。

构建固件：

```sh
sh build.sh
```

待 Mac 识别到 NUCLEO 的 ST-LINK 后，再用 ST-LINK 下载 `nucleo_uart_echo.bin`。此测试固件会覆盖 NUCLEO 当前程序；测试完成后可再刷入正式小车程序。

## 本次硬件验证记录

- 2026-11-12：MaixCAM2 经 `/dev/ttyS2` 发送 `PING:MAIXCAM2\r\n`，NUCLEO-F446RE 原样回传，UART2 ↔ USART1 的 TX、RX、GND 接线已通过实测。
- 测试前备份的 512 KiB 原固件位于 `backups/nucleo_flash_backup_2026-11-12.bin`，SHA-256 为 `6e7adb9e390f22ba10483e7d27d88a15c253d6a57e62905b59cced8a9b7b14ce`。
- 这份原厂固件备份只保存在本机 `backups/` 目录，已加入 Git 忽略规则，不会上传到仓库。
- 回显测试后已恢复原固件；整片 Flash 读回内容与备份逐字节一致。
- 后续已在 `../uart_protocol_v1/` 烧录结构化消息协议 V1；NUCLEO 当前运行协议 V1，原固件备份仍保留在此目录。
