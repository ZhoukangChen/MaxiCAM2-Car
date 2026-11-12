# MaixCAM2 → NUCLEO-F446RE 视觉数据协议 V1

此版本不依赖比赛场地。MaixCAM2 发送模拟路线和障碍物结果；NUCLEO 接收、校验 CRC、返回 ACK，并点亮 LD2 表示最近收到有效数据。连续 300 ms 没有有效帧后，LD2 熄灭。此版本不驱动电机。

## 帧格式

数据帧格式为 `@payload*CRC16\r\n`。CRC 使用 CRC-16/CCITT-FALSE，覆盖 `@` 后、`*` 前的 ASCII 字节。

字段顺序：

`版本,序号,路线有效,路线横向偏差千分值,航向偏差毫度,障碍有效,障碍中心千分值,障碍宽度千分值,置信度千分值`

横向偏差正值表示识别到的路线中心位于图像中心右侧；障碍中心为图像从左到右 0–1000；当前字段表示图像内的归一化位置，不代表毫米距离。毫米测距需要后续标定或接入 ToF。

## 运行

构建固件：

```sh
sh build.sh
```

烧录前使用 `uart_link_test/backups/nucleo_flash_backup_2026-11-12.bin` 保留的原程序备份。NUCLEO 的 USART1 为 PA9 TX、PA10 RX；MaixCAM2 使用 UART2 的 U2T/B0、U2R/B1，115200、8N1。

Maix 端模拟测试脚本：`maix_uart_protocol_sim.py`。它发送 5 组模拟数据，核对返回帧中的序号、状态和 CRC，再发送一组 CRC 错误帧，确认 NUCLEO 拒收。

## 本次实测

- 2026-11-12：已在实际 MaixCAM2 与 NUCLEO-F446RE 上联调，5 组模拟帧全部收到序号和字段匹配的 `OK` ACK。
- 再发送一帧故意损坏 CRC 的数据后，NUCLEO 返回 `ERR`、错误码 `2`；模拟端验证了 ACK 的 CRC。
- NUCLEO 当前烧录 `nucleo_uart_protocol_v1.bin`。LD2 新鲜数据指示逻辑已写入；LED 在数据停止后的熄灭行为需要现场目视确认。
- 本次固件以 `-Wall -Wextra` 编译，无编译警告；固件写入后由 ST-LINK 校验成功。
- 原厂演示固件备份仅保存在本机 `../uart_link_test/backups/` 目录，该目录已加入 Git 忽略规则；SHA-256 记录见 `../uart_link_test/README.md`。
