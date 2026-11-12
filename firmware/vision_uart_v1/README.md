# MaixCAM2 视觉处理与 UART V1

此目录开始实现不依赖比赛场地的第一版视觉数据处理：输入灰度图，沿三个高度扫描黑线，估计路线横向偏移和局部方向；再从深色连通区域里找一个与路线走廊相交的障碍候选框。结果通过已验证的 UART 协议发送给 NUCLEO。

## 当前算法边界

- 路线输入假定为浅色背景上的深色线，阈值由 Otsu 直方图分割并限制在安全范围内；现场照明、曝光和相机角度仍需用赛道复核。
- `line_dx_permille` 是图像归一化偏移，正值表示路线中心位于画面中心右侧；`heading_millideg` 是由扫描线中心变化估出的局部方向误差。
- 障碍物输出是“深色连通区域候选”，坐标和宽度是图像归一化量，不是毫米距离，也不等于已经判断会碰撞。白色或与背景接近的物体需要后续补充边缘/颜色方法或 ToF。
- 当前 `confidence_permille` 在路线与障碍同时有效时取两者较小值，表示保守的整帧置信度。

## 无场地测试

[黑色弯道正样例](../../figure/vision-test/curved_track.svg) 是确定性的路线和深色物体测试图；[空白负样例](../../figure/vision-test/blank_scene.svg) 只有分散物体、没有路线。对应 PNG 也放在同一目录，上传到 MaixCAM2 后可离线运行：

```sh
python3 maix_vision_uart_v1.py --image /tmp/curved_track.png
```

负样例也用同一命令，图像路径替换为 `/tmp/blank_scene.png`；预期 `line_valid=0`、`obstacle_valid=0`。可在最后增加保存路径，例如 `python3 maix_vision_uart_v1.py --image /tmp/curved_track.png /tmp/curved_overlay.png`。

读取真实相机并将结果发给 NUCLEO：

```sh
python3 maix_vision_uart_v1.py --live 60 /tmp/vision_debug
```

使用 UART2 `/dev/ttyS2`，115200、8N1。`--live` 的第三个参数是相机端保存目录；会保存 3 张带标注 PNG。绿色标出路线扫描中心，红框标出障碍候选，顶部显示偏差、置信度和阈值。下载到 Mac 后把图片放到 `figure/vision-debug/`。运行过程中 NUCLEO 的 LD2 应在有效数据到来时点亮，连续 300 ms 无有效帧后熄灭。现阶段这个阈值只用于软件联调，尚未和电机安全停机闭环。

## 当前验证结果

- 合成弯道正样例：识别到 3 行路线点，`line_valid=1`、偏移 `-187`、方向 `-2490 mdeg`；识别到的障碍候选中心 `500`、宽度 `65`（均为归一化值）。
- 合成空白负样例：`line_valid=0`、`obstacle_valid=0`、置信度 `0`。
- 2026-11-12 摘掉镜头保护盖后进行真实相机采样：读取 45 帧，保存第 0、15、30 帧到 `../../figure/vision-debug/cap_removed/`。三帧都能看到桌面和线缆，不再是保护盖未摘时的随机噪声；画面没有赛道黑线，因此 `line_valid=0`、`obstacle_valid=0`、置信度 `0` 是当前场景下的预期结果，不能据此判断弯道循线能力。
- 同次实时联调中，MaixCAM2 发出的视觉帧收到 NUCLEO 有效 ACK。用户已目视确认 NUCLEO 的 LD2 在数据到达时点亮、停止发送后约 300 ms 熄灭。
- 下一步需要让镜头看到白底上的约 20 mm 黑色测试线，再检查真实画面中的扫描点、路线中心、偏差和障碍候选。当前阈值与障碍规则仍需用测试线和实际障碍物继续校准。
