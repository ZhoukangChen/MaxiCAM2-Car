from maix import camera, image, uart
import math
import os
import sys
import time

# 中文注释：本版使用 640×480 灰度图；场地阈值和车宽仍需到赛道标定。
WIDTH = 640
HEIGHT = 480
CAMERA_FPS = 15
LINE_THRESHOLD_MIN = 32
LINE_THRESHOLD_MAX = 150
LINE_SCAN_FRACTIONS = (0.82, 0.70, 0.58)
MIN_LINE_WIDTH_PIXELS = 8
MAX_LINE_WIDTH_FRACTION = 0.22
UART_SEND_EVERY = 2


def clamp(value, low, high):
    return max(low, min(high, value))


# 中文注释：CRC-16/CCITT-FALSE，与 NUCLEO 协议固件保持一致。
def crc16_ccitt(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_uart_frame(sequence, result):
    fields = (
        1,
        sequence,
        result["line_valid"],
        result["line_dx_permille"],
        result["heading_millideg"],
        result["obstacle_valid"],
        result["obstacle_x_permille"],
        result["obstacle_w_permille"],
        result["confidence_permille"],
    )
    payload = ",".join(str(value) for value in fields).encode("ascii")
    checksum = crc16_ccitt(payload)
    return b"@" + payload + b"*" + f"{checksum:04X}".encode("ascii") + b"\r\n"


def estimate_dark_threshold(img):
    # 中文注释：用前方路面 ROI 的 Otsu 阈值分开深色区域和浅色背景，再限制阈值范围。
    width = img.width()
    height = img.height()
    y0 = int(height * 0.42)
    histogram = img.get_histogram(roi=[0, y0, width, height - y0], bins=64)
    threshold = int(histogram.get_threshold().l_value())
    if threshold < LINE_THRESHOLD_MIN or threshold > LINE_THRESHOLD_MAX:
        threshold = 80
    return threshold


def dark_runs_on_row(img, y, threshold, x_step=2):
    # 中文注释：在一条水平扫描线上合并相邻深色像素，得到可能的黑线区段。
    width = img.width()
    runs = []
    run_start = None
    run_last = None
    run_samples = 0

    for x in range(0, width, x_step):
        pixel = img.get_pixel(x, y)
        dark = bool(pixel and int(pixel[0]) <= threshold)
        if dark:
            if run_start is None:
                run_start = x
                run_samples = 0
            run_last = x
            run_samples += 1
        elif run_start is not None:
            run_width = run_last - run_start + x_step
            if run_samples >= 2 and max(MIN_LINE_WIDTH_PIXELS, int(width * 0.0125)) <= run_width <= int(width * MAX_LINE_WIDTH_FRACTION):
                runs.append((int((run_start + run_last) / 2), run_width))
            run_start = None
            run_last = None
            run_samples = 0

    if run_start is not None:
        run_width = run_last - run_start + x_step
        if run_samples >= 2 and max(MIN_LINE_WIDTH_PIXELS, int(width * 0.0125)) <= run_width <= int(width * MAX_LINE_WIDTH_FRACTION):
            runs.append((int((run_start + run_last) / 2), run_width))
    return runs


def select_run(runs, expected_x, expected_width=None):
    if not runs:
        return None

    def score(run):
        center, width = run
        width_penalty = 0 if expected_width is None else abs(width - expected_width) * 0.15
        # 中文注释：优先选择与上一行位置接近、且宽度相近的深色区段。
        return abs(center - expected_x) + width_penalty

    return min(runs, key=score)


def interpolate_route_x(y, detections):
    # 中文注释：在相邻扫描行之间插值，粗略估计物体所在高度处的路线中心。
    points = sorted(detections, key=lambda item: item[0])
    if len(points) < 2:
        return None

    for index in range(len(points) - 1):
        y0, x0, _ = points[index]
        y1, x1, _ = points[index + 1]
        if y0 <= y <= y1:
            fraction = (y - y0) / max(1, y1 - y0)
            return x0 + fraction * (x1 - x0)

    # 中文注释：物体落在扫描区外时，使用最近两行的方向作有限外推。
    if y < points[0][0]:
        y0, x0, _ = points[0]
        y1, x1, _ = points[1]
    else:
        y0, x0, _ = points[-2]
        y1, x1, _ = points[-1]
    fraction = (y - y0) / max(1, y1 - y0)
    return x0 + fraction * (x1 - x0)


def detect_dark_obstacle_candidate(img, threshold, line_detections):
    # 中文注释：只找与路线走廊重叠的深色连通区域；这是候选框，不是已标定的物理距离。
    if len(line_detections) < 2:
        return 0, 0, 0, 0, None

    width = img.width()
    height = img.height()
    minimum_area = max(100, int(width * height * 0.0008))
    blobs = img.find_blobs(
        thresholds=[[0, threshold]],
        roi=[0, 0, width, height],
        x_stride=3,
        y_stride=2,
        area_threshold=minimum_area,
        pixels_threshold=minimum_area,
        merge=False,
    )

    nearest = None
    nearest_y = -1
    near_line_width = max(item[2] for item in line_detections)
    corridor_half_width = max(int(width * 0.08), int(near_line_width * 2.5))

    for blob in blobs:
        bx, by, bw, bh = blob.x(), blob.y(), blob.w(), blob.h()
        bottom = by + bh
        area = blob.pixels()

        # 中文注释：忽略触及图像底边的连通区域，因为它通常就是循迹黑线本身。
        if bottom >= height - 3 or area < minimum_area:
            continue
        if bw > int(width * 0.60) or bh > int(height * 0.70):
            continue

        center_y = int(blob.cy())
        route_x = interpolate_route_x(center_y, line_detections)
        if route_x is None:
            continue
        intersects_corridor = (bx <= route_x + corridor_half_width and
                               bx + bw >= route_x - corridor_half_width)
        if intersects_corridor and center_y > nearest_y:
            nearest = blob
            nearest_y = center_y

    if nearest is None:
        return 0, 0, 0, 0, None

    x_permille = int(clamp(nearest.cx() * 1000 / max(1, width - 1), 0, 1000))
    width_permille = int(clamp(nearest.w() * 1000 / width, 0, 1000))
    box_area = max(1, nearest.w() * nearest.h())
    density = nearest.pixels() / box_area
    confidence = int(clamp(350 + density * 500, 0, 900))
    box = (nearest.x(), nearest.y(), nearest.w(), nearest.h(), nearest.pixels())
    return 1, x_permille, width_permille, confidence, box


def analyze_frame(img):
    width = img.width()
    height = img.height()
    threshold = estimate_dark_threshold(img)
    row_y = [int(height * fraction) for fraction in LINE_SCAN_FRACTIONS]
    detections = []
    expected_x = width / 2
    expected_width = None

    for y in row_y:
        run = select_run(dark_runs_on_row(img, y, threshold), expected_x, expected_width)
        if run is None:
            continue
        center_x, run_width = run
        if detections and abs(center_x - expected_x) > width * 0.15:
            continue
        detections.append((y, center_x, run_width))
        expected_x = center_x
        expected_width = run_width

    line_valid = 1 if len(detections) >= 2 else 0
    line_dx = 0
    heading_millideg = 0
    line_confidence = 0

    if line_valid:
        near = max(detections, key=lambda item: item[0])
        far = min(detections, key=lambda item: item[0])
        line_dx = int(clamp((near[1] - width / 2) * 2000 / width, -1000, 1000))
        heading_deg = math.degrees(math.atan2(near[1] - far[1], max(1, near[0] - far[0])))
        heading_millideg = int(clamp(round(heading_deg * 1000), -180000, 180000))

        ordered = sorted(detections, key=lambda item: item[0])
        jumps = [abs(ordered[i + 1][1] - ordered[i][1]) / width for i in range(len(ordered) - 1)]
        continuity = 1.0 if not jumps else max(0.0, 1.0 - (sum(jumps) / len(jumps)) / 0.28)
        line_confidence = int((len(detections) / len(row_y)) * 1000 * (0.75 + 0.25 * continuity))

    obstacle_valid = 0
    obstacle_x = 0
    obstacle_width = 0
    obstacle_confidence = 0
    obstacle_box = None
    if line_valid:
        (obstacle_valid, obstacle_x,
         obstacle_width, obstacle_confidence, obstacle_box) = detect_dark_obstacle_candidate(img, threshold, detections)

    if line_valid and obstacle_valid:
        confidence = min(line_confidence, obstacle_confidence)
    elif line_valid:
        confidence = line_confidence
    elif obstacle_valid:
        confidence = obstacle_confidence
    else:
        confidence = 0

    return {
        "line_valid": line_valid,
        "line_dx_permille": line_dx,
        "heading_millideg": heading_millideg,
        "obstacle_valid": obstacle_valid,
        "obstacle_x_permille": obstacle_x,
        "obstacle_w_permille": obstacle_width,
        "confidence_permille": confidence,
        "threshold": threshold,
        "line_rows": len(detections),
        "line_points": detections,
        "obstacle_box": obstacle_box,
    }


def draw_debug_overlay(rgb_img, result, frame_number):
    # 中文注释：在彩色原图上画扫描点、路线中心、障碍候选框和状态字样。
    width = rgb_img.width()
    height = rgb_img.height()
    center_x = width // 2
    rgb_img.draw_line(center_x, 0, center_x, height - 1, image.COLOR_GRAY, 1)

    points = sorted(result["line_points"], key=lambda item: item[0])
    if result["line_valid"] and points:
        for index, (y, x, run_width) in enumerate(points):
            rgb_img.draw_line(0, y, width - 1, y, image.COLOR_GRAY, 1)
            rgb_img.draw_rect(max(0, x - run_width // 2), y - 2, run_width, 5, image.COLOR_GREEN, 2)
            rgb_img.draw_circle(x, y, 6, image.COLOR_GREEN, -1)
            if index > 0:
                previous_y, previous_x, _ = points[index - 1]
                rgb_img.draw_line(previous_x, previous_y, x, y, image.COLOR_GREEN, 4)

    box = result["obstacle_box"]
    if result["obstacle_valid"] and box is not None:
        x, y, box_width, box_height, _ = box
        rgb_img.draw_rect(x, y, box_width, box_height, image.COLOR_RED, 3)
        label_y = max(54, y - 18)
        rgb_img.draw_string(x, label_y, "OBS CANDIDATE", color=image.COLOR_RED, scale=1, wrap=False)

    # 中文注释：顶部深色信息栏展示识别状态、偏差、置信度和阈值。
    rgb_img.draw_rect(0, 0, width, 50, image.COLOR_BLACK, -1)
    line_label = "LINE OK" if result["line_valid"] else "LINE LOST"
    heading_deg = result["heading_millideg"] / 1000.0
    rgb_img.draw_string(
        8, 5,
        "F{} {} dx={} head={:.1f}deg".format(
            frame_number, line_label, result["line_dx_permille"], heading_deg
        ),
        color=image.COLOR_WHITE, scale=1, wrap=False,
    )
    rgb_img.draw_string(
        8, 27,
        "OBS={} x={} w={} conf={}% th={}".format(
            result["obstacle_valid"], result["obstacle_x_permille"],
            result["obstacle_w_permille"], result["confidence_permille"] // 10,
            result["threshold"],
        ),
        color=image.COLOR_YELLOW, scale=1, wrap=False,
    )
    return rgb_img


def inspect_image(path, save_path=None):
    rgb_img = image.load(path, format=image.Format.FMT_RGB888)
    if rgb_img is None:
        raise RuntimeError("无法加载测试图像：" + path)
    gray_img = rgb_img.to_format(image.Format.FMT_GRAYSCALE)
    result = analyze_frame(gray_img)
    print("图像尺寸：", rgb_img.width(), "x", rgb_img.height())
    print("视觉结果：", result)
    if save_path:
        os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
        draw_debug_overlay(rgb_img, result, 0)
        save_err = rgb_img.save(save_path, quality=95)
        print("已保存调试帧：", save_path, save_err)


def run_live(frame_limit, save_dir=None):
    # 中文注释：读取彩色相机帧用于存图，转换出的灰度帧用于识别并发送给 NUCLEO。
    cam = camera.Camera(width=WIDTH, height=HEIGHT, format=image.Format.FMT_RGB888, fps=CAMERA_FPS)
    cam.skip_frames(8)
    serial = uart.UART("/dev/ttyS2", 115200)
    sequence = 0
    ack_buffer = b""
    save_indices = {0, frame_limit // 3, (frame_limit * 2) // 3} if save_dir else set()
    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    for frame_index in range(frame_limit):
        rgb_img = cam.read()
        if rgb_img is None:
            continue
        gray_img = rgb_img.to_format(image.Format.FMT_GRAYSCALE)
        result = analyze_frame(gray_img)

        if frame_index % UART_SEND_EVERY == 0:
            sequence = (sequence + 1) & 0xFFFF
            serial.write(build_uart_frame(sequence, result))

        # 中文注释：顺手读取 ACK，避免相机端接收缓冲区堆积。
        received = serial.read()
        if received:
            ack_buffer += received
            while b"\n" in ack_buffer:
                line, ack_buffer = ack_buffer.split(b"\n", 1)
                if line.startswith(b"!"):
                    print("NUCLEO:", line)

        if frame_index in save_indices:
            save_path = os.path.join(save_dir, "live_{:03d}.png".format(frame_index))
            draw_debug_overlay(rgb_img, result, frame_index)
            save_err = rgb_img.save(save_path, quality=95)
            print("已保存调试帧：", save_path, save_err)

        if frame_index % 5 == 0:
            print("FRAME", frame_index, "TH", result["threshold"],
                  "LINE", result["line_valid"], result["line_dx_permille"],
                  "HEAD", result["heading_millideg"],
                  "POINTS", result["line_points"],
                  "OBS", result["obstacle_valid"], result["obstacle_box"],
                  "CONF", result["confidence_permille"])
        time.sleep(0.005)


if __name__ == "__main__":
    # 中文注释：离线图像和真实相机两种模式都可以保存带标注的 PNG 调试帧。
    if len(sys.argv) >= 3 and sys.argv[1] == "--image":
        save_path = sys.argv[3] if len(sys.argv) >= 4 else None
        inspect_image(sys.argv[2], save_path)
    else:
        frame_limit = int(sys.argv[2]) if len(sys.argv) >= 3 and sys.argv[1] == "--live" else 60
        save_dir = sys.argv[3] if len(sys.argv) >= 4 and sys.argv[1] == "--live" else None
        run_live(frame_limit, save_dir)
