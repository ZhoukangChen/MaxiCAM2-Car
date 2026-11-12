from maix import uart
import time

# 中文注释：CRC-16/CCITT-FALSE，初值 0xFFFF，多项式 0x1021。
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# 中文注释：字段顺序为版本、序号、路线有效、横向偏差、航向偏差、障碍物有效、障碍中心、障碍宽度、置信度。
def build_frame(fields) -> bytes:
    payload = ",".join(str(value) for value in fields).encode("ascii")
    checksum = crc16_ccitt(payload)
    return b"@" + payload + b"*" + f"{checksum:04X}".encode("ascii") + b"\r\n"


def decode_response(data: bytes):
    for line in data.splitlines():
        if not line.startswith(b"!") or b"*" not in line:
            continue
        payload, checksum_text = line[1:].rsplit(b"*", 1)
        try:
            checksum = int(checksum_text, 16)
            fields = payload.decode("ascii").split(",")
        except (ValueError, UnicodeDecodeError):
            continue
        if crc16_ccitt(payload) == checksum:
            return fields
    return None


serial = uart.UART("/dev/ttyS2", 115200)
receive_buffer = b""


def wait_for_sequence(sequence, max_polls=150):
    # 中文注释：轮询串口并缓存碎片，直到拿到指定序号的完整 ACK 或超时。
    global receive_buffer
    for _ in range(max_polls):
        chunk = serial.read()
        if chunk:
            receive_buffer += chunk
        while b"\n" in receive_buffer:
            line, receive_buffer = receive_buffer.split(b"\n", 1)
            fields = decode_response(line)
            if fields and len(fields) >= 3 and fields[1] == str(sequence):
                return fields
        time.sleep(0.01)
    return None


samples = [
    (1, 1, -300, -12000, 0, 0, 0, 0),
    (1, 1, -100, -4000, 0, 0, 0, 0),
    (1, 1, 0, 0, 1, 720, 160, 860),
    (1, 1, 250, 18000, 1, 640, 220, 920),
    (1, 0, 0, 0, 1, 500, 350, 700),
]

passed = 0
for sequence, values in enumerate(samples, start=1):
    fields = (values[0], sequence) + values[1:]
    frame = build_frame(fields)
    serial.write(frame)
    print("发送模拟帧：", frame)
    response = wait_for_sequence(sequence)
    if response is None:
        print("等待 ACK 超时，序号：", sequence)
        continue
    expected = ["1"] + [str(value) for value in fields[1:]]
    expected.insert(2, "OK")
    if response == expected:
        print("ACK 与模拟数据一致：", response)
        passed += 1
    else:
        print("ACK 字段不一致：", response, "预期：", expected)

# 中文注释：故意改坏 CRC，确认 NUCLEO 会拒绝损坏的数据帧。
bad_frame = bytearray(build_frame((1, 999, 1, 0, 0, 0, 0, 0, 0)))
bad_frame[-4] = ord("0") if bad_frame[-4] != ord("0") else ord("1")
serial.write(bytes(bad_frame))
print("发送 CRC 错误帧：", bytes(bad_frame))
bad_response = wait_for_sequence(999)
print("损坏帧响应：", bad_response)

print("有效模拟帧通过：", passed, "/", len(samples))
if passed != len(samples) or bad_response is None or len(bad_response) < 4 or bad_response[2] != "ERR" or bad_response[3] != "2":
    raise RuntimeError("协议联调未完全通过")
print("模拟数据与 CRC 拒收测试通过。")
