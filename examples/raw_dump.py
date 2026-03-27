"""Capture raw serial bytes and analyze frame structure for CRC detection."""

import sys
import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 921_600
ROWS = 20
COLS = 8
PAYLOAD_SIZE = ROWS * COLS * 2  # 384 bytes
HEADER_A = b"\xaa\xaa"
HEADER_B = b"\xbb\xbb"

CAPTURE_SECONDS = 2

ser = serial.Serial(PORT, BAUD, timeout=0.1)

# Send init command (same as hub.py)
init_cmd = bytes.fromhex("01 06 60 10 00 01 57 CF")
ser.write(init_cmd)
ser.flush()
time.sleep(0.1)
ser.reset_input_buffer()

print(f"Capturing {CAPTURE_SECONDS}s of raw data...")
raw = bytearray()
t0 = time.time()
while time.time() - t0 < CAPTURE_SECONDS:
    chunk = ser.read(4096)
    if chunk:
        raw += chunk

ser.close()
print(f"Captured {len(raw)} bytes\n")

# Find all header positions
headers = []
for i in range(len(raw) - 1):
    if raw[i:i+2] == HEADER_A:
        headers.append((i, 'A'))
    elif raw[i:i+2] == HEADER_B:
        headers.append((i, 'B'))

# Filter out overlapping matches (e.g. 0xAA 0xAA 0xAA would match twice)
filtered = []
for pos, tag in headers:
    if filtered and pos < filtered[-1][0] + 2:
        continue
    filtered.append((pos, tag))

print(f"Found {len(filtered)} header candidates\n")

# Analyze distances between consecutive headers
print("=== Header-to-header distances (first 30) ===")
distances = []
for i in range(min(30, len(filtered) - 1)):
    pos1, tag1 = filtered[i]
    pos2, tag2 = filtered[i + 1]
    dist = pos2 - pos1
    distances.append(dist)
    print(f"  {tag1}@{pos1:>6} -> {tag2}@{pos2:>6}  distance={dist}  (payload would be {dist - 2})")

if distances:
    from collections import Counter
    print(f"\n=== Distance frequency ===")
    for dist, count in Counter(distances).most_common():
        extra = dist - 2 - PAYLOAD_SIZE
        print(f"  distance={dist}  count={count}  (expected {PAYLOAD_SIZE + 2}, extra={extra} bytes)")

# Show raw bytes around first few headers
print(f"\n=== Bytes around first 5 headers ===")
print(f"Expected payload = {PAYLOAD_SIZE} bytes ({ROWS}x{COLS}x2)")
print(f"Expected frame (no CRC) = {PAYLOAD_SIZE + 2} bytes")

for i in range(min(5, len(filtered))):
    pos, tag = filtered[i]
    # Show header + a few bytes before and after the expected frame end
    end = pos + 2 + PAYLOAD_SIZE
    
    # Bytes before header
    pre_start = max(0, pos - 4)
    pre = raw[pre_start:pos]
    
    # Bytes right after expected payload end
    post = raw[end:min(end + 8, len(raw))]
    
    print(f"\n  Header {tag} @ offset {pos}:")
    print(f"    Pre-header bytes:  {pre.hex(' ')}")
    print(f"    Header:            {raw[pos:pos+2].hex(' ')}")
    print(f"    First 8 payload:   {raw[pos+2:pos+10].hex(' ')}")
    print(f"    Last 8 payload:    {raw[end-8:end].hex(' ')}")
    print(f"    Post-payload (8B): {post.hex(' ')}")
    
    # Check if post bytes are a header
    if len(post) >= 2:
        if post[:2] == HEADER_A:
            print(f"    -> Next header (A) starts IMMEDIATELY after payload. No CRC.")
        elif post[:2] == HEADER_B:
            print(f"    -> Next header (B) starts IMMEDIATELY after payload. No CRC.")
        else:
            print(f"    -> Next bytes are NOT a header. Possible CRC or padding.")
