"""Modbus RTU helpers for reading and writing sensor settings.

The sensors keep streaming frames while they answer, so replies are searched
for inside the incoming byte stream and accepted only when their CRC matches.
"""

from __future__ import annotations

from typing import List

FUNC_READ = 0x03
FUNC_WRITE = 0x06

#: Output data type register (SensX25): 0 = ADC counts, 1 = pressure.
REG_OUTPUT_MODE = 0x6005


def crc16(data: bytes) -> bytes:
    """Modbus CRC-16 (poly 0xA001, init 0xFFFF), low byte first."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, crc >> 8])


def request(address: int, function: int, register: int, value: int) -> bytes:
    """Build an 8-byte request: read (``value`` = register count) or write."""
    body = bytes([
        address, function,
        register >> 8, register & 0xFF,
        value >> 8, value & 0xFF,
    ])
    return body + crc16(body)


def find_read_replies(rx: bytes, address: int) -> List[int]:
    """Return the value of every valid single-register read reply in *rx*.

    A reply is ``[address][0x03][0x02][value hi][value lo][crc lo][crc hi]``.
    Behind a hub every sensor answers, so several replies can be present.
    """
    head = bytes([address, FUNC_READ, 0x02])
    values: List[int] = []
    i = rx.find(head)
    while 0 <= i <= len(rx) - 7:
        if crc16(rx[i : i + 5]) == rx[i + 5 : i + 7]:
            values.append((rx[i + 3] << 8) | rx[i + 4])
        i = rx.find(head, i + 1)
    return values
