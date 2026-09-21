#!/usr/bin/env python3
"""Connect to the harness monitor bridge and report what the firmware is saying.

A minimal WebSocket client, written by hand so the check has no dependencies:
it performs the upgrade handshake, reads binary frames, decodes the firmware
packets in them and prints how many arrived and which variables they carried.
Used by `just check-monitor` to prove the bridge works end to end.
"""

import argparse
import base64
import os
import socket
import struct
import sys
import time

HEADER = 0x42
TAIL = 0x7F
ESCAPE = 0x7D
MAP_REQUEST = 0x02
MAP_RESPONSE = 0x03
VARIABLE = 0x04


def handshake(sock: socket.socket, host: str, port: int) -> None:
    """Perform the WebSocket upgrade and consume the response headers."""
    key = base64.b64encode(os.urandom(16)).decode()
    request = (
        f"GET / HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
        f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
    )
    sock.sendall(request.encode())

    response = b""
    while b"\r\n\r\n" not in response:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("the server closed during the handshake")
        response += chunk

    if b"101" not in response.split(b"\r\n", 1)[0]:
        raise ConnectionError(f"the server refused the upgrade: {response.split(b'\r\n', 1)[0]!r}")


def read_exactly(sock: socket.socket, count: int) -> bytes:
    """Read exactly count bytes, or raise if the connection ends first."""
    data = b""
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise ConnectionError("the server closed mid frame")
        data += chunk
    return data


def read_frame(sock: socket.socket) -> bytes:
    """Read one unmasked server frame and return its payload."""
    first, second = read_exactly(sock, 2)
    length = second & 0x7F

    if length == 126:
        length = struct.unpack("!H", read_exactly(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", read_exactly(sock, 8))[0]

    payload = read_exactly(sock, length) if length else b""
    return payload if (first & 0x0F) == 0x02 else b""


def serialize(message_type: int, identifier: int = 0, payload: bytes = b"\x00") -> bytes:
    """Build one firmware packet, exactly as comm::Packet::serialize does.

    Fields are big endian, an escaped byte is kept as is after the escape byte
    rather than xored, and the checksum sums everything but the header.
    """
    escaped = bytearray()
    for byte in payload:
        if byte in (HEADER, TAIL, ESCAPE):
            escaped.append(ESCAPE)
        escaped.append(byte)

    body = bytes([HEADER, message_type, identifier >> 8, identifier & 0xFF, len(payload) >> 8, len(payload) & 0xFF])
    body += bytes(escaped)
    checksum = sum(body[1:]) & 0xFF
    if checksum in (HEADER, TAIL, ESCAPE):
        checksum += 1
    return body + bytes([checksum, TAIL])


def send_frame(sock: socket.socket, payload: bytes) -> None:
    """Send one masked binary frame, as a client must."""
    mask = os.urandom(4)
    masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
    header = bytes([0x82])

    if len(payload) < 126:
        header += bytes([0x80 | len(payload)])
    elif len(payload) < (1 << 16):
        header += bytes([0x80 | 126]) + struct.pack("!H", len(payload))
    else:
        header += bytes([0x80 | 127]) + struct.pack("!Q", len(payload))

    sock.sendall(header + mask + masked)


def packets(stream: bytearray) -> list[tuple[int, int]]:
    """Pull every complete packet out of the stream, returning type and id."""
    found = []
    index = 0
    consumed = 0

    while index < len(stream):
        if stream[index] != HEADER or index + 6 > len(stream):
            index += 1
            continue

        end = index + 6
        while end < len(stream):
            if stream[end] == ESCAPE:
                end += 2
                continue
            if stream[end] == TAIL:
                break
            end += 1

        if end >= len(stream):
            break

        found.append((stream[index + 1], (stream[index + 2] << 8) | stream[index + 3]))
        index = end + 1
        consumed = index

    del stream[:consumed]
    return found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--listen-seconds", type=float, default=3.0)
    parser.add_argument("--expect-packets", type=int, default=1)
    parser.add_argument(
        "--request-map", action="store_true", help="ask the firmware for its variable map, as the monitor does"
    )
    arguments = parser.parse_args()

    deadline = time.monotonic() + arguments.connect_timeout
    sock = None

    while time.monotonic() < deadline:
        try:
            sock = socket.create_connection((arguments.host, arguments.port), timeout=1.0)
            break
        except OSError:
            time.sleep(0.05)

    if sock is None:
        print(f"FAIL could not reach ws://{arguments.host}:{arguments.port}")
        return 1

    with sock:
        handshake(sock, arguments.host, arguments.port)
        sock.settimeout(arguments.listen_seconds)

        if arguments.request_map:
            send_frame(sock, serialize(MAP_REQUEST))

        stream = bytearray()
        seen = []
        stop = time.monotonic() + arguments.listen_seconds

        while time.monotonic() < stop:
            try:
                stream += read_frame(sock)
            except (TimeoutError, socket.timeout, ConnectionError):
                break
            seen += packets(stream)

        kinds = {}
        for kind, _ in seen:
            kinds[kind] = kinds.get(kind, 0) + 1

        print(f"packets={len(seen)} by_type={kinds}")

        if len(seen) < arguments.expect_packets:
            print(f"FAIL expected at least {arguments.expect_packets} packet(s)")
            return 1

        if arguments.request_map and MAP_RESPONSE not in kinds:
            print("FAIL the firmware never answered the variable map request")
            return 1

    print("ok   the monitor bridge is talking")
    return 0


if __name__ == "__main__":
    sys.exit(main())
