"""Zero-dependency stdlib RFC 6455 Foxglove WebSocket v1 read-only subscriber."""
import base64
import hashlib
import json
import os
import socket
import ssl
import struct
import time
from urllib.parse import urlparse

_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
_MAX_FRAME_BYTES = 32 * 1024 * 1024 + 64


class FoxgloveError(RuntimeError):
    """Raised when the Foxglove WebSocket link fails or closes unexpectedly."""


class _BufferedSocket:
    def __init__(self, sock, initial=b""):
        self._sock = sock
        self._buf = bytearray(initial)

    def read_exact(self, size, stop_event, deadline=None):
        while len(self._buf) < size:
            if stop_event is not None and stop_event.is_set():
                return None
            if deadline is not None and time.monotonic() >= deadline:
                raise TimeoutError("Timed out reading from Foxglove WebSocket")
            try:
                chunk = self._sock.recv(max(4096, min(65536, size - len(self._buf))))
            except socket.timeout:
                continue
            if not chunk:
                raise FoxgloveError("Foxglove WebSocket closed by peer")
            self._buf.extend(chunk)
        out = bytes(self._buf[:size])
        del self._buf[:size]
        return out


def _mask_payload(payload, mask_key):
    data = bytearray(payload)
    for idx in range(len(data)):
        data[idx] ^= mask_key[idx & 3]
    return bytes(data)


def _send_frame(sock, opcode, payload=b""):
    length = len(payload)
    header = bytearray([0x80 | (opcode & 0x0F)])
    if length < 126:
        header.append(0x80 | length)
    elif length < 65536:
        header.append(0x80 | 126)
        header.extend(struct.pack("!H", length))
    else:
        header.append(0x80 | 127)
        header.extend(struct.pack("!Q", length))
    mask = os.urandom(4)
    header.extend(mask)
    sock.sendall(bytes(header) + _mask_payload(payload, mask))


def _recv_message(reader, sock, stop_event, timeout_s):
    """Receive one complete RFC 6455 message (handling fragmentation and ping/pong)."""
    deadline = None if timeout_s is None else time.monotonic() + timeout_s
    fragments = []
    message_opcode = None

    while True:
        first_two = reader.read_exact(2, stop_event, deadline=deadline)
        if first_two is None:
            return None, None
        b0, b1 = first_two[0], first_two[1]
        fin = (b0 & 0x80) != 0
        opcode = b0 & 0x0F
        masked = (b1 & 0x80) != 0
        length = b1 & 0x7F

        if length == 126:
            ext = reader.read_exact(2, stop_event, deadline=deadline)
            if ext is None:
                return None, None
            (length,) = struct.unpack("!H", ext)
        elif length == 127:
            ext = reader.read_exact(8, stop_event, deadline=deadline)
            if ext is None:
                return None, None
            (length,) = struct.unpack("!Q", ext)

        if length > _MAX_FRAME_BYTES:
            raise FoxgloveError(f"Oversized WebSocket frame: {length} bytes")

        mask_key = None
        if masked:
            mask_key = reader.read_exact(4, stop_event, deadline=deadline)
            if mask_key is None:
                return None, None

        payload = b""
        if length > 0:
            payload = reader.read_exact(length, stop_event, deadline=deadline)
            if payload is None:
                return None, None
            if masked:
                payload = _mask_payload(payload, mask_key)

        if opcode == 0x8:  # CLOSE
            try:
                _send_frame(sock, 0x8, payload[:2])
            except OSError:
                pass
            raise FoxgloveError("Foxglove WebSocket closed cleanly")
        if opcode == 0x9:  # PING
            _send_frame(sock, 0xA, payload)
            continue
        if opcode == 0xA:  # PONG
            continue

        if opcode in (0x1, 0x2):
            if message_opcode is not None:
                raise FoxgloveError("Unexpected new data frame before continuation finished")
            message_opcode = opcode
            fragments = [payload]
        elif opcode == 0x0:
            if message_opcode is None:
                raise FoxgloveError("Unexpected continuation frame")
            fragments.append(payload)
        else:
            raise FoxgloveError(f"Unsupported WebSocket opcode: {opcode}")

        if fin:
            full = b"".join(fragments)
            return message_opcode, full


def _connect_ws(url, connect_timeout=5.0, io_timeout=0.5):
    parsed = urlparse(url)
    if parsed.scheme not in ("ws", "wss"):
        raise ValueError(f"Unsupported WebSocket URL scheme: {url}")
    host = parsed.hostname or "127.0.0.1"
    port = parsed.port or (443 if parsed.scheme == "wss" else 80)
    path = parsed.path or "/"
    if parsed.query:
        path = f"{path}?{parsed.query}"

    raw_sock = socket.create_connection((host, port), timeout=connect_timeout)
    try:
        if parsed.scheme == "wss":
            ctx = ssl.create_default_context()
            sock = ctx.wrap_socket(raw_sock, server_hostname=host)
        else:
            sock = raw_sock
        sock.settimeout(connect_timeout)

        key = base64.b64encode(os.urandom(16)).decode("ascii")
        host_header = f"{host}:{port}" if parsed.port else host
        request = (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {host_header}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "Sec-WebSocket-Protocol: foxglove.websocket.v1\r\n"
            "\r\n"
        ).encode("ascii")
        sock.sendall(request)

        buf = bytearray()
        while b"\r\n\r\n" not in buf:
            chunk = sock.recv(4096)
            if not chunk:
                raise FoxgloveError("Connection closed during WebSocket handshake")
            buf.extend(chunk)
            if len(buf) > 65536:
                raise FoxgloveError("Oversized WebSocket handshake header")

        header_end = buf.index(b"\r\n\r\n")
        header_text = bytes(buf[:header_end]).decode("iso-8859-1")
        remainder = bytes(buf[header_end + 4:])

        lines = header_text.split("\r\n")
        if not lines or " 101 " not in lines[0]:
            raise FoxgloveError(f"WebSocket handshake rejected: {lines[0] if lines else 'empty'}")

        headers = {}
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()

        expected_accept = base64.b64encode(
            hashlib.sha1((key + _WS_GUID).encode("ascii")).digest()
        ).decode("ascii")
        if headers.get("sec-websocket-accept") != expected_accept:
            raise FoxgloveError("Invalid Sec-WebSocket-Accept from Foxglove server")

        sock.settimeout(io_timeout)
        return sock, _BufferedSocket(sock, remainder)
    except BaseException:
        raw_sock.close()
        raise


def stream_pointcloud2(
    url,
    topic,
    stop_event,
    *,
    connect_timeout=5.0,
    first_frame_timeout=10.0,
    frame_timeout=3.0,
):
    """Read-only generator yielding (log_time_ns, receive_wall_ms, receive_mono_ms, payload_bytes)."""
    sock, reader = _connect_ws(url, connect_timeout=connect_timeout)
    channel_id = None
    subscription_id = 1
    got_first_frame = False
    try:
        while not stop_event.is_set():
            timeout = frame_timeout if got_first_frame else first_frame_timeout
            opcode, data = _recv_message(reader, sock, stop_event, timeout_s=timeout)
            if opcode is None:
                return
            recv_wall_ms = round(time.time() * 1000.0, 3)
            recv_mono_ms = round(time.monotonic() * 1000.0, 3)

            if opcode == 0x1:
                event = json.loads(data.decode("utf-8"))
                op = event.get("op")
                if op == "advertise" and channel_id is None:
                    for ch in event.get("channels") or ():
                        if ch.get("topic") == topic:
                            if ch.get("encoding") != "ros1" or ch.get("schemaName") != "sensor_msgs/PointCloud2":
                                raise FoxgloveError(
                                    f"Unexpected schema for {topic}: {ch.get('encoding')}/{ch.get('schemaName')}"
                                )
                            channel_id = ch["id"]
                            sub_msg = json.dumps({
                                "op": "subscribe",
                                "subscriptions": [{"id": subscription_id, "channelId": channel_id}],
                            }).encode("utf-8")
                            _send_frame(sock, 0x1, sub_msg)
                            break
                elif op == "unadvertise" and channel_id in (event.get("channelIds") or ()):
                    raise FoxgloveError(f"Topic {topic} unadvertised by Foxglove bridge")
            elif opcode == 0x2 and len(data) >= 13 and data[0] == 1:
                sub_id, log_time_ns = struct.unpack_from("<IQ", data, 1)
                if sub_id == subscription_id:
                    got_first_frame = True
                    yield log_time_ns, recv_wall_ms, recv_mono_ms, data[13:]
    finally:
        try:
            _send_frame(sock, 0x8, b"")
        except OSError:
            pass
        sock.close()
