#!/usr/bin/env python3
"""Length-framed TCP transport for offline distributed trajectory optimization."""

import json
import socket
import struct
import time


class FramedSocket:
    def __init__(self, role, host, port, timeout=15.0):
        if role not in ("server", "client"):
            raise ValueError("role must be server or client")
        self.role = role
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self.socket = None
        self.messages_sent = 0
        self.messages_received = 0
        self.bytes_sent = 0
        self.bytes_received = 0

    def connect(self):
        if self.role == "server":
            listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((self.host, self.port))
            listener.listen(1)
            listener.settimeout(self.timeout)
            self.socket, _ = listener.accept()
            listener.close()
        else:
            deadline = time.monotonic() + self.timeout
            last_error = None
            while time.monotonic() < deadline:
                candidate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    candidate.connect((self.host, self.port))
                    self.socket = candidate
                    break
                except OSError as exc:
                    last_error = exc
                    candidate.close()
                    time.sleep(0.05)
            if self.socket is None:
                raise RuntimeError("trajectory optimizer socket connect failed: {}".format(last_error))
        self.socket.settimeout(self.timeout)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def close(self):
        if self.socket is not None:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.socket.close()
            self.socket = None

    def _receive_exact(self, count):
        chunks = []
        remaining = count
        while remaining:
            chunk = self.socket.recv(remaining)
            if not chunk:
                raise EOFError("trajectory optimizer peer closed the socket")
            chunks.append(chunk)
            remaining -= len(chunk)
        data = b"".join(chunks)
        self.bytes_received += len(data)
        return data

    def send(self, payload):
        encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
        frame = struct.pack("!I", len(encoded)) + encoded
        self.socket.sendall(frame)
        self.messages_sent += 1
        self.bytes_sent += len(frame)

    def receive(self):
        length = struct.unpack("!I", self._receive_exact(4))[0]
        if length <= 0 or length > 64 * 1024 * 1024:
            raise ValueError("invalid trajectory optimizer frame length {}".format(length))
        payload = json.loads(self._receive_exact(length).decode("utf-8"))
        self.messages_received += 1
        return payload

    def exchange(self, payload):
        self.send(payload)
        return self.receive()

    def statistics(self):
        return {
            "messages_sent": self.messages_sent,
            "messages_received": self.messages_received,
            "bytes_sent": self.bytes_sent,
            "bytes_received": self.bytes_received,
        }

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
