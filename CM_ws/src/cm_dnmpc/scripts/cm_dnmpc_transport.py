#!/usr/bin/env python3
"""Length-framed TCP transport used by the two local controllers."""

import json
import socket
import struct
import time


class PredictionSocket:
    def __init__(self, role, bind_host, peer_host, port, timeout_s, reconnect_s):
        self.role = role
        self.bind_host = bind_host
        self.peer_host = peer_host
        self.port = int(port)
        self.timeout_s = float(timeout_s)
        self.reconnect_s = float(reconnect_s)
        self.listener = None
        self.connection = None
        self.messages = 0
        self.bytes_sent = 0
        self.bytes_received = 0

    def _create_listener(self):
        if self.listener is not None:
            return
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.bind_host, self.port))
        listener.listen(1)
        listener.settimeout(self.reconnect_s)
        self.listener = listener

    def connect_once(self):
        if self.connection is not None:
            return True
        try:
            if self.role == "server":
                self._create_listener()
                connection, _ = self.listener.accept()
            else:
                connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                connection.settimeout(self.reconnect_s)
                connection.connect((self.peer_host, self.port))
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            connection.settimeout(self.timeout_s)
            self.connection = connection
            return True
        except OSError:
            return False

    def wait_connected(self, stop):
        while not stop():
            if self.connect_once():
                return True
            time.sleep(self.reconnect_s)
        return False

    def _receive_exact(self, count):
        data = bytearray()
        while len(data) < count:
            block = self.connection.recv(count - len(data))
            if not block:
                raise ConnectionError("peer closed")
            data.extend(block)
        self.bytes_received += len(data)
        return bytes(data)

    def exchange(self, message):
        if self.connection is None and not self.connect_once():
            return None
        try:
            payload = json.dumps(
                message, separators=(",", ":")).encode("utf-8")
            frame = struct.pack("!I", len(payload)) + payload
            self.connection.sendall(frame)
            self.bytes_sent += len(frame)
            size = struct.unpack("!I", self._receive_exact(4))[0]
            if size <= 0 or size > 1000000:
                raise ValueError("invalid frame")
            reply = json.loads(self._receive_exact(size).decode("utf-8"))
            self.messages += 1
            return reply
        except (OSError, ValueError, ConnectionError, json.JSONDecodeError):
            self.close_connection()
            return None

    def close_connection(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except OSError:
                pass
        self.connection = None

    def close(self):
        self.close_connection()
        if self.listener is not None:
            try:
                self.listener.close()
            except OSError:
                pass
        self.listener = None


class LocalDuplexPredictionSocket:
    """Two unidirectional TCP links for two controllers on one computer.

    Each controller listens on its own receive port and connects its outgoing
    link to the other controller's receive port.  There is deliberately no
    left-server/right-client identity in this simulation transport.
    """

    def __init__(self, bind_host, peer_host, receive_port, send_port,
                 timeout_s, reconnect_s):
        self.bind_host = bind_host
        self.peer_host = peer_host
        self.receive_port = int(receive_port)
        self.send_port = int(send_port)
        self.timeout_s = float(timeout_s)
        self.reconnect_s = float(reconnect_s)
        self.listener = None
        self.incoming = None
        self.outgoing = None
        self.messages = 0
        self.bytes_sent = 0
        self.bytes_received = 0

    def _create_listener(self):
        if self.listener is not None:
            return
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.bind_host, self.receive_port))
        listener.listen(1)
        listener.settimeout(self.reconnect_s)
        self.listener = listener

    def _connect_outgoing_once(self):
        if self.outgoing is not None:
            return
        connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            connection.settimeout(self.reconnect_s)
            connection.connect((self.peer_host, self.send_port))
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            connection.settimeout(self.timeout_s)
            self.outgoing = connection
        except OSError:
            connection.close()

    def _accept_incoming_once(self):
        if self.incoming is not None:
            return
        try:
            connection, _ = self.listener.accept()
            connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            connection.settimeout(self.timeout_s)
            self.incoming = connection
        except OSError:
            return

    def connect_once(self):
        self._create_listener()
        self._connect_outgoing_once()
        self._accept_incoming_once()
        return self.incoming is not None and self.outgoing is not None

    def wait_connected(self, stop):
        while not stop():
            if self.connect_once():
                return True
            time.sleep(self.reconnect_s)
        return False

    def set_timeout(self, timeout_s):
        self.timeout_s = float(timeout_s)
        for connection in (self.incoming, self.outgoing):
            if connection is not None:
                connection.settimeout(self.timeout_s)

    def _receive_exact(self, count):
        data = bytearray()
        while len(data) < count:
            block = self.incoming.recv(count - len(data))
            if not block:
                raise ConnectionError("peer closed")
            data.extend(block)
        self.bytes_received += len(data)
        return bytes(data)

    def exchange(self, message):
        if ((self.incoming is None or self.outgoing is None) and
                not self.connect_once()):
            return None
        try:
            payload = json.dumps(
                message, separators=(",", ":")).encode("utf-8")
            frame = struct.pack("!I", len(payload)) + payload
            self.outgoing.sendall(frame)
            self.bytes_sent += len(frame)
            size = struct.unpack("!I", self._receive_exact(4))[0]
            if size <= 0 or size > 1000000:
                raise ValueError("invalid frame")
            reply = json.loads(self._receive_exact(size).decode("utf-8"))
            self.messages += 1
            return reply
        except (OSError, ValueError, ConnectionError, json.JSONDecodeError):
            self.close_connections()
            return None

    def close_connections(self):
        for connection in (self.incoming, self.outgoing):
            if connection is not None:
                try:
                    connection.close()
                except OSError:
                    pass
        self.incoming = None
        self.outgoing = None

    def close(self):
        self.close_connections()
        if self.listener is not None:
            try:
                self.listener.close()
            except OSError:
                pass
        self.listener = None
