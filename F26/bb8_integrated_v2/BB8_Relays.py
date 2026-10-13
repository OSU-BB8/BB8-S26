"""Relay requests to the Body supervisor. GPIO20 is owned ONLY by bb8_body.py."""
import socket,time
SOCKET='/tmp/bb8-relay.sock'
class BB8Relays:
    def __init__(self):
        self.enabled=False
    def _request(self,cmd):
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
            s.settimeout(1.0)
            s.connect(SOCKET)
            s.sendall(cmd.encode())
            response=s.recv(64).decode().strip()
        if response!='OK':raise RuntimeError('Supervisor denied relay '+cmd)
    def enable(self):
        if not self.enabled:
            self._request('ON')
            self.enabled=True
            time.sleep(.15)
    def disable(self):
        self._request('OFF')
        self.enabled=False
    def close(self):
        self.disable()
