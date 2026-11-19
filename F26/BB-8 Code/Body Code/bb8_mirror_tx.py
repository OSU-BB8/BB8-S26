"""Best-effort BB-8 test stand sender. No effect on main controls if absent/offline."""
import json, os, socket, time
from pathlib import Path

class MirrorTX:
    def __init__(self, mode):
        self.mode = mode
        self.sock = None
        self.dest = None
        try:
            cfg = json.loads((Path(__file__).resolve().parent / 'config.json').read_text())
            if cfg.get('test_unit_host'):
                self.dest = (cfg['test_unit_host'], int(cfg.get('test_unit_port', 5015)))
                self.token = cfg['token']
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sock.setblocking(False)
        except (OSError, ValueError, KeyError, TypeError):
            pass

    def send(self, distance=0.0, angle=0.0, stop=False):
        if not self.sock:
            return
        packet = {'protocol':'bb8-mirror-v1', 'kind':'motion', 'token':self.token,
                  'mode':self.mode, 'distance_ft':float(distance),
                  'angle_deg':float(angle), 'stop':bool(stop), 'sent_at':time.time()}
        try:
            self.sock.sendto(json.dumps(packet).encode(), self.dest)
        except (OSError, ValueError, TypeError):
            pass
