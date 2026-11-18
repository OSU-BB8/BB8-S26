#!/usr/bin/env python3
"""Own GPIO20/GPIO21; serialize mode changes while UDP reception stays responsive."""
import json
import os
from pathlib import Path
import queue
import signal
import socket
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parent
SCRIPTS = {
    'PS5': 'PS5_Controller.py', 'BEACON': 'Beacon_Target_Control.py',
    'APRILTAG': 'Beacon_Target_Control.py', 'DEMO': 'Movement_Demo.py',
}


class BodySupervisor:
    def __init__(self, config, root=ROOT, relay_socket_path='/tmp/bb8-relay.sock'):
        self.config = config
        self.root = Path(root)
        self.relay_socket_path = relay_socket_path
        self.lock = threading.RLock()
        self.commands = queue.Queue()
        self.shutdown = threading.Event()
        self.estop_started = threading.Event()
        self.head_estop_ack = threading.Event()
        self.generation = 0
        self.state = 'IDLE'
        self.mode = None
        self.proc = None
        self.last_remote = time.monotonic()
        self.last_head = None
        self.relay_device = None
        self.teensy_enable = None
        self.sock = None
        self.relay_socket = None
        self.workers = []
        self.mirror_socket = None
        self.mirror_sequence = 0

    def outputs_off(self):
        # Caller holds lock, preventing a concurrent relay ON request.
        if self.teensy_enable is not None:
            self.teensy_enable.off()
        if self.relay_device is not None:
            self.relay_device.off()

    def tx(self, command, selected=None):
        if self.sock is None:
            return
        packet = {'token': self.config['token'], 'command': command, 'mode': selected}
        try:
            self.sock.sendto(json.dumps(packet).encode(),
                             (self.config['head_host'], self.config['head_port']))
        except OSError as exc:
            print('[HEAD] Send error:', exc, flush=True)

    def mirror_status(self):
        """Optional, best-effort telemetry; never affects primary control."""
        if self.mirror_socket is None:
            return
        with self.lock:
            running = (self.state == 'RUNNING' and self.proc is not None
                       and self.proc.poll() is None and not self.estop_started.is_set()
                       and not self.shutdown.is_set())
            self.mirror_sequence += 1
            packet = {'protocol': 'bb8-mirror-v1', 'kind': 'status',
                      'token': self.config['token'], 'seq': self.mirror_sequence,
                      'state': self.state, 'mode': self.mode,
                      'enabled': running}
        try:
            self.mirror_socket.sendto(json.dumps(packet).encode(),
                                      (self.config['test_unit_host'],
                                       int(self.config.get('test_unit_port', 5015))))
        except OSError as exc:
            print('[MIRROR] Send error:', exc, flush=True)

    def mirror_loop(self):
        while not self.shutdown.is_set():
            self.mirror_status()
            self.shutdown.wait(.2)

    def request_start(self, selected):
        if selected not in SCRIPTS:
            return
        with self.lock:
            if self.estop_started.is_set() or self.shutdown.is_set():
                return
            self.last_remote = time.monotonic()
            if self.mode == selected and (self.state == 'STARTING' or
                    (self.state == 'RUNNING' and self.proc is not None and
                     self.proc.poll() is None)):
                return
            self.generation += 1
            self.outputs_off()
            self.state, self.mode = 'STARTING', selected
            self.commands.put(('START', selected, self.generation))

    def request_stop(self, emergency=False):
        with self.lock:
            if self.estop_started.is_set():
                return
            if emergency:
                self.estop_started.set()
                self.head_estop_ack.clear()
            self.generation += 1  # Cancel queued/in-progress starts immediately.
            self.state = 'ESTOP' if emergency else 'STOPPING'
            self.outputs_off()
            self.commands.put(('ESTOP' if emergency else 'STOP', None, self.generation))

    def request_shutdown(self, *_):
        self.shutdown.set()
        with self.lock:
            self.generation += 1
            self.outputs_off()

    def active(self, generation):
        return (generation == self.generation and not self.estop_started.is_set()
                and not self.shutdown.is_set())

    def stop_child(self):
        # Only the main thread creates/reaps children. Waiting must not hold lock.
        with self.lock:
            child = self.proc
        if child is not None:
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    child.wait(timeout=1.5)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(child.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    child.wait(timeout=1)
            with self.lock:
                if self.proc is child:
                    self.proc = None

    def stop(self, generation=None):
        with self.lock:
            self.outputs_off()
        self.stop_child()
        for _ in range(3):
            self.tx('STOP')
        with self.lock:
            if not self.estop_started.is_set() and (generation is None or
                                                    generation == self.generation):
                self.state, self.mode = 'IDLE', None
        print('[BODY] Child stopped / outputs OFF', flush=True)

    def start(self, selected, generation):
        if not self.active(generation):
            return
        self.stop_child()
        if not self.active(generation):
            return
        if self.config['relay_active_high'] is None:
            print('[BLOCKED] Verify relay_active_high before starting', flush=True)
            self.stop(generation)
            return
        script = self.root / SCRIPTS[selected]
        if not script.is_file():
            print('[BLOCKED] Missing mode script:', script, flush=True)
            self.stop(generation)
            return
        for _ in range(3):
            self.tx('STOP')
        self.tx('START', selected)
        if selected in ('BEACON', 'APRILTAG'):
            deadline = time.monotonic() + 5
            while self.active(generation):
                with self.lock:
                    ready = self.last_head is not None and time.monotonic() - self.last_head <= 1.5
                if ready:
                    break
                if time.monotonic() >= deadline:
                    print('[BLOCKED] Head heartbeat missing', flush=True)
                    self.stop(generation)
                    return
                self.shutdown.wait(.05)
        # Cancellation and enable share the same lock as STOP/ESTOP.
        with self.lock:
            if not self.active(generation):
                return
            if self.teensy_enable is None:
                raise RuntimeError('GPIO21 enable is not initialized')
            self.proc = subprocess.Popen(
                [sys.executable, '-u', str(script)], cwd=self.root,
                start_new_session=True)
            # A signal handler can run on this thread during Popen.
            if not self.active(generation):
                return
            self.state, self.mode = 'RUNNING', selected
            self.teensy_enable.on()
            if not self.active(generation):
                self.outputs_off()
        print('[BODY] RUNNING', selected, flush=True)

    def request_poweroff(self):
        try:
            result = subprocess.run(['sudo', '-n', '/usr/bin/systemctl', 'poweroff'],
                                    timeout=5, capture_output=True, text=True)
            if result.returncode:
                print('[SHUTDOWN FAILED]', result.stderr.strip(), flush=True)
        except (OSError, subprocess.SubprocessError) as exc:
            print('[SHUTDOWN FAILED]', exc, flush=True)

    def estop(self):
        # Main thread waits for the head and poweroff; receiver stays alive.
        with self.lock:
            self.outputs_off()
        self.stop_child()
        with self.lock:
            self.mode = None
        print('[BODY] ESTOP latched: child stopped, outputs OFF', flush=True)
        deadline = time.monotonic() + 4
        while not self.shutdown.is_set() and time.monotonic() < deadline:
            self.tx('ESTOP')
            if self.head_estop_ack.wait(.2):
                break
        print('[BODY] Head ESTOP ACK:', self.head_estop_ack.is_set(), flush=True)
        if self.config['allow_shutdown'] and not self.shutdown.is_set():
            self.request_poweroff()
        else:
            print('[BODY] Shutdown disabled or service stopping', flush=True)
        self.shutdown.set()  # Only AFTER poweroff has finished.

    def status(self):
        with self.lock:
            age = None if self.last_head is None else time.monotonic() - self.last_head
            return {
                'type': 'STATUS_REPLY', 'state': self.state, 'mode': self.mode,
                'head_connected': age is not None and age <= self.config['head_timeout_seconds'],
                'head_heartbeat_age_s': None if age is None else round(age, 1),
                'relay_configured': self.relay_device is not None,
                'relay_enabled': bool(self.relay_device.value) if self.relay_device is not None else False,
                'teensy_enable': bool(self.teensy_enable.value) if self.teensy_enable is not None else False,
                'mode_process_alive': self.proc is not None and self.proc.poll() is None,
                'shutdown_enabled': self.config['allow_shutdown'],
            }

    def handle(self, msg, addr=None):
        if not isinstance(msg, dict) or msg.get('token') != self.config['token']:
            return
        command = str(msg.get('command', '')).upper()
        if command == 'HEAD_HEARTBEAT':
            if addr and addr[0] == self.config['head_host']:
                with self.lock:
                    self.last_head = time.monotonic()
        elif command == 'HEAD_ACK':
            if addr and addr[0] == self.config['head_host'] and msg.get('for') == 'ESTOP':
                self.head_estop_ack.set()
        elif command == 'STATUS':
            response = self.status()
            if addr is None:
                print('[BODY]', json.dumps(response), flush=True)
            else:
                self.sock.sendto(json.dumps(response).encode(), addr)
        elif command == 'ESTOP':
            self.request_stop(emergency=True)
        elif command == 'STOP':
            self.request_stop()
        elif command == 'START':
            self.request_start(str(msg.get('mode', '')).upper())
        elif command == 'HEARTBEAT':
            with self.lock:
                self.last_remote = time.monotonic()

    def receiver(self):
        while not self.shutdown.is_set():
            try:
                raw, addr = self.sock.recvfrom(4096)
                self.handle(json.loads(raw), addr)
            except socket.timeout:
                pass
            except (OSError, ValueError) as exc:
                if not self.shutdown.is_set():
                    print('[BODY] Receive error:', exc, flush=True)

    def relay_request(self, command):
        with self.lock:
            if command == 'OFF':
                if self.relay_device is not None:
                    self.relay_device.off()
                return 'OK'
            permitted = (not self.shutdown.is_set() and not self.estop_started.is_set()
                         and self.state == 'RUNNING' and self.proc is not None
                         and self.proc.poll() is None and self.relay_device is not None)
            if command == 'ON' and permitted:
                self.relay_device.on()
                return 'OK'
            return 'DENIED'

    def relay_server(self):
        while not self.shutdown.is_set():
            try:
                conn, _ = self.relay_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                if not self.shutdown.is_set():
                    self.request_shutdown()
                return
            with conn:
                conn.settimeout(.5)
                try:
                    command = conn.recv(128).decode().strip().upper()
                    conn.sendall(self.relay_request(command).encode())
                except (OSError, ValueError) as exc:
                    print('[RELAY]', exc, flush=True)

    def console(self):
        while not self.shutdown.is_set():
            try:
                command = input('bb8> ').strip().upper()
            except EOFError:
                return
            if command in SCRIPTS:
                self.request_start(command)
            elif command in ('STOP', 'ESTOP', 'STATUS'):
                self.handle({'token': self.config['token'], 'command': command})

    def check_failsafes(self):
        with self.lock:
            if self.state != 'RUNNING':
                return
            now = time.monotonic()
            reason = None
            if self.proc is None or self.proc.poll() is not None:
                reason = 'Child exited'
            elif self.config['remote_required'] and now - self.last_remote > self.config['remote_timeout_seconds']:
                reason = 'Remote heartbeat lost'
            elif self.mode in ('BEACON', 'APRILTAG') and (self.last_head is None or
                    now - self.last_head > self.config['head_timeout_seconds']):
                reason = 'Head supervisor lost'
            if reason:
                print('[FAILSAFE]', reason, flush=True)
                self.request_stop()

    def initialize(self):
        from gpiozero import OutputDevice
        self.teensy_enable = OutputDevice(21, active_high=True, initial_value=False)
        if self.config['relay_active_high'] is not None:
            self.relay_device = OutputDevice(20, active_high=self.config['relay_active_high'], initial_value=False)
        if self.config.get("test_unit_host"):
            self.mirror_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.config['body_port']))
        self.sock.settimeout(.2)
        # Bind before receiving START so children cannot race relay-server startup.
        try:
            os.unlink(self.relay_socket_path)
        except FileNotFoundError:
            pass
        self.relay_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.relay_socket.bind(self.relay_socket_path)
        os.chmod(self.relay_socket_path, 0o600)
        self.relay_socket.listen(4)
        self.relay_socket.settimeout(.2)

    def close(self):
        self.request_shutdown()
        try:
            self.stop()
        finally:
            for worker in self.workers:
                worker.join(timeout=1)
            with self.lock:
                self.outputs_off()
                if self.teensy_enable is not None:
                    self.teensy_enable.close()
                if self.relay_device is not None:
                    self.relay_device.close()
            if self.sock is not None:
                self.sock.close()
            if self.mirror_socket is not None:
                self.mirror_socket.close()
            if self.relay_socket is not None:
                self.relay_socket.close()
                try:
                    os.unlink(self.relay_socket_path)
                except FileNotFoundError:
                    pass

    def run(self):
        old_handlers = {}
        try:
            for sig in (signal.SIGTERM, signal.SIGINT):
                old_handlers[sig] = signal.signal(sig, self.request_shutdown)
            self.initialize()
            for target in (self.relay_server, self.receiver):
                worker = threading.Thread(target=target, daemon=True)
                self.workers.append(worker)
                worker.start()
            if self.mirror_socket is not None:
                worker = threading.Thread(target=self.mirror_loop, daemon=True)
                self.workers.append(worker)
                worker.start()
            if self.config['debug_console']:
                threading.Thread(target=self.console, daemon=True).start()
            while not self.shutdown.is_set():
                self.check_failsafes()
                try:
                    command, selected, generation = self.commands.get(timeout=.1)
                except queue.Empty:
                    continue
                if command == 'ESTOP':
                    self.estop()
                elif self.active(generation):
                    if command == 'START':
                        self.start(selected, generation)
                    else:
                        self.stop(generation)
        finally:
            try:
                self.close()
            finally:
                for sig, handler in old_handlers.items():
                    signal.signal(sig, handler)


def main():
    config = json.loads((ROOT / 'config.json').read_text())
    if 'BB8_DEBUG_CONSOLE' in os.environ:
        config['debug_console'] = os.environ['BB8_DEBUG_CONSOLE'] == '1'
    BodySupervisor(config).run()


if __name__ == '__main__':
    main()
