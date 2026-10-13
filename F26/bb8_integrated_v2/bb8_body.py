#!/usr/bin/env python3
"""Single owner of BB-8 modes. Children own GPIO20 and Teensy serial only while running."""
import json,os,signal,socket,subprocess,sys,threading,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
C=json.loads((ROOT/'config.json').read_text())
lock=threading.RLock(); relay_lock=threading.RLock(); relay_device=None; relay_socket_path='/tmp/bb8-relay.sock'; state='IDLE'; mode=None; proc=None; last_remote=time.monotonic(); last_head=time.monotonic(); running=True
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); sock.bind(('0.0.0.0',C['body_port'])); sock.settimeout(.25)
head=(C['head_host'],C['head_port'])
head_estop_ack=threading.Event()
estop_started=threading.Event()
def tx(cmd,selected=None):
    sock.sendto(json.dumps({'token':C['token'],'command':cmd,'mode':selected}).encode(),head)
def init_relay():
    global relay_device
    if C['relay_active_high'] is None:
        return
    from gpiozero import OutputDevice
    relay_device=OutputDevice(20,active_high=C['relay_active_high'],initial_value=False)

def relay_off():
    with relay_lock:
        if relay_device is not None: relay_device.off()

def relay_server():
    try: os.unlink(relay_socket_path)
    except FileNotFoundError: pass
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    server.bind(relay_socket_path)
    os.chmod(relay_socket_path,0o600)
    server.listen(4)
    server.settimeout(.2)
    try:
        while running:
            try: conn,_=server.accept()
            except socket.timeout: continue
            with conn:
                conn.settimeout(.5)
                try:
                    command=conn.recv(128).decode().strip().upper()
                    with lock:
                        permitted=(state=='RUNNING' and proc is not None and proc.poll() is None and relay_device is not None)
                        with relay_lock:
                            if command=='ON' and permitted: relay_device.on(); reply='OK'
                            elif command=='OFF': relay_off(); reply='OK'
                            else: reply='DENIED'
                    conn.sendall(reply.encode())
                except (OSError,ValueError) as exc: print('[RELAY]',exc,flush=True)
    finally:
        server.close()
        try:os.unlink(relay_socket_path)
        except FileNotFoundError:pass

def stop_child():
    global proc
    if proc and proc.poll() is None:
        os.killpg(proc.pid,signal.SIGTERM)
        try: proc.wait(timeout=1.5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid,signal.SIGKILL); proc.wait(timeout=1)
    proc=None
    relay_off()
def stop():
    global state,mode
    with lock:
        if state=='ESTOP': return
        state='STOPPING'
        stop_child()
        for _ in range(3): tx('STOP')
        mode=None;state='IDLE';print('[BODY] IDLE / relay OFF',flush=True)
def start(selected):
    global state,mode,proc
    with lock:
        if state=='ESTOP': return
        if selected not in ('PS5','BEACON','APRILTAG'): return
        if state!='IDLE': stop()
        if C['relay_active_high'] is None:
            print('[BLOCKED] Set relay_active_high after verifying hardware',flush=True);return
        state='STARTING';tx('START',selected)
        # Do not block the UDP receiver: head heartbeat is handled in receiver thread.
        # A fresh heartbeat is required for beacon operation.
        if selected in ('BEACON','APRILTAG'):
            deadline=time.monotonic()+5
            while time.monotonic()<deadline and time.monotonic()-last_head>1.5:
                time.sleep(.05)
            if time.monotonic()-last_head>1.5:
                print('[BLOCKED] Head heartbeat missing',flush=True);stop();return
        script='PS5_Controller.py' if selected=='PS5' else 'Beacon_Target_Control.py'
        env=os.environ.copy();env['BB8_RELAY_ACTIVE_HIGH']='1' if C['relay_active_high'] else '0'
        proc=subprocess.Popen([sys.executable,'-u',str(ROOT/script)],cwd=ROOT,env=env,start_new_session=True)
        mode=selected;state='RUNNING';print('[BODY] RUNNING',mode,flush=True)
def request_poweroff():
    # Requires narrow passwordless sudoers rule documented in README.
    try:
        result=subprocess.run(['sudo','-n','/usr/bin/systemctl','poweroff'],
                              timeout=5, capture_output=True, text=True)
        if result.returncode:
            print('[SHUTDOWN FAILED]', result.stderr.strip(), flush=True)
    except Exception as exc:
        print('[SHUTDOWN FAILED]', repr(exc), flush=True)

def estop():
    global state,mode,running
    if estop_started.is_set():return
    estop_started.set()
    with lock:
        state='ESTOP'
        # Kill the mode process first; do not call stop() (ESTOP is latched).
        stop_child()
        mode=None
        relay_off()
    print('[BODY] ESTOP latched: child stopped, relays OFF',flush=True)
    head_estop_ack.clear()
    # Retransmit until Head acknowledges, but never wait forever.
    deadline=time.monotonic()+4.0
    while time.monotonic()<deadline and not head_estop_ack.is_set():
        try:tx('ESTOP')
        except OSError as exc:print('[HEAD] ESTOP send error',repr(exc),flush=True)
        head_estop_ack.wait(.2)
    print('[BODY] Head ESTOP ACK:',head_estop_ack.is_set(),flush=True)
    running=False
    if C['allow_shutdown']:
        request_poweroff()
    else:
        print('[BODY] Shutdown disabled in config; staying powered for testing',flush=True)
def handle(msg,addr=None):
    global last_remote,last_head
    if msg.get('token')!=C['token']: return
    cmd=str(msg.get('command','')).upper()
    if cmd=='HEAD_HEARTBEAT':
        if addr and addr[0]==C['head_host']: last_head=time.monotonic()
        return
    if cmd=='HEAD_ACK':
        if addr and addr[0]==C['head_host'] and msg.get('for')=='ESTOP':head_estop_ack.set()
        return
    if cmd=='ESTOP':
        if not estop_started.is_set():threading.Thread(target=estop,daemon=True).start()
        return
    if estop_started.is_set():return
    if cmd=='HEARTBEAT':last_remote=time.monotonic();return
    if cmd=='START':
        last_remote=time.monotonic();threading.Thread(target=start,args=(str(msg.get('mode','')).upper(),),daemon=True).start()
    elif cmd=='STOP':threading.Thread(target=stop,daemon=True).start()
    elif cmd=='STATUS':
        with lock:
            response = {
                'type': 'STATUS_REPLY',
                'state': state,
                'mode': mode,
                'head_connected': time.monotonic() - last_head <= C['head_timeout_seconds'],
                'head_heartbeat_age_s': round(time.monotonic() - last_head, 1),
                'relay_configured': relay_device is not None,
                'relay_enabled': bool(relay_device.value) if relay_device is not None else False,
                'mode_process_alive': bool(proc is not None and proc.poll() is None),
                'shutdown_enabled': C['allow_shutdown'],
            }
        if addr is not None:
            sock.sendto(json.dumps(response).encode('utf-8'), addr)
        else:
            print('[BODY]', json.dumps(response), flush=True)
def receiver():
    while running:
        try: raw,addr=sock.recvfrom(4096);handle(json.loads(raw),addr)
        except socket.timeout:pass
        except Exception as e:print('[BODY] receive error',repr(e),flush=True)
def console():
    while running:
        try: cmd=input('bb8> ').strip().upper()
        except EOFError:return
        if cmd in ('PS5','BEACON','APRILTAG'):handle({'token':C['token'],'command':'START','mode':cmd})
        elif cmd in ('STOP','ESTOP','STATUS'):handle({'token':C['token'],'command':cmd})
def main():
    global running
    if C['relay_active_high'] is None:
        print('GPIO polarity unverified: configure before hardware operation',flush=True)
    else:init_relay()
    threading.Thread(target=relay_server,daemon=True).start()
    threading.Thread(target=receiver,daemon=True).start()
    if C['debug_console']:threading.Thread(target=console,daemon=True).start()
    try:
        while running:
            with lock:
                if state=='RUNNING' and proc and proc.poll() is not None:
                    print('[FAILSAFE] Child exited',flush=True);stop()
                if state=='RUNNING' and C['remote_required'] and time.monotonic()-last_remote>C['remote_timeout_seconds']:
                    print('[FAILSAFE] Remote heartbeat lost',flush=True);stop()
                if state=='RUNNING' and mode in ('BEACON','APRILTAG') and time.monotonic()-last_head>C['head_timeout_seconds']:
                    print('[FAILSAFE] Head supervisor lost',flush=True);stop()
            time.sleep(.1)
    except KeyboardInterrupt:pass
    finally:
        if state!='ESTOP':stop()
        relay_off()
        if relay_device is not None: relay_device.close()
        sock.close()
if __name__=='__main__':main()
