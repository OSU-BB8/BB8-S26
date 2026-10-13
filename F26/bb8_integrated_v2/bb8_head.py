#!/usr/bin/env python3
import json,os,signal,socket,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent; C=json.loads((ROOT/'config.json').read_text());proc=None;mode=None
sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('0.0.0.0',C['head_port']));sock.settimeout(.2)
def tx(command, **extra):sock.sendto(json.dumps({'token':C['token'],'command':command,**extra}).encode(),(C['body_host'],C['body_port']))
def stop():
    global proc,mode
    if proc and proc.poll() is None:
        os.killpg(proc.pid,signal.SIGTERM)
        try:proc.wait(timeout=1.5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=1)
    proc=None;mode=None
def main():
    global proc,mode
    last=0
    try:
        while True:
            if time.monotonic()-last>.5:
                tx('HEAD_HEARTBEAT');last=time.monotonic()
            try:raw,addr=sock.recvfrom(2048)
            except socket.timeout:
                if proc and proc.poll() is not None:stop()
                continue
            if addr[0]!=C['body_host']:continue
            try:msg=json.loads(raw)
            except ValueError:continue
            if msg.get('token')!=C['token']:continue
            cmd=msg.get('command');selected=msg.get('mode')
            if cmd in ('STOP','ESTOP'):
                stop()
                tx('HEAD_ACK', **{'for':cmd})
            elif cmd=='START':
                if selected==mode and (proc is None or proc.poll() is None):tx('HEAD_ACK');continue
                stop()
                if selected=='BEACON':proc=subprocess.Popen([sys.executable,'-u',str(ROOT/'BB8_Beacon_Sender.py')],cwd=ROOT,start_new_session=True)
                elif selected=='APRILTAG':proc=subprocess.Popen([C.get('apriltag_python',sys.executable),'-u',str(ROOT/'BB8_AprilTag_Detection.py')],cwd=ROOT,start_new_session=True)
                mode=selected;tx('HEAD_ACK')
            if cmd=='ESTOP':
                if C['allow_shutdown']:
                    try:
                        result=subprocess.run(['sudo','-n','/usr/bin/systemctl','poweroff'],timeout=5,capture_output=True,text=True)
                        if result.returncode:print('[HEAD] Shutdown failed:',result.stderr.strip(),flush=True)
                    except Exception as exc:print('[HEAD] Shutdown failed:',repr(exc),flush=True)
                return
    finally:stop();sock.close()
if __name__=='__main__':main()
