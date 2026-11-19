#!/usr/bin/env python3
"""BB-8 test stand: dry-run by default. No IMU. Enable hardware only after wiring verification."""
import argparse, json, math, socket, time
TIMEOUT=.6
MODES={'PS5','BEACON','APRILTAG','DEMO'}

class Outputs:
    def __init__(self, hardware=False):
        self.hardware=hardware
        self.last=None
        if hardware:
            from gpiozero import PhaseEnableMotor, OutputDevice
            import board, busio
            from adafruit_pca9685 import PCA9685
            from adafruit_motor.servo import Servo
            # Original Movement_Functions wiring; check driver mode/3.3V compatibility first.
            self.m1=PhaseEnableMotor(phase=5,enable=13,pwm=True)
            self.m2=PhaseEnableMotor(phase=6,enable=18,pwm=True)
            self.turn=PhaseEnableMotor(phase=26,enable=19,pwm=True)
            self.r1=OutputDevice(20,active_high=True,initial_value=False)
            self.r2=OutputDevice(21,active_high=True,initial_value=False)
            self.pca=PCA9685(busio.I2C(board.SCL,board.SDA))
            self.pca.frequency=50
            self.s1=Servo(self.pca.channels[0]); self.s2=Servo(self.pca.channels[4])
            self.stop()
    def stop(self):
        if self.hardware:
            self.m1.value=0; self.m2.value=0; self.turn.value=0
            self.r1.off(); self.r2.off()
            self.pca.channels[0].duty_cycle=0
            self.pca.channels[4].duty_cycle=0
        if self.last != ('stop',): print('[STAND] STOP',flush=True)
        self.last=('stop',)
    def move(self, distance, angle):
        # Demonstration mapping, not an exact reproduction of Teensy dynamics.
        # Forward/back speed follows target distance; pivot is requested at large angles.
        a=math.radians(angle)
        drive=max(-.25,min(.25,(distance/20)*math.cos(a)*.25))
        turn=max(-.25,min(.25,-math.sin(a)*.25))
        swing=max(78,min(102,90+drive*36))
        command=(round(drive,3),round(turn,3),round(swing,1))
        if self.hardware:
            self.r1.on(); self.r2.on()
            self.m1.value=drive; self.m2.value=drive
            self.turn.value=turn
            offset=swing-90-1.0
            self.s1.angle=90+offset; self.s2.angle=90-offset
        if command!=self.last: print('[STAND] drive, turn, swing:',command,flush=True)
        self.last=command
    def close(self):
        self.stop()
        if self.hardware:
            for x in (self.m1,self.m2,self.turn,self.r1,self.r2): x.close()
            self.pca.deinit()

class Stand:
    def __init__(self,outputs):
        self.out=outputs; self.enabled=False; self.mode=None
        self.last_status=0; self.last_motion=0; self.last_seq=-1
    def handle(self,p,now):
        if p.get('protocol')!='bb8-mirror-v1':return
        if p.get('kind')=='status':
            seq=p.get('seq')
            if type(seq)!=int or seq<=self.last_seq:return
            self.last_seq=seq; self.last_status=now
            newmode=p.get('mode') if p.get('mode') in MODES else None
            enabled=p.get('enabled') is True and p.get('state')=='RUNNING'
            if not enabled or newmode!=self.mode:self.out.stop();self.last_motion=0
            self.enabled=enabled;self.mode=newmode
        elif p.get('kind')=='motion':
            if not self.enabled or now-self.last_status>TIMEOUT or p.get('mode')!=self.mode:return
            if p.get('stop') is True:self.out.stop();self.last_motion=0;return
            try:
                d=float(p['distance_ft']);a=float(p['angle_deg'])
            except (KeyError,ValueError,TypeError):return
            if not(math.isfinite(d) and math.isfinite(a) and 0<=d<=100 and -180<=a<=180):return
            self.last_motion=now
            if d <= 0.01:
                self.out.stop()
            else:
                self.out.move(d,a)
    def watchdog(self,now):
        if now-self.last_status>TIMEOUT or now-self.last_motion>TIMEOUT:self.out.stop()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--token',required=True)
    ap.add_argument('--port',type=int,default=5015)
    ap.add_argument('--enable-hardware',action='store_true',help='CAUTION: energizes motors; verify wiring and use physical cutoff')
    args=ap.parse_args()
    outputs=Outputs(args.enable_hardware);stand=Stand(outputs)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);sock.bind(('0.0.0.0',args.port));sock.settimeout(.05)
    print('Stand receiver', 'HARDWARE' if args.enable_hardware else 'DRY RUN',flush=True)
    try:
        while True:
            try:
                raw,addr=sock.recvfrom(2048)
                if len(raw)>1500:continue
                msg=json.loads(raw)
                if isinstance(msg,dict) and msg.get('token')==args.token:
                    stand.handle(msg,time.monotonic())
            except (ValueError,UnicodeError):pass
            except socket.timeout:pass
            stand.watchdog(time.monotonic())
    except KeyboardInterrupt:pass
    finally:outputs.close();sock.close()
if __name__=='__main__':main()
