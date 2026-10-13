# E-STOP: both Raspberry Pis

Updated Body and Head supervisors: an E-STOP stops the Body mode process, commands GPIO 20 OFF, repeatedly sends ESTOP to the Head until an acknowledgement (up to 4 seconds), then requests Body shutdown. The Head stops its mode process, acknowledges, and requests its own shutdown.

## Install
Copy `bb8_body.py` to the Body Pi and `bb8_head.py` to the Head Pi, keeping the rest of the v4 statusfix files and existing config. Set `allow_shutdown` to `true` **on both Pis** only after testing. Keep `relay_active_high` set to the verified hardware polarity.

Each service runs as an unprivileged user. Grant **only** passwordless poweroff to the service account via `sudo visudo -f /etc/sudoers.d/bb8-poweroff`:

Body Pi:
```
bb8 ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff
```
Head Pi:
```
bb8camera ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff
```

Check the binary location with `command -v systemctl`; if not `/usr/bin/systemctl`, change both code and sudoers accordingly. Run `sudo visudo -c` to validate. Test permissions on a planned shutdown only: `sudo -n /usr/bin/systemctl poweroff` will immediately shut the Pi down.

Restart each supervisor: `sudo systemctl restart bb8-body.service` on Body; `sudo systemctl restart bb8-head.service` on Head.

## Testing
With motor power disconnected, first leave `allow_shutdown: false` on both. Start a mode, send `estop`, and check `journalctl -u bb8-body -n 60` and `journalctl -u bb8-head -n 60`. Confirm Body reports Head ESTOP ACK true and relays OFF. The E-STOP latch requires a supervisor restart for further modes. Next enable shutdown on both and perform a controlled power-off test.

## Safety
A network E-STOP cannot guarantee that the Head receives the command or powers off if Wi-Fi is down. Body shuts itself down after a bounded acknowledgement wait regardless. A hardware motor power cutoff is required for a real emergency stop. On Pi shutdown, GPIO is not guaranteed to remain driven OFF: use a fail-safe relay interface and hardware pull resistors. If the Body is unable to power off due to permissions, logs report failure; the ESTOP latch still stops mode processes and keeps relays commanded OFF while the supervisor remains alive.
