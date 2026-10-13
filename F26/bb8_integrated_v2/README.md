# BB-8 integrated mode control v2 (bench-test release)

## Files
- Body: `bb8_body.py`, `BB8_Relays.py`, `PS5_Controller.py`, `Beacon_Target_Control.py`
- Head: `bb8_head.py`, `BB8_Beacon_Sender.py`
- Teensy: `BB8_Teensy_Controller.ino` (unchanged firmware)
- `bb8_remote_debug.py`: UDP command emulator
- AprilTag is intentionally a placeholder.

## Setup
1. **Motors disconnected for first tests.** Disable old `bb8.service` and any legacy scripts/services, so only one process owns `/dev/serial0`, UDP 5005, or GPIO20.
2. Install on each Pi in `/home/bb8/bb8-control`; replace `User=bb8` and paths if your account differs. Copy matching files and the same `config.json` to both Pis.
3. `sudo apt install python3-serial python3-gpiozero python3-pygame` on Body; `python3-serial` on Head. Head may need its existing UWB dependencies. Use the existing Python virtual environment if required.
4. Configure both Pis to join Head Pi hotspot; set actual Head IP (example 10.42.0.1) and Body IP (existing beacon sender expects 10.42.0.27). Reserve/static-address these addresses.
5. Set a long random `token` identically on both. Set `debug_console` true for terminal debugging; set false for systemd.
6. **Measure GPIO20 relay polarity while motors disconnected.** Set `relay_active_high` to `true` only if GPIO HIGH energizes relays, otherwise `false`. Leaving `null` prevents starting modes. Relay board must have a hardware default-off bias when Pi is unpowered or booting.
7. Edit `BB8_Beacon_Sender.py` BODY_PI_IP to match actual Body IP; DISPLAY_IP to match laptop IP. UDP 5005 remains unchanged. Existing sender still holds a stale target up to 4 seconds: review this carefully before motion tests.
8. Run `python3 bb8_head.py` on Head, then `python3 bb8_body.py` on Body. Test `status`, `apriltag`, `stop`, then `beacon` and `ps5` with motors disconnected.
9. For boot: `sudo cp bb8-body.service /etc/systemd/system/` (Body), equivalent head service (Head), `sudo systemctl daemon-reload`, `sudo systemctl enable --now bb8-body.service`; inspect with `journalctl -u bb8-body.service -f`. Disable console in service mode.
10. `allow_shutdown=false` by default. Only set true after successful ESTOP dry-run, and verify service user has poweroff authorization.

## Important limitations / safety
- Existing child programs own Teensy UART and GPIO20 while running; supervisor owns GPIO20 only in IDLE. This avoids simultaneous GPIO/serial access but means supervisor cannot independently force GPIO off if a child is hung until it terminates. Add an independent hardware watchdog/interlock before field use.
- Wi-Fi E-stop is **not safety-rated**; use a hardwired normally-closed motor-power E-stop. UDP may be lost or spoofed; shared token is basic filtering, not encryption/authentication.
- PS5 mode requires a DualSense connected directly to Body Pi over Bluetooth, as in the uploaded script. The future Wi-Fi remote selects the mode; it does not replace the PS5 joystick.
- Debug terminal bypasses remote heartbeat intentionally. `remote_required` defaults false. The included remote emulator sends discrete commands, not periodic heartbeats; do not enable remote_required until real remote heartbeat is implemented.
- Head supervisor heartbeat is monitored in beacon mode. No end-to-end acknowledged startup or shutdown, and no guaranteed head shutdown if Wi-Fi fails. A physical power disconnect remains essential.
- `BB8_Teensy_Controller.ino` unchanged. Firmware has STOP and a watchdog; it does not implement a separate safety-rated ARM acknowledgment.
- Do not run `bb8_body.py` via systemd with debug_console true; use remote emulator from a second machine, or direct terminal mode with service stopped.


## v3 fixes (GPIO busy / head heartbeat)
- Body supervisor exclusively claims GPIO20 at startup. Mode programs call the local Unix relay request socket instead of claiming GPIO themselves.
- The relay socket only accepts ON while a mode child is running; OFF is always allowed.
- Head heartbeat processing continues while a mode is starting.
- PS5 status accepts Teensy `S=` as well as `S,`.
- Stop any old supervisor first. Start only one `bb8_body.py` process.
- Confirm relay polarity and keep motor power disconnected for first tests.
- For testing: `python3 bb8_body.py` on Body Pi, `python3 bb8_head.py` on Head Pi.
- Copy all files together; do not mix v2 and v3.
- GPIO cannot be guaranteed OFF during Pi boot before software initializes: add hardware default-off and physical motor cutoff.
- Software E-stop and heartbeat are not a substitute for hardwired safety controls.


## v4 AprilTag mode
- Copy `BB8_AprilTag_Detection.py` to Head Pi beside `bb8_head.py`. Body Pi reuses `Beacon_Target_Control.py` as the distance/angle receiver and Teensy bridge.
- `APRILTAG` mode starts the detector on Head Pi and receiver on Body Pi. `STOP` terminates both. Only tag ID 1 transmits movement targets; other tags print/appear in the optional preview. When ID 1 disappears, packets cease, causing Body receiver to stop after 0.5 seconds and disable relays. No hold-last-target in AprilTag mode.
- Install Picamera2, OpenCV, NumPy, pupil_apriltags in the **same Python interpreter** configured as `apriltag_python` in Head `config.json`. If you installed pupil_apriltags in `~/apriltag-env`, set `apriltag_python` to `/home/bb8camera/apriltag-env/bin/python` (check that Picamera2 is also available in that environment).
- `BB8_BODY_IP` environment variable can override detector target IP; default 10.42.0.27. Preview disabled by default for systemd. Set `BB8_SHOW_VIDEO=1` only in a graphical manual session.
- Keep both Pi `config.json` token/addresses in sync. Head service user must be `bb8camera` and path must match actual installation; do not blindly copy the example service.
- Distance uses original 3D camera-to-tag distance and angle uses atan2(x,z); values are camera-relative, not automatically body-relative. Calibrate FOV/camera intrinsics and account for head yaw before autonomous use.
- **Known safety caveat:** the v3 Body supervisor checks head heartbeat but does not await mode-specific START acknowledgement, and UDP target packets are unauthenticated. Keep motor power disconnected during initial tests and use a physical motor cutoff.
- Test: stop systemd services before manual runs; start both supervisors; enter `apriltag` on Body; observe Head detection and Body receiver; remove tag 1 and verify relay OFF within timeout; test `stop`, `beacon`, and `estop` with motor power disconnected.
