#!/usr/bin/env python3
"""
BB-8 Beacon PC Display

Expected Head Pi packet:
distance_ft,angle_deg,distance_confidence,angle_confidence,
good_distance_count,good_angle_count,raw_distance_count,raw_angle_count,state
"""

import math
import socket
import threading
import time
import tkinter as tk

UDP_PORT = 5006
PACKET_TIMEOUT = 2.0
MAX_RANGE_FT = 20.0

BG = "#11151a"
PANEL = "#1b222b"
GRID = "#3b4654"
TEXT = "#e8eef5"
MUTED = "#9ba8b5"
GOOD = "#55d187"
WARN = "#ffca55"
BAD = "#ff6b6b"
ROBOT = "#64a9ff"


class Receiver:
    def __init__(self):
        self.lock = threading.Lock()
        self.telemetry = None
        self.last_rx = None
        self.sender_ip = None
        self.running = True

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", UDP_PORT))
        self.sock.settimeout(0.25)

        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while self.running:
            try:
                data, address = self.sock.recvfrom(1024)
                parts = data.decode("ascii", errors="strict").strip().split(",")

                if len(parts) != 9:
                    continue

                telemetry = {
                    "distance": float(parts[0]),
                    "angle": float(parts[1]),
                    "distance_conf": float(parts[2]),
                    "angle_conf": float(parts[3]),
                    "good_distance": int(parts[4]),
                    "good_angle": int(parts[5]),
                    "raw_distance": int(parts[6]),
                    "raw_angle": int(parts[7]),
                    "state": parts[8],
                }

                with self.lock:
                    self.telemetry = telemetry
                    self.last_rx = time.monotonic()
                    self.sender_ip = address[0]

            except socket.timeout:
                pass
            except (UnicodeDecodeError, ValueError):
                pass
            except OSError:
                break

    def get(self):
        with self.lock:
            t = self.telemetry
            last = self.last_rx
            ip = self.sender_ip

        live = last is not None and time.monotonic() - last <= PACKET_TIMEOUT
        return t, live, ip

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass


class App:
    def __init__(self, root):
        self.root = root
        self.rx = Receiver()

        root.title("BB-8 Beacon Tracker")
        root.geometry("1100x760")
        root.minsize(900, 620)
        root.configure(bg=BG)
        root.protocol("WM_DELETE_WINDOW", self.close)

        tk.Label(
            root, text="BB-8 BEACON TRACKER",
            bg=BG, fg=TEXT, font=("Segoe UI", 20, "bold")
        ).pack(pady=(12, 4))

        self.status = tk.Label(
            root, text="Waiting for Head Pi...",
            bg=BG, fg=BAD, font=("Segoe UI", 11)
        )
        self.status.pack(pady=(0, 8))

        content = tk.Frame(root, bg=BG)
        content.pack(fill="both", expand=True, padx=16, pady=(0, 10))

        self.canvas = tk.Canvas(content, bg=PANEL, highlightthickness=0)
        self.canvas.pack(side="left", fill="both", expand=True)

        self.side = tk.Frame(content, bg=PANEL, width=260)
        self.side.pack(side="right", fill="y", padx=(10, 0))
        self.side.pack_propagate(False)

        tk.Label(
            self.side, text="FILTER DATA",
            bg=PANEL, fg=TEXT, font=("Segoe UI", 15, "bold")
        ).pack(pady=(22, 20))

        self.side_labels = {}
        for key, title in [
            ("state", "Reading"),
            ("distance", "Distance Average"),
            ("angle", "Angle Average"),
            ("dconf", "Distance Confidence"),
            ("aconf", "Angle Confidence"),
            ("dsamples", "Good Distance Samples"),
            ("asamples", "Good Angle Samples"),
        ]:
            frame = tk.Frame(self.side, bg=PANEL)
            frame.pack(fill="x", padx=18, pady=7)
            tk.Label(
                frame, text=title, bg=PANEL, fg=MUTED,
                font=("Segoe UI", 10)
            ).pack(anchor="w")
            value = tk.Label(
                frame, text="--", bg=PANEL, fg=TEXT,
                font=("Consolas", 13, "bold")
            )
            value.pack(anchor="w")
            self.side_labels[key] = value

        self.data = tk.Label(
            root, text="", bg=BG, fg=TEXT, font=("Consolas", 13)
        )
        self.data.pack(pady=(0, 14))

        self.update_ui()

    def draw(self, t, live):
        c = self.canvas
        c.delete("all")
        w = max(c.winfo_width(), 1)
        h = max(c.winfo_height(), 1)
        cx = w / 2
        cy = h * 0.72
        R = min(w * 0.42, h * 0.62)

        for ft in (5, 10, 15, 20):
            r = R * ft / MAX_RANGE_FT
            c.create_oval(cx-r, cy-r, cx+r, cy+r, outline=GRID)
            c.create_text(
                cx+5, cy-r+10, text=f"{ft} ft",
                fill=MUTED, anchor="w"
            )

        c.create_line(cx-R, cy, cx+R, cy, fill=GRID)
        c.create_line(cx, cy+R*.3, cx, cy-R, fill=GRID)
        c.create_text(cx, cy-R-15, text="0° / FRONT", fill=TEXT)
        c.create_text(cx+R+8, cy, text="+90°", fill=MUTED, anchor="w")
        c.create_text(cx-R-8, cy, text="-90°", fill=MUTED, anchor="e")

        c.create_oval(
            cx-17, cy-17, cx+17, cy+17,
            fill=ROBOT, outline="white", width=2
        )
        c.create_text(cx, cy+30, text="BB-8", fill=TEXT)

        if not live or t is None:
            c.create_text(
                cx, 35, text="NO LIVE BEACON DATA",
                fill=BAD, font=("Segoe UI", 15, "bold")
            )
            return

        d = t["distance"]
        a = t["angle"]
        rad = math.radians(a)
        shown = min(d, MAX_RANGE_FT)

        bx = cx + R * (shown/MAX_RANGE_FT) * math.sin(rad)
        by = cy - R * (shown/MAX_RANGE_FT) * math.cos(rad)

        c.create_line(cx, cy, bx, by, fill=WARN, width=3, arrow=tk.LAST)
        c.create_oval(
            bx-12, by-12, bx+12, by+12,
            fill=WARN, outline="white", width=2
        )
        c.create_text(
            bx, by-27,
            text=f'{d:.2f} ft\n{a:+.1f}°',
            fill=TEXT, justify="center"
        )

    def update_side(self, t, live):
        if not live or t is None:
            for label in self.side_labels.values():
                label.config(text="--", fg=MUTED)
            return

        state = t["state"]
        self.side_labels["state"].config(
            text="FRESH" if state == "GOOD" else "HELD",
            fg=GOOD if state == "GOOD" else WARN
        )
        self.side_labels["distance"].config(text=f'{t["distance"]:.3f} ft')
        self.side_labels["angle"].config(text=f'{t["angle"]:+.2f}°')
        self.side_labels["dconf"].config(text=f'{t["distance_conf"]:.1f}%')
        self.side_labels["aconf"].config(text=f'{t["angle_conf"]:.1f}%')
        self.side_labels["dsamples"].config(
            text=f'{t["good_distance"]} / {t["raw_distance"]}'
        )
        self.side_labels["asamples"].config(
            text=f'{t["good_angle"]} / {t["raw_angle"]}'
        )

    def update_ui(self):
        t, live, ip = self.rx.get()

        if live and t:
            d = t["distance"]
            a = t["angle"]
            rad = math.radians(a)
            x = d * math.sin(rad)
            y = d * math.cos(rad)

            self.status.config(
                text=f"HEAD PI CONNECTED  •  {ip}  •  UDP {UDP_PORT}",
                fg=GOOD
            )
            self.data.config(
                text=(
                    f"Distance: {d:6.2f} ft    "
                    f"Angle: {a:+7.2f}°    "
                    f"X: {x:+6.2f} ft    "
                    f"Y: {y:+6.2f} ft"
                )
            )
        else:
            self.status.config(
                text=f"WAITING FOR HEAD PI  •  UDP {UDP_PORT}",
                fg=BAD
            )
            self.data.config(text="No live beacon packets received.")

        self.draw(t, live)
        self.update_side(t, live)
        self.root.after(50, self.update_ui)

    def close(self):
        self.rx.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()
