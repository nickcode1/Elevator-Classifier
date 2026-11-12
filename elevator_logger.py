#!/usr/bin/env python3
"""
Elevator ride logger (laptop side) for the ElevatorLogger.ino firmware.  Version 2.

The board streams sensor data the whole time. This program
  * detects every ride's start and stop from the accelerometer and saves one
    segment per ride, with 3 s of standstill before and after (no button presses),
  * lets you type or correct the ride's metadata while the ride is being recorded,
    after it, or at any later time (click the ride in the table),
  * keeps the complete uncut recording too, so nothing is lost if a ride is missed.

Files written to the output folder:
    ride_<id>.csv            one file per detected ride
    rides_metadata.csv       one row per ride (building, car, direction, floors, ...)
    raw/session_<time>.csv   the whole stream, uncut

Setup (once):   pip install pyserial        (tkinter comes with Python)
Run:            python elevator_logger.py --tag nick
                python elevator_logger.py --port COM5 --tag nick
Try it without a board:      python elevator_logger.py --demo
Re-cut a saved raw stream:   python elevator_logger.py --replay elevator_data/raw/session_....csv

How the detection works (lecture: thresholds, hysteresis, baseline tracking):
  1. With the board at rest the program learns which way gravity points, so the
     board can lie in any orientation. Vertical acceleration = reading along that
     direction minus gravity; it is positive when the cabin accelerates upward.
  2. The activity signal is the vertical acceleration smoothed over 0.25 s.
  3. Hysteresis: a ride starts when the signal stays above the HIGH threshold
     (5 x idle noise) and the start is traced back to where it left the LOW
     threshold (2 x idle noise).
  4. At constant speed the accelerometer reads about zero, so "quiet" alone does not
     end a ride. A ride is a start pulse followed by an OPPOSITE stop pulse; it ends
     when the opposite pulse has been seen and the signal is quiet again for 1 s.
  5. The gravity baseline and the idle noise are only updated while no ride is active.
  6. The barometer confirms it: an event that changed height by less than 1.5 m
     (a bump, picking the board up) is not saved as a ride.
"""

import argparse
import collections
import csv
import datetime as dt
import math
import os
import queue
import random
import statistics
import sys
import threading
import time

# ----------------------------------------------------------------- detector settings
G = 9.80665             # m/s^2 per g
PA_PER_M = 12.0         # 0.12 hPa per metre (assignment)
PRE_ROLL_S = 3.0        # standstill kept before each ride
POST_ROLL_S = 3.0       # standstill kept after each ride
SMOOTH_S = 0.25         # moving-average length of the activity signal
START_SIGMA = 5.0       # HIGH threshold = 5 x idle noise ...
START_MIN = 0.12        # ... but never below this (m/s^2) ...
START_MAX = 0.30        # ... or above this
STOP_SIGMA = 2.0        # LOW threshold = 2 x idle noise ...
STOP_MIN = 0.05         # ... but never below this (m/s^2)
START_HOLD_S = 0.25     # signal must stay above HIGH this long (rejects knocks)
QUIET_END_S = 1.0       # quiet time after the stop pulse that ends a ride
MIN_RIDE_S = 2.0        # shorter events are not rides
MIN_HEIGHT_M = 1.5      # smaller height changes are not rides
MAX_RIDE_S = 120.0      # give up on an event that never ends
CAL_S = 2.0             # stillness needed to learn the gravity direction
CAL_MAX_STD = 0.20      # m/s^2: "still" means every axis varies less than this
TILT_LIMIT = 1.0        # m/s^2 sideways for 1.5 s = the board was moved, re-learn gravity
TILT_HOLD_S = 1.5
BASELINE_TAU_S = 5.0    # how fast the gravity baseline follows slow drift while idle
NOISE_TAU_S = 10.0      # how fast the idle-noise estimate adapts

BUILDINGS = ["HartHouse", "MC", "Myhal", "Rosebrugh"]
RAW_HEADER = "t_us,ax_g,ay_g,az_g,p_pa,temp_c,rh_pct,baro_seq"
META_HEADER = [
    "ride_id", "file", "date", "time_start", "building", "car", "direction",
    "start_floor", "end_floor", "floors_travelled", "n_people", "orientation", "notes",
    "discard", "direction_auto", "duration_s", "est_height_m", "dp_pa", "v_peak_ms",
    "move_start_s", "move_stop_s", "n_samples", "rate_hz", "session_file", "session_t_s",
]
SETUP_FIELDS = ("building", "car", "orientation")
RIDE_FIELDS = ("start_floor", "end_floor", "n_people", "direction", "notes", "discard")


# =========================================================================== detector
class RideDetector:
    """Feed it one sample at a time; it returns a list of (event, details).

    Events: calibrating, calibrated, ride_start, ride_cancel, ride_end,
            ride_saved (details include the rows of the segment), ride_rejected.
    """

    def __init__(self, rate_hz=100):
        self.rate = rate_hz
        self.n_smooth = max(1, round(SMOOTH_S * rate_hz))
        self.pre_n = int(PRE_ROLL_S * rate_hz)
        self.post_n = int(POST_ROLL_S * rate_hz)
        self.buf = collections.deque(maxlen=int(600 * rate_hz))  # last 10 minutes of rows
        self.first = 0        # sample number of buf[0]
        self.idx = -1         # sample number of the newest row
        self.t_prev = None
        self.pending = None   # finished ride waiting for its post-roll
        self.var = (START_MIN / START_SIGMA) ** 2
        self.af = 0.0         # activity signal (smoothed vertical acceleration, m/s^2)
        self.hi = START_MIN
        self.lo = STOP_MIN
        self._recalibrate(None, "starting")

    # ------------------------------------------------------------------ helpers
    def _recalibrate(self, ev, why):
        self.state = "CAL"
        self.cal = []
        self.ma = collections.deque()
        self.ma_sum = 0.0
        self.hf = 0.0
        self.tilt_t = 0.0
        self._reset_run()
        if ev is not None:
            ev.append(("calibrating", {"why": why}))

    def _reset_run(self):
        self.run_start = None   # sample where the signal left the LOW threshold
        self.v_run = 0.0
        self.hi_t = 0.0

    def _row(self, i):
        return self.buf[i - self.first]

    def noise(self):
        return math.sqrt(self.var)

    # --------------------------------------------------------------------- feed
    def feed(self, t_us, ax, ay, az, p, raw):
        ev = []
        self.idx += 1
        if len(self.buf) == self.buf.maxlen:
            self.first += 1
        self.buf.append((t_us, ax, ay, az, p, raw))
        t = t_us / 1e6
        dt_ = 0.0 if self.t_prev is None else min(max(t - self.t_prev, 0.0), 0.1)
        self.t_prev = t

        if self.state == "CAL":
            self._calibrate(t, ax, ay, az, ev)
        else:
            self._track(t, dt_, ax, ay, az, ev)
        if self.pending and self.idx >= self.pending["due"]:
            self._finalize(ev)
        return ev

    def _calibrate(self, t, ax, ay, az, ev):
        self.cal.append((t, ax, ay, az))
        if t - self.cal[0][0] < CAL_S:
            return
        cols = list(zip(*self.cal))[1:]
        if max(statistics.pstdev(c) for c in cols) * G <= CAL_MAX_STD:
            self.g0 = [statistics.fmean(c) for c in cols]
            self.state = "IDLE"
            self._reset_run()
            ev.append(("calibrated", {}))
        else:
            self.cal = self.cal[len(self.cal) // 2:]  # not still yet: keep trying

    def _track(self, t, dt_, ax, ay, az, ev):
        gx, gy, gz = self.g0
        gn = math.sqrt(gx * gx + gy * gy + gz * gz) or 1.0
        dx, dy, dz = ax - gx, ay - gy, az - gz
        av = (dx * gx + dy * gy + dz * gz) / gn * G                 # vertical, m/s^2 (up = +)
        side = math.sqrt(max((dx * dx + dy * dy + dz * dz) * G * G - av * av, 0.0))
        self.hf += (side - self.hf) * min(dt_ / 0.5, 1.0)

        self.ma.append(av)
        self.ma_sum += av
        if len(self.ma) > self.n_smooth:
            self.ma_sum -= self.ma.popleft()
        af = self.af = self.ma_sum / len(self.ma)
        a = abs(af)
        sigma = math.sqrt(self.var)
        hi = self.hi = min(max(START_SIGMA * sigma, START_MIN), START_MAX)
        lo = self.lo = min(max(STOP_SIGMA * sigma, STOP_MIN), hi / 2)

        # Board picked up or turned over: gravity moved to another axis -> learn it again
        self.tilt_t = self.tilt_t + dt_ if self.hf > TILT_LIMIT else 0.0
        if self.tilt_t >= TILT_HOLD_S:
            if self.state == "MOVING":
                ev.append(("ride_cancel", {"why": "board was moved"}))
            self._recalibrate(ev, "board was moved")
            return

        if self.state == "IDLE":
            if a < hi:  # baseline tracking, only while no ride is active
                k = min(dt_ / BASELINE_TAU_S, 1.0)
                self.g0 = [gx + k * dx, gy + k * dy, gz + k * dz]
            if a < lo:
                self._reset_run()
                self.var += min(dt_ / NOISE_TAU_S, 1.0) * (af * af - self.var)
                return
            if self.run_start is None:
                self.run_start = self.idx
            self.v_run += af * dt_
            self.hi_t = self.hi_t + dt_ if a > hi else 0.0
            if self.hi_t >= START_HOLD_S:
                if self.pending:        # previous ride's post-roll is cut short
                    self._finalize(ev)
                self.state = "MOVING"
                self.sign0 = 1 if af > 0 else -1
                self.start_idx = max(self.run_start, self.idx - 2 * self.rate) - self.n_smooth // 2
                self.t_trig = t
                self.v = self.v_run
                self.vpeak = abs(self.v)
                self.dist = 0.0
                self.opp = False
                self.opp_t = self.quiet_t = self.same_t = 0.0
                self.quiet_idx = None
                self.last_sign = self.sign0
                ev.append(("ride_start", {"direction": "up" if self.sign0 > 0 else "down", "t": t}))
            return

        # ------------------------------------------------------------- MOVING
        self.v += af * dt_
        self.vpeak = max(self.vpeak, abs(self.v))
        self.dist += abs(self.v) * dt_
        sign = 1 if af > 0 else -1
        self.opp_t = self.opp_t + dt_ if (a > hi and sign != self.sign0) else 0.0
        if self.opp_t >= START_HOLD_S:
            self.opp = True                                   # the opposite (stop) pulse
        if a < lo:
            if self.quiet_idx is None:
                self.quiet_idx = self.idx
            self.quiet_t += dt_
            self.same_t = 0.0
        else:
            self.quiet_idx = None
            self.quiet_t = 0.0
            self.same_t = self.same_t + dt_ if sign == self.last_sign else 0.0
            self.last_sign = sign

        if self.opp and self.quiet_t >= QUIET_END_S and (abs(self.v) < 0.35 * self.vpeak or self.quiet_t >= 4.0):
            stop_idx = self.quiet_idx - self.n_smooth // 2
            self.pending = {"start": self.start_idx, "stop": stop_idx, "due": stop_idx + self.post_n,
                            "sign": self.sign0, "vpeak": self.vpeak, "dist": self.dist}
            self.state = "IDLE"
            self._reset_run()
            ev.append(("ride_end", {"t": self._row(max(stop_idx, self.first))[0] / 1e6}))
        elif not self.opp and self.quiet_t >= 1.5 and abs(self.v) < 0.15:
            self.state = "IDLE"                               # a knock, not a ride
            self._reset_run()
            ev.append(("ride_cancel", {"why": "just a bump"}))
        elif self.same_t > 8.0 or t - self.t_trig > MAX_RIDE_S:
            ev.append(("ride_cancel", {"why": "signal never settled"}))
            self._recalibrate(ev, "signal never settled")

    def _median_p(self, i0, i1):
        i0, i1 = max(i0, self.first), min(i1, self.idx)
        vals = [self._row(i)[4] for i in range(i0, i1 + 1)]
        vals = [v for v in vals if v > 0]
        return statistics.median(vals) if vals else None

    def _finalize(self, ev):
        """Cut the finished ride out of the buffer and check it against the barometer."""
        pd, self.pending = self.pending, None
        s, e = max(pd["start"], self.first), max(pd["stop"], self.first)
        seg0 = max(s - self.pre_n, self.first)
        seg1 = min(e + self.post_n, self.idx)
        t_s, t_e = self._row(s)[0], self._row(e)[0]
        duration = (t_e - t_s) / 1e6
        r = self.rate
        p0 = self._median_p(s - int(2.5 * r), s - int(0.5 * r))
        p1 = self._median_p(e + int(0.5 * r), e + int(2.5 * r))
        if p0 is not None and p1 is not None:
            dp = p1 - p0
            height = -dp / PA_PER_M
            moved = abs(height)
        else:  # no barometer: fall back to the distance from integrating acceleration
            dp, height = None, pd["sign"] * pd["dist"]
            moved = pd["dist"]
        if duration < MIN_RIDE_S or moved < MIN_HEIGHT_M:
            ev.append(("ride_rejected", {"why": f"only {moved:.1f} m in {duration:.1f} s"}))
            return
        rows = [self._row(i) for i in range(seg0, seg1 + 1)]
        t0 = rows[0][0]
        ev.append(("ride_saved", {
            "rows": rows,
            "direction_auto": "up" if height > 0 else "down",
            "duration_s": f"{duration:.2f}",
            "est_height_m": f"{height:.1f}",
            "dp_pa": "" if dp is None else f"{dp:.1f}",
            "v_peak_ms": f"{pd['vpeak']:.2f}",
            "move_start_s": f"{(t_s - t0) / 1e6:.2f}",
            "move_stop_s": f"{(t_e - t0) / 1e6:.2f}",
            "n_samples": str(len(rows)),
            "rate_hz": f"{(len(rows) - 1) / ((rows[-1][0] - t0) / 1e6):.2f}" if rows[-1][0] > t0 else "",
            "session_t_s": f"{t_s / 1e6:.2f}",
        }))


def parse_row(line):
    """CSV line from the board -> (t_us, ax, ay, az, p, fields) or None if garbled."""
    f = line.split(",")
    try:
        if len(f) != 8:
            return None
        vals = [float(x) for x in f[1:]]
        return int(f[0]), vals[0], vals[1], vals[2], vals[3], f
    except ValueError:
        return None


def write_segment(path, rows):
    t0 = rows[0][0]
    with open(path, "w", newline="") as f:
        f.write(RAW_HEADER + "\n")
        for r in rows:
            f.write(f"{r[0] - t0}," + ",".join(r[5][1:]) + "\n")


# ============================================================================= boards
class SerialBoard:
    """Serial link. A background thread puts every received line on self.q."""

    def __init__(self, port, baud):
        import serial
        self.serial = serial
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.q = queue.Queue()
        self.dead = False
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        buf = b""
        while True:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except (self.serial.SerialException, OSError, TypeError):
                self.dead = True
                return
            if chunk:
                buf += chunk
                *lines, buf = buf.split(b"\n")
                for raw in lines:
                    s = raw.decode("ascii", errors="replace").strip()
                    if s:
                        self.q.put(s)

    def send(self, text):
        try:
            self.ser.write((text + "\n").encode("ascii"))
            self.ser.flush()
        except (self.serial.SerialException, OSError):
            self.dead = True

    def close(self):
        self.send("led 0")
        self.send("stop")
        time.sleep(0.2)
        try:
            self.ser.close()
        except Exception:
            pass


class DemoBoard:
    """Pretend board for trying the program: a tilted board in an elevator that makes
    a ride every ~20 s (--demo)."""

    def __init__(self, speed=1.0):
        self.q = queue.Queue()
        self.dead = False
        self.speed = speed
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def send(self, text):
        pass

    def close(self):
        self.running = False

    @staticmethod
    def _accel(tr, floors):
        """Upward cabin acceleration tr seconds into a ride of the given floors."""
        a_max, ramp, hold = 0.8, 0.6, 0.9
        pulse = 2 * ramp + hold
        cruise = max(abs(floors) * 3.6 / (a_max * (ramp + hold)) - pulse, 0.0)

        def shape(x):
            if x < 0 or x > pulse:
                return 0.0
            if x < ramp:
                return x / ramp
            if x < ramp + hold:
                return 1.0
            return (pulse - x) / ramp
        s = 1 if floors > 0 else -1
        return s * a_max * (shape(tr) - shape(tr - pulse - cruise)), 2 * pulse + cruise

    def _run(self):
        rnd = random.Random(1)
        up = (0.10, -0.17, 0.98)                    # gravity direction in the board frame (tilted)
        plan = [1, -1, 3, -2, 2, -3]
        n = v = h = 0.0
        ride_t0, ride_no, p_out, seq = 5.0, 0, 101325, 0
        wall0 = time.time()
        while self.running:
            t = n / 100.0
            a, length = self._accel(t - ride_t0, plan[ride_no % len(plan)])
            if t - ride_t0 > length:
                ride_t0, ride_no = t + 9.0, ride_no + 1
            if abs(v) > 0.01 or abs(a) > 0.01:
                a += 0.03 * math.sin(2 * math.pi * 12.5 * t)   # cabin vibration while moving
            v += a * 0.01
            h += v * 0.01
            f = 1.0 + a / G
            acc = [f * u + rnd.gauss(0, 0.0012) for u in up]
            if int(n) % 5 == 0:
                p_out, seq = int(101325 - PA_PER_M * h + rnd.gauss(0, 1.0)), seq + 1
            self.q.put(f"{int(t * 1e6)},{acc[0]:.4f},{acc[1]:.4f},{acc[2]:.4f},{p_out},22.40,35.00,{seq}")
            n += 1
            wait = wall0 + t / self.speed - time.time()
            if wait > 0:
                time.sleep(wait)


def find_port(requested):
    from serial.tools import list_ports
    if requested:
        return requested
    ports = [p for p in list_ports.comports() if "bluetooth" not in (p.description or "").lower()]
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        sys.exit("No serial ports found. Plug the board in, or pass --port.")
    print("Several serial ports found:")
    for i, p in enumerate(ports, 1):
        print(f"  {i}. {p.device}  ({p.description})")
    while True:
        s = input("Which one is the board? ").strip()
        if s.isdigit() and 1 <= int(s) <= len(ports):
            return ports[int(s) - 1].device


def connect(port, baud):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is missing. Install it with:  pip install pyserial")
    port = find_port(port)
    print(f"Connecting to {port} ...")
    try:
        board = SerialBoard(port, baud)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}\n(Is the Arduino Serial Monitor still open?)")
    for _ in range(6):                      # opening the port can reset the board: keep asking
        board.send("stop")
        board.send("id")
        end = time.time() + 1.5
        while time.time() < end:
            try:
                s = board.q.get(timeout=0.1)
            except queue.Empty:
                continue
            if s.startswith("# ELEVLOG"):
                print("Board found:", s[2:])
                if "bma400=0" in s:
                    sys.exit("The accelerometer was not detected at boot - check the board.")
                if "bme680=0" in s:
                    print("!! Pressure sensor not detected: rides will be checked by acceleration only.")
                board.send("start")
                return board
    sys.exit("The board did not answer. Check that ElevatorLogger.ino is uploaded and the port/baud are right.")


# ====================================================================== metadata file
def load_metadata(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return [dict(r) for r in csv.DictReader(f)]


def save_metadata(path, rides):
    tmp = path + ".tmp"
    with open(tmp, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=META_HEADER, extrasaction="ignore", restval="")
        w.writeheader()
        for r in rides:
            if r.get("_state", "done") == "done":
                w.writerow(r)
    os.replace(tmp, path)


def floors_between(a, b):
    try:
        return str(abs(int(b) - int(a)))
    except (TypeError, ValueError):
        return ""


# ================================================================================ GUI
class App:
    COLS = [("ride_id", "Ride", 90), ("time_start", "Time", 65), ("building", "Building", 80),
            ("car", "Car", 45), ("direction", "Dir", 50), ("start_floor", "From", 45),
            ("end_floor", "To", 45), ("n_people", "People", 50), ("orientation", "Orientation", 150),
            ("duration_s", "Time s", 55), ("est_height_m", "Height m", 65), ("discard", "Discard", 55),
            ("notes", "Notes", 160)]

    def __init__(self, root, board, out_dir, tag, rate=100):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk = tk, ttk
        self.root, self.board, self.out, self.tag, self.rate = root, board, out_dir, tag, rate
        self.det = RideDetector(rate)
        self.meta_path = os.path.join(out_dir, "rides_metadata.csv")
        self.rides = load_metadata(self.meta_path)
        nums = [int(r["ride_id"].split("_")[-1]) for r in self.rides
                if r.get("ride_id", "").split("_")[-1].isdigit()
                and (not tag or r["ride_id"].startswith(tag + "_"))]
        self.next_num = max(nums, default=0) + 1
        last = self.rides[-1] if self.rides else {}
        self.defaults = {"building": last.get("building", ""), "car": last.get("car", ""),
                         "orientation": last.get("orientation", ""), "n_people": last.get("n_people", ""),
                         "start_floor": last.get("end_floor", ""), "end_floor": ""}
        self.latest = None      # newest ride of this session
        self.cur = None         # ride in progress or waiting for its post-roll
        self.sel = None         # older ride picked in the table for editing
        self.loading = False
        self.save_job = None
        self.msg = ""           # last result, shown in the banner while idle
        self.t_now = 0.0
        self.ride_t0 = 0.0
        self.prev_t_us = None
        self.gaps = self.bad = self.n_rows = 0
        self.rate_count, self.rate_time, self.rate_now = 0, time.time(), 0.0
        self.plot = collections.deque(maxlen=1200)   # (t, activity) at 20 Hz = last 60 s
        self.spans = []                               # [t_start, t_stop or None] of rides

        os.makedirs(os.path.join(out_dir, "raw"), exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_name = f"session_{stamp}{'_' + tag if tag else ''}.csv"
        self.raw = open(os.path.join(out_dir, "raw", self.session_name), "w", newline="")
        self.raw.write(RAW_HEADER + "\n")

        self._build()
        self._load_form()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(40, self._poll)
        root.after(250, self._draw_plot)

    # --------------------------------------------------------------- window
    def _build(self):
        tk, ttk = self.tk, self.ttk
        root = self.root
        root.title(f"Elevator ride logger - raw/{self.session_name}")
        root.geometry("1040x700")
        self.banner = tk.Label(root, text="", font=("TkDefaultFont", 18, "bold"), fg="white", bg="#b36b00", pady=10)
        self.banner.pack(fill="x")
        self.stats = tk.Label(root, text="", anchor="w", padx=8)
        self.stats.pack(fill="x")

        self.vars = {f: tk.StringVar() for f in SETUP_FIELDS + RIDE_FIELDS}
        self.entries = {}
        forms = tk.Frame(root)
        forms.pack(fill="x", padx=8, pady=4)

        self.setup_box = ttk.LabelFrame(forms, text="Setup")
        self.setup_box.pack(side="left", fill="both", expand=True, padx=(0, 6))
        row = tk.Frame(self.setup_box)
        row.pack(anchor="w", padx=6, pady=4)
        tk.Label(row, text="Building", width=11, anchor="w").pack(side="left")
        for b in BUILDINGS:
            ttk.Radiobutton(row, text=b, value=b, variable=self.vars["building"]).pack(side="left", padx=3)
        self._entry(self.setup_box, "car", "Car", 10)
        self._entry(self.setup_box, "orientation", "Orientation", 30)
        tk.Label(self.setup_box, text="e.g. flat_on_floor_screen_up", fg="#666").pack(anchor="w", padx=6)

        self.ride_box = ttk.LabelFrame(forms, text="Ride")
        self.ride_box.pack(side="left", fill="both", expand=True)
        row = tk.Frame(self.ride_box)
        row.pack(anchor="w", padx=6, pady=4)
        for f, label in (("start_floor", "From floor"), ("end_floor", "To floor"), ("n_people", "People")):
            tk.Label(row, text=label).pack(side="left")
            e = ttk.Entry(row, textvariable=self.vars[f], width=6)
            e.pack(side="left", padx=(3, 12))
            self.entries[f] = e
        row = tk.Frame(self.ride_box)
        row.pack(anchor="w", padx=6, pady=2)
        tk.Label(row, text="Direction", width=11, anchor="w").pack(side="left")
        for d in ("up", "down"):
            ttk.Radiobutton(row, text=d, value=d, variable=self.vars["direction"]).pack(side="left", padx=3)
        tk.Label(row, text="(filled in automatically)", fg="#666").pack(side="left", padx=6)
        self._entry(self.ride_box, "notes", "Notes", 40)
        row = tk.Frame(self.ride_box)
        row.pack(fill="x", padx=6, pady=2)
        ttk.Checkbutton(row, text="Discard this ride (bad data)", variable=self.vars["discard"],
                        onvalue="1", offvalue="0").pack(side="left")
        self.back_btn = ttk.Button(row, text="Back to the latest ride", command=self.back_to_live)
        self.back_btn.pack(side="right")

        for f, v in self.vars.items():
            v.trace_add("write", lambda *_, f=f: self._on_edit(f))

        self.canvas = tk.Canvas(root, height=150, bg="white", highlightthickness=1, highlightbackground="#bbb")
        self.canvas.pack(fill="x", padx=8, pady=4)

        tk.Label(root, text="Rides (click one to correct it)", anchor="w", padx=8).pack(fill="x")
        frame = tk.Frame(root)
        frame.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        self.tree = ttk.Treeview(frame, columns=[c[0] for c in self.COLS], show="headings", selectmode="browse")
        for key, title, width in self.COLS:
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, anchor="w")
        sb = ttk.Scrollbar(frame, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=sb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.tree.bind("<<TreeviewSelect>>", self._on_select)
        for r in self.rides:
            if not self.tree.exists(r["ride_id"]):
                self.tree.insert("", 0, iid=r["ride_id"], values=self._values(r))

    def _entry(self, parent, field, label, width):
        row = self.tk.Frame(parent)
        row.pack(anchor="w", padx=6, pady=2)
        self.tk.Label(row, text=label, width=11, anchor="w").pack(side="left")
        e = self.ttk.Entry(row, textvariable=self.vars[field], width=width)
        e.pack(side="left")
        self.entries[field] = e

    def _values(self, r):
        vals = []
        for key, _, _ in self.COLS:
            v = r.get(key, "")
            if key == "discard":
                v = "yes" if v == "1" else ""
            if key in ("duration_s", "est_height_m") and r.get("_state", "done") != "done":
                v = "..."
            vals.append(v)
        return vals

    # ----------------------------------------------------------------- form
    def _load_form(self):
        """Show the ride picked in the table, or (live) the setup for the next ride
        plus the latest ride."""
        self.loading = True
        if self.sel is not None:
            for f in SETUP_FIELDS + RIDE_FIELDS:
                self.vars[f].set(self.sel.get(f, "") or ("0" if f == "discard" else ""))
            self.setup_box.config(text=f"Setup of ride {self.sel['ride_id']}")
            self.ride_box.config(text=f"EDITING ride {self.sel['ride_id']} (picked in the table)")
            self.back_btn.state(["!disabled"])
        else:
            active = self.cur is not None
            src = self.cur if active else self.defaults
            for f in SETUP_FIELDS:
                self.vars[f].set(src.get(f, ""))
            ride = self.latest or self.defaults
            for f in RIDE_FIELDS:
                self.vars[f].set(ride.get(f, "") or ("0" if f == "discard" else ""))
            self.setup_box.config(text="Setup - this ride and all rides after it" if active
                                  else "Setup - for the next rides")
            if self.latest is None:
                title = "First ride - fill in what you already know"
            elif self.latest.get("_state") == "moving":
                title = f"Ride {self.latest['ride_id']} - IN PROGRESS"
            else:
                title = f"Ride {self.latest['ride_id']} - finished (still editable)"
            self.ride_box.config(text=title)
            self.back_btn.state(["disabled"])
        self.loading = False

    def _on_edit(self, f):
        if self.loading:
            return
        v = self.vars[f].get().strip()
        if self.sel is not None:                      # correcting an older ride
            self.sel[f] = v
            self._touch(self.sel)
            return
        if f in SETUP_FIELDS:
            self.defaults[f] = v
            if self.cur is not None:                  # ride in progress: applies to it too
                self.cur[f] = v
                self._touch(self.cur)
            elif f == "building":
                self.defaults["start_floor"] = ""     # new building: old floor no longer applies
            return
        if f == "n_people":
            self.defaults[f] = v                      # carried over to the next ride
        if self.latest is None:                       # no ride yet: remember it for the first one
            if f in ("start_floor", "end_floor"):
                self.defaults[f] = v
            return
        self.latest[f] = v
        if f == "direction":
            self.latest["_dir_manual"] = True
        if f == "end_floor":
            self.defaults["start_floor"] = v          # the next ride starts where this one ended
        self._touch(self.latest)

    def _touch(self, r):
        r["floors_travelled"] = floors_between(r.get("start_floor"), r.get("end_floor"))
        if self.tree.exists(r["ride_id"]):
            self.tree.item(r["ride_id"], values=self._values(r))
        if r.get("_state", "done") == "done":
            if self.save_job:
                self.root.after_cancel(self.save_job)
            self.save_job = self.root.after(400, self._save)

    def _save(self):
        self.save_job = None
        save_metadata(self.meta_path, self.rides)

    def _on_select(self, _event):
        picked = self.tree.selection()
        if not picked:
            return
        rec = next((r for r in self.rides if r["ride_id"] == picked[0]), None)
        self.sel = None if rec is self.latest else rec
        self._load_form()

    def back_to_live(self):
        self.sel = None
        self.tree.selection_remove(self.tree.selection())
        self._load_form()

    # --------------------------------------------------------------- stream
    def _poll(self):
        try:
            for _ in range(3000):
                line = self.board.q.get_nowait()
                if line[0] == "#":
                    if "ERROR" in line:
                        self.msg = "Board: " + line[1:].strip()
                    continue
                if not (line[0].isdigit() or line[0] == "-"):
                    continue
                row = parse_row(line)
                if row is None:
                    self.bad += 1
                    continue
                self.raw.write(line + "\n")
                t_us = row[0]
                if self.prev_t_us is not None and t_us - self.prev_t_us > 1.5e6 / self.rate:
                    self.gaps += 1
                self.prev_t_us = t_us
                self.t_now = t_us / 1e6
                self.n_rows += 1
                self.rate_count += 1
                for kind, d in self.det.feed(*row):
                    self._on_event(kind, d)
                if self.n_rows % 5 == 0:
                    self.plot.append((self.t_now, self.det.af))
        except queue.Empty:
            pass
        now = time.time()
        if now - self.rate_time >= 1.0:
            self.rate_now = self.rate_count / (now - self.rate_time)
            self.rate_count, self.rate_time = 0, now
            self.raw.flush()
        self._update_banner()
        self.root.after(40, self._poll)

    def _on_event(self, kind, d):
        if kind == "ride_start":
            self.ride_t0 = d["t"]
            self.spans.append([d["t"], None])
            self.board.send("led 1")
            now = dt.datetime.now()
            ride_id = f"{self.tag}_{self.next_num:04d}" if self.tag else f"{self.next_num:04d}"
            rec = {"ride_id": ride_id, "file": f"ride_{ride_id}.csv",
                   "date": now.strftime("%Y-%m-%d"), "time_start": now.strftime("%H:%M:%S"),
                   "direction": d["direction"], "notes": "", "discard": "0",
                   "session_file": self.session_name, "_state": "moving"}
            for f in SETUP_FIELDS + ("n_people", "start_floor", "end_floor"):
                rec[f] = self.defaults.get(f, "")
            self.defaults["end_floor"] = ""
            self.rides.append(rec)
            self.latest = self.cur = rec
            self.tree.insert("", 0, iid=ride_id, values=self._values(rec))
            if self.sel is None:
                self._load_form()
                self.entries["end_floor"].focus_set()        # just type the destination floor
                self.entries["end_floor"].selection_range(0, "end")
        elif kind == "ride_end":
            if self.spans and self.spans[-1][1] is None:
                self.spans[-1][1] = d["t"]
            self.board.send("led 0")
            if self.cur:
                self.cur["_state"] = "saving"
                if self.sel is None:
                    self._load_form_titles_only()
        elif kind == "ride_saved":
            rec = self.cur
            self.cur = None
            if rec is None:
                return
            rows = d.pop("rows")
            write_segment(os.path.join(self.out, rec["file"]), rows)
            rec.update(d)
            if not rec.get("_dir_manual"):
                rec["direction"] = d["direction_auto"]      # the barometer has the final word
            rec["_state"] = "done"
            self.next_num += 1
            self.defaults["start_floor"] = rec.get("end_floor", "")
            self.msg = (f"Saved ride {rec['ride_id']}: {rec['direction_auto']} {abs(float(d['est_height_m'])):.1f} m "
                        f"in {d['duration_s']} s")
            self._touch(rec)
            if self.sel is None:
                self._load_form()
        elif kind in ("ride_cancel", "ride_rejected"):
            if self.spans and kind == "ride_cancel":
                self.spans.pop()
            elif self.spans:
                self.spans[-1].append("rejected")
            self.board.send("led 0")
            rec, self.cur = self.cur, None
            if rec is not None:                              # keep what was typed for the real ride
                self.rides.remove(rec)
                self.tree.delete(rec["ride_id"])
                for f in ("start_floor", "end_floor", "n_people"):
                    self.defaults[f] = rec.get(f, "")
                done = [r for r in self.rides if r.get("_state") == "moving" or r.get("session_file") == self.session_name]
                self.latest = done[-1] if done else None
                if self.sel is rec:
                    self.sel = None
                if self.sel is None:
                    self._load_form()
            self.msg = f"Ignored a movement that was not a ride ({d['why']})"
        elif kind == "calibrating":
            self.msg = ""

    def _load_form_titles_only(self):
        self.setup_box.config(text="Setup - this ride and all rides after it")
        self.ride_box.config(text=f"Ride {self.latest['ride_id']} - finished, saving...")

    def _update_banner(self):
        det = self.det
        if self.board.dead:
            text, colour = "LOST CONNECTION TO THE BOARD - check the cable and restart", "#a00000"
        elif self.n_rows == 0:
            text, colour = "Waiting for data from the board...", "#666666"
        elif det.state == "CAL":
            text, colour = "LEARNING GRAVITY - keep the board still", "#b36b00"
        elif det.state == "MOVING":
            arrow = "UP" if det.sign0 > 0 else "DOWN"
            text = f"RIDE IN PROGRESS   {arrow}   {self.t_now - self.ride_t0:.0f} s"
            colour = "#c00000"
        elif det.pending:
            text, colour = "Ride finished - recording 3 s of standstill...", "#b36b00"
        else:
            text, colour = "READY - waiting for a ride", "#1a7f37"
            if self.msg:
                text += "      |      " + self.msg
        self.banner.config(text=text, bg=colour)
        saved = sum(1 for r in self.rides if r.get("session_file") == self.session_name and r.get("_state") == "done")
        self.stats.config(text=f"{self.rate_now:5.1f} samples/s    gaps: {self.gaps}    garbled lines: {self.bad}    "
                               f"idle noise: {det.noise():.3f} m/s2    start threshold: {det.hi:.2f} m/s2    "
                               f"rides this session: {saved}")

    def _draw_plot(self):
        c = self.canvas
        c.delete("all")
        w, h = max(c.winfo_width(), 100), max(c.winfo_height(), 50)
        t1 = self.t_now
        t0 = t1 - 60.0
        peak = max([abs(a) for _, a in self.plot] + [1.0]) * 1.15
        x = lambda t: (t - t0) / 60.0 * w
        y = lambda a: h / 2 - a / peak * (h / 2 - 4)
        for span in self.spans[-12:]:
            s, e = span[0], span[1] if span[1] is not None else t1
            if e >= t0:
                c.create_rectangle(x(max(s, t0)), 0, x(e), h, outline="",
                                   fill="#e6e6e6" if len(span) > 2 else "#ffd9d9")
        c.create_line(0, y(0), w, y(0), fill="#999")
        for thr in (self.det.hi, -self.det.hi):
            c.create_line(0, y(thr), w, y(thr), fill="#d08000", dash=(4, 3))
        pts = [coord for t, a in self.plot if t >= t0 for coord in (x(t), y(a))]
        if len(pts) >= 4:
            c.create_line(*pts, fill="#0050b0", width=1.5)
        c.create_text(6, 4, anchor="nw", fill="#444",
                      text=f"Vertical acceleration, last 60 s (+ = upward, full scale {peak:.1f} m/s2). "
                           "Dashed = start threshold, red band = detected ride")
        self.root.after(250, self._draw_plot)

    def close(self):
        save_metadata(self.meta_path, self.rides)
        self.raw.close()
        self.board.close()
        self.root.destroy()


# ============================================================================= replay
def replay(path, out_dir, rate):
    """Run the detector over a saved raw stream and cut it into rides again."""
    det = RideDetector(rate)
    name = os.path.splitext(os.path.basename(path))[0]
    dest = os.path.join(out_dir, "replay_" + name)
    os.makedirs(dest, exist_ok=True)
    n = 0
    print(f"{'ride':>4} {'start s':>9} {'time s':>7} {'height m':>9} {'dir':>5}")
    with open(path) as f:
        for line in f:
            row = parse_row(line.strip())
            if row is None:
                continue
            for kind, d in det.feed(*row):
                if kind == "ride_saved":
                    n += 1
                    write_segment(os.path.join(dest, f"ride_{n:03d}.csv"), d["rows"])
                    print(f"{n:>4} {d['session_t_s']:>9} {d['duration_s']:>7} {d['est_height_m']:>9} {d['direction_auto']:>5}")
                elif kind == "ride_rejected":
                    print(f"   - ignored a movement at {row[0] / 1e6:.1f} s ({d['why']})")
    print(f"{n} rides written to {dest}")


def main():
    ap = argparse.ArgumentParser(description="Record elevator rides automatically, with metadata.")
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="elevator_data", help="output folder (default: elevator_data)")
    ap.add_argument("--tag", default="", help="short name put in front of ride IDs, e.g. your name")
    ap.add_argument("--rate", type=int, default=100, help="accelerometer rows per second (ACCEL_HZ in the firmware)")
    ap.add_argument("--demo", action="store_true", help="simulated board, to try the program")
    ap.add_argument("--replay", metavar="RAW_CSV", help="re-cut a saved raw session file into rides")
    args = ap.parse_args()
    tag = "".join(c for c in args.tag if c.isalnum())
    os.makedirs(args.out, exist_ok=True)

    if args.replay:
        replay(args.replay, args.out, args.rate)
        return
    try:
        import tkinter as tk
    except ImportError:
        sys.exit("tkinter is missing. On Linux: sudo apt install python3-tk")
    board = DemoBoard() if args.demo else connect(args.port, args.baud)
    root = tk.Tk()
    App(root, board, args.out, tag, args.rate)
    print(f"Saving to: {os.path.abspath(args.out)}")
    root.mainloop()


if __name__ == "__main__":
    main()
