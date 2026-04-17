"""
theseus_read.py  —  Theseus ground terminal reader
===================================================
Reads THS packets from the LoRa ground module and displays
a live updating terminal dashboard. No Synnax, no GUI required.

Usage:
    python theseus_read.py --port COM8 --baud 9600
    python theseus_read.py --port COM8 --baud 9600 --log

With --log, saves every packet to a timestamped CSV in the current folder.

Packet format:
    THS,<alt_m>,<press_pa>,<temp_c>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\r\n
"""

import argparse
import csv
import os
import sys
import time
from datetime import datetime

import serial


FIELDS = ["alt_m", "press_pa", "temp_c", "ax", "ay", "az", "gx", "gy", "gz"]


# ---------------------------------------------------------------------------
# Terminal helpers
# ---------------------------------------------------------------------------
def clear():
    os.system('cls' if os.name == 'nt' else 'clear')


def bar(val, mn, mx, width=20, fill='█', empty='░'):
    if mx == mn:
        frac = 0.0
    else:
        frac = max(0.0, min(1.0, (val - mn) / (mx - mn)))
    filled = int(frac * width)
    return fill * filled + empty * (width - filled)


def color(text, code):
    """ANSI color. Works on Windows 10+ terminal and all Unix."""
    return f"\033[{code}m{text}\033[0m"


def temp_color(tc):
    if tc < 40:   return color(f"{tc:.1f} C", "92")   # green
    elif tc < 60: return color(f"{tc:.1f} C", "93")   # yellow
    else:         return color(f"{tc:.1f} C", "91")   # red


def az_color(az):
    if abs(az) > 3.0: return color(f"{az:+.3f} g", "91")   # red — high G event
    elif abs(az) > 2.0: return color(f"{az:+.3f} g", "93") # yellow
    else: return color(f"{az:+.3f} g", "97")                # white


# ---------------------------------------------------------------------------
# Packet parser — same robust buffer approach as visualizer
# ---------------------------------------------------------------------------
def parse_packets(buf):
    """Extract all complete THS packets from buffer. Returns (packets, remaining_buf)."""
    packets = []
    while 'THS,' in buf:
        s    = buf.find('THS,')
        rest = buf[s + 4:]
        nxt  = rest.find('THS,')

        if nxt == -1:
            end = -1
            for d in ['\r\n', '\n', '\r']:
                i = rest.find(d)
                if i != -1 and (end == -1 or i < end):
                    end = i
            if end == -1:
                break
            chunk = rest[:end].strip()
            buf   = rest[end:]
        else:
            chunk = rest[:nxt].strip().rstrip(',\r\n')
            buf   = rest[nxt:]

        parts = chunk.split(',')
        if len(parts) == 9:
            try:
                packets.append([float(x) for x in parts])
            except ValueError:
                pass

    return packets, buf


# ---------------------------------------------------------------------------
# Stats tracker
# ---------------------------------------------------------------------------
class Stats:
    def __init__(self):
        self.count     = 0
        self.start     = time.time()
        self.alt_max   = -9999.0
        self.alt_min   =  9999.0
        self.az_max    = -9999.0
        self.temp_max  = -9999.0
        self.last_time = time.time()
        self.hz        = 0.0

    def update(self, f):
        now = time.time()
        dt  = now - self.last_time
        if dt > 0:
            self.hz = 0.9 * self.hz + 0.1 * (1.0 / dt)
        self.last_time = now
        self.count    += 1
        if f[0] > self.alt_max:  self.alt_max  = f[0]
        if f[0] < self.alt_min:  self.alt_min  = f[0]
        if f[5] > self.az_max:   self.az_max   = f[5]
        if f[2] > self.temp_max: self.temp_max = f[2]

    def elapsed(self):
        return time.time() - self.start


# ---------------------------------------------------------------------------
# Display
# ---------------------------------------------------------------------------
def render(f, stats, link_age, log_path):
    alt, press, temp = f[0], f[1], f[2]
    ax,  ay,  az     = f[3], f[4], f[5]
    gx,  gy,  gz     = f[6], f[7], f[8]

    link_str = color("LINK OK", "92") if link_age < 1.5 else color(f"LINK LOST {link_age:.1f}s", "91")
    hz_str   = f"{stats.hz:.1f} Hz" if stats.hz > 0 else "---"

    print(f"\n{'='*55}")
    print(f"  THESEUS TELEMETRY          {link_str}   {hz_str}")
    print(f"  packets: {stats.count}   elapsed: {stats.elapsed():.0f}s")
    if log_path:
        print(f"  logging: {os.path.basename(log_path)}")
    print(f"{'='*55}")

    # altitude
    alt_bar = bar(alt, stats.alt_min if stats.alt_min < 9999 else 0, max(stats.alt_max, alt + 1))
    print(f"\n  ALTITUDE    {color(f'{alt:.2f} m', '96'):>18}   max: {stats.alt_max:.1f} m")
    print(f"  [{alt_bar}]")

    # pressure
    print(f"\n  PRESSURE    {color(f'{press:.1f} Pa', '97'):>18}")

    # temperature
    print(f"\n  BAY TEMP    {temp_color(temp):>18}   max: {stats.temp_max:.1f} C")

    print(f"\n  {'─'*50}")

    # acceleration
    print(f"  ACCEL       X {ax:+7.3f} g   Y {ay:+7.3f} g")
    print(f"              Z {az_color(az):>18}   max: {stats.az_max:.2f} g")

    # gyro
    print(f"\n  GYRO        X {gx:+7.2f} °/s")
    print(f"              Y {gy:+7.2f} °/s")
    print(f"              Z {gz:+7.2f} °/s")

    print(f"\n  {'─'*50}")
    print(f"  Ctrl+C to stop\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Theseus ground terminal reader")
    parser.add_argument('--port',  default='COM8')
    parser.add_argument('--baud',  type=int, default=9600)
    parser.add_argument('--log',   action='store_true', help='Save CSV log')
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baud} baud...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    print("Waiting for THS packets... (Ctrl+C to stop)\n")

    log_file   = None
    log_writer = None
    log_path   = None

    if args.log:
        ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = f"theseus_{ts}.csv"
        log_file = open(log_path, 'w', newline='')
        log_writer = csv.writer(log_file)
        log_writer.writerow(["timestamp_s"] + FIELDS)
        print(f"Logging to {log_path}")

    stats       = Stats()
    buf         = ""
    last_packet = 0.0
    last_f      = None

    try:
        while True:
            w = ser.in_waiting
            if w > 0:
                buf += ser.read(w).decode('utf-8', errors='replace')
            else:
                time.sleep(0.005)
                continue

            packets, buf = parse_packets(buf)

            for f in packets:
                stats.update(f)
                last_packet = time.time()
                last_f      = f

                if log_writer:
                    log_writer.writerow([f"{time.time():.3f}"] + [f"{v:.4f}" for v in f])
                    log_file.flush()

            if last_f is not None:
                link_age = time.time() - last_packet
                clear()
                render(last_f, stats, link_age, log_path)

    except KeyboardInterrupt:
        print("\nStopped.")
        if log_file:
            log_file.close()
            print(f"Log saved: {log_path}  ({stats.count} packets)")


if __name__ == '__main__':
    main()
