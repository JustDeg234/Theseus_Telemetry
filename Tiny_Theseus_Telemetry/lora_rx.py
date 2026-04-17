"""
lora_rx.py  —  Theseus LoRa ground receiver -> Synnax
=====================================================
Reads THS packets from the ground LoRa serial port and streams
all 9 telemetry channels into the existing Theseus Synnax instance.

Usage:
    python lora_rx.py --port COM8 --baud 9600
    python lora_rx.py --setup        # create channels only, then exit

Run alongside BLTDAQ backend:
    Terminal 1: synnax start --listen localhost:9091 --insecure
    Terminal 2: python backend/src/main.py live_go
    Terminal 3: python lora_rx.py --port COM8 --baud 9600

Synnax channels created:
    theseus_time        (index)
    theseus_alt_m       altitude in metres
    theseus_press_pa    pressure in Pa
    theseus_temp_c      bay temperature in C
    theseus_ax/ay/az    acceleration in g
    theseus_gx/gy/gz    gyro in deg/s

Dependencies:
    pip install synnax numpy pyserial
"""

import argparse
import os
import sys
import time

import numpy as np
import serial
import synnax as sy


TIME_CH  = "theseus_time"
DATA_CHS = [
    "theseus_alt_m",
    "theseus_press_pa",
    "theseus_temp_c",
    "theseus_ax",
    "theseus_ay",
    "theseus_az",
    "theseus_gx",
    "theseus_gy",
    "theseus_gz",
]


def connect_synnax() -> sy.Synnax:
    host     = os.getenv("SYNNAX_HOST",     "localhost").strip() or "localhost"
    port     = int(os.getenv("SYNNAX_PORT", "9091"))
    username = os.getenv("SYNNAX_USERNAME", "synnax")
    password = os.getenv("SYNNAX_PASSWORD", "seldon")
    secure   = os.getenv("SYNNAX_SECURE",  "false").lower() in {"1", "true", "yes"}

    candidates = ["localhost", "127.0.0.1"] if host in {"localhost", "127.0.0.1"} else [host]
    errors = []
    for h in candidates:
        try:
            return sy.Synnax(host=h, port=port,
                             username=username, password=password,
                             secure=secure)
        except Exception as exc:
            errors.append(f"{h}:{port} -> {exc}")

    raise RuntimeError(
        "Cannot connect to Synnax.\n"
        "Start it with: synnax start --listen localhost:9091 --insecure\n"
        f"Attempts: {' | '.join(errors)}"
    )


def create_channels(client: sy.Synnax) -> None:
    time_ch = client.channels.create(
        name=TIME_CH,
        data_type=sy.DataType.TIMESTAMP,
        is_index=True,
        retrieve_if_name_exists=True,
    )
    for name in DATA_CHS:
        client.channels.create(
            name=name,
            data_type=sy.DataType.FLOAT32,
            index=time_ch.key,
            retrieve_if_name_exists=True,
        )
    print(f"[SYNNAX] Channels ready: {TIME_CH} + {len(DATA_CHS)} data channels")


def parse_packets(buf: str):
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


def run(port: str, baud: int) -> None:
    print(f"[SERIAL] Opening {port} @ {baud} baud...")
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        raise SystemExit(f"[ERROR] Cannot open {port}: {e}")

    print("[SYNNAX] Connecting...")
    client = connect_synnax()
    create_channels(client)

    good = 0
    buf  = ""

    print(f"\n[RX] Streaming to Synnax — Ctrl+C to stop\n")

    with client.open_writer(
        start=sy.TimeStamp.now(),
        channels=[TIME_CH] + DATA_CHS,
    ) as writer:
        while True:
            try:
                w = ser.in_waiting
                if w > 0:
                    buf += ser.read(w).decode('utf-8', errors='replace')
                else:
                    time.sleep(0.005)
                    continue

                packets, buf = parse_packets(buf)

                for f in packets:
                    now   = sy.TimeStamp.now()
                    frame = {TIME_CH: np.array([int(now)], dtype=np.int64)}
                    for i, ch in enumerate(DATA_CHS):
                        frame[ch] = np.array([f[i]], dtype=np.float32)
                    writer.write(frame)
                    good += 1

                    if good % 20 == 0:
                        print(
                            f"[RX] alt={f[0]:7.2f} m  "
                            f"temp={f[2]:5.1f} C  "
                            f"az={f[5]:+5.2f} g  "
                            f"total={good} packets"
                        )

            except KeyboardInterrupt:
                raise
            except Exception:
                continue


def main():
    parser = argparse.ArgumentParser(description="Theseus LoRa -> Synnax")
    parser.add_argument('--port',  default='COM8')
    parser.add_argument('--baud',  type=int, default=9600)
    parser.add_argument('--setup', action='store_true',
                        help='Create channels only then exit')
    args = parser.parse_args()

    if args.setup:
        create_channels(connect_synnax())
        print("[SYNNAX] Setup complete.")
        return

    try:
        run(args.port, args.baud)
    except KeyboardInterrupt:
        print("\n[RX] Stopped.")


if __name__ == '__main__':
    main()
