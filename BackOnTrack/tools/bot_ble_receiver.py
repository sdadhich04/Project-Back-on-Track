#!/usr/bin/env python3
"""
bot_ble_receiver.py — BLE receiver for the "Back on Track" posture monitor.

Connects to the ESP32-S3 firmware over BLE, subscribes to the sensor
characteristic, decodes each 72-byte bot_ble_packet_t, prints a live readout,
and logs every packet to CSV for offline signal processing (MATLAB/Python).

Wire format (must match ble_telemetry.h, little-endian, __attribute__((packed))):
    uint32  timestamp_ms
    float   upper_pitch, upper_roll, upper_yaw
    float   upper_q_i, upper_q_j, upper_q_k, upper_q_r
    float   lower_pitch, lower_roll, lower_yaw
    float   lower_q_i, lower_q_j, lower_q_k, lower_q_r
    float   emg_filtered, emg_voltage_mv
    uint8   status_flags   (bit0 upper, bit1 lower, bit2 emg, bit3 calibrated)
    uint8   seq            (wraps 0..255; used for drop detection)
    uint16  emg_raw_adc
    => 72 bytes total

Requires:  pip install bleak
Usage:
    python bot_ble_receiver.py                  # connect, print + log to CSV
    python bot_ble_receiver.py --csv run1.csv   # choose CSV path
    python bot_ble_receiver.py --calibrate      # send CALIBRATE on connect
    python bot_ble_receiver.py --quiet          # CSV only, no per-packet print
    python bot_ble_receiver.py --duration 30    # auto-stop after 30 s
Stop anytime with Ctrl+C — a summary (packets, drops, rate) is printed.
"""

import argparse
import asyncio
import csv
import struct
import sys
import time
from datetime import datetime

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("bleak is not installed. Run:  pip install bleak")

# ----- GATT identifiers (computed from ble_telemetry.c make_uuid128) -----
DEVICE_NAME = "BackOnTrack"
SENSOR_CHAR_UUID  = "12341234-5678-1234-1234-12347856ab01"  # notify + read
CONTROL_CHAR_UUID = "12341234-5678-1234-1234-12347856ab02"  # write

# Control commands (bot_ble_cmd_t)
CMD_START_CALIBRATE = 0x01
CMD_RESET_SESSION   = 0x02
CMD_PING            = 0xFF

PACKET_FMT  = "<I7f7f2fBBH"
PACKET_SIZE = struct.calcsize(PACKET_FMT)   # 72

CSV_HEADER = [
    "host_time_s", "timestamp_ms", "seq",
    "upper_pitch", "upper_roll", "upper_yaw",
    "upper_q_i", "upper_q_j", "upper_q_k", "upper_q_r",
    "lower_pitch", "lower_roll", "lower_yaw",
    "lower_q_i", "lower_q_j", "lower_q_k", "lower_q_r",
    "emg_filtered", "emg_voltage_mv", "emg_raw_adc",
    "upper_valid", "lower_valid", "emg_valid", "calibrated",
]


def decode(data: bytes) -> dict:
    """Unpack one bot_ble_packet_t into a dict."""
    if len(data) != PACKET_SIZE:
        raise ValueError(f"expected {PACKET_SIZE} bytes, got {len(data)}")
    v = struct.unpack(PACKET_FMT, data)
    flags = v[17]
    return {
        "timestamp_ms": v[0],
        "upper_pitch": v[1], "upper_roll": v[2], "upper_yaw": v[3],
        "upper_q_i": v[4], "upper_q_j": v[5], "upper_q_k": v[6], "upper_q_r": v[7],
        "lower_pitch": v[8], "lower_roll": v[9], "lower_yaw": v[10],
        "lower_q_i": v[11], "lower_q_j": v[12], "lower_q_k": v[13], "lower_q_r": v[14],
        "emg_filtered": v[15], "emg_voltage_mv": v[16],
        "status_flags": flags, "seq": v[18], "emg_raw_adc": v[19],
        "upper_valid": bool(flags & 0x01),
        "lower_valid": bool(flags & 0x02),
        "emg_valid":   bool(flags & 0x04),
        "calibrated":  bool(flags & 0x08),
    }


class Receiver:
    def __init__(self, writer, quiet):
        self.writer = writer
        self.quiet = quiet
        self.count = 0
        self.dropped = 0
        self.last_seq = None
        self.t0 = None

    def on_notify(self, _sender, data: bytearray):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        try:
            pkt = decode(bytes(data))
        except ValueError as e:
            print(f"\n[warn] bad packet: {e}")
            return

        # Dropped-packet detection via the 8-bit sequence counter.
        if self.last_seq is not None:
            gap = (pkt["seq"] - self.last_seq - 1) & 0xFF
            if gap:
                self.dropped += gap
        self.last_seq = pkt["seq"]
        self.count += 1

        self.writer.writerow([
            round(now - self.t0, 4), pkt["timestamp_ms"], pkt["seq"],
            pkt["upper_pitch"], pkt["upper_roll"], pkt["upper_yaw"],
            pkt["upper_q_i"], pkt["upper_q_j"], pkt["upper_q_k"], pkt["upper_q_r"],
            pkt["lower_pitch"], pkt["lower_roll"], pkt["lower_yaw"],
            pkt["lower_q_i"], pkt["lower_q_j"], pkt["lower_q_k"], pkt["lower_q_r"],
            pkt["emg_filtered"], pkt["emg_voltage_mv"], pkt["emg_raw_adc"],
            int(pkt["upper_valid"]), int(pkt["lower_valid"]),
            int(pkt["emg_valid"]), int(pkt["calibrated"]),
        ])

        if not self.quiet:
            cal = "CAL" if pkt["calibrated"] else "---"
            up = "U" if pkt["upper_valid"] else "-"
            lo = "L" if pkt["lower_valid"] else "-"
            em = "E" if pkt["emg_valid"] else "-"
            sys.stdout.write(
                f"\r#{self.count:6d} seq={pkt['seq']:3d} drop={self.dropped:4d} "
                f"[{up}{lo}{em} {cal}] "
                f"UP p/r/y={pkt['upper_pitch']:7.2f}/{pkt['upper_roll']:7.2f}/{pkt['upper_yaw']:7.2f} "
                f"EMG={pkt['emg_filtered']:6.1f} ({pkt['emg_raw_adc']:4d})  "
            )
            sys.stdout.flush()


async def find_device(name, timeout):
    print(f"Scanning for \"{name}\" ({timeout:.0f}s)...")
    dev = await BleakScanner.find_device_by_name(name, timeout=timeout)
    if dev is None:
        # Fallback: scan all and match name case-insensitively
        for d in await BleakScanner.discover(timeout=timeout):
            if (d.name or "").lower() == name.lower():
                return d
    return dev


async def run(args):
    dev = await find_device(args.name, args.scan_timeout)
    if dev is None:
        sys.exit(f"Device \"{args.name}\" not found. Is it powered and advertising?")
    print(f"Found {dev.name} [{dev.address}] — connecting...")

    async with BleakClient(dev) as client:
        # bleak/OS negotiates the MTU; firmware prefers 100.
        try:
            print(f"Connected. Negotiated MTU = {client.mtu_size} bytes")
        except Exception:
            print("Connected.")

        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(CSV_HEADER)
        rx = Receiver(writer, args.quiet)

        await client.start_notify(SENSOR_CHAR_UUID, rx.on_notify)
        print(f"Subscribed. Logging to {args.csv}. Ctrl+C to stop.\n")

        if args.calibrate:
            await asyncio.sleep(0.3)
            await client.write_gatt_char(CONTROL_CHAR_UUID,
                                         bytes([CMD_START_CALIBRATE]), response=False)
            print("[sent] START_CALIBRATE")
        if args.reset:
            await client.write_gatt_char(CONTROL_CHAR_UUID,
                                         bytes([CMD_RESET_SESSION]), response=False)
            print("[sent] RESET_SESSION")

        start = time.monotonic()
        try:
            while True:
                await asyncio.sleep(0.2)
                if args.duration and (time.monotonic() - start) >= args.duration:
                    break
                if not client.is_connected:
                    print("\n[warn] disconnected by peer")
                    break
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            try:
                await client.stop_notify(SENSOR_CHAR_UUID)
            except Exception:
                pass
            csv_file.close()

        elapsed = max(time.monotonic() - (rx.t0 or start), 1e-6)
        total = rx.count + rx.dropped
        loss = (100.0 * rx.dropped / total) if total else 0.0
        print(f"\n\nStopped. {rx.count} packets in {elapsed:.1f}s "
              f"(~{rx.count/elapsed:.1f} Hz), {rx.dropped} dropped ({loss:.1f}%).")
        print(f"CSV saved: {args.csv}")


def main():
    ap = argparse.ArgumentParser(description="Back on Track BLE receiver")
    ap.add_argument("--name", default=DEVICE_NAME, help="BLE device name")
    ap.add_argument("--csv", default=None, help="CSV output path")
    ap.add_argument("--scan-timeout", type=float, default=10.0)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="auto-stop after N seconds (0 = run until Ctrl+C)")
    ap.add_argument("--calibrate", action="store_true",
                    help="send START_CALIBRATE after connecting")
    ap.add_argument("--reset", action="store_true",
                    help="send RESET_SESSION after connecting")
    ap.add_argument("--quiet", action="store_true", help="CSV only, no console readout")
    args = ap.parse_args()

    if args.csv is None:
        args.csv = f"bot_{datetime.now():%Y%m%d_%H%M%S}.csv"

    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
