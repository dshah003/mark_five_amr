#!/usr/bin/env python3
"""
test_serial.py — Interactive serial test for Robot_Node_BTS7960 firmware.

Tests the exact same protocol as serial_bridge.py without needing ROS2.

Usage:
  python3 arduino_ws/test_serial.py
  python3 arduino_ws/test_serial.py --port /dev/ttyACM1

Commands (type at the prompt):
  f          Forward  (linear=0.2, angular=0)
  b          Backward (linear=-0.2, angular=0)
  l          Turn left  (linear=0, angular=0.5)
  r          Turn right (linear=0, angular=-0.5)
  s          Stop (send zero velocity)
  v L A      Send custom velocity: L=linear m/s, A=angular rad/s
               e.g.  v 0.15 0.3
  t          Print last received tick counts
  q          Quit
"""

import argparse
import sys
import threading
import time
import serial

# ── configuration ──────────────────────────────────────────────────────────────
DEFAULT_PORT = "/dev/ttyACM0"
BAUD = 115200
TIMEOUT = 0.1

# ── shared state ───────────────────────────────────────────────────────────────
last_left_ticks = 0
last_right_ticks = 0
tick_lock = threading.Lock()


def reader_thread(ser: serial.Serial, stop_event: threading.Event):
    """Background thread: read tick messages from Arduino and print them."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line.startswith("t,"):
                    parts = line.split(",")
                    if len(parts) == 3:
                        global last_left_ticks, last_right_ticks
                        with tick_lock:
                            last_left_ticks = int(parts[1])
                            last_right_ticks = int(parts[2])
                        print(f"\r[ticks] left={last_left_ticks:6d}  right={last_right_ticks:6d}", end="", flush=True)
                elif line:
                    print(f"\r[arduino] {line}")
        except (serial.SerialException, ValueError):
            break
        time.sleep(0.01)


def send_velocity(ser: serial.Serial, linear: float, angular: float):
    cmd = f"v,{linear:.4f},{angular:.4f}\n"
    ser.write(cmd.encode("utf-8"))
    print(f"\r[sent]   {cmd.strip()}", flush=True)


def main():
    parser = argparse.ArgumentParser(description="BTS7960 serial test (no ROS2 needed)")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    args = parser.parse_args()

    print(f"Connecting to {args.port} at {BAUD} baud...")
    try:
        ser = serial.Serial(args.port, BAUD, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        print("Is the Arduino plugged in? Try: ls /dev/ttyACM*")
        sys.exit(1)

    time.sleep(2)  # Give Arduino time to reset after opening serial
    print(f"Connected. Waiting for tick data...")
    print(__doc__)

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    try:
        while True:
            try:
                cmd = input("\ncmd> ").strip().lower()
            except EOFError:
                break

            if cmd == "q":
                break
            elif cmd == "f":
                send_velocity(ser, 0.2, 0.0)
            elif cmd == "b":
                send_velocity(ser, -0.2, 0.0)
            elif cmd == "l":
                send_velocity(ser, 0.0, 0.5)
            elif cmd == "r":
                send_velocity(ser, 0.0, -0.5)
            elif cmd == "s":
                send_velocity(ser, 0.0, 0.0)
            elif cmd.startswith("v "):
                parts = cmd.split()
                if len(parts) == 3:
                    try:
                        send_velocity(ser, float(parts[1]), float(parts[2]))
                    except ValueError:
                        print("Usage: v <linear> <angular>  e.g. v 0.15 0.3")
                else:
                    print("Usage: v <linear> <angular>  e.g. v 0.15 0.3")
            elif cmd == "t":
                with tick_lock:
                    print(f"left={last_left_ticks}  right={last_right_ticks}")
            elif cmd == "":
                pass
            else:
                print("Unknown command. Type q to quit.")
    finally:
        print("\nStopping motors...")
        try:
            send_velocity(ser, 0.0, 0.0)
            time.sleep(0.2)
        except Exception:
            pass
        stop_event.set()
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
