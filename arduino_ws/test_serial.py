#!/usr/bin/env python3
"""
test_serial.py — Send velocity commands to Arduino over serial (no ROS2 needed).

Usage:
  python3 test_serial.py
  python3 test_serial.py --port /dev/ttyACM1

Commands:
  f    Forward
  b    Backward
  l    Turn left
  r    Turn right
  s    Stop
  v L A  Custom: linear (m/s) and angular (rad/s), e.g. v 0.15 0.3
  q    Quit
"""

import argparse
import sys
import time
import serial

DEFAULT_PORT = "/dev/ttyACM0"
BAUD = 115200


def send(ser, linear, angular):
    cmd = f"v,{linear:.4f},{angular:.4f}\n"
    ser.write(cmd.encode())
    print(f"  -> {cmd.strip()}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_PORT)
    args = parser.parse_args()

    print(f"Connecting to {args.port} ...")
    try:
        ser = serial.Serial(args.port, BAUD, timeout=0)
    except serial.SerialException as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    # Drain any buffered Arduino output so it doesn't clog the port
    time.sleep(2)
    ser.reset_input_buffer()

    print("Connected. Commands: f b l r s  |  v L A  |  q")

    try:
        while True:
            cmd = input("cmd> ").strip().lower()
            # Drain incoming bytes silently so the buffer stays clear
            ser.reset_input_buffer()

            if cmd == "q":
                break
            elif cmd == "f":
                send(ser, 0.2, 0.0)
            elif cmd == "b":
                send(ser, -0.2, 0.0)
            elif cmd == "l":
                send(ser, 0.0, 0.5)
            elif cmd == "r":
                send(ser, 0.0, -0.5)
            elif cmd == "s":
                send(ser, 0.0, 0.0)
            elif cmd.startswith("v "):
                parts = cmd.split()
                if len(parts) == 3:
                    try:
                        send(ser, float(parts[1]), float(parts[2]))
                    except ValueError:
                        print("Usage: v <linear> <angular>  e.g.  v 0.15 0.3")
                else:
                    print("Usage: v <linear> <angular>  e.g.  v 0.15 0.3")
            elif cmd == "":
                pass
            else:
                print("Unknown command.")
    finally:
        print("Stopping motors...")
        send(ser, 0.0, 0.0)
        time.sleep(0.2)
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
