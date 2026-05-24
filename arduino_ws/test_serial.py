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


def send(ser, char, label):
    ser.write(char.encode())
    print(f"  -> {label}")


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
                send(ser, "f", "Forward")
            elif cmd == "b":
                send(ser, "b", "Backward")
            elif cmd == "l":
                send(ser, "l", "Spin left")
            elif cmd == "r":
                send(ser, "r", "Spin right")
            elif cmd == "s":
                send(ser, "s", "Stop")
            elif cmd == "":
                pass
            else:
                print("Unknown command. Use: f b l r s q")
    finally:
        print("Stopping motors...")
        send(ser, 0.0, 0.0)
        time.sleep(0.2)
        ser.close()
        print("Done.")


if __name__ == "__main__":
    main()
