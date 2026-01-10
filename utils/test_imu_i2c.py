#!/usr/bin/env python3
"""
Standalone I2C Test for BNO085 IMU

This script tests basic I2C communication with the BNO085 sensor
without requiring ROS2. Run this first to verify hardware connection.

Hardware Connection (Jetson Nano):
  BNO085 VIN  → Pin 1  (3.3V)
  BNO085 GND  → Pin 6  (GND)
  BNO085 SDA  → Pin 3  (I2C Bus 1 SDA / GPIO2)
  BNO085 SCL  → Pin 5  (I2C Bus 1 SCL / GPIO3)

Expected I2C Address: 0x4A or 0x4B

Usage:
  sudo python3 test_imu_i2c.py
"""

import sys

try:
    import smbus2
except ImportError:
    print("Error: smbus2 library not found")
    print("Install it with: pip3 install smbus2")
    sys.exit(1)


def scan_i2c_bus(bus_number=1):
    """Scan I2C bus for connected devices"""
    print(f"\n{'='*60}")
    print(f"Scanning I2C Bus {bus_number} for BNO085 IMU...")
    print(f"{'='*60}\n")

    try:
        bus = smbus2.SMBus(bus_number)
    except FileNotFoundError:
        print(f"ERROR: I2C bus {bus_number} not found!")
        print("Make sure I2C is enabled on your Jetson Nano.")
        print("Check with: ls -l /dev/i2c-*")
        return None
    except PermissionError:
        print("ERROR: Permission denied!")
        print("Run this script with sudo: sudo python3 test_imu_i2c.py")
        return None

    devices_found = []

    print("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f")

    for row in range(0, 128, 16):
        print(f"{row:02x}: ", end="")
        for col in range(16):
            addr = row + col
            if addr < 3 or addr > 0x77:
                print("   ", end="")
                continue

            try:
                bus.read_byte(addr)
                print(f"{addr:02x} ", end="")
                devices_found.append(addr)
            except:
                print("-- ", end="")
        print()

    bus.close()

    print(f"\n{'='*60}")
    if devices_found:
        print(f"Found {len(devices_found)} device(s): {[hex(d) for d in devices_found]}")

        # Check for BNO085 addresses
        bno085_addrs = [0x4A, 0x4B]
        bno085_found = [addr for addr in devices_found if addr in bno085_addrs]

        if bno085_found:
            print(f"\n✓ SUCCESS: BNO085 IMU detected at address {hex(bno085_found[0])}")
            print("\nNext steps:")
            print("1. Test ROS2 driver: python3 test_imu_ros.py")
            print("2. Or launch robot: ros2 launch mark_five_bot robot.launch.py")
            return bno085_found[0]
        else:
            print(f"\n⚠ WARNING: Devices found, but none match BNO085 addresses (0x4A or 0x4B)")
            print("Devices found:", [hex(d) for d in devices_found])
            print("\nDouble-check your wiring:")
            print("  VIN → 3.3V (Pin 1)")
            print("  GND → GND  (Pin 6)")
            print("  SDA → SDA  (Pin 3)")
            print("  SCL → SCL  (Pin 5)")
            return None
    else:
        print("✗ FAIL: No I2C devices found!")
        print("\nTroubleshooting:")
        print("1. Check physical connections")
        print("2. Verify IMU has power (some have power LEDs)")
        print("3. Check I2C is enabled: ls -l /dev/i2c-*")
        print("4. Try a different I2C bus if available")
        return None

    print(f"{'='*60}\n")


def test_bno085_communication(bus_number=1, address=0x4A):
    """Test basic read/write with BNO085"""
    print(f"\n{'='*60}")
    print(f"Testing communication with BNO085 at address {hex(address)}...")
    print(f"{'='*60}\n")

    try:
        bus = smbus2.SMBus(bus_number)

        # Try to read a byte (this is a basic connectivity test)
        data = bus.read_byte(address)
        print(f"✓ Successfully read from device: 0x{data:02x}")

        bus.close()
        print("\n✓ Basic I2C communication working!")
        return True

    except Exception as e:
        print(f"✗ Communication failed: {e}")
        return False


if __name__ == "__main__":
    print("\n" + "="*60)
    print("BNO085 IMU I2C Connection Test")
    print("="*60)

    # Scan for devices
    addr = scan_i2c_bus(bus_number=1)

    # If BNO085 found, test communication
    if addr:
        test_bno085_communication(bus_number=1, address=addr)

    print()
