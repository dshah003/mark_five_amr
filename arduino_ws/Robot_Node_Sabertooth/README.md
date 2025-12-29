# Robot_Node_Sabertooth

Arduino firmware for Mark Five AMR using **Sabertooth 2x12 v1.00** motor driver.

This is an alternative to the original `Robot_Node.ino` which uses the L293DNE H-Bridge.

**Communication:** Simple serial protocol with ROS2 Python bridge (no ros2arduino/micro-ROS needed)

## Why Sabertooth 2x12?

| Specification | Sabertooth 2x12 | L293DNE |
|---------------|-----------------|---------|
| Voltage Drop  | <0.5V (MOSFET)  | ~3-4V (Darlington) |
| Current (cont)| 12A per channel | 0.6A per channel |
| Current (peak)| 25A per channel | 1.2A per channel |
| Protection    | Thermal, overcurrent, undervoltage | Thermal only |

## Requirements

### Hardware
- Arduino Mega 2560
- Sabertooth 2x12 motor driver (tested with v1.00)
- 2x Hall encoder DC geared motors (25GA370 or similar, 12V)
- 12V power supply (3A+ recommended)

### Software
- **Sabertooth Arduino Library** by Dimension Engineering
  - Install via: Sketch -> Include Library -> Manage Libraries -> Search "Sabertooth"
  - Or download from: https://www.dimensionengineering.com/info/arduino

**Note:** ros2arduino and micro-ROS are NOT needed. The simple serial protocol is more reliable on Arduino Mega's limited 8KB RAM.

## Serial Protocol

The Arduino uses a lightweight text-based serial protocol:

```
Arduino TX (publishes):  "t,<left_ticks>,<right_ticks>\n"
Arduino RX (receives):   "v,<linear_x>,<angular_z>\n"
```

The ROS2 `serial_bridge.py` node translates this to ROS2 topics.

## DIP Switch Configuration

**Packetized Serial Mode, Address 128:**

```
Switch: 1   2   3   4   5   6
        OFF OFF ON  ON  ON  ON
```

Use the DIP Wizard for other configurations:
http://www.dimensionengineering.com/datasheets/SabertoothDIPWizard/start.htm

## Wiring Diagram

```
+-------------+                     +-----------------+
| Arduino     |                     |  Sabertooth 2x12|
| Mega 2560   |                     |                 |
|             |                     |  S1    S2       |
|  Pin 18 TX1 +-------------------->|  S1             |
|             |                     |                 |
|         GND +-------------------->|  0V             |
+-------------+                     |                 |
                                    |  B+    B-       |
      +12V DC Supply +------------->|  B+             |
      (3A+ recommended)             |                 |
                   GND +----------->|  B-             |
                                    |                 |
                                    |  M1A  M1B       |
      Left Motor <------------------|  M1A  M1B       |
                                    |                 |
                                    |  M2A  M2B       |
      Right Motor <-----------------|  M2A  M2B       |
                                    +-----------------+
```

## Pin Connections

### Arduino Mega to Sabertooth

| Arduino Pin | Sabertooth | Description |
|-------------|------------|-------------|
| Pin 18 (TX1)| S1         | Serial TX (commands) |
| GND         | 0V         | Common ground |

### Arduino Mega to Encoders

| Arduino Pin | Encoder | Description |
|-------------|---------|-------------|
| Pin 21 (INT0) | Left Encoder A  | Interrupt - tick counting |
| Pin 20        | Left Encoder B  | Direction sensing |
| Pin 3 (INT1)  | Right Encoder A | Interrupt - tick counting |
| Pin 2         | Right Encoder B | Direction sensing |

### Motor Connections

| Sabertooth | Motor |
|------------|-------|
| M1A, M1B   | Left Motor terminals |
| M2A, M2B   | Right Motor terminals |

**Note:** If motors spin in wrong direction, swap M1A/M1B or M2A/M2B, or swap motor(1)/motor(2) in code.

## ROS2 Topics (via serial_bridge.py)

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/left_ticks` | std_msgs/Int16 | Left wheel encoder ticks |
| `/right_ticks` | std_msgs/Int16 | Right wheel encoder ticks |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |

## Installation

1. **Install Sabertooth library** in Arduino IDE:
   - Sketch -> Include Library -> Manage Libraries
   - Search "Sabertooth" -> Install

2. **Set DIP switches**: `OFF OFF ON ON ON ON`

3. **Wire connections** per the diagram above

4. **Upload firmware** to Arduino Mega:
   - Board: Arduino Mega 2560
   - Port: /dev/ttyACM0

5. **Launch ROS2 robot**:
   ```bash
   ros2 launch mark_five_bot bringup.launch.py
   ```
   This automatically starts the serial_bridge node.

6. **Test with ROS2**:
   ```bash
   # Terminal 1 - Launch robot (already running)

   # Terminal 2 - Monitor encoder ticks
   ros2 topic echo /left_ticks
   ros2 topic echo /right_ticks

   # Terminal 3 - Keyboard teleop
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

## Troubleshooting

### Motors don't move

1. Check DIP switch settings: `OFF OFF ON ON ON ON`
2. Verify power supply connected to B+/B-
3. Check S1 connected to Arduino Pin 18
4. Ensure common ground between Arduino and Sabertooth
5. Verify Sabertooth library is installed
6. Check serial_bridge node is running: `ros2 node list`

### No topics appearing

1. Check serial_bridge is running:
   ```bash
   ros2 node list | grep serial_bridge
   ```
2. Check Arduino is connected to correct port (`/dev/ttyACM0`)
3. Verify baud rate matches (115200)
4. Test serial directly:
   ```bash
   cat /dev/ttyACM0
   # Should see: t,0,0 (tick messages)
   ```

### Motors spin wrong direction

Swap motor wire connections (M1A/M1B or M2A/M2B), or in the code swap:
```cpp
ST.motor(1, leftWithSign);   // Try swapping 1 and 2
ST.motor(2, rightWithSign);
```

### Robot drifts to one side

Adjust the `DRIFT_MULTIPLIER` constant in the code (default: 120).

## Robot Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| WHEEL_RADIUS | 0.055 m | Wheel radius |
| WHEEL_BASE | 0.14 m | Distance between wheels |
| TICKS_PER_REVOLUTION | 540 | Encoder ticks per wheel revolution |
| TICKS_PER_METER | 3125 | Encoder ticks per meter traveled |

## Power Supply Requirements

| Specification | Recommended |
|---------------|-------------|
| Voltage | 12-14V DC |
| Current | 3A minimum, 5A preferred |
| Type | Regulated DC supply or 3S LiPo battery |

**Important:** Do not use a 5V->12V boost converter. The Sabertooth requires direct 12V with adequate current capacity.

## Files

- `Robot_Node_Sabertooth.ino` - Main Arduino firmware
- `README.md` - This file

## Test Code

For standalone motor testing (without ROS2), see:
- `arduino_ws/test_sabertooth/test_sabertooth.ino` - Full test sequence
- `arduino_ws/test_sabertooth_minimal/test_sabertooth_minimal.ino` - Minimal test

## Migration from ROS1

The ROS2 version uses a **simple serial protocol** instead of rosserial. Key changes:
- No ROS libraries needed on Arduino (just Sabertooth library)
- ROS2 `serial_bridge.py` node handles translation
- Much lower memory footprint on Arduino
- More reliable than ros2arduino/micro-ROS on Arduino Mega
