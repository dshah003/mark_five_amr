# Robot_Node_BTS7960

Arduino firmware for Mark Five AMR using **2x BTS7960 (IBT-2)** H-bridge motor driver modules.

**Communication:** Simple serial protocol with ROS2 Python bridge (no library installation needed)

## Why BTS7960?

| Specification | BTS7960 (IBT-2) | Sabertooth 2x12 | L293DNE |
|---------------|-----------------|-----------------|---------|
| Voltage Drop  | <0.5V (MOSFET)  | <0.5V (MOSFET)  | ~3-4V (Darlington) |
| Current (cont)| 43A per channel | 12A per channel | 0.6A per channel |
| Current (peak)| 43A per channel | 25A per channel | 1.2A per channel |
| Control       | PWM (0-255)     | Serial library  | PWM |
| Modules needed| 2 (one per motor)| 1              | 1 |
| Library       | None (built-in) | Dimension Eng.  | None |

## Requirements

### Hardware
- Arduino Mega 2560
- 2x BTS7960 / IBT-2 motor driver modules
- 2x Hall encoder DC geared motors (12V)
- 12V power supply (5A+ recommended)

### Software
- No external library needed — uses standard Arduino `analogWrite()`

## Serial Protocol

```
Arduino TX (publishes):  "t,<left_ticks>,<right_ticks>\n"
Arduino RX (receives):   "v,<linear_x>,<angular_z>\n"
```

The ROS2 `serial_bridge.py` node translates this to ROS2 topics.

## Wiring

### BTS7960 to Arduino Mega

Each BTS7960 module has 6 control pins (RPWM, LPWM, R_EN, L_EN, VCC, GND):

```
+-------------+         +-------------------+         +-------------------+
| Arduino     |         | Left BTS7960      |         | Right BTS7960     |
| Mega 2560   |         |                   |         |                   |
|             |         |  RPWM  (forward)  |         |  RPWM  (forward)  |
|  Pin 4 -----+-------->|  RPWM             |         |                   |
|  Pin 5 -----+-------->|  LPWM             |         |                   |
|  Pin 6 -----+-------->|  R_EN             |         |                   |
|  Pin 7 -----+-------->|  L_EN             |         |                   |
|             |         |                   |         |                   |
|  Pin 8 -----+---------+-------------------+-------->|  RPWM             |
|  Pin 9 -----+---------+-------------------+-------->|  LPWM             |
|  Pin 10 ----+---------+-------------------+-------->|  R_EN             |
|  Pin 11 ----+---------+-------------------+-------->|  L_EN             |
|             |         |                   |         |                   |
|  5V --------+-------->|  VCC              |-------->|  VCC              |
|  GND -------+-------->|  GND              |-------->|  GND (logic)      |
+-------------+         |                   |         |                   |
                        |  B+   B-          |         |  B+   B-          |
  12V Supply ---------->|  B+               |-------->|  B+               |
  GND ----------------->|  B-               |-------->|  B-               |
                        |                   |         |                   |
                        |  M+ (OUT1)        |         |  M+ (OUT1)        |
  Left Motor  <---------|  M+ / M-          | Right   |  M+ / M-  ------->| Right Motor
                        +-------------------+         +-------------------+
```

**Important:** The B+/B- power terminals carry motor current (up to 43A peak). Use adequately rated wire (16AWG minimum).

### Pin Summary

| Arduino Pin | BTS7960 Module | Signal | Description |
|-------------|----------------|--------|-------------|
| 4  | Left  | RPWM   | Left motor forward PWM |
| 5  | Left  | LPWM   | Left motor reverse PWM |
| 6  | Left  | R_EN   | Left module enable (right half-bridge) |
| 7  | Left  | L_EN   | Left module enable (left half-bridge) |
| 8  | Right | RPWM   | Right motor forward PWM |
| 9  | Right | LPWM   | Right motor reverse PWM |
| 10 | Right | R_EN   | Right module enable (right half-bridge) |
| 11 | Right | L_EN   | Right module enable (left half-bridge) |
| 21 (INT0) | — | Left Encoder A  | Interrupt — tick counting |
| 20        | — | Left Encoder B  | Direction sensing |
| 3  (INT1) | — | Right Encoder A | Interrupt — tick counting |
| 2         | — | Right Encoder B | Direction sensing |

## Installation

1. **Open `Robot_Node_BTS7960.ino`** in Arduino IDE — no library installation needed

2. **Wire connections** per the diagram above

3. **Upload firmware** to Arduino Mega:
   - Board: Arduino Mega 2560
   - Port: /dev/ttyACM0

4. **Launch ROS2 robot**:
   ```bash
   ros2 launch mark_five_bot bringup.launch.py
   ```

5. **Test with ROS2**:
   ```bash
   ros2 topic echo /left_ticks
   ros2 topic echo /right_ticks
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

## Tuning

Initial constants are scaled from the previous Sabertooth values and will need tuning:

| Constant | Value | Description |
|----------|-------|-------------|
| `K_P` | 278 | PWM per (m/s) — tune first |
| `b` | 52 | Static PWM offset (deadband compensation) |
| `PWM_MIN` | 60 | Below this → 0 (deadband) |
| `PWM_MAX` | 100 | Safety cap during initial testing (0–255) |
| `PWM_TURN` | 80 | Fixed PWM for turn commands |
| `DRIFT_MULTIPLIER` | 120 | Straight-line correction gain |

**Tuning procedure:**
1. Set `PWM_MAX = 60` and send a `linear_x = 0.1` command — robot should move slowly
2. If it doesn't move, increase `b` or lower `PWM_MIN`
3. Gradually increase `PWM_MAX` to the desired top speed
4. Adjust `DRIFT_MULTIPLIER` if the robot curves during straight-line motion

## Troubleshooting

### Motors don't move
1. Check EN pins: `R_EN` and `L_EN` must be HIGH (set in `setup()`)
2. Check B+/B- have 12V connected to both modules
3. Verify 5V/GND logic connections to VCC/GND on each module
4. Check serial_bridge is running: `ros2 node list | grep serial_bridge`

### Motors spin wrong direction
Swap the motor wire connections (M+ and M−) on the module, **or** swap the sign in code:
```cpp
// In set_pwm_values(), swap leftWithSign sign:
int leftWithSign = (pwmLeftReq >= 0) ? -pwmLeftOut : pwmLeftOut;
```

### Robot drifts to one side
Adjust `DRIFT_MULTIPLIER` (increase to correct more, decrease if oscillating).

### No encoder ticks
- Verify encoder wires on pins 21/20 (left) and 3/2 (right)
- Check encoder power supply (typically 5V from Arduino)

## Robot Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| WHEEL_RADIUS | 0.035 m | Wheel radius (70mm diameter) |
| WHEEL_BASE | 0.36 m | Distance between wheels |
| TICKS_PER_REVOLUTION | 540 | Encoder ticks per revolution |
| TICKS_PER_METER | 2456 | Encoder ticks per meter |
