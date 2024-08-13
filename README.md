# mark_five_amr
Jetson nano based mobile robot with SLAM capability


## Notes 

- One Revolution = 540 Encoder ticks. 
- Wheel diameter = 0.055 m
- Wheel Radius = 0.0275 m
- Max Motor speed = 130rpm
- Circumference = 2*pi*r = 0.17279

*If PWM range is 50-100, robot velocity will be in the range:*
- Linear velocity = Angular_Velocity * r OR rpm * Circumference (divide by 60 to get m/s)
- Linear velocity Min = 130 rotations/min * 0.50 duty cycle * 0.17279 m / 60 sec = 0.187 m/s
- Linear velocity Max = 130 rotations/min * 1.0 duty cycle * 0.17279 m / 60 sec = 0.3744 m/s
- Velocity Range = 0.187 - 0.3744 m/s

@TODO: Calculate robot velocity via Encoder and verify the above calculations.

### References:

Math: http://wiki.ros.org/diff_drive_controller
Overall autonomous system design: https://github.com/danielsnider/ros-rover  

