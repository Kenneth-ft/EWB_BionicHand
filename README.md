# EWB Robotics Team Bionic Hand V4

Hardware Library for the Bionic Hand

This library contains baremetal programming functions for controlling the i2c devices of the bionic hand. The fingers of the hand are cable-driven by MG996R continuous servos, which are controlled by a PCA9685 16-Channel PWM Driver Module.

Finger joint angles are measured by AS5600 Hall-Effect Encoders. These encoders are connected to the arduino using TCA9548A Multiplexers. 

# Finger Assignments
| index | Name | MCP,PIP,DIP joint index|
|---|---|---|
| 0 | Index | 0,1,2 |
| 1 | Middle | 3,4,5 |
| 2 | Ring | 6,7,8 |
| 3 | Pinky | 9,10,11 |
| 4 | Thumb | 12,13 | 

# Currently Available Functions