# EWB Robotics Team Bionic Hand V4

Hardware Library for the Bionic Hand

This library contains baremetal programming functions for controlling the i2c devices of the bionic hand. The fingers of the hand are cable-driven by MG996R continuous servos, which are controlled by a PCA9685 16-Channel PWM Driver Module.

Finger joint angles are measured by AS5600 Hall-Effect Encoders. These encoders are connected to the arduino using TCA9548A Multiplexers. 

# Finger Assignments
| index | Name | MCP, PIP, DIP joint index|
|---|---|---|
| 0 | Index | 0, 1, 2 |
| 1 | Middle | 3, 4, 5 |
| 2 | Ring | 6, 7, 8 |
| 3 | Pinky | 9, 10, 11 |
| 4 | Thumb | 12, 13 | 

# Currently Available Functions


<details>
<summary><b>setFingerSpeed()</b></summary>
<br>

**Parameters:**
- `fingerID` (uint8_t): 0 to 4, selects which finger to control
- `speed` (float): -1.0 to 1.0, where -1.0 is full speed in one direction, 1.0 is full speed in the other direction, and 0.0 is stop

**Returns:**
- void

</details>

<details>
<summary><b>getFingerAngles()</b></summary>
<br>

Fills the passed array with three angles (MCP, PIP, DIP)
You must pass an array of size 3!

**Parameters:**
- `fingerID` (uint8_t): 0 to 4, selects which finger to control
- `outputAngles[3]` (float): the output array to fill (must be size 3)

**Returns:**
- void

</details>

<details>
<summary><b>readRawSensor()</b></summary>
<br>

**Parameters:**
- `sensorIndex` (uint8_t): selects the sensor to read (see table above)

**Returns:**
- angle in degrees (float)

</details>
