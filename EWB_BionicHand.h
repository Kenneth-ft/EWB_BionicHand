#ifndef EWB_BIONIC_HAND_H
#define EWB_BIONIC_HAND_H

#include <Arduino.h>
#include <Wire.h>

#define SIMULATION_MODE // uncommented: no hand. commented: yes hand.

struct FingerPose
{
    float mcp;
    float pip;
    float dip; // Will be -1.0 or NAN for the thumb
};

class EWBHand
{
public:
    // Constructor: You can add setup parameters here if needed
    EWBHand();

    // Initialization (Call this in void setup())
    void begin();

    // --- CONTROLS ---
    // speed: -1.0 to 1.0. fingerID: 0-4
    void setFingerSpeed(uint8_t fingerID, float speed);

    void getFingerPose(uint8_t fingerID, FingerPose &pose);

    // Useful for debugging specific sensors (0-13)
    float readRawSensor(uint8_t sensorIndex);

    float getJointVelocity(uint8_t sensorIndex);

    void calibrate();

private:
    // Helper functions (Hidden from the user)

    void _selectMuxChannel(uint8_t globalSensorIndex);
    float _readAS5600(); // Reads currently selected sensor

    // Internal state variables could go here
    uint8_t _currentMuxChannel = 255; // Cache to avoid redundant switching
    float _simulatedAngles[14];       // 14 joints

    float _angleOffset[14];

    float _lastAngles[14];
    unsigned long _lastTimes[14];

    float _filteredVelocity[14];
};

#endif