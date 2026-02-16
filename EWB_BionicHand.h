#ifndef EWB_BIONIC_HAND_H
#define EWB_BIONIC_HAND_H

#include <Arduino.h>
#include <Wire.h>


#define SIMULATION_MODE //uncommented: no hand. commented: yes hand.

class EWBHand {
  public:
    // Constructor: You can add setup parameters here if needed
    EWBHand();

    // Initialization (Call this in void setup())
    void begin();

    // --- CONTROLS ---
    // speed: -1.0 to 1.0. fingerID: 0-4
    void setFingerSpeed(uint8_t fingerID, float speed);
    
    // Fills the passed array with 3 angles (MCP, PIP, DIP)
    // You must pass an array of size 3!
    void getFingerAngles(uint8_t fingerID, float outputAngles[3]);

    // Useful for debugging specific sensors (0-13)
    float readRawSensor(uint8_t sensorIndex);

  private:
    // Helper functions (Hidden from the user)

    void _selectMuxChannel(uint8_t globalSensorIndex);
    float _readAS5600(); // Reads currently selected sensor
    
    // Internal state variables could go here
    uint8_t _currentMuxChannel = 255; // Cache to avoid redundant switching
    float _simulatedAngles[14]; //14 joints
};

#endif