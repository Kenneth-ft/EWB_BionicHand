#include "EWB_BionicHand.h"

// i2c Hardware Addresses
#define PCA9685_ADDR 0x40 // Default address
#define MUX_A_ADDR 0x70 // First Multiplexer (Joint 0-7)
#define MUX_B_ADDR 0x71 // Second Multiplexer (Joint 8-13)
#define AS5600_ADDR 0x36

#define PCA9685_MODE1 0x00
#define PCA9685_PRE_SCALE 0xFE
#define PCA9685_LED0_ON_L 0x06 // Register for Channel 0 start



// Constructor (Usually empty)
EWBHand::EWBHand() {

    // Reset sim angles
    for(int i=0; i<14; i++) _simulatedAngles[i] = 0.0;
}

void EWBHand::begin() {
    Serial.begin(115200);
    
    #ifdef SIMULATION_MODE
        Serial.println("--- EWB HAND: SIMULATION MODE ACTIVE ---");
        Serial.println("No hardware required. Commands will be printed.");
    #else
    Serial.println("--- EWB HAND: HARDWARE MODE ---");
    Wire.begin();
    Wire.setClock(400000); // 400kHz Fast Mode
  
  // Initialize PCA9685 (Wake up & Set 50Hz)
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(PCA9685_MODE1);
    Wire.write(0x10); // Sleep mode (needed to change prescaler)
    Wire.endTransmission();
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(PCA9685_PRE_SCALE);
    Wire.write(121); // Prescaler for ~50Hz
    Wire.endTransmission();
    Wire.beginTransmission(PCA9685_ADDR);
    Wire.write(PCA9685_MODE1);
    Wire.write(0x20); // Auto-increment mode enabled, wake up
    Wire.endTransmission();
    #endif
  // Remember to use 'Wire.beginTransmission(PCA_ADDR)' etc.
}



// --- PUBLIC METHODS ---

// Helper to get all 3 angles for a finger
// Assumption: Finger 0 = Sensors 0,1,2. Finger 1 = Sensors 3,4,5...
void EWBHand::getFingerAngles(uint8_t fingerID, float outputAngles[3]) {
    uint8_t startIdx = fingerID * 3; 
    // Careful! Thumb might only have 2 joints? Adjust math if so.
    
    for(int i=0; i<3; i++) {
        outputAngles[i] = readRawSensor(startIdx + i);
    }
}

// Reads a specific sensor by Index (0-13)
float EWBHand::readRawSensor(uint8_t sensorIndex) {
    #ifdef SIMULATION_MODE
        if(sensorIndex < 14) return _simulatedAngles[sensorIndex];
        return 0.0;
    #else
        _selectMuxChannel(sensorIndex);
        return _readAS5600();
    #endif
}

void EWBHand::setFingerSpeed(uint8_t fingerID, float speed) {
  // 1. Clamp speed between -1.0 and 1.0
  if (speed > 1.0) speed = 1.0;
  if (speed < -1.0) speed = -1.0;
  // 2. Map speed to pulse width (1000us - 2000us)
  // 1500us is center/stop
  float pulse_us = 1500.0 + (speed*500.0); 
  // 3. Convert to ticks (4096 ticks = 20ms = 20000us)
  uint16_t off_tick = (uint16_t)((pulse_us / 20000.0) * 4096.0);
  // 4. I2C Transaction
  // Auto-increment is active, can write all 4 bytes in one go

  #ifdef SIMULATION_MODE
    Serial.print("[SIM] Motor ");
    Serial.print(fingerID);
    Serial.print(" set to speed: ");
    Serial.println(speed);

    // Fake Physics: Move all 3 joints of this finger
    int startIdx = fingerID * 3;
    for(int i=0; i<3; i++) {
        if(startIdx+i < 14) _simulatedAngles[startIdx+i] += (speed * 5.0);
    }
  #else
  Wire.beginTransmission(PCA9685_ADDR);
  Wire.write(PCA9685_LED0_ON_L + (4 * fingerID)); // Calculate register offset
  Wire.write(0); // ON_L
  Wire.write(0); // ON_H
  Wire.write(off_tick & 0xFF); //OFF_L
  Wire.write((off_tick >> 8) & 0xFF); //OFF_H
  Wire.endTransmission();
  #endif
  // Map fingerID to PCA channel (maybe Finger 0 is Channel 0, etc.)
}

// --- PRIVATE HELPERS ---

void EWBHand::_selectMuxChannel(uint8_t globalSensorIndex) {
  // Logic to switch between Mux A and B based on ID
  if (globalSensorIndex > 15) return; // do nothing if index is invalid

  // Map sensor 0-7 to Mux A, 8-15 to Mux B
  uint8_t muxAddr = (globalSensorIndex < 8) ? MUX_A_ADDR : MUX_B_ADDR;
  uint8_t muxChan = (globalSensorIndex < 8) ? globalSensorIndex : (globalSensorIndex - 8);

  // Send the switch command
  Wire.beginTransmission(muxAddr);
  Wire.write(1 << muxChan);
  Wire.endTransmission();
}

// --- Raw AS5600 Interaction ---
float EWBHand::_readAS5600() {
  // Point to the RAW ANGLE register (0x0C)
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission();

  // Request 2 bytes (High byte + Low byte)
  Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
  
  if (Wire.available() >= 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    
    // Combine them: (High << 8) | Low
    uint16_t rawAngle = (highByte << 8) | lowByte; 

    // Convert 0-4095 range to 0-360 degrees
    return (rawAngle * 360.0) / 4096.0;
  }
  
  return -1.0; // Error flag (sensor unplugged or wires crossed)
}