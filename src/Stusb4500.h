#ifndef STUSB4500_H
#define STUSB4500_H

#include <Arduino.h>
#include <Wire.h>

// I2C and Pin Definitions
#define STUSB4500_ADDRESS 0x28
#define ALERT_PIN 2

// STUSB4500 Register Addresses
#define PDO_NUM_REG 0x70
#define V_SNK_PDO2_REG 0x85
#define V_SNK_PDO3_REG 0x87
#define RDO_REG_STATUS 0x95
#define NVM_CTRL_REG 0x25 // Register used for soft reset/NVM loading

// Class to manage the STUSB4500 chip
class Stusb4500 {
public:
    Stusb4500();
    void begin();
    void setVoltage(int voltage);
    void checkAndProcessAlert();

private:
    int currentNegotiatedVoltage;
    void i2cWrite(uint8_t reg, uint8_t data);
    uint8_t i2cRead(uint8_t reg);
};

// Global ISR function declaration
void handleAlert();

#endif // STUSB4500_H