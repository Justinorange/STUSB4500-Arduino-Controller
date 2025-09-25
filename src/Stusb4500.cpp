#include "Stusb4500.h"

// Global flag for the ISR
volatile bool alert_flag = false;

// Interrupt Service Routine
void handleAlert() {
    alert_flag = true;
}

Stusb4500::Stusb4500() {
    currentNegotiatedVoltage = 0;
}

void Stusb4500::begin() {
    // Initialize I2C communication
    Wire.begin();

    // Set the ALERT pin as an input with a pull-up resistor
    pinMode(ALERT_PIN, INPUT_PULLUP);
    // Attach an interrupt to the ALERT pin, to be triggered on a falling edge
    attachInterrupt(digitalPinToInterrupt(ALERT_PIN), handleAlert, FALLING);

    // Initial soft reset of the STUSB4500 NVM to load default settings
    i2cWrite(NVM_CTRL_REG, 0x01); // Trigger NVM read
    delay(10); // Wait for NVM to load
}

void Stusb4500::setVoltage(int voltage) {
    uint16_t pdo2_voltage = 0;
    uint16_t pdo3_voltage = 0;
    uint8_t num_pdos = 1; // Default is one PDO (5V)

    // According to the datasheet, voltage is programmed in 50mV steps.
    uint16_t voltage_val = voltage / 0.05;

    if (voltage == 5) {
        // To request 5V, we just set the number of PDOs to 1.
        num_pdos = 1;
    } else {
        // For other voltages, set them as PDO2 and make PDO3 a copy for robust negotiation.
        num_pdos = 3;
        pdo2_voltage = voltage_val;
        pdo3_voltage = voltage_val;
    }

    // Write the voltage for PDO2
    i2cWrite(V_SNK_PDO2_REG, pdo2_voltage & 0xFF);
    i2cWrite(V_SNK_PDO2_REG + 1, (pdo2_voltage >> 8) & 0x03);

    // Write the voltage for PDO3
    i2cWrite(V_SNK_PDO3_REG, pdo3_voltage & 0xFF);
    i2cWrite(V_SNK_PDO3_REG + 1, (pdo3_voltage >> 8) & 0x03);

    // Set the number of active PDOs
    uint8_t pdo_num_val = (num_pdos == 1) ? 0b01 : 0b11;
    uint8_t current_pdo_reg = i2cRead(PDO_NUM_REG);
    current_pdo_reg &= 0x3F; // Clear bits 6-7
    current_pdo_reg |= (pdo_num_val << 6);
    i2cWrite(PDO_NUM_REG, current_pdo_reg);

    // Trigger a new negotiation with a soft reset
    i2cWrite(NVM_CTRL_REG, 0x01);

    Serial.print("Configuration sent for ");
    Serial.print(voltage);
    Serial.println("V. Awaiting negotiation...");
}

void Stusb4500::checkAndProcessAlert() {
    if (alert_flag) {
        uint8_t pdo_index = i2cRead(RDO_REG_STATUS) & 0x0F; // Read Sink PDO number in use
        int new_voltage = 0;

        switch (pdo_index) {
            case 1: // PDO1 is always 5V
                new_voltage = 5;
                break;
            case 2: { // PDO2
                uint16_t pdo2_val = i2cRead(V_SNK_PDO2_REG) | ((i2cRead(V_SNK_PDO2_REG + 1) & 0x03) << 8);
                new_voltage = (int)((pdo2_val * 50L) / 1000L);
                break;
            }
            case 3: { // PDO3
                uint16_t pdo3_val = i2cRead(V_SNK_PDO3_REG) | ((i2cRead(V_SNK_PDO3_REG + 1) & 0x03) << 8);
                new_voltage = (int)((pdo3_val * 50L) / 1000L);
                break;
            }
        }

        if (new_voltage != 0 && new_voltage != currentNegotiatedVoltage) {
            currentNegotiatedVoltage = new_voltage;
            Serial.print("✅ Negotiation successful. Voltage is now: ");
            Serial.print(currentNegotiatedVoltage);
            Serial.println("V");
        } else {
            Serial.println("ℹ️ Alert received from STUSB4500 (no voltage change or status update).");
        }

        alert_flag = false; // Reset the flag
    }
}

// Helper function to write a byte to an I2C device register
void Stusb4500::i2cWrite(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(STUSB4500_ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

// Helper function to read a byte from an I2C device register
uint8_t Stusb4500::i2cRead(uint8_t reg) {
    Wire.beginTransmission(STUSB4500_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false); // Send a restart message
    Wire.requestFrom(STUSB4500_ADDRESS, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0; // Return 0 on error
}