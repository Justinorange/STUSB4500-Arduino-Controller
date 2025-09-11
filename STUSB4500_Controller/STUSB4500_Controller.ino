/*
  STUSB4500 USB PD Voltage Controller for Arduino Nano

  This sketch communicates with the STUSB4500 USB PD controller chip
  over I2C to change the negotiated power profile (voltage).

  Hardware Setup:
  - Arduino Nano
  - STUSB4500 Breakout Board
  - Arduino A4 (SDA) -> STUSB4500 SDA
  - Arduino A5 (SCL) -> STUSB4500 SCL
  - Arduino D2 -> STUSB4500 ALERT
  - Arduino GND -> STUSB4500 GND
  - Arduino 5V -> STUSB4500 VDD (if not powered from VBUS)
  - Ensure necessary pull-up resistors are in place for I2C lines and ALERT.

  How to Use:
  1. Upload this code to your Arduino Nano.
  2. Open the Serial Monitor (baud rate: 9600).
  3. Type one of the following commands and press Enter:
     - '5' to request 5V
     - '9' to request 9V
     - '12' to request 12V
     - '15' to request 15V
     - '20' to request 20V
  4. The STUSB4500 will then negotiate the requested voltage with the USB PD source.
  5. The ALERT pin will signal when a new negotiation is complete or if an event occurs.
*/

#include <Wire.h>

// I2C address of the STUSB4500. Default is 0x28.
// Can be 0x29, 0x2A, or 0x2B depending on ADDR0 and ADDR1 pins.
#define STUSB4500_ADDRESS 0x28

// STUSB4500 Register Addresses
#define PDO_NUM_REG 0x70
#define V_SNK_PDO2_REG 0x85
#define V_SNK_PDO3_REG 0x87

// Digital pin connected to the STUSB4500's ALERT pin
#define ALERT_PIN 2

// A volatile boolean flag to be used by the interrupt service routine
volatile bool alert_flag = false;

void setup() {
  // Initialize Serial communication at 9600 baud
  Serial.begin(9600);
  Serial.println("STUSB4500 Voltage Controller Initialized");
  Serial.println("Enter a voltage (5, 9, 12, 15, or 20) to request a new power profile.");

  // Initialize I2C communication
  Wire.begin();

  // Set the ALERT pin as an input with a pull-up resistor
  pinMode(ALERT_PIN, INPUT_PULLUP);
  // Attach an interrupt to the ALERT pin, to be triggered on a falling edge
  attachInterrupt(digitalPinToInterrupt(ALERT_PIN), handleAlert, FALLING);
  
  // Initial soft reset of the STUSB4500 NVM to load default settings
  // This helps ensure the chip is in a known state.
  // Note: For production use, you would pre-program the NVM.
  // This example dynamically changes PDOs in RAM.
  i2cWrite(0x25, 0x01); // Trigger NVM read
  delay(10); // Wait for NVM to load
}

void loop() {
  // Check if there is data available to read from the serial port
  if (Serial.available() > 0) {
    // Read the incoming integer
    int target_voltage = Serial.parseInt();

    // Process the command
    switch (target_voltage) {
      case 5:
        Serial.println("Requesting 5V...");
        setVoltage(5);
        break;
      case 9:
        Serial.println("Requesting 9V...");
        setVoltage(9);
        break;
      case 12:
        Serial.println("Requesting 12V...");
        setVoltage(12);
        break;
      case 15:
        Serial.println("Requesting 15V...");
        setVoltage(15);
        break;
      case 20:
        Serial.println("Requesting 20V...");
        setVoltage(20);
        break;
      default:
        Serial.println("Invalid voltage. Please enter 5, 9, 12, 15, or 20.");
        break;
    }
  }

  // If the alert flag was set by the ISR, print a message and reset it
  if (alert_flag) {
    // An alert can be for various reasons. We check the negotiated power data object (PDO)
    // to see if the voltage contract has changed.
    uint8_t pdo_index = i2cRead(RDO_REG_STATUS) & 0x0F; // Read Sink PDO number in use

    int new_voltage = 0;

    switch (pdo_index) {
      case 1: // PDO1 is always 5V
        new_voltage = 5;
        break;
      case 2: // PDO2
        {
          // Read the 10-bit value from the register
          uint16_t pdo2_val = i2cRead(V_SNK_PDO2_REG) | ((i2cRead(V_SNK_PDO2_REG + 1) & 0x03) << 8);
          // Convert to volts (value * 50mV)
          new_voltage = (int)((pdo2_val * 50L) / 1000L);
        }
        break;
      case 3: // PDO3
        {
          // Read the 10-bit value from the register
          uint16_t pdo3_val = i2cRead(V_SNK_PDO3_REG) | ((i2cRead(V_SNK_PDO3_REG + 1) & 0x03) << 8);
          // Convert to volts (value * 50mV)
          new_voltage = (int)((pdo3_val * 50L) / 1000L);
        }
        break;
    }

    if (new_voltage != 0 && new_voltage != currentNegotiatedVoltage) {
      currentNegotiatedVoltage = new_voltage;
      Serial.print("Negotiation successful. Voltage is now: ");
      Serial.print(currentNegotiatedVoltage);
      Serial.println("V");
    } else {
      Serial.println("Alert received from STUSB4500 (no voltage change or status update).");
    }

    alert_flag = false;
  }
}

// Function to set the desired voltage by configuring the PDO profiles.
// The STUSB4500 supports up to 3 Sink PDOs.
// PDO1 is fixed at 5V. We will configure PDO2 and PDO3 to get other voltages.
void setVoltage(int voltage) {
  uint16_t pdo2_voltage = 0;
  uint16_t pdo3_voltage = 0;
  uint8_t num_pdos = 1; // Default is one PDO (5V)

  // According to the datasheet, the voltage is programmed in 50mV steps.
  // So, value = voltage / 0.05
  uint16_t voltage_val = voltage / 0.05;

  if (voltage == 5) {
    // To request 5V, we just set the number of PDOs to 1.
    // The STUSB4500 will then only negotiate the default 5V PDO.
    num_pdos = 1;
  } else {
    // For other voltages, we will set them as PDO2 and make PDO3 a copy.
    // This makes the negotiation more robust.
    num_pdos = 3;
    pdo2_voltage = voltage_val;
    pdo3_voltage = voltage_val;
  }
  
  // Write the voltage for PDO2 (registers 0x85 and 0x86)
  i2cWrite(V_SNK_PDO2_REG, pdo2_voltage & 0xFF);
  i2cWrite(V_SNK_PDO2_REG + 1, (pdo2_voltage >> 8) & 0x03);

  // Write the voltage for PDO3 (registers 0x87 and 0x88)
  i2cWrite(V_SNK_PDO3_REG, pdo3_voltage & 0xFF);
  i2cWrite(V_SNK_PDO3_REG + 1, (pdo3_voltage >> 8) & 0x03);

  // Set the number of active PDOs
  // Bits 6-7 of register 0x70 control this.
  // 0b00 or 0b01 -> 1 PDO
  // 0b10 -> 2 PDOs
  // 0b11 -> 3 PDOs
  uint8_t pdo_num_val = 0;
  if (num_pdos == 1) pdo_num_val = 0b01;
  if (num_pdos == 2) pdo_num_val = 0b10;
  if (num_pdos == 3) pdo_num_val = 0b11;
  
  uint8_t current_pdo_reg = i2cRead(PDO_NUM_REG);
  current_pdo_reg &= 0x3F; // Clear bits 6-7
  current_pdo_reg |= (pdo_num_val << 6);
  i2cWrite(PDO_NUM_REG, current_pdo_reg);
  
  // After changing PDOs, we need to trigger a new negotiation.
  // A soft reset is an effective way to do this.
  i2cWrite(0x25, 0x01); // Send soft reset command
  
  Serial.print("Configuration sent for ");
  Serial.print(voltage);
  Serial.println("V. Awaiting negotiation...");
}

// Interrupt Service Routine for the ALERT pin
void handleAlert() {
  alert_flag = true;
}

// Helper function to write a byte to an I2C device register
void i2cWrite(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(STUSB4500_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// Helper function to read a byte from an I2C device register
uint8_t i2cRead(uint8_t reg) {
  Wire.beginTransmission(STUSB4500_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false); // Send a restart message, keeping the connection active
  Wire.requestFrom(STUSB4500_ADDRESS, (uint8_t)1); // Request 1 byte from the specified register
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}
