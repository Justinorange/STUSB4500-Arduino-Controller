/**
 * @file STUSB4500_Controller.ino
 * @brief Arduino sketch to control the STUSB4500 USB PD Sink Controller.
 * @details This code allows a user to select a USB Power Delivery voltage profile (5, 9, 12, 15, or 20V)
 * via the Serial Monitor. It uses a polling mechanism over I2C to check for status updates,
 * removing the need for the ALERT interrupt pin.
 *
 * Hardware Connections:
 * - Arduino Nano D4 (SDA) -> STUSB4500 SDA
 * - Arduino Nano D5 (SCL) -> STUSB4500 SCL
 * - Arduino Nano GND     -> STUSB4500 GND
 * - Arduino Nano 5V      -> Pull-up resistors for SDA & SCL (e.g., 4.7k ohm)
 *
 * The STUSB4500's ADDR0 and ADDR1 pins should be connected to GND to use the default I2C address 0x28.
 * The STUSB4500's RESET pin should be pulled high (to 3.3V or 5V).
 *
 * Based on the STUSB4500 datasheet and AN5225 Application Note.
 */

#include <Wire.h>

// The default I2C address for the STUSB4500 when ADDR pins are grounded.
const uint8_t STUSB4500_ADDR = 0x28;

// --- STUSB4500 Register Map (from AN5225) ---
const uint8_t REG_ALERT_STATUS_1      = 0x0C; // Interrupt status register 1
const uint8_t REG_PORT_STATUS_0       = 0x16; // Port status register
const uint8_t REG_RDO_STATUS_0        = 0x1D; // Negotiated RDO status LSB
const uint8_t REG_RESET_CTRL          = 0x23; // Software Reset Control Register
const uint8_t REG_SNK_PDO_NUM         = 0x54; // Number of Sink PDOs to advertise
const uint8_t REG_SNK_PDO2_0          = 0x89; // Start address for Sink PDO2 (32-bit value)

// --- Polling Timer ---
unsigned long last_poll_time = 0;
const long poll_interval = 500; // Check status every 500 ms

/**
 * @brief Initializes I2C and Serial communication.
 */
void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to open

  Wire.begin();

  Serial.println("--- STUSB4500 USB PD Controller (Polling Mode) ---");

  // Check if the STUSB4500 is connected and responding.
  Wire.beginTransmission(STUSB4500_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("Error: STUSB4500 not found on I2C bus!");
    Serial.println("Please check wiring. Halting.");
    while (1); // Stop execution
  }
  Serial.println("STUSB4500 detected successfully.");
  Serial.println("Enter a voltage (5, 9, 12, 15, or 20) and press Enter to negotiate.");
  Serial.println();
}

/**
 * @brief Main loop to handle user input and poll for status changes.
 */
void loop() {
  // Check if the user has entered a command in the Serial Monitor.
  if (Serial.available() > 0) {
    long voltage = Serial.parseInt();
    
    // Clear the serial buffer of any remaining characters (like newline)
    while(Serial.read() != -1);

    if (voltage > 0) {
      setVoltage(voltage);
    }
  }

  // Periodically poll the device for status updates.
  unsigned long current_millis = millis();
  if (current_millis - last_poll_time >= poll_interval) {
    last_poll_time = current_millis;
    checkDeviceStatus();
  }
}


/**
 * @brief Configures the STUSB4500 to request a specific voltage.
 * @param v The target voltage (5, 9, 12, 15, or 20).
 */
void setVoltage(int v) {
  Serial.print("Attempting to set voltage to ");
  Serial.print(v);
  Serial.println("V...");

  // For 5V, we just advertise one PDO. The chip defaults to PDO1, which is always 5V.
  if (v == 5) {
    // Set number of advertised Sink PDOs to 1.
    writeRegister(REG_SNK_PDO_NUM, 0b00);
    Serial.println("Configured for 5V (default PDO1).");
  } 
  // For other voltages, we configure PDO2 and advertise two PDOs.
  else if (v == 9 || v == 12 || v == 15 || v == 20) {
    // The STUSB4500 uses a 32-bit format for each PDO.
    // Bits [19:10]: Voltage in 50mV units
    // Bits [9:0]: Current in 10mA units

    // Calculate the 10-bit value for the voltage.
    uint16_t voltage_val = (uint16_t)(v / 0.05);

    // Set a default max current of 1.5A for the request.
    // The source will provide the lowest current it can support at that voltage.
    uint16_t current_val = (uint16_t)(1.5 / 0.01); // 150 -> 1.5A

    // Construct the 32-bit PDO value. Other bits are left at 0 for simplicity.
    uint32_t pdo_value = ((uint32_t)voltage_val << 10) | current_val;

    // Write the 4 bytes of the PDO value to the PDO2 registers.
    write4Bytes(REG_SNK_PDO2_0, pdo_value);

    // Set the number of advertised Sink PDOs to 2 (PDO1 and our custom PDO2).
    writeRegister(REG_SNK_PDO_NUM, 0b10);

    Serial.print("Configured PDO2 for ");
    Serial.print(v);
    Serial.println("V @ 1.5A max.");
  } else {
    Serial.println("Invalid voltage. Please use 5, 9, 12, 15, or 20.");
    return;
  }

  // Trigger a software reset to force the chip to re-read the configuration
  // from its RAM registers and start a new negotiation with the source.
  Serial.println("Sending soft reset to start negotiation...");
  writeRegister(REG_RESET_CTRL, 0x01); // Set SW_RESET bit
  delay(20); // Wait a moment for the reset to process
}

/**
 * @brief Polls the STUSB4500 and decodes status registers if an event has occurred.
 */
void checkDeviceStatus() {
  uint8_t status = readRegister(REG_ALERT_STATUS_1);

  // If status is 0, no events have occurred since the last check.
  if (status == 0) {
    return;
  }

  Serial.println("\n--- Status Update ---");

  // Bit 0: Port Attach/Detach Status Change
  if (status & 0b00000001) {
    // Check the ATTACHED_STATUS bit in the PORT_STATUS register
    uint8_t port_status = readRegister(REG_PORT_STATUS_0);
    if (port_status & 0x01) {
      Serial.println("Status: Power source attached.");
    } else {
      Serial.println("Status: Power source detached.");
    }
  }

  // Bit 5: Power Delivery Contract negotiation has completed (success or fail)
  if (status & 0b00100000) {
     // Read the RDO status register to see which PDO was negotiated
     uint8_t pdo_status_lsb = readRegister(REG_RDO_STATUS_0);
     uint8_t negotiated_pdo_num = (pdo_status_lsb & 0b01110000) >> 4;
     
     if (negotiated_pdo_num > 0) {
        Serial.print("Status: PD Contract established. Active PDO is #");
        Serial.println(negotiated_pdo_num);
        Serial.println("-> Voltage negotiation successful!");
     } else {
        Serial.println("Status: PD Contract failed or is at default 5V.");
     }
  }
  
  Serial.println("----------------------\n");

  // IMPORTANT: Clear the alert by writing the status value back to the register.
  writeRegister(REG_ALERT_STATUS_1, status);
}

// --- I2C Helper Functions ---

/**
 * @brief Writes a single byte to a specified register.
 * @param reg The register address.
 * @param value The byte to write.
 */
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(STUSB4500_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

/**
 * @brief Reads a single byte from a specified register.
 * @param reg The register address.
 * @return The byte read from the register.
 */
uint8_t readRegister(uint8_t reg) {
  uint8_t value = 0;
  Wire.beginTransmission(STUSB4500_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // Send restart to keep connection active
  
  Wire.requestFrom(STUSB4500_ADDR, (uint8_t)1);
  if (Wire.available()) {
    value = Wire.read();
  }
  return value;
}

/**
 * @brief Writes four bytes (a 32-bit value) to sequential registers.
 * @param startReg The starting register address.
 * @param value The 32-bit value to write.
 */
void write4Bytes(uint8_t startReg, uint32_t value) {
  Wire.beginTransmission(STUSB4500_ADDR);
  Wire.write(startReg);
  Wire.write((uint8_t)(value & 0xFF));        // LSB
  Wire.write((uint8_t)((value >> 8) & 0xFF));
  Wire.write((uint8_t)((value >> 16) & 0xFF));
  Wire.write((uint8_t)((value >> 24) & 0xFF)); // MSB
  Wire.endTransmission();
}

