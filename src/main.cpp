/*
  STUSB4500 USB PD Voltage Controller for Arduino Nano

  This sketch uses the Stusb4500 library to communicate with the STUSB4500
  USB PD controller chip over I2C to change the negotiated power profile.

  How to Use:
  1. Upload this code to your Arduino.
  2. Open the Serial Monitor (baud rate: 9600).
  3. Type a voltage (5, 9, 12, 15, or 20) and press Enter.
*/

#include "Stusb4500.h"

// Create an instance of the Stusb4500 class
Stusb4500 pd_controller;

void setup() {
    // Initialize Serial communication
    Serial.begin(9600);
    Serial.println("STUSB4500 Voltage Controller Initialized");
    Serial.println("Enter a voltage (5, 9, 12, 15, or 20) to request a new power profile.");

    // Initialize the STUSB4500 controller
    pd_controller.begin();
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
        pd_controller.setVoltage(5);
        break;
      case 9:
        Serial.println("Requesting 9V...");
        pd_controller.setVoltage(9);
        break;
      case 12:
        Serial.println("Requesting 12V...");
        pd_controller.setVoltage(12);
        break;
      case 15:
        Serial.println("Requesting 15V...");
        pd_controller.setVoltage(15);
        break;
      case 20:
        Serial.println("Requesting 20V...");
        pd_controller.setVoltage(20);
        break;
      default:
        Serial.println("Invalid voltage. Please enter 5, 9, 12, 15, or 20.");
        break;
    }
  }

    // Check for and process any alerts from the STUSB4500
    pd_controller.checkAndProcessAlert();
}