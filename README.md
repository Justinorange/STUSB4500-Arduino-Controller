STUSB4500 Power Controller
A KiCAD project for a custom USB Power Delivery (USB-PD) sink board using the STUSB4500. This controller allows an external microcontroller, like an Arduino, to request specific voltages (5V, 9V, 12V, 15V, or 20V) from a USB-C PD power source via I2C.

Hardware
The board is designed in KiCAD and includes:

STUSB4500 USB-PD Sink Controller

USB-C input for power

I2C interface for an Arduino

Power output terminals

Arduino Connections
Connect the board to your Arduino as follows:

Arduino D4 (SDA) → STUSB4500 SDA

Arduino D5 (SCL) → STUSB4500 SCL

Arduino GND → STUSB4500 GND

Add 4.7kΩ pull-up resistors to SDA and SCL.

Ensure the ADDR0 and ADDR1 pins on the STUSB4500 are grounded for the default I2C address (0x28).

Software
The included Arduino sketch (STUSB4500_Controller.ino) handles all communication.

How to Use
Upload the code to your Arduino.

Open the Serial Monitor at 9600 baud.

Enter a voltage (5, 9, 12, 15, or 20) and press Enter.

The code will configure the STUSB4500 and initiate a negotiation with the power source.

Status updates will be printed to the Serial Monitor.

Code Overview
The sketch uses the Wire.h library for I2C. It reads user input from the Serial Monitor to determine the desired voltage. It then writes a 32-bit PDO (Power Data Object) to the STUSB4500's registers before performing a software reset to start the negotiation. The code polls the device status every 500ms to report if a PD contract was successful or if the power source was disconnected.
