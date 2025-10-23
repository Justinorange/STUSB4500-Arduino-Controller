STUSB4500 Power Controller
A KiCAD project for a custom USB Power Delivery (USB-PD) sink board using the STUSB4500. This controller allows an external microcontroller, like an Arduino, to request specific voltages (5V, 9V, 12V, 15V, or 20V) from a USB-C PD power source via I2C.

Hardware
The board is designed in KiCAD and includes:

STUSB4500 USB-PD Sink Controller

USB-C input for power

I2C interface for an Arduino (via 5 pin JST)

Power output terminals (2 pin JST)

I2C Connections (designed around ESP32 Arduino nano)
Connect the board to your Arduino as follows:

Arduino A4 (SDA) → STUSB4500 SDA

Arduino A5 (SCL) → STUSB4500 SCL

Arduino GND → STUSB4500 GND

Arduino D0 → STUSB4500 RST

Arduino Vin → STUSB4500 VDD


How to Use
Upload the code to your Arduino.

Open the Serial Monitor at 9600 baud.

Enter a voltage (5, 9, 12, 15, or 20) and press Enter.

The code will configure the STUSB4500 and initiate a negotiation with the power source.

Status updates will be printed to the Serial Monitor.