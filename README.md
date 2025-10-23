# STUSB4500 USB-C Power Delivery (PD) Sink Controller

A custom **KiCAD project** for building a **USB Power Delivery (USB-PD) Sink Board** utilizing the STMicroelectronics **STUSB4500** controller.

This board provides a powerful and flexible way to draw high-power **variable voltage** from any standard USB-C PD power source. It is designed to be fully controllable by an external host microcontroller, via the **I²C bus**.

---

## Features

* **USB-PD Protocol Negotiation**: Automatically handles the USB-PD handshake process.
* **I²C Control**: Allows a host MCU (e.g., Arduino, ESP32) to digitally request specific voltages.
* **Configurable Voltages**: Supports requesting standard USB-PD voltages: **5V, 9V, 12V, 15V, or 20V**.
* **Compact Design**: Designed as a simple, custom PCB project in KiCAD.

---

## Hardware Overview

The board is designed around the dedicated **STUSB4500** chip, which simplifies the PD sink implementation significantly by handling the communication protocol stack internally.

| Component | Description |
| :--- | :--- |
| **Controller IC** | STUSB4500 USB-PD Sink Controller |
| **Input** | USB-C receptacle for PD power source input |
| **Host Interface** | 5-pin JST connector for I²C communication with the microcontroller |
| **Power Output** | 2-pin JST connector for regulated DC output power |

The full schematic and PCB layout files are available in the KiCAD project directory (`/kicad/`).

---

## I²C Connection Pinout

This pinout is designed for compatibility with the ESP32 Arduino Nano, using a standard **5-pin JST header**.

| STUSB4500 Board Pin | Signal | Arduino (e.g., Nano/Uno) | Function |
| :--- | :--- | :--- | :--- |
| **SDA** | I²C Data | A4 | I²C Serial Data Line |
| **SCL** | I²C Clock | A5 | I²C Serial Clock Line |
| **RST** | Reset | D0 | External Reset Pin for STUSB4500 |
| **VDD** | Power Input | Vin (20V) | Power Supply for Arduino/STUSB4500 Logic |
| **GND** | Ground | GND | Common Ground Reference |

---

## How to Use (Arduino Example)

Follow these steps to quickly test and control the power negotiation using an Arduino or compatible microcontroller.

### Prerequisites

* **Code**: You will need the specific library/code to communicate with the STUSB4500 over I²C.
* **Connection**: Ensure the board is wired correctly according to the table above.
* **Power**: Connect a **USB-C PD power source** (charger) to the board's USB-C input.

### Negotiation Workflow

1.  **Upload**: Upload the I²C configuration code to your Arduino.
2.  **Monitor**: Open the **Serial Monitor** at 9600 baud.
3.  **Input**: Enter the desired target voltage (**5, 9, 12, 15, or 20**) and press Enter.
4.  **Negotiation**: The Arduino code will:
    * Configure the STUSB4500's I²C registers with the requested voltage.
    * The **STUSB4500 chip** will initiate a negotiation request to the power source.
5.  **Status**: The negotiated status (Success or Failure) and the resulting stable output voltage will be printed back to the Serial Monitor.
6.  **Output**: The requested voltage will be delivered to the **2-pin JST output terminal**.