# Syringe Driver Project ReadMe

## Project Overview

This repository contains the firmware and supporting CAN communication libraries for a **multi-channel syringe driver system**, designed to precisely control motor-driven syringes through a CAN (Controller Area Network) interface. The system integrates an **MCP2515-based CAN transceiver** and a **UIROBOT UIM342 motor controller** to enable synchronized and reliable operation of syringe actuators used in automated liquid handling systems.

The firmware is implemented for **Arduino-compatible microcontrollers** and includes low-level CAN communication libraries (`mcp_can.cpp/.h`), an abstraction layer (`UirSimpleCAN.cpp/.h`), and the main control sketch (`syringe_driver.ino`).

---

## Features

* **CAN Bus Communication:** Full integration with MCP2515 CAN controllers for stable communication with motor driver modules.
* **UIM342 Motor Support:** Optimized routines for sending and receiving control words to UIROBOT UIM342 drivers.
* **Configurable Syringe Channels:** Supports multiple syringe channels over the same bus with unique CAN IDs.
* **Real-time Configuration Access:** Functions to read or set key parameters such as position, serial number, and motion limits.
* **Error Handling:** Implements MCP2515 error detection and mode management (configuration, loopback, sleep, etc.).
* **Hardware Abstraction:** The firmware is modular—communication logic (in `UirSimpleCAN`) is separate from low-level drivers (`mcp_can`).

---

## System Architecture

### 1. Syringe Driver Firmware

The core firmware (`syringe_driver.ino`) initializes CAN communication and manages syringe actuation logic. It coordinates motor commands, reads motion states, and sends actuation control words to the UIM342 motor via CAN.

### 2. UirSimpleCAN Layer

The `UirSimpleCAN` module provides higher-level functions to simplify interaction with the CAN bus. It defines `ReadMLConfig`, `ReadSNConfig`, `ReadPPConfig`, and `SetToUim342`, which handle:

* Querying motor parameters (motion limits, serial numbers, position profiles)
* Setting motor parameters or commands via encoded CAN frames
* Wrapping and parsing CAN frames according to UIM342 communication protocol.

### 3. MCP_CAN Library

This library is a derivative of the open-source Seeed Studio and Cory J. Fowler MCP2515 library. It manages SPI communication with the MCP2515 chip, including:

* Resetting and initializing the controller
* Reading/writing registers and CAN buffers
* Handling transmit/receive operations and error states
* Setting filters, masks, baud rates, and operational modes

The MCP_CAN class provides the interface between Arduino SPI hardware and the physical CAN transceiver.

---

## Hardware Requirements

* **Microcontroller:** Arduino Uno, Mega, or equivalent
* **CAN Transceiver:** MCP2515 module with TJA1050 or similar CAN driver
* **Motor Controller:** UIROBOT UIM342 (CAN-compatible)
* **Power Supply:** As required by syringe motors (typically 24V for UIM342)
* **Syringe Mechanics:** Stepper-motor-driven syringe or linear actuator assembly

---

## Wiring

| Component   | Arduino Pin   | Description               |
| ----------- | ------------- | ------------------------- |
| MCP2515 CS  | D10           | Chip select for SPI       |
| MCP2515 SO  | MISO          | Master In Slave Out       |
| MCP2515 SI  | MOSI          | Master Out Slave In       |
| MCP2515 SCK | SCK           | Serial Clock              |
| MCP2515 INT | D2 (optional) | Interrupt line (optional) |
| UIM342 CANH | CAN High      | Connects to CAN bus line  |
| UIM342 CANL | CAN Low       | Connects to CAN bus line  |

Ensure proper termination (120Ω resistors) at both ends of the CAN bus.

---

## Software Setup

1. **Install Arduino IDE**

   * Download from [https://www.arduino.cc/en/software](https://www.arduino.cc/en/software)

2. **Add Libraries**

   * Add the included `mcp_can` and `UirSimpleCAN` files to your project directory.

3. **Load Firmware**

   * Open `syringe_driver.ino` in Arduino IDE.
   * Select the appropriate board and port.
   * Upload the sketch to your microcontroller.

4. **Serial Monitor**

   * Use 115200 baud for monitoring CAN responses and debug messages.

---

## Example Usage

Once uploaded, the system initializes the MCP2515 CAN controller, sets the baud rate, and starts communicating with the connected UIM342 motor modules. Typical commands include:

* **Read Motion Limit Configuration**

  * Uses `ReadMLConfig(pid, cid, cw, datalen, data)`
* **Read Serial Number**

  * Uses `ReadSNConfig(pid, cid, cw, datalen, data)`
* **Write Control Command**

  * Uses `SetToUim342(can_id, data_len, data)`

Each function sends and receives CAN frames, printing diagnostic messages to the Serial Monitor for validation.

---

## License

This project includes code covered by two licenses:

* **UirSimpleCAN:** MIT License (© 2022 UIROBOT)
* **MCP_CAN Library:** GNU Lesser General Public License v2.1 (© Seeed Technology Inc. and Cory J. Fowler, 2012–2017)

The syringe driver firmware (`syringe_driver.ino`) and integration logic are released under the **MIT License**, permitting open use, modification, and distribution.

---

## Contributing

Contributions to improve code structure, add features (like PID motion control or multi-node synchronization), or enhance documentation are welcome.

To contribute:

1. Fork the repository.
2. Create a feature branch.
3. Commit your changes with descriptive messages.
4. Open a Pull Request.

---

## Acknowledgements

* **UIROBOT** for providing UIM342 motor protocol documentation.
* **Seeed Studio** and **Cory J. Fowler** for the foundational MCP2515 CAN library.
* Open-source CAN and Arduino communities for continued development and testing frameworks.

---

## Disclaimer

This project is intended for educational, research, and prototype automation applications. It is provided **“as is”**, without warranty of any kind. The user assumes all responsibility for safe operation and compliance with applicable standards.
