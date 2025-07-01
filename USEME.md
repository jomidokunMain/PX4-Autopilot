
# WheelEncoder

This repository contains the hardware design and firmware for a **Wheel Encoder system** developed for **BMROVER**. It was initially created to integrate wheel encoder data into PX4 messages using an Arduino-based setup.

The Wheel Encoder system includes:
- A custom acquisition board with **4 inputs** for **Magnetic Hall Encoders (Dual Hall Phase)**.
- Interfaces including **I2C, UART, and CAN**.
- Mounting hardware for the BMROVER vehicle.
- PX4 driver integration for seamless autopilot support on the BMROVER platform.

---

## Usage Overview

### 1. Install Firmware on PX4 Board

#### Option 1: Build from Source

1. **Clone the Repository**

   ```bash
   git clone https://github.com/jomidokunMain/PX4-Autopilot.git -b wheelEncoder --recursive
   ```

2. **Build Firmware**

   Navigate into the cloned directory:

   ```bash
   cd PX4-Autopilot
   ```

   - For **PX4-v5 board**:
     - Without airframe:
       ```bash
       make px4_fmu-v5_default
       ```
     - With airframe:
       ```bash
       make px4_fmu-v5_rover
       ```

   - For **PX4-v6c board**:
     - Without airframe:
       ```bash
       make px4_fmu-v6c_default
       ```
     - With airframe:
       ```bash
       make px4_fmu-v6c_rover
       ```

3. **Upload Firmware (Flash the Board)**

   Append `upload` to the make command to flash the firmware via USB. For example:

   ```bash
   make px4_fmu-v6c_default upload
   ```

   > ⚠️ Ensure you use the same target (`board version v5 or v5c ` and `default` or `rover`) that you used during compilation.

---

#### Option 2: Use Precompiled Firmware

Precompiled binaries are provided in the [`firmware_board`](firmware_board) directory.

1. Download the appropriate firmware file for your board.
2. Open **QGroundControl (QGC)**.
3. Connect your PX4 board.
4. Go to **Vehicle Setup → Firmware → Custom Firmware**.
5. Upload the downloaded binary and follow the on-screen instructions.

---

## Repository Structure

- **Orginal PX-Autopilot/**

- **firmware_board/**
  **Board:** Precompiled binaries for different PX4 targets for quick flashing via QGroundControl.
  - **Hardware/**
	- **Arduino/**
	Contains Arduino sketches for the wheel encoder and a test I2C encoder reader.


---

## PX4 Firmware Fork

A custom fork of PX4 (based on v1.14 Beta) includes the Wheel Encoder driver and integrates it using standard `WheelEncoders.msg` uORB messages.

- PX4 Fork Repository: [https://github.com/jomidokunMain/PX4-Autopilot/tree/wheelEncoder](https://github.com/jomidokunMain/PX4-Autopilot/tree/wheelEncoder)

---

## License

This project is released under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- Developed as part of the BMROVER project at [BIMI Lab](https://mrover.org/).
- Inspired by the need for robust wheel encoder integration in rover applications.
