# STM32_Self_Balanced_Car

A two-wheeled self-balancing car project implemented on the **STM32F407VET6** microcontroller, designed to maintain upright stability using PID control logic.

The car utilizes real-time data from an IMU sensor (MPU6050) to calculate tilt angles and adjusts the torque of the **GA25-370** motors accordingly via the **L298N** driver.

---

### Authors

Gia Huy Vo @Ghuyv2412

---

### Hardware Specifications

* **MCU:** STM32F407VET6 (ARM Cortex-M4)
* **Motor Driver:** L298N Dual H-Bridge
* **Motors:** GA25-370 DC Gear Motors (12V, 280RPM) with Hall Effect Encoders
* **Sensor:** MPU6050 (3-axis Gyroscope & 3-axis Accelerometer)
* **Power:** 11.1V - 12V Li-po Battery

---

### Folders

**Stm32_firmware**: Contains the core C code developed in STM32CubeIDE.

* **Pid_controller**: Implementation of the PID algorithm (Proportional-Integral-Derivative) for balance and speed control.
* **Encoder_interface**: Modules for reading high-speed pulses from the GA25 motors to calculate velocity.
* **Imu_processing**: Filters (Complementary) to process raw data from the MPU6050 for accurate angle estimation.

**Hardware_design**: Schematics and PCB layout files.

* **Schematics**: Circuit diagrams for the STM32, L298N, and sensor MPU6050.
* **3d_model**: STL files for the car chassis and motor brackets.

**Tuning_tools**: Scripts used to visualize data and tune PID parameters.

---

### Notes

* The system operates at a high frequency (e.g., 5ms control loop) to ensure real-time stability.
* Complementary filtering is used by default for angle calculation to minimize computational overhead.
* Current I/O configuration is optimized for the VET6 development board pinout.
