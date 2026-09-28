#STM32H7A3ZI-Q + MPU6050

1) Project Overview

This project is developed using STM32CubeIDE and STM32 HAL for the STM32H7A3ZI-Q microcontroller.
The MPU6050 is connected to the STM32 through the I²C interface and provides:

3-axis accelerometer data
3-axis gyroscope data
Raw sensor measurements
Accelerometer measurements in g
Gyroscope measurements in °/s
Basic tilt/angle calculation

The project is part of the development work for a self-balancing Segway control system, where IMU measurements are used for attitude estimation and control.

2) Hardware
  a) Microcontroller - STM32 NUCLEO-H7A3ZI-Q

MCU: STM32H7A3ZITx
ARM Cortex-M7
STM32H7 series
STM32CubeIDE development environment
IMU Sensor

  b) MPU6050

3-axis accelerometer
3-axis gyroscope
I²C communication
Default I²C address: 0x68
Operating logic voltage: 3.3 V

3) Hardware Connection
   a) MPU6050 to	STM32H7A3ZI-Q
VCC	3.3 V
GND	GND
SCL	I²C SCL
SDA	I²C SDA

The exact GPIO configuration is defined in the STM32CubeMX .ioc project file.

4) Communication
   a) I²C

The MPU6050 communicates with the STM32 using I²C.

Default device address: 0x68

The firmware checks the MPU6050 device and reads the WHO_AM_I register during initialization.

Expected value: WHO_AM_I = 0x68
Sensor Configuration

The current firmware configures the MPU6050 approximately as follows:
Accelerometer
Range: ±2 g
Gyroscope
Range: ±250 °/s

These settings provide high sensitivity for low-range motion measurement, which is useful for tilt and balancing applications.

Example Sensor Output

Example raw sensor data: RAW | AX: -152 AY: -576 AZ: 15564
                               GX: -124 GY: 130 GZ: 169

Converted physical measurements: PHY | A[g] X=0.009 Y=0.035 Z=0.949
                                     | G[dps] X=0.94 Y=0.99 Z=1.29

Example accelerometer-based pitch: ACCEL PITCH = 0.55 deg

5) Software
Development Environment
STM32CubeIDE
STM32CubeMX
C programming language
STM32 HAL
ARM GCC toolchain

6) Project Configuration
The STM32CubeMX configuration is included in: MPU6050_STM.ioc
