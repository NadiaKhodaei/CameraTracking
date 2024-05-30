<<<<<<< HEAD
# cameraTracking
=======

# CameraTracking STM32 Project

## Overview

This project consists of multiple Python scripts designed to interface with an STM32 microcontroller and perform various tasks such as capturing camera images, tracking objects, and reading gyroscope and accelerometer data. The scripts utilize libraries like OpenCV, Jetson Inference, and PySerial for handling camera input, object detection, and serial communication, respectively.

## Contents

1. **camera.py**
2. **cameraTracking.py**
3. **motor_imu.py**
4. **test.py**

## Requirements

- Python 3.x
- OpenCV
- NumPy
- PySerial
- Jetson Inference
- Jetson Utils

## Installation

1. **Clone the repository:**
   ```sh
   git clone https://github.com/NadiaKhodaei/gyrodataStm32.git
   cd gyrodataStm32
   ```

2. **Install dependencies:**
   ```sh
   pip install opencv-python numpy pyserial jetson-inference jetson-utils
   ```

## Usage

### camera.py

This script captures video from the default camera and displays it in a window.

**Usage:**
```sh
python camera.py
```

### cameraTracking.py

This script uses a CSI camera and Jetson Inference to detect objects and control a servo motor to center the detected object.

**Usage:**
```sh
python cameraTracking.py
```

### motor_imu.py

This script captures video, detects objects using Jetson Inference, reads IMU data from an STM32 microcontroller, and controls a servo motor to center the detected object.

**Usage:**
```sh
python motor_imu.py
```

### test.py

This script connects to an STM32 microcontroller via serial communication, sends commands to the microcontroller, and processes the received gyroscope data.

**Usage:**
```sh
python test.py
```

## File Descriptions

### camera.py

This script captures video from the default camera and displays it. Press 'q' to exit the video display.

### cameraTracking.py

This script uses a CSI camera and a pre-trained model from Jetson Inference to detect objects. It communicates with an STM32 microcontroller to control a servo motor based on the detected object's position.

### motor_imu.py

This script captures video, detects objects using Jetson Inference, and reads IMU data from an STM32 microcontroller. It controls a servo motor to center the detected object based on the IMU data.

### test.py

This script connects to an STM32 microcontroller via serial communication, sends commands to the microcontroller, and processes the received gyroscope data. It prints the gyroscope data and performs baseline noise measurement.
>>>>>>> master
