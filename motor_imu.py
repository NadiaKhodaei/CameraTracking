import cv2
import numpy as np
import time
import serial
from jetson_inference import detectNet
from jetson_utils import videoSource
import serial

def move_servo(x):
    packet = bytearray() ## create packet
    packet.append(0x01) ## start with command to set pwm

    ## this part we convert our pwm value to "int32" which is 4 bytes and add it to the packet
    ## this only works for positive values up to 2,147,483,647 - cause python
    packet += ((x).to_bytes(4,byteorder='big')) #(18-132) /65 center, 120 right, 17 left
    print("I am going to send:", packet)

    stm32.write(packet) ## send
    time.sleep(.1)

    data = stm32.readline().decode("utf-8") ## read response
    print(data)

# set up serial connection
port_name = "/dev/ttyACM0"
stm32 = serial.Serial(port_name, 9600, timeout=1)
print("port open")

# Constants
Kp = 0.2  # Proportional gain for controller

# Initialize camera and neural network
camera = videoSource("csi://0")  # Using CSI camera
net = detectNet("peoplenet-pruned", threshold=0.5)

# Setup Serial Port for STM32
port_name = "/dev/ttyACM0"
stm32 = serial.Serial(port_name, 9600, timeout=1)

def send_pwm(pwm_value):
    packet = bytearray([0x01])  # Command to set PWM
    packet += pwm_value.to_bytes(4, byteorder='big', signed=True)  # Convert to signed big endian bytes
    stm32.write(packet)  # Send packet
    time.sleep(0.1)
    return stm32.readline().decode("utf-8")  # Read response
def request_imu():
    try:
        stm32.write(bytearray([0x03]))  # Send packet
        time.sleep(0.5)
        return stm32.readline().decode("utf-8")  # Read response
    except serial.SerialException as e:
        print("Error reading from serial port:", e)
        return None
print("Starting object centering")
while True:
    img = camera.Capture()
    detections = net.Detect(img, overlay="box,labels,conf")
    annotated_img = np.array(img)
    raw_data = request_imu()
    if raw_data:
        data = raw_data.lstrip('\x00').rstrip('\r\n')
        data_list = data.split(',')
    try:
            # Assuming the first three values are accelerometer readings and the next three are gyroscope readings
            accel_values = [float(value) for value in data_list[:3]]
            gyro_values = [float(value) for value in data_list[3:]]

            print("Accelerometer readings = X: {}, Y: {}, Z: {}".format(*accel_values))
            print("Gyroscope readings = X: {}, Y: {}, Z: {}".format(*gyro_values))
    except ValueError as e:
            print("Error converting data to float:", e)
    else:
        print("Failed to read data from STM32.")
    if detections:
        # Find the detection with the highest confidence
        max_conf_detection = max(detections, key=lambda det: det.Confidence)
        
        detected_box_center = np.array([
            (max_conf_detection.Left + max_conf_detection.Right) / 2, 
            (max_conf_detection.Top + max_conf_detection.Bottom) / 2
        ])
        image_center = np.array([img.width / 2, img.height / 2])
        
        error = image_center - detected_box_center
        pwm_signal = int(Kp * error[0])
        response = send_pwm(pwm_signal)
        print("Sent PWM:", pwm_signal, "Response:", response)

        # max error is img.width/2. max servo range is 18-132
        pwm_signal = int(18 + error[0] / (img.width / 2) * 114)
        pwm_signal = max(18, min(pwm_signal, 132))  # Clamping the value
        move_servo(pwm_signal)

    else:
        print("No detections")
        send_pwm(0)  # No object detected, set PWM to neutral position (adjust this value as needed)

    # cv2.imshow("Annotated Image", annotated_img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break

# Cleanup
cv2.destroyAllWindows()
stm32.close(