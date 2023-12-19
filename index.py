import serial
import RPi.GPIO as GPIO
import time

gpio_pin = 17  # You can change this to the desired GPIO pin
serial_port = "/dev/ttyACM1"  # Change this to the appropriate serial port
baud_rate = 921600

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_pin, GPIO.OUT)

def connect_pin_to_ground():
    GPIO.output(gpio_pin, GPIO.LOW)
    print(f"Connected GPIO pin {gpio_pin} to ground.")

def disconnect_pin():
    GPIO.output(gpio_pin, GPIO.HIGH)
    print(f"Disconnected GPIO pin {gpio_pin} from ground.")

def parse_serial_data(data):
    # Extract the MAC address from the received data
    mac_address = data.split(' ')[0].strip()

    # Check if the MAC address is not all zeros
    if mac_address != "00:00:00:00:00:00":
        connect_pin_to_ground()
        # Keep the pin connected for 5 seconds
        time.sleep(5)
        disconnect_pin()

# Set up serial communication
ser = serial.Serial(serial_port, baud_rate)

try:
    while True:
        # Read data from serial port
        serial_data = ser.readline().decode('utf-8').strip()
        print(f"Received data: {serial_data}")

        # Parse and process the received data
        parse_serial_data(serial_data)

except KeyboardInterrupt:
    print("Program terminated by user.")
    disconnect_pin()
    GPIO.cleanup()
    ser.close()
