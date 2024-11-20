import serial
import time

# Define Arduino port and baud rate
arduino_port = '/dev/ttyUSB1'  # Replace with your Arduino's port (e.g., 'COM3' on Windows, '/dev/ttyUSB0' on Linux)
baud_rate = 115200             # Match the baud rate in your Arduino sketch

# Initialize serial connection
try:
    arduino = serial.Serial(port=arduino_port, baudrate=baud_rate, timeout=1)
    time.sleep(2)  # Give the connection a moment to settle
except Exception as e:
    print(f"Failed to connect to Arduino: {e}")
    exit()

# Function to read and decode data from Arduino
def read_arduino_data():
    try:
        # Read raw bytes
        raw_data = arduino.readline()
        # Decode bytes to string
        decoded_data = raw_data.decode('utf-8').strip()
        return decoded_data
    except UnicodeDecodeError as e:
        print(f"Decoding error: {e}")
        return None

# Continuously read data
print("Reading data from Arduino. Press Ctrl+C to stop.")
try:
    while True:
        data = read_arduino_data()
        if data:
            print(f"Received: {data}")
except KeyboardInterrupt:
    print("\nStopped by user.")


