import serial
import time
import csv
import threading
import keyboard  # Import the keyboard library
import pandas as pd
from matplotlib import pyplot as plt
import os

# Create CSV file and add headers
with open('SensorData.csv', mode = 'a', newline='') as sensor_file:
        sensor_writer = csv.writer(sensor_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        sensor_writer.writerow(["x", "y", "theta", "timestamp"])

# Initialize Arduino connection (adjust port for Ubuntu)
arduino = serial.Serial(port='/dev/ttyUSB1', baudrate=115200, timeout=1)  # Ubuntu uses /dev/ttyUSB* or /dev/ttyACM*
time.sleep(2)

# Function to extract x and y data from Arduino
def extract_data(data):
    try: 
        x, y, theta = data.split(",")
        return float(x), float(y), float(theta)
    except ValueError: 
          print("Invalid data received")
          return None, None, None 

# Read data from Arduino and log to CSV
def read_data(): 
    while True: 
        # data = arduino.readline().decode('utf-8').strip()
        data = arduino.readline() 
        print(f"Raw data type: {type(data)}")  # This should print <class 'bytes'>
        data = arduino.readline().decode('UTF-8').strip()
            
        if data: 
            x, y, theta = extract_data(data)
            if x is not None and y is not None: 
                timestamp = time.asctime()
                print(f"Logged data: x = {x}, y = {y}, theta = {theta}, Timestamp = {timestamp}")

                # Write data to CSV
                with open('SensorData.csv', mode='a', newline='') as sensor_file:
                    sensor_writer = csv.writer(sensor_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    sensor_writer.writerow([x, y, theta, timestamp])

# Define the key press actions using the keyboard library
def handle_keypress():
    while True:
        if keyboard.is_pressed('w'):  # If 'w' is pressed
            arduino.write(b'2')
            print("Moving forward")
        elif keyboard.is_pressed('x'):  # If 'x' is pressed
            arduino.write(b'3')
            print('Moving backward')
        elif keyboard.is_pressed('a'):  # If 'a' is pressed
            arduino.write(b'4')
            print("Turning left")
        elif keyboard.is_pressed('d'):  # If 'd' is pressed
            arduino.write(b'5')
            print("Turning right")
        elif keyboard.is_pressed('s'):  # If 's' is pressed
            arduino.write(b'0')
            print('Robot stopped')
        elif keyboard.is_pressed('esc'):  # If ESC is pressed
            print("Exiting...")
            break  # Exit the loop when ESC is pressed

# Plot data from CSV
def plot_data():
    try:
        # Load data from the CSV file
        df = pd.read_csv('SensorData.csv')
        x = df['x']
        y = df['y']

        # Create a plot
        plt.figure(figsize=(8, 8))
        
        # Add a line connecting the points
        plt.plot(x, y, marker='o', markersize=4, linestyle='-', color='blue', label="Path")

        # Customize the plot
        plt.xlabel('X Coordinate (meters)')
        plt.ylabel('Y Coordinate (meters)')
        plt.title('Robot Traveling Path')
        plt.legend()
        plt.grid(True)
        plt.axis('equal') 
        plt.tight_layout()

        # Save the plot to the file
        plot_filename = os.path.join(os.getcwd(), "robot_path_plot.png")
        plt.savefig(plot_filename)
        print(f"Plot saved as {plot_filename}")

    except FileNotFoundError:
        print("SensorData.csv not found.")
    except KeyError:
        print("Unexpected CSV format. Ensure columns are labeled 'x' and 'y'.")

# Start data reading thread
read_thread = threading.Thread(target=read_data, daemon=True)
read_thread.start()

# Start handling key presses
handle_keypress()

# After exiting keypress loop, plot the data
plot_data()
