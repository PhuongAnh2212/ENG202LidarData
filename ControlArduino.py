import serial
import time 
import csv 
import threading 
from pynput import keyboard 
import pandas as pd 
from matplotlib import pyplot as plt 

with open('SensorData.csv', mode = 'a', newline='') as sensor_file:
        sensor_writer = csv.writer(sensor_file, delimiter=',', quotechar='"', quoting = csv.QUOTE_MINIMAL)
        sensor_writer.writerow(["x","y", "timestamp"])
arduino = serial.Serial(port = '/dev/cu.usbserial-130', baudrate = 115200, timeout = 1)
time.sleep(2)

def extract_data(data):
    try: 
        x,y = data.split(",")
        return float(x),float(y)
    except ValueError: 
          print("Invalid data received")
          return None, None 

def read_data(): #read data from Arduino and log to CSV
      while True: 
            data = arduino.readline().decode('utf-8').strip()
            if data: 
                  x,y = extract_data(data)
                  if x is not None and y is not None: 
                        timestamp=time.asctime()
                        print(f"Logged data: x = {x},y={y}, Timestamp={timestamp}")

                        with open('SensorData.csv', mode ='a',newline='') as sensor_file:
                              sensor_writer = csv.writer(sensor_file, delimiter = ',',quotechar='"', quoting = csv.QUOTE_MINIMAL)
                              sensor_writer.writerow([x,y,timestamp])


def on_press(key):
        try: 
            if key.char == 'w':
                  arduino.write(b'2')
                  print("moving forward")
            elif key.char == 'x':
                  arduino.write(b'3')
                  print('moving backward')
            elif key.char == 'a':
                  arduino.write(b'4')
                  print("turning left")
            elif key.char == 'd':
                  arduino.write(b'5')
                  print("turning right")
            elif key.char == 's':
                  arduino.write(b'0')
                  print('robot stopped')
        except AttributeError:
              print("ignoring other key...")

def on_release(key):
      if key == keyboard.Key.esc: 
            print("Existing..")
            return False 

def plot_data():
    try:
        # Load data from the CSV file
        df = pd.read_csv('SensorData.csv')
        x = df['x']
        y = df['y']

        # Create a plot
        plt.figure(figsize=(8, 6))
        
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
        
        # Show the plot
        plt.show()

    except FileNotFoundError:
        print("SensorData.csv not found.")
    except KeyError:
        print("Unexpected CSV format. Ensure columns are labeled 'x' and 'y'.")


read_thread = threading.Thread(target= read_data, daemon= True)
read_thread.start()
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
      listener.join()

plot_data()



    