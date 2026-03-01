import serial
import numpy as np
import matplotlib.pyplot as plt


#init 
ROWS = 2
COLS = 2
# Change COM port to Arduino
ser = serial.Serial('COM4', 115200, timeout=1)

plt.ion()  # interactive mode
fig, ax = plt.subplots()
data = np.zeros((ROWS, COLS))  # Initialize data array
heatmap = ax.imshow(data, cmap='hot', vmin=0, vmax=1023)
plt.colorbar(heatmap)

while True:
    row_data = []
    while True:
        line = ser.readline().decode().strip()
        if not line:
            continue
        if line == "---":
            break
        values = [int(x) for x in line.split(",")]
        row_data.append(values)
    
    if row_data:
        data = np.array(row_data)
        heatmap.set_data(data)
        plt.draw()
        plt.pause(0.05)

