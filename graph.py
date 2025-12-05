import serial
import matplotlib.pyplot as plt
from collections import deque

# ---- CONFIG ----
PORT = "/dev/ttyACM0"   # Change to your port if needed
BAUD = 9600
WINDOW = 300            # number of points on screen

# ---- INIT SERIAL ----
ser = serial.Serial("/dev/ttyACM0", 9600)

# ---- DATA BUFFER ----
data = deque([0]*WINDOW, maxlen=WINDOW)

# ---- PLOT SETUP ----
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(data)
ax.set_ylim(0, 100000)      # adjust depending on your sensor values
ax.set_title("Real-time IR Signal")
ax.set_xlabel("Samples")
ax.set_ylabel("Value")

# ---- LOOP ----
while True:
    try:
        raw = ser.readline().decode().strip()
        if raw.isdigit():
            value = int(raw)
            data.append(value)
            line.set_ydata(data)
            ax.set_ylim(min(data) - 1000, max(data) + 1000)
            plt.pause(0.001)
    except KeyboardInterrupt:
        break
    except:
        pass
