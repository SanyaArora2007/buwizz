#!/home/sanyaarora/buwizz/venv/bin/python3
"""Continuously print distance from the VL53L0X time-of-flight sensor.

Run it on the device with either:
    ./sensor.py
    python3 sensor.py
Press Ctrl+C to stop.
"""
import time
import board
import adafruit_vl53l0x

i2c = board.I2C()                       # uses pin 3 (SDA) and pin 5 (SCL)
tof = adafruit_vl53l0x.VL53L0X(i2c)     # VL53L0X (shares address 0x29 with L1X)

print("Reading... Ctrl+C to stop", flush=True)
try:
    while True:
        mm = tof.range                  # millimeters to target
        print(f"{mm} mm  ({mm / 10:.1f} cm)", flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopped.")
