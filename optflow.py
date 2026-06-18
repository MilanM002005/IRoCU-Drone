# Optical Flow Data readings display

from dronekit import connect
import time

print("Connecting...")

vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

print("Connected")

while True:


    try:

        print("\n----- OPTICAL FLOW DATA -----")

        print("Rangefinder Distance:",
            vehicle.rangefinder.distance)

        print("Rangefinder Voltage:",
            vehicle.rangefinder.voltage)

        print("Altitude:",
            vehicle.location.global_relative_frame.alt)

        print("Velocity:",
            vehicle.velocity)

        print("Mode:",
            vehicle.mode.name)

        time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping...")
        break


vehicle.close()
