from dronekit import connect, VehicleMode
import time

print("Connecting...")

vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

print("Connected")

print("Switching to GUIDED_NOGPS mode")
vehicle.mode = VehicleMode("GUIDED_NOGPS")

time.sleep(3)

print("Arming motors...")
vehicle.armed = True

timeout = 20

while timeout > 0:

    if vehicle.armed:
        print("Drone Armed!")
        break

    print("Waiting for arm...")
    time.sleep(1)
    timeout -= 1

if not vehicle.armed:
    print("ARM FAILED")
    vehicle.close()
    exit()

print("Taking off...")

vehicle.simple_takeoff(1)

for i in range(15):
    alt = vehicle.location.global_relative_frame.alt
    print("Altitude:", alt)
    time.sleep(1)

print("Landing...")

vehicle.mode = VehicleMode("LAND")

vehicle.close()