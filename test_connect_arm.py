# To test connection to drone. First it changes mode to stabilize and then arms the drone.


from dronekit import connect, VehicleMode
import time

print("Connecting...")

vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

print("Connected")

print("Changing to STABILIZE mode")
vehicle.mode = VehicleMode("STABILIZE")

time.sleep(3)

print("Arming motors")
vehicle.armed = True

time.sleep(5)

print("Armed status:", vehicle.armed)

vehicle.close()
