from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

print("Connecting...")

vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

print("Connected")

print("Switching to GUIDED_NOGPS")
vehicle.mode = VehicleMode("GUIDED_NOGPS")

time.sleep(2)

print("Arming motors...")
vehicle.armed = True

while not vehicle.armed:
    print("Waiting for arming...")
time.sleep(1)

print("ARMED")

def send_attitude_target(thrust):


    msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        1,
        1,
        0b00000000,
        [1, 0, 0, 0],
        0,
        0,
        0,
        thrust
    )

    vehicle.send_mavlink(msg)


print("Increasing thrust...")

for i in range(50):


    send_attitude_target(0.7)

    alt = vehicle.location.global_relative_frame.alt

    print("Altitude:", alt)

    time.sleep(0.1)


print("Reducing thrust...")

for i in range(30):


    send_attitude_target(0.5)

    time.sleep(0.1)


print("Landing...")

vehicle.mode = VehicleMode("LAND")

vehicle.close()
