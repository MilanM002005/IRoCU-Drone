from pymavlink import mavutil
import time

master = mavutil.mavlink_connection(
    '/dev/ttyACM0',
    baud=115200
)

master.wait_heartbeat()

print("Arming")

master.arducopter_arm()

master.motors_armed_wait()

print("ARMED")

time.sleep(5)

print("Disarming")

master.arducopter_disarm()

master.motors_disarmed_wait()

print("DISARMED")