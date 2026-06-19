from pymavlink import mavutil

print("Connecting...")

master = mavutil.mavlink_connection(
    '/dev/ttyACM0',
    baud=115200
)

master.wait_heartbeat()

print(
    f"Connected!\n"
    f"System ID: {master.target_system}\n"
    f"Component ID: {master.target_component}"
)