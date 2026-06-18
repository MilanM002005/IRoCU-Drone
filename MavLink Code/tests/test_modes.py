from pymavlink import mavutil
import time

master = mavutil.mavlink_connection(
    '/dev/ttyACM0',
    baud=115200
)

master.wait_heartbeat()

mode_mapping = master.mode_mapping()

for mode in ["STABILIZE", "GUIDED_NOGPS"]:

    mode_id = mode_mapping[mode]

    print(f"Switching to {mode}")

    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    time.sleep(3)