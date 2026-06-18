from pymavlink import mavutil

master = mavutil.mavlink_connection(
    '/dev/ttyACM0',
    baud=115200
)

master.wait_heartbeat()

print("Listening...")

while True:

    msg = master.recv_match(blocking=True)

    if msg:
        print(msg.get_type())