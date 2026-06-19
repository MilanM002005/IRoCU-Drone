from dronekit import connect, VehicleMode
import threading
import time
import sys
import tty
import termios

# ==========================================================

# CONFIG

# ==========================================================

CONNECTION_STRING = "/dev/ttyACM0"
BAUDRATE = 115200

TARGET_ALTITUDE = 1.0       # meters
TAKEOFF_THRUST = 0.51

NEUTRAL_TIME = 2            # seconds
STABILIZE_TIME = 8          # seconds
LOITER_TIME = 5            # seconds

MAX_SAFE_ALT = 2.0          # watchdog

abort_flag = threading.Event()
kill_flag = threading.Event()

# ==========================================================

# KEYBOARD SAFETY

# Q = LAND

# R = DISARM

# ==========================================================

def keyboard_listener():

```
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

try:
    tty.setraw(fd)

    while True:

        key = sys.stdin.read(1)

        if key.lower() == "q":
            print("\n[ABORT] LAND requested")
            abort_flag.set()
            break

        if key.lower() == "r":
            print("\n[KILL] DISARM requested")
            kill_flag.set()
            break

finally:
    termios.tcsetattr(
        fd,
        termios.TCSADRAIN,
        old_settings
    )
```

# ==========================================================

# HELPERS

# ==========================================================

def get_ekf_alt(vehicle):

```
try:
    down = vehicle.location.local_frame.down

    if down is not None:
        return -down

except:
    pass

return None
```

def get_lidar_alt(vehicle):

```
try:
    return vehicle.rangefinder.distance
except:
    return None
```

def print_telemetry(vehicle, state):

```
ekf = get_ekf_alt(vehicle)
lidar = get_lidar_alt(vehicle)

ekf_str = "N/A" if ekf is None else f"{ekf:.2f}"
lidar_str = "N/A" if lidar is None else f"{lidar:.2f}"

print(
    f"[{state}] "
    f"MODE={vehicle.mode.name} | "
    f"EKF={ekf_str}m | "
    f"LIDAR={lidar_str}m"
)
```

def wait_for_mode(vehicle, target_mode, timeout=10):

```
start = time.time()

while time.time() - start < timeout:

    if vehicle.mode.name == target_mode:
        return True

    time.sleep(0.2)

return False
```

def send_attitude_target(vehicle, thrust):

```
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
```

# ==========================================================

# SAFETY

# ==========================================================

def emergency_land(vehicle):

```
print("\n[SAFETY] LANDING")

vehicle.mode = VehicleMode("LAND")

while vehicle.armed:

    print_telemetry(vehicle, "LANDING")

    time.sleep(1)

print("[OK] DISARMED")
```

def watchdog(vehicle):

```
alt = get_ekf_alt(vehicle)

if alt is not None and alt > MAX_SAFE_ALT:

    print(
        f"\n[WATCHDOG] "
        f"Altitude exceeded {MAX_SAFE_ALT}m"
    )

    emergency_land(vehicle)
    return True

return False
```

# ==========================================================

# MAIN

# ==========================================================

def main():

```
listener = threading.Thread(
    target=keyboard_listener,
    daemon=True
)

listener.start()

print("=" * 60)
print("AUTONOMOUS FLIGHT TEST")
print("Q = LAND")
print("R = DISARM")
print("=" * 60)

# ------------------------------------------------------
# CONNECT
# ------------------------------------------------------

print("\n[1] CONNECTING")

vehicle = connect(
    CONNECTION_STRING,
    baud=BAUDRATE,
    wait_ready=True
)

print("[OK] CONNECTION SUCCESSFUL")

# ------------------------------------------------------
# GUIDED_NOGPS
# ------------------------------------------------------

print("\n[2] SWITCHING TO GUIDED_NOGPS")

vehicle.mode = VehicleMode("GUIDED_NOGPS")

if not wait_for_mode(vehicle, "GUIDED_NOGPS"):

    print("[ERROR] GUIDED_NOGPS FAILED")
    vehicle.close()
    return

print("[OK] GUIDED_NOGPS ACTIVE")

# ------------------------------------------------------
# ARM
# ------------------------------------------------------

print("\n[3] ARMING MOTORS")

vehicle.armed = True

while not vehicle.armed:

    print("[INFO] WAITING FOR ARM")

    time.sleep(1)

print("[OK] MOTORS ARMED")

# ------------------------------------------------------
# TAKEOFF
# ------------------------------------------------------

print(
    f"\n[4] TAKING OFF "
    f"({TAKEOFF_THRUST*100:.0f}% THRUST)"
)

while True:

    if abort_flag.is_set():
        emergency_land(vehicle)
        return

    if kill_flag.is_set():
        vehicle.armed = False
        return

    if watchdog(vehicle):
        return

    alt = get_ekf_alt(vehicle)

    print_telemetry(vehicle, "TAKEOFF")

    if alt is not None and alt >= TARGET_ALTITUDE:

        print(
            f"\n[OK] TARGET ALTITUDE "
            f"{TARGET_ALTITUDE}m REACHED"
        )

        break

    send_attitude_target(
        vehicle,
        TAKEOFF_THRUST
    )

    time.sleep(0.1)

# ------------------------------------------------------
# NEUTRALIZE
# ------------------------------------------------------

print(
    f"\n[5] NEUTRALIZING "
    f"FOR {NEUTRAL_TIME}s"
)

start = time.time()

while time.time() - start < NEUTRAL_TIME:

    send_attitude_target(vehicle, 0.5)

    print_telemetry(vehicle, "NEUTRAL")

    time.sleep(0.1)

# ------------------------------------------------------
# ALT_HOLD
# ------------------------------------------------------

print("\n[6] SWITCHING TO ALT_HOLD")

vehicle.mode = VehicleMode("ALT_HOLD")

if not wait_for_mode(vehicle, "ALT_HOLD"):

    print("[ERROR] ALT_HOLD FAILED")

    emergency_land(vehicle)
    return

print("[OK] ALT_HOLD ACTIVE")

# ------------------------------------------------------
# STABILIZE
# ------------------------------------------------------

print(
    f"\n[7] STABILIZING "
    f"FOR {STABILIZE_TIME}s"
)

for i in range(STABILIZE_TIME, 0, -1):

    print_telemetry(vehicle, f"STABILIZE {i}")

    time.sleep(1)

# ------------------------------------------------------
# LOITER
# ------------------------------------------------------

print("\n[8] SWITCHING TO LOITER")

vehicle.mode = VehicleMode("LOITER")

if not wait_for_mode(vehicle, "LOITER"):

    print("[ERROR] LOITER FAILED")

    emergency_land(vehicle)
    return

print("[OK] LOITER ACTIVE")

print("\n[9] HOLDING POSITION")

start = time.time()

while time.time() - start < LOITER_TIME:

    if abort_flag.is_set():
        emergency_land(vehicle)
        return

    if kill_flag.is_set():
        vehicle.armed = False
        return

    print_telemetry(vehicle, "LOITER")

    time.sleep(1)

# ------------------------------------------------------
# LAND
# ------------------------------------------------------

print("\n[10] LANDING")

vehicle.mode = VehicleMode("LAND")

if not wait_for_mode(vehicle, "LAND"):

    print("[WARNING] LAND MODE VERIFY FAILED")

while vehicle.armed:

    print_telemetry(vehicle, "LANDING")

    time.sleep(1)

print("\n[11] DISARMED")
print("[12] MISSION COMPLETE")

vehicle.close()
```

if **name** == "**main**":
main()
