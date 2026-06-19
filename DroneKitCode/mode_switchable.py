# Both modes for Loiter and ALT_HOLD are included here for demonstration.
# Change the MODE variable to switch between them. Example : MODE = "ALT_HOLD" or MODE = "LOITER".
# LOITER mode requires GPS lock (outdoors), while ALT_HOLD works indoors using bar

from dronekit import connect, VehicleMode
import time

# ─── CONFIG ───────────────────────────────────────────
TARGET_ALT_M    = 1.0     # Target altitude in metres
LOITER_SECONDS  = 30       # How long to loiter at target
TAKEOFF_THRUST  = 0.52    # ~52% — slow safe climb indoors
MODE = "ALT_HOLD"  # "LOITER" or "ALT_HOLD"
# ──────────────────────────────────────────────────────
# ⚠️  LOITER needs GPS lock (outdoor use)
#     For INDOOR use → change MODE to "ALT_HOLD"
#     ALT_HOLD uses barometer only, no GPS needed
# ──────────────────────────────────────────────────────
    # Change to "ALT_HOLD" for indoors


def send_attitude_target(vehicle, thrust):
    """
    Send level attitude + thrust via MAVLink.
    Quaternion [1,0,0,0] = perfectly level, no rotation.
    Thrust: 0.0 (off) → 1.0 (full). Keep low indoors!
    """
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        1,
        1,
        0b00000000,
        [1, 0, 0, 0],   # level quaternion
        0, 0, 0,         # zero roll/pitch/yaw rates
        thrust
    )
    vehicle.send_mavlink(msg)


# ─── 1. CONNECT ───────────────────────────────────────
print("Connecting to flight controller...")
vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)
print("Connected.")


# ─── 2. MODE → GUIDED_NOGPS ───────────────────────────
print("Switching to GUIDED_NOGPS for takeoff...")
vehicle.mode = VehicleMode("STABILIZE")
time.sleep(2)


# ─── 3. ARM ───────────────────────────────────────────
print("Arming motors...")
vehicle.armed = True
while not vehicle.armed:
    print("  Waiting for arm confirmation...")
    time.sleep(0.5)
print("ARMED.")


# ─── 4. TAKEOFF — climb until 1 m ────────────────────
print(f"Taking off... climbing to {TARGET_ALT_M} m")
while True:
    alt = vehicle.location.global_relative_frame.alt or 0.0
    print(f"  Altitude: {alt:.2f} m")

    if alt >= TARGET_ALT_M * 0.95:    # 95% of target = close enough
        print(f"  Target altitude {TARGET_ALT_M} m reached!")
        break

    send_attitude_target(vehicle, TAKEOFF_THRUST)
    time.sleep(0.1)


# ─── 5. SWITCH TO LOITER (or ALT_HOLD) ───────────────
print(f"Switching to {MODE} mode...")
vehicle.mode = VehicleMode(MODE)
time.sleep(1)  # Give FC time to stabilise in new mode

# Confirm mode switched successfully
if vehicle.mode.name != MODE:
    print(f"WARNING: Mode switch failed! Still in {vehicle.mode.name}")
    print("Proceeding to land for safety...")
    vehicle.mode = VehicleMode("LAND")
else:
    print(f"{MODE} active. Holding position for {LOITER_SECONDS} seconds...")

    # ─── 6. LOITER for 5 seconds ──────────────────────
    loiter_start = time.time()
    while time.time() - loiter_start < LOITER_SECONDS:
        alt = vehicle.location.global_relative_frame.alt or 0.0
        elapsed = time.time() - loiter_start
        print(f"  Loitering — {elapsed:.1f}s / {LOITER_SECONDS}s  |  alt: {alt:.2f} m")
        time.sleep(0.5)

    print("Loiter complete.")


# ─── 7. LAND ──────────────────────────────────────────
print("Switching to LAND mode...")
vehicle.mode = VehicleMode("LAND")

while True:
    alt = vehicle.location.global_relative_frame.alt or 0.0
    print(f"  Descending — altitude: {alt:.2f} m")
    if alt <= 0.1:
        print("Landed safely.")
        break
    time.sleep(0.5)


vehicle.close()
print("Done.")