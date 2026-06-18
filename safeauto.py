from dronekit import connect, VehicleMode
import time

TARGET_ALT_M      = 1.0    # Target altitude in metres
ALT_HOLD_SECONDS  = 5      # Hold time at 1m
TAKEOFF_THRUST    = 0.52   # 52% — slow liftoff thrust



def send_attitude_target(vehicle, thrust):
    """
    Send level attitude + thrust via MAVLink.
    Used ONLY during initial liftoff in GUIDED_NOGPS.
    Quaternion [1,0,0,0] = perfectly level.
    """
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        1,
        1,
        0b00000000,
        [1, 0, 0, 0],
        0, 0, 0,
        thrust
    )
    vehicle.send_mavlink(msg)


def wait_for_mode(vehicle, mode_name, timeout=5):
    """
    Wait until vehicle confirms mode switch.
    Returns True if successful, False if timed out.
    """
    start = time.time()
    while vehicle.mode.name != mode_name:
        if time.time() - start > timeout:
            return False
        time.sleep(0.2)
    return True


# ─── 1. CONNECT ───────────────────────────────────────
print("Connecting to flight controller...")
vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)
print(f"✅ Connected. | Battery: {vehicle.battery.voltage:.1f}V")


# ─── 2. MODE → GUIDED_NOGPS ───────────────────────────
print("Switching to GUIDED_NOGPS...")
vehicle.mode = VehicleMode("GUIDED_NOGPS")
if not wait_for_mode(vehicle, "GUIDED_NOGPS"):
    print(" GUIDED_NOGPS failed! Aborting.")
    vehicle.close()
    exit()
print(f" Mode: {vehicle.mode.name}")
time.sleep(1)


# ─── 3. ARM ───────────────────────────────────────────
print("Arming motors...")
vehicle.armed = True
while not vehicle.armed:
    print("  Waiting for arm...")
    time.sleep(0.5)
print("✅ ARMED.")


# ─── 4. LIFTOFF in GUIDED_NOGPS ───────────────────────
# Just get off the ground safely (0.3m)
LIFTOFF_ALT = 0.3  

print(f"Lifting off to {LIFTOFF_ALT}m (optflow activation height)...")
while True:
    alt = vehicle.location.global_relative_frame.alt or 0.0
    print(f"  Altitude: {alt:.2f}m")

    if alt >= LIFTOFF_ALT:
        print(f"✅ Liftoff altitude {alt:.2f}m reached!")
        break

    send_attitude_target(vehicle, TAKEOFF_THRUST)
    time.sleep(0.1)


# ─── 5. SWITCH TO LOITER ──────────────────────────────

print("Switching to LOITER (optical flow + rangefinder)...")
vehicle.mode = VehicleMode("LOITER")
if not wait_for_mode(vehicle, "LOITER"):
    print(f"LOITER failed! Still in {vehicle.mode.name}")
    print("  → Landing for safety...")
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()
    exit()
print(f"✅ Mode: {vehicle.mode.name}")


# ─── 6. CLIMB TO TARGET ALT IN LOITER ────────────────

print(f"LOITER active — climbing to {TARGET_ALT_M}m...")
while True:
    alt = vehicle.location.global_relative_frame.alt or 0.0
    print(f"  LOITER climbing — Altitude: {alt:.2f}m / {TARGET_ALT_M}m")

    if alt >= TARGET_ALT_M * 0.95:
        print(f"✅ Target altitude {alt:.2f}m reached in LOITER!")
        break

    time.sleep(0.2)   


# ─── 7. SWITCH TO ALT_HOLD ────────────────────────────
print("Switching to ALT_HOLD...")
vehicle.mode = VehicleMode("ALT_HOLD")
if not wait_for_mode(vehicle, "ALT_HOLD"):
    print(f" ALT_HOLD failed! Still in {vehicle.mode.name}")
    print("  → Landing for safety...")
    vehicle.mode = VehicleMode("LAND")
    vehicle.close()
    exit()
print(f"✅ Mode confirmed: {vehicle.mode.name}")
print(f"Holding altitude for {ALT_HOLD_SECONDS} seconds...")


# ─── 8. HOLD 5 SECONDS ────────────────────────────────
hold_start = time.time()
while time.time() - hold_start < ALT_HOLD_SECONDS:
    alt = vehicle.location.global_relative_frame.alt or 0.0
    elapsed = time.time() - hold_start
    print(f"  ALT_HOLD — {elapsed:.1f}s/{ALT_HOLD_SECONDS}s | alt: {alt:.2f}m | mode: {vehicle.mode.name}")
    time.sleep(0.5)
print("✅ Hold complete.")


# ─── 9. LAND ──────────────────────────────────────────
print("Switching to LAND...")
vehicle.mode = VehicleMode("LAND")
if not wait_for_mode(vehicle, "LAND"):
    print(" LAND mode failed — manual intervention needed!")
    vehicle.close()
    exit()
print(f"✅ Mode: {vehicle.mode.name} — descending...")

while True:
    alt = vehicle.location.global_relative_frame.alt or 0.0
    print(f"  Descending — altitude: {alt:.2f}m")
    if alt <= 0.1:
        print("✅ Landed safely.")
        break
    time.sleep(0.5)


vehicle.close()
print("Done.")