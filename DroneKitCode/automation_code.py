# Take off in GUIDED_NOGPS, switch to LOITER and hold for 30 seconds, then land

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import threading
import sys
import tty
import termios

# ---------------- CONFIG ----------------

TARGET_ALT = 1.0
LOITER_TIME = 20
TAKEOFF_THRUST = 0.51

# ----------------------------------------

abort_flag = threading.Event()
kill_flag  = threading.Event()


def keyboard_listener():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            key = sys.stdin.read(1)
            if key.lower() == 'q':
                print("\n[Q] Emergency abort triggered")
                abort_flag.set()
                break
            if key.lower() == 'r':
                print("\n[R] KILL SWITCH — disarming immediately")
                kill_flag.set()
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def get_ekf_altitude(vehicle):
    ned = vehicle.location.local_frame
    if ned is None or ned.down is None:
        return None
    return -ned.down


def get_rangefinder_alt(vehicle):
    if vehicle.rangefinder is None:
        return None
    return vehicle.rangefinder.distance


def print_header():
    print("\n" + "-" * 55)
    print(f"  {'TIME':<8} {'PHASE':<12} {'EKF ALT':>10} {'LIDAR ALT':>10} {'TIMER':>8}")
    print("-" * 55)


def print_status(phase, ekf_alt, rf_alt, elapsed=None, total=None):
    ts       = time.strftime("%H:%M:%S")
    ekf_str  = f"{ekf_alt:.2f}m" if ekf_alt is not None else "N/A"
    rf_str   = f"{rf_alt:.2f}m"  if rf_alt  is not None else "N/A"
    time_str = f"{elapsed}/{total}s" if elapsed is not None else ""
    print(f"  {ts:<8} {phase:<12} {ekf_str:>10} {rf_str:>10} {time_str:>8}")


def set_ekf_origin(vehicle):
    vehicle._master.mav.set_gps_global_origin_send(
        vehicle._master.target_system,
        0, 0, 0
    )
    print("  EKF origin set")


def send_attitude_target(vehicle, thrust):
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 1, 1,
        0b00000000,
        [1, 0, 0, 0],
        0, 0, 0,
        thrust
    )
    vehicle.send_mavlink(msg)


def send_loiter_hold(vehicle):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b111111111000,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)


def wait_for_mode(vehicle, mode_name, timeout=10):
    start = time.time()
    while vehicle.mode.name != mode_name:
        if time.time() - start > timeout:
            return False
        time.sleep(0.2)
    return True


def kill_motors(vehicle):
    print("\n[KILL] Disarming motors NOW")
    vehicle.armed = False
    time.sleep(0.5)
    print("[KILL] Motors cut")


def emergency_land(vehicle):
    print("\n[ABORT] Switching to LAND...")
    vehicle.mode = VehicleMode("LAND")
    wait_for_mode(vehicle, "LAND", timeout=5)

    landing_start = time.time()
    while vehicle.armed:
        if kill_flag.is_set():
            kill_motors(vehicle)
            return
        ekf_alt = get_ekf_altitude(vehicle)
        rf_alt  = get_rangefinder_alt(vehicle)
        elapsed = int(time.time() - landing_start)
        print_status("EMRG LAND", ekf_alt, rf_alt, elapsed, 30)
        if time.time() - landing_start > 30:
            print("\n  Landing timeout — disarming manually")
            vehicle.armed = False
            break
        time.sleep(1)
    print("\n  Disarmed")


# ── MAIN ──────────────────────────────────────────────────────

listener = threading.Thread(target=keyboard_listener, daemon=True)
listener.start()

print("=" * 58)
print("  Press Q to abort and land")
print("  Press R to kill motors immediately")
print("=" * 58)

try:
    print("\n  Connecting...")
    vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)
    print(f"  Connected  |  Battery: {vehicle.battery.voltage}V")

    # ARM
    print("\n  Switching to GUIDED_NOGPS...")
    vehicle.mode = VehicleMode("GUIDED_NOGPS")

    if not wait_for_mode(vehicle, "GUIDED_NOGPS"):
        print("  Mode change failed")
        vehicle.close()
        exit()

    print("  Arming motors...")
    vehicle.armed = True

    while not vehicle.armed:
        if kill_flag.is_set():
            kill_motors(vehicle)
            vehicle.close()
            exit()
        if abort_flag.is_set():
            print("  Abort during arm — disarming")
            vehicle.armed = False
            vehicle.close()
            exit()
        print("  Waiting for arm...")
        time.sleep(1)

    print("  ARMED")

    set_ekf_origin(vehicle)
    time.sleep(0.5)

    # TAKEOFF
    print(f"\n  Taking off to {TARGET_ALT}m")
    print_header()

    while True:
        if kill_flag.is_set():
            kill_motors(vehicle)
            vehicle.close()
            exit()
        if abort_flag.is_set():
            emergency_land(vehicle)
            vehicle.close()
            exit()

        ekf_alt = get_ekf_altitude(vehicle)
        rf_alt  = get_rangefinder_alt(vehicle)

        if ekf_alt is None:
            print("  EKF altitude not ready...")
            time.sleep(0.2)
            continue

        print_status("TAKEOFF", ekf_alt, rf_alt)

        if ekf_alt >= TARGET_ALT:
            print("=" * 58)
            print("  Target altitude reached")
            break

        send_attitude_target(vehicle, TAKEOFF_THRUST)
        time.sleep(0.1)

    # LOITER
    print("\n  Switching to LOITER...")
    vehicle.mode = VehicleMode("LOITER")

    if not wait_for_mode(vehicle, "LOITER"):
        print("  LOITER failed, landing for safety")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()
        exit()

    print("  LOITER ACTIVE")
    print_header()

    start = time.time()

    while time.time() - start < LOITER_TIME:
        if kill_flag.is_set():
            kill_motors(vehicle)
            vehicle.close()
            exit()
        if abort_flag.is_set():
            emergency_land(vehicle)
            vehicle.close()
            exit()

        ekf_alt = get_ekf_altitude(vehicle)
        rf_alt  = get_rangefinder_alt(vehicle)
        elapsed = int(time.time() - start)
        print_status("LOITER", ekf_alt, rf_alt, elapsed, LOITER_TIME)

        send_loiter_hold(vehicle)
        time.sleep(0.1)

    # LAND
    print("=" * 58)
    print("\n  Switching to LAND...")
    vehicle.mode = VehicleMode("LAND")

    if not wait_for_mode(vehicle, "LAND"):
        print("  LAND mode failed")
        vehicle.close()
        exit()

    print("  LANDING")
    print_header()

    landing_start = time.time()

    while vehicle.armed:
        if kill_flag.is_set():
            kill_motors(vehicle)
            break
        if abort_flag.is_set():
            print("\n  Abort during landing — forcing disarm")
            vehicle.armed = False
            break

        ekf_alt = get_ekf_altitude(vehicle)
        rf_alt  = get_rangefinder_alt(vehicle)
        elapsed = int(time.time() - landing_start)
        print_status("LANDING", ekf_alt, rf_alt, elapsed, 30)

        if time.time() - landing_start > 30:
            print("\n  Landing timeout — disarming manually")
            vehicle.armed = False
            break

        time.sleep(1)

    print("=" * 58)
    print("\n  Mission Complete")
    print("=" * 58)

except Exception as e:
    print(f"\n  ERROR: {e}")
    try:
        if kill_flag.is_set():
            kill_motors(vehicle)
        else:
            emergency_land(vehicle)
    except:
        pass

finally:
    try:
        vehicle.close()
    except:
        pass