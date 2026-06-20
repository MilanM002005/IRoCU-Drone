"""
Mission: arm -> GUIDED_NOGPS -> climb to 1 m -> hold 5 s -> land -> disarm

Sensor setup this targets: optical flow (horizontal velocity / relative
position, EK3_SRC1_VELXY=5) + rangefinder (height above ground,
EK3_SRC1_POSZ=2). No GPS.

GUIDED_NOGPS does not accept position/velocity setpoints - only attitude +
thrust (SET_ATTITUDE_TARGET). So "takeoff" and "land" here are a manual
closed-loop thrust controller using the EKF's fused altitude
(LOCAL_POSITION_NED.z, which is rangefinder-sourced on this vehicle) as
feedback. This is NOT the same as MAV_CMD_NAV_TAKEOFF / the LAND mode.

>>> SAFETY - READ BEFORE RUNNING <<<
- First run tethered, over a soft surface, ideally with props clipped/off,
  just to watch the thrust controller behave before trusting it in flight.
- Keep the RC in hand and ready to flip to STABILIZE/LAND or cut power.
- HOVER_THRUST below is a *guess* (0.50). Read the real value first:
      print(drone.get_param("MOT_THST_HOVER"))
  and set HOVER_THRUST to that before flying.
- Start with CLIMB_KP lower than you think you need.
- Flight surface matters: optical flow needs texture and even lighting.
  A flat painted floor or glass will degrade flow quality and is the
  most likely real-world failure mode here - that's why this script
  actively monitors flow quality and rangefinder/EKF agreement, not
  just the EKF flag summary, and aborts to a landing if either degrades
  for a sustained period mid-flight.
"""

import time
from drone import Drone

# ---- Tunable parameters -------------------------------------------------
TARGET_ALT = 1.0        # metres
HOLD_TIME = 5.0          # seconds
HOVER_THRUST = 0.50      # GUESS - replace with MOT_THST_HOVER, see above
CLIMB_KP = 0.15          # thrust added per metre of altitude error
MIN_THRUST = 0.20        # hard floor, never command less than this in air
MAX_THRUST = 0.75        # hard ceiling on commanded thrust
ALT_TOLERANCE = 0.08     # metres - "close enough" to target to start hold
CLIMB_TIMEOUT = 12.0     # seconds - abort climb if not reached by then
CEILING_ALT = 1.4        # metres - hard safety ceiling, forces descent
LOOP_DT = 0.05           # seconds - 20 Hz control loop
LANDED_ALT = 0.08        # metres - below this is considered "on the ground"
DESCEND_THRUST_BIAS = 0.10  # thrust below hover used for descent

# Sensor-health monitoring (tailored to optical flow + rangefinder)
MIN_FLOW_QUALITY = 10           # ArduPilot's FLOW_QUAL_MIN default
RANGEFINDER_EKF_MISMATCH = 0.30  # metres - raw rangefinder vs EKF alt
MAX_CONSECUTIVE_BAD = 6          # ~0.3s at 20Hz before triggering abort
# ---------------------------------------------------------------------------


class SensorMonitor:
    """Tracks optical-flow quality and rangefinder/EKF agreement across
    loop iterations, so one bad reading doesn't trigger an abort but a
    sustained drop does."""

    def __init__(self):
        self._bad_flow = 0
        self._bad_rangefinder = 0

    def check(self, drone, ekf_alt):

        quality = drone.get_flow_quality()

        if quality is not None and quality < MIN_FLOW_QUALITY:
            self._bad_flow += 1
        else:
            self._bad_flow = 0

        if self._bad_flow >= MAX_CONSECUTIVE_BAD:
            return f"optical flow quality degraded (quality={quality})"

        rf_alt = drone.get_rangefinder_distance()

        if rf_alt is not None and ekf_alt is not None:
            if abs(rf_alt - ekf_alt) > RANGEFINDER_EKF_MISMATCH:
                self._bad_rangefinder += 1
            else:
                self._bad_rangefinder = 0

        if self._bad_rangefinder >= MAX_CONSECUTIVE_BAD:
            return (
                f"rangefinder/EKF altitude mismatch "
                f"(rangefinder={rf_alt:.2f}m, ekf={ekf_alt:.2f}m)"
            )

        return None


def safe_descend_and_disarm(drone, reason=""):
    """Best-effort controlled descent, then cuts thrust and disarms.
    Used both for the planned landing and for every abort path."""

    print(f"Landing. ({reason})")

    start = time.time()

    while time.time() - start < 10:

        alt = drone.get_ekf_altitude()

        if alt is None:
            # No altitude data at all - don't guess, just cut thrust.
            break

        if alt <= LANDED_ALT:
            break

        thrust = HOVER_THRUST - DESCEND_THRUST_BIAS
        thrust = max(MIN_THRUST, min(MAX_THRUST, thrust))

        drone.send_attitude_target(thrust)
        time.sleep(LOOP_DT)

    # Cut thrust and disarm regardless of how we got here.
    drone.send_attitude_target(MIN_THRUST)
    time.sleep(0.3)

    ok = drone.disarm()
    print("Disarmed." if ok else "WARNING: disarm did not confirm - check vehicle!")


def preflight_sensor_check(drone):

    drone.check_ekf_source_config()  # warns only, doesn't block

    if not drone.ekf_ready_optical_flow():
        return False

    quality = drone.get_flow_quality()

    if quality is None:
        print("ABORT: no OPTICAL_FLOW messages received.")
        return False

    if quality < MIN_FLOW_QUALITY:
        print(f"ABORT: optical flow quality too low ({quality}). "
              f"Check lighting/texture under the vehicle.")
        return False

    rf = drone.get_rangefinder_distance()

    if rf is None:
        print("ABORT: no DISTANCE_SENSOR messages received.")
        return False

    print(f"Pre-flight OK: flow_quality={quality}, rangefinder={rf:.2f} m")

    return True


def run_mission():

    drone = Drone()
    drone.connect()

    try:
        # ---- Pre-flight checks ----
        if not preflight_sensor_check(drone):
            drone.drain_statustext(prefix="[FC] ")
            return

        # ---- Mode ----
        drone.set_mode("GUIDED_NOGPS")

        if not drone.wait_mode("GUIDED_NOGPS", timeout=10):
            print("ABORT: failed to enter GUIDED_NOGPS.")
            return

        print("Mode: GUIDED_NOGPS")

        # ---- Arm ----
        if not drone.arm():
            print("ABORT: arming failed (see STATUSTEXT above for PreArm reason).")
            return

        print("Armed.")

        monitor = SensorMonitor()

        # ---- Climb to TARGET_ALT ----
        print(f"Climbing to {TARGET_ALT} m...")

        climb_start = time.time()
        reached = False

        while time.time() - climb_start < CLIMB_TIMEOUT:

            alt = drone.get_ekf_altitude()
            drone.drain_statustext(prefix="[FC] ")

            if alt is None:
                print("ABORT: lost altitude feed during climb.")
                safe_descend_and_disarm(drone, "lost altitude feed")
                return

            if alt >= CEILING_ALT:
                print(f"ABORT: ceiling exceeded ({alt:.2f} m).")
                safe_descend_and_disarm(drone, "ceiling exceeded")
                return

            bad = monitor.check(drone, alt)
            if bad:
                print(f"ABORT: {bad}")
                safe_descend_and_disarm(drone, bad)
                return

            error = TARGET_ALT - alt

            if abs(error) <= ALT_TOLERANCE:
                reached = True
                break

            thrust = HOVER_THRUST + CLIMB_KP * error
            thrust = max(MIN_THRUST, min(MAX_THRUST, thrust))

            drone.send_attitude_target(thrust)
            print(f"  alt={alt:.2f} m  thrust={thrust:.2f}")
            time.sleep(LOOP_DT)

        if not reached:
            print("ABORT: did not reach target altitude in time.")
            safe_descend_and_disarm(drone, "climb timeout")
            return

        print(f"Reached ~{TARGET_ALT} m. Holding for {HOLD_TIME}s...")

        # ---- Altitude hold ----
        hold_start = time.time()

        while time.time() - hold_start < HOLD_TIME:

            alt = drone.get_ekf_altitude()
            drone.drain_statustext(prefix="[FC] ")

            if alt is None:
                print("ABORT: lost altitude feed during hold.")
                safe_descend_and_disarm(drone, "lost altitude feed")
                return

            if alt >= CEILING_ALT:
                print(f"ABORT: ceiling exceeded during hold ({alt:.2f} m).")
                safe_descend_and_disarm(drone, "ceiling exceeded")
                return

            bad = monitor.check(drone, alt)
            if bad:
                print(f"ABORT: {bad}")
                safe_descend_and_disarm(drone, bad)
                return

            error = TARGET_ALT - alt
            thrust = HOVER_THRUST + CLIMB_KP * error
            thrust = max(MIN_THRUST, min(MAX_THRUST, thrust))

            drone.send_attitude_target(thrust)
            print(f"  alt={alt:.2f} m  thrust={thrust:.2f}")
            time.sleep(LOOP_DT)

        print("Hold complete.")

        # ---- Land ----
        safe_descend_and_disarm(drone, "mission complete")

    except KeyboardInterrupt:
        print("Interrupted by user - emergency descend/disarm.")
        safe_descend_and_disarm(drone, "keyboard interrupt")

    except Exception as e:
        print(f"Unhandled error: {e} - emergency descend/disarm.")
        safe_descend_and_disarm(drone, "exception")

    finally:
        drone.close()


if __name__ == "__main__":
    run_mission()