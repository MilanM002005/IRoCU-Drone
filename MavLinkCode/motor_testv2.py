"""
Mission: arm -> GUIDED_NOGPS -> climb to 1 m -> hold 5 s
         -> ALT_HOLD (3 s) -> LOITER (10 s) -> LAND -> disarm

Sensor setup this targets: optical flow (horizontal velocity / relative
position, EK3_SRC1_VELXY=5) + rangefinder (height above ground,
EK3_SRC1_POSZ=2). No GPS.

GUIDED_NOGPS does not accept position/velocity setpoints - only attitude +
thrust (SET_ATTITUDE_TARGET). So the initial "takeoff" and hold are a manual
closed-loop thrust controller using the EKF's fused altitude
(LOCAL_POSITION_NED.z, which is rangefinder-sourced on this vehicle) as
feedback. This is NOT the same as MAV_CMD_NAV_TAKEOFF / the LAND mode.

ALT_HOLD and LOITER are RC-stick-driven modes. Since this script flies with
no pilot on the sticks, it sends a CENTERED RC_CHANNELS_OVERRIDE (roll/pitch/
yaw at neutral PWM, throttle at the mid/hover detent the FC expects in
ALT_HOLD/LOITER) for the duration of those two phases, then clears the
override before LAND so a hardware RC (if connected) regains authority and
so no stale override is left active after the script exits.

>>> IMPORTANT - LOITER ON OPTICAL FLOW <<<
LOITER requires the EKF to have a *confident horizontal position* estimate,
not just velocity. With flow-only (no GPS), ArduPilot can derive position by
integrating flow velocity, but this estimate can be poor (drifty, or briefly
invalid right after a mode change). The vehicle may refuse to enter LOITER,
or may enter it and immediately show large position uncertainty. This script
treats "mode reported as LOITER" as necessary but NOT sufficient - it also
re-checks EKF health and altitude sanity every loop while in LOITER, same as
it always did for GUIDED_NOGPS, and aborts to LAND if anything looks wrong.

Per explicit instruction, the optical-flow-quality / rangefinder-EKF-mismatch
SensorMonitor from the original climb/hold phases is NOT applied during
ALT_HOLD/LOITER. Hard altitude bounds (ceiling, lost-altitude-feed) and mode/
arm-state confirmation are still checked in those phases, since those are
basic flight-safety bounds rather than sensor-quality heuristics.

>>> SAFETY - READ BEFORE RUNNING <<<
- First run tethered, over a soft surface, ideally with props clipped/off,
  just to watch the thrust controller and RC-override behavior before
  trusting it in flight.
- Keep the RC in hand and ready to flip to STABILIZE/LAND or cut power.
  NOTE: while this script holds an RC override active (ALT_HOLD/LOITER
  phases), a *physical* RC stick movement will NOT be seen by the FC until
  the override is cleared or times out - know how your RC_CHANNELS_OVERRIDE
  handling/failsafe is configured before flying this.
- HOVER_THRUST below is a *guess* (0.50). Read the real value first:
      print(drone.get_param("MOT_THST_HOVER"))
  and set HOVER_THRUST (and re-check MAX_THRUST) to that before flying.
- Start with CLIMB_KP lower than you think you need.
- Flight surface matters: optical flow needs texture and even lighting.
  A flat painted floor or glass will degrade flow quality and is the
  most likely real-world failure mode here - that's why this script
  actively monitors flow quality and rangefinder/EKF agreement during the
  GUIDED_NOGPS phases, not just the EKF flag summary, and aborts to a
  landing if either degrades for a sustained period.
- LOITER specifically can behave unpredictably on flow-only EKF position.
  Be ready to take over or accept the scripted abort-to-LAND.
"""

import time
from drone import Drone

# ---- Tunable parameters -------------------------------------------------
TARGET_ALT = 1.0        # metres
HOLD_TIME = 5.0          # seconds, GUIDED_NOGPS hold
HOVER_THRUST = 0.50      # GUESS - replace with MOT_THST_HOVER, see above
CLIMB_KP = 0.15          # thrust added per metre of altitude error
MIN_THRUST = 0.20        # hard floor, never command less than this in air
MAX_THRUST = 0.51        # hard ceiling on commanded thrust
ALT_TOLERANCE = 0.08     # metres - "close enough" to target to start hold
CLIMB_TIMEOUT = 12.0     # seconds - abort climb if not reached by then
CEILING_ALT = 1.4        # metres - hard safety ceiling, forces descent
LOOP_DT = 0.05           # seconds - 20 Hz control loop
LANDED_ALT = 0.08        # metres - below this is considered "on the ground"
DESCEND_THRUST_BIAS = 0.10  # thrust below hover used for descent

# ---- New: mode-hold phases after the initial GUIDED_NOGPS hold ----------
ALTHOLD_TIME = 3.0       # seconds to hold in ALT_HOLD
LOITER_TIME = 10.0       # seconds to hold in LOITER
MODE_CHANGE_TIMEOUT = 6.0  # seconds to wait for each mode to be confirmed
LAND_CONFIRM_TIMEOUT = 30.0  # seconds to wait for LAND mode to disarm itself

# Centered/neutral RC override values (typical 1000-2000us PWM convention).
# Throttle at 1500 = mid-stick, which is the "hold altitude" detent in
# ALT_HOLD/LOITER (NOT zero throttle - zero/low PWM here would mean
# "descend" or trigger a throttle failsafe depending on RC_OPTIONS).
RC_NEUTRAL = {
    "roll": 1500,
    "pitch": 1500,
    "throttle": 1500,
    "yaw": 1500,
}

# Sensor-health monitoring (tailored to optical flow + rangefinder)
# Applies ONLY to the original GUIDED_NOGPS climb/hold phases.
MIN_FLOW_QUALITY = 10           # ArduPilot's FLOW_QUAL_MIN default
RANGEFINDER_EKF_MISMATCH = 0.30  # metres - raw rangefinder vs EKF alt
MAX_CONSECUTIVE_BAD = 6          # ~0.3s at 20Hz before triggering abort
# ---------------------------------------------------------------------------


class SensorMonitor:
    """Tracks optical-flow quality and rangefinder/EKF agreement across
    loop iterations, so one bad reading doesn't trigger an abort but a
    sustained drop does. Used only during GUIDED_NOGPS climb/hold."""

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
    """Best-effort controlled descent via direct thrust command, then cuts
    thrust and disarms. Used for the original GUIDED_NOGPS abort paths.
    Clears any RC override first, since this may be called from a phase
    that had one active."""

    print(f"Landing (manual thrust controller). ({reason})")

    drone.clear_rc_override()

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


def abort_via_land_mode(drone, reason=""):
    """Abort path used once we've left GUIDED_NOGPS (i.e. during/after the
    ALT_HOLD or LOITER phases). At that point we no longer have a clean
    attitude-target thrust loop driving the vehicle (mode has moved on from
    GUIDED_NOGPS), so the safe abort is to clear any RC override and let the
    flight controller's own LAND mode handle descent, same as the planned
    end-of-mission landing."""

    print(f"Aborting to LAND mode. ({reason})")
    drone.clear_rc_override()
    land_and_confirm(drone)


def land_and_confirm(drone):
    """Switch to LAND, wait for the FC to report the mode change, then wait
    for the vehicle to disarm on its own (ArduPilot's LAND mode disarms
    automatically on touchdown by default). Falls back to a manual disarm
    call if it doesn't, but does NOT attempt to drive thrust manually here -
    LAND mode owns the descent."""

    drone.set_mode("LAND")

    if not drone.wait_mode("LAND", timeout=MODE_CHANGE_TIMEOUT):
        print("WARNING: LAND mode not confirmed - sending disarm directly.")
        ok = drone.disarm()
        print("Disarmed." if ok else "WARNING: disarm did not confirm - check vehicle!")
        return

    print("Mode: LAND. Waiting for touchdown/auto-disarm...")

    start = time.time()

    while time.time() - start < LAND_CONFIRM_TIMEOUT:

        drone.drain_statustext(prefix="[FC] ")

        if not drone.is_armed():
            print("Disarmed (automatic, on touchdown).")
            return

        time.sleep(LOOP_DT)

    print("WARNING: LAND mode did not auto-disarm within timeout - "
          "sending disarm directly. CHECK VEHICLE.")
    ok = drone.disarm()
    print("Disarmed." if ok else "WARNING: disarm did not confirm - check vehicle!")


def hold_in_mode(drone, mode_name, duration):
    """Switches to the given mode, holds a centered RC override for
    `duration` seconds, and continuously checks hard flight-safety bounds
    (altitude feed present, ceiling, still armed, still in the expected
    mode). Returns None on success, or an abort reason string.

    Does NOT run the optical-flow/rangefinder SensorMonitor - that check is
    scoped to the GUIDED_NOGPS phases only, per spec.
    """

    drone.set_mode(mode_name)

    if not drone.wait_mode(mode_name, timeout=MODE_CHANGE_TIMEOUT):
        return f"failed to enter {mode_name}"

    print(f"Mode: {mode_name}. Holding {duration}s with centered RC override...")

    drone.send_rc_override(**RC_NEUTRAL)

    start = time.time()

    while time.time() - start < duration:

        drone.drain_statustext(prefix="[FC] ")

        if not drone.is_armed():
            return f"vehicle disarmed unexpectedly during {mode_name}"

        current_mode = drone.get_mode()
        if current_mode is not None and current_mode != mode_name:
            return f"mode changed away from {mode_name} unexpectedly (now {current_mode})"

        alt = drone.get_ekf_altitude()

        if alt is None:
            return f"lost altitude feed during {mode_name}"

        if alt >= CEILING_ALT:
            return f"ceiling exceeded during {mode_name} ({alt:.2f} m)"

        # Keep refreshing the override - some autopilots time out RC
        # overrides after ~1-3s of no update and revert to last RC input.
        drone.send_rc_override(**RC_NEUTRAL)

        time.sleep(LOOP_DT)

    return None


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

        # ---- Altitude hold (GUIDED_NOGPS, manual thrust controller) ----
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

        print("GUIDED_NOGPS hold complete.")

        # ---- ALT_HOLD (3s, centered RC override) ----
        bad = hold_in_mode(drone, "ALT_HOLD", ALTHOLD_TIME)
        if bad:
            print(f"ABORT: {bad}")
            abort_via_land_mode(drone, bad)
            return

        print("ALT_HOLD complete.")

        # ---- LOITER (10s, centered RC override) ----
        # NOTE: on flow-only EKF position, LOITER may refuse to engage or
        # may show poor position hold. wait_mode() confirms the FC accepted
        # the mode; hold_in_mode() then continues to check hard altitude/
        # arm-state bounds (but not flow/rangefinder quality) for the
        # duration.
        bad = hold_in_mode(drone, "LOITER", LOITER_TIME)
        if bad:
            print(f"ABORT: {bad}")
            abort_via_land_mode(drone, bad)
            return

        print("LOITER complete.")

        # ---- Land (FC's own LAND mode, not the manual thrust controller) ----
        drone.clear_rc_override()
        land_and_confirm(drone)

    except KeyboardInterrupt:
        print("Interrupted by user - emergency landing.")
        # We don't know which phase we were in, so clear any override and
        # prefer LAND mode (works whether we were in GUIDED_NOGPS, ALT_HOLD,
        # or LOITER) over the manual thrust controller.
        drone.clear_rc_override()
        land_and_confirm(drone)

    except Exception as e:
        print(f"Unhandled error: {e} - emergency landing.")
        drone.clear_rc_override()
        land_and_confirm(drone)

    finally:
        drone.clear_rc_override()
        drone.close()


if __name__ == "__main__":
    run_mission()