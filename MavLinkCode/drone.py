from pymavlink import mavutil
import time


class Drone:

    def __init__(
        self,
        connection_string="/dev/ttyACM0",
        baud=115200
    ):

        self.connection_string = connection_string
        self.baud = baud
        self.master = None

    # -------------------------
    # Connection
    # -------------------------

    def connect(self, timeout=10):

        print("Connecting to Pixhawk...")

        self.master = mavutil.mavlink_connection(
            self.connection_string,
            baud=self.baud
        )

        hb = self.master.wait_heartbeat(timeout=timeout)

        if hb is None:
            raise TimeoutError(
                "No heartbeat received - check connection_string/baud"
            )

        print(
            f"Connected "
            f"(SYS={hb.get_srcSystem()}, "
            f"COMP={hb.get_srcComponent()})"
        )

        self.request_data_streams()

    def close(self):

        if self.master:
            self.master.close()

    def request_data_streams(self):
        """Explicitly requests the message streams this mission depends
        on, since nothing else (e.g. a GCS) may be requesting them on a
        raw pymavlink connection."""

        messages_hz = {
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 20,  # EKF pos/vel
            mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR: 10,     # raw rangefinder
            mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW: 10,        # raw flow quality
            mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 5,    # EKF health flags
        }

        for msg_id, hz in messages_hz.items():
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                int(1e6 / hz),
                0, 0, 0, 0, 0
            )

    # -------------------------
    # Modes
    # -------------------------

    def set_mode(self, mode_name):

        mapping = self.master.mode_mapping()

        if mode_name not in mapping:
            raise ValueError(
                f"Unknown mode '{mode_name}'. "
                f"Available: {list(mapping.keys())}"
            )

        mode_id = mapping[mode_name]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

    def wait_mode(self, mode_name, timeout=10):

        target_mode = self.master.mode_mapping()[mode_name]

        start = time.time()

        while time.time() - start < timeout:

            msg = self.master.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1
            )

            if msg and msg.custom_mode == target_mode:
                return True

        return False

    # -------------------------
    # Arm / Disarm
    # -------------------------

    def arm(self, timeout=10):
        """Arms and waits for confirmation, printing any STATUSTEXT
        (e.g. PreArm failure reasons) seen while waiting."""

        self.drain_statustext()

        self.master.arducopter_arm()

        start = time.time()

        while time.time() - start < timeout:

            self.drain_statustext(prefix="[FC] ")

            if self.master.motors_armed():
                return True

            time.sleep(0.2)

        return False

    def disarm(self, force=False, timeout=10):

        if force:
            # param2 = 21196 forces disarm even mid-flight (use with care)
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 21196, 0, 0, 0, 0, 0
            )
        else:
            self.master.arducopter_disarm()

        start = time.time()

        while time.time() - start < timeout:

            if not self.master.motors_armed():
                return True

            time.sleep(0.2)

        return False

    # -------------------------
    # Telemetry - EKF (fused)
    # -------------------------

    def get_ekf_altitude(self, timeout=0.2):
        """Height above ground, as fused by EKF3 - sourced from the
        rangefinder if EK3_SRC1_POSZ=2 on this vehicle."""

        msg = self.master.recv_match(
            type="LOCAL_POSITION_NED",
            blocking=True,
            timeout=timeout
        )

        if msg is None:
            return None

        return -msg.z

    def get_climb_rate(self, timeout=0.2):

        msg = self.master.recv_match(
            type="LOCAL_POSITION_NED",
            blocking=True,
            timeout=timeout
        )

        if msg is None:
            return None

        return -msg.vz  # NED down -> up-positive

    # -------------------------
    # Telemetry - raw sensors
    # -------------------------

    def get_rangefinder_distance(self, timeout=0.2):
        """Raw rangefinder reading (metres), independent of the EKF
        fusion - useful for sanity-checking get_ekf_altitude()."""

        msg = self.master.recv_match(
            type="DISTANCE_SENSOR",
            blocking=True,
            timeout=timeout
        )

        if msg is None:
            return None

        return msg.current_distance / 100.0  # cm -> m

    def get_flow_quality(self, timeout=0.2):
        """Raw optical flow quality, 0-255. ArduPilot's FLOW_QUAL_MIN
        param (default 10) is the threshold below which it stops
        trusting flow data - low light / low texture / glass floors
        are the usual causes of a drop here."""

        msg = self.master.recv_match(
            type="OPTICAL_FLOW",
            blocking=True,
            timeout=timeout
        )

        if msg is None:
            return None

        return msg.quality

    def drain_statustext(self, prefix=""):
        """Non-blocking: prints all currently-buffered STATUSTEXT
        messages. Call this regularly so PreArm/EKF messages aren't
        silently dropped."""

        while True:

            msg = self.master.recv_match(type="STATUSTEXT", blocking=False)

            if msg is None:
                break

            print(f"{prefix}{msg.text}")

    # -------------------------
    # EKF / source health
    # -------------------------

    def check_ekf_source_config(self):
        """Sanity-checks that EK3 is actually configured for this
        vehicle's sensors: optical flow for horizontal velocity,
        rangefinder for vertical (AGL) position. Warns only - doesn't
        block - since param read failures shouldn't be mission-fatal."""

        velxy = self.get_param("EK3_SRC1_VELXY")
        posz = self.get_param("EK3_SRC1_POSZ")

        ok = True

        if velxy is None or int(velxy) != 5:  # 5 = OpticalFlow
            print(f"WARNING: EK3_SRC1_VELXY={velxy}, expected 5 (OpticalFlow)")
            ok = False

        if posz is None or int(posz) != 2:  # 2 = RangeFinder
            print(f"WARNING: EK3_SRC1_POSZ={posz}, expected 2 (RangeFinder)")
            ok = False

        return ok

    def ekf_ready_optical_flow(self, timeout=3):
        """Checks the EKF flags that actually matter for this sensor
        setup:
          EKF_ATTITUDE        - IMU/AHRS
          EKF_VELOCITY_HORIZ  - fed by optical flow (EK3_SRC1_VELXY)
          EKF_VELOCITY_VERT   - fed by baro/rangefinder fusion
          EKF_POS_HORIZ_REL   - relative position, integrated from flow
          EKF_POS_VERT_AGL    - rangefinder-sourced height above ground

        Deliberately does NOT require EKF_POS_HORIZ_ABS / EKF_POS_VERT_ABS,
        since those need GPS/absolute references this vehicle doesn't have.
        """

        msg = self.master.recv_match(
            type="EKF_STATUS_REPORT",
            blocking=True,
            timeout=timeout
        )

        if msg is None:
            print("No EKF_STATUS_REPORT received - is it being streamed?")
            return False

        flags = msg.flags

        required = (
            mavutil.mavlink.EKF_ATTITUDE |
            mavutil.mavlink.EKF_VELOCITY_HORIZ |
            mavutil.mavlink.EKF_VELOCITY_VERT |
            mavutil.mavlink.EKF_POS_HORIZ_REL |
            mavutil.mavlink.EKF_POS_VERT_AGL
        )

        ok = (flags & required) == required

        if not ok:
            missing = required & ~flags
            print(f"EKF not ready. flags=0b{flags:09b} missing=0b{missing:09b}")

        return ok

    def get_param(self, param_name, timeout=5):
        """Reads a single parameter, e.g. drone.get_param('MOT_THST_HOVER')."""

        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode("utf-8"),
            -1
        )

        msg = self.master.recv_match(
            type="PARAM_VALUE",
            blocking=True,
            timeout=timeout
        )

        if msg and msg.param_id.strip("\x00") == param_name:
            return msg.param_value

        return None

    # -------------------------
    # Attitude Target
    # -------------------------

    def send_attitude_target(self, thrust, type_mask=0b00000111):
        # type_mask 0b00000111: ignore body roll/pitch/yaw rate,
        # use attitude quaternion + thrust only.

        self.master.mav.set_attitude_target_send(
            0,
            self.master.target_system,
            self.master.target_component,
            type_mask,
            [1, 0, 0, 0],
            0,
            0,
            0,
            thrust
        )