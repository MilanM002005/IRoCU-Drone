"""
Standalone diagnostic: figures out WHY EKF_POS_HORIZ_REL and
EKF_POS_VERT_AGL are missing - misconfigured EK3 source params vs.
sensor data not actually reaching the EKF.

Run this instead of mission.py until it prints all-clear.
"""

import time
from drone import Drone


def main():

    drone = Drone()
    drone.connect()

    print("\n--- EK3 source params (what the EKF is SET to use) ---")
    drone.check_ekf_source_config()

    velxy = drone.get_param("EK3_SRC1_VELXY")
    posz = drone.get_param("EK3_SRC1_POSZ")
    print(f"EK3_SRC1_VELXY = {velxy}  (expect 5 = OpticalFlow)")
    print(f"EK3_SRC1_POSZ  = {posz}  (expect 2 = RangeFinder)")

    print("\n--- Raw sensor data (is it actually arriving?) ---")
    for i in range(10):
        quality = drone.get_flow_quality()
        rf = drone.get_rangefinder_distance()
        print(f"  [{i}] flow_quality={quality}  rangefinder={rf}")
        time.sleep(0.5)

    print("\n--- EKF flags over time (is it converging?) ---")
    for i in range(10):
        ready = drone.ekf_ready_optical_flow(timeout=2)
        print(f"  [{i}] ready={ready}")
        time.sleep(1)

    drone.close()


if __name__ == "__main__":
    main()