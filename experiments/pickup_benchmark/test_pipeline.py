import sys
sys.path.insert(0, '/home/sean/ros2_ws/experiments/pickup_benchmark')

import rclpy
from pickup_experiment import PickupExperiment, TRIAL_TIMEOUT_SEC
import pickup_experiment
from layouts import LAYOUTS

# shrink timeout for a fast smoke test
pickup_experiment.TRIAL_TIMEOUT_SEC = 60.0


def main():
    rclpy.init()
    node = PickupExperiment()
    result = node.run_trial(roller_omega=45.0, blind_speed=0.3, layout=LAYOUTS[0], layout_id=0)
    print('RESULT:', result, flush=True)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
