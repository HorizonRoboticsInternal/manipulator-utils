import dill
import numpy as np
from tqdm import trange
from time import sleep
from typing import List
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class WX250sManipulator:
    def __init__(self):
        self.bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")
        self.arm = self.bot.arm
        self.joint_names = self.arm.group_info.joint_names
        self.num_joints = len(self.joint_names)

    def get_gripper_tf(self):
        return self.arm.T_sb

    def get_joint_positions(self):
        return self.arm.core.joint_states.position[:6]

    def go_to_home(self, moving_time=3.0):
        self.arm.set_joint_positions([0]*self.num_joints, moving_time=moving_time)

    def point_straight(self, moving_time=3.0):
        self.go_to_home(moving_time)
        sleep(0.5)

        lower_jp = [0.0, -0.00020950394840690834, 0.9273922091469897, 0.0, -0.9271827051985841, 0.0]
        self.arm.set_joint_positions(lower_jp, moving_time=moving_time)
        sleep(0.5)

    def execute_trajectory(self, joint_trajectory: np.ndarray, wp_time: float,
                           record_joint_positions: bool = False
                           ) -> np.ndarray:
        """
        Executes a joint position trajectory with optional recording.

        :param joint_trajectory: An array of joint position targets to hit.
        :param wp_time: The amount of time to travel between each waypoint.
        :param record_joint_positions: Flag indicating whether joint positions will be recorded during execution.
        :return: An array of recorded joint positions. Returns None if record_joint_positions is False
        """
        recorded_joint_positions = [] if record_joint_positions else None

        num_joint_positions = len(joint_trajectory)
        print("Trajectory Completion Progress")
        for i in trange(num_joint_positions, ncols=100):
            joint_position = joint_trajectory[i]
            self.arm.set_joint_positions(joint_position,
                                         moving_time=wp_time,
                                         blocking=True,
                                         recorded_joint_positions=recorded_joint_positions)

        return np.asarray(recorded_joint_positions)

    def shutdown(self):
        self.bot.shutdown()
        print("Shutdown successfully.")


def main():
    arm = WX250sManipulator()

    arm.point_straight()

    with open("data/joint_trajectories.pkl", "rb") as f:
        joint_trajectories = dill.load(f)

    # Execute trajectories and record hardware joint angles
    recorded_joint_positions: List[np.ndarray] = []
    for joint_traj in joint_trajectories:
        recorded_joint_positions.append(arm.execute_trajectory(joint_traj, wp_time=0.05,
                                                               record_joint_positions=True))
        sleep(0.5)

    with open("data/hw_trajectories.pkl", "wb") as f:
        dill.dump(recorded_joint_positions, f)

    print("Successfully recorded joint positions.")

    arm.shutdown()


if __name__ == '__main__':
    main()
