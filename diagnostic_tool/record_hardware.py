import dill
import numpy as np
from time import sleep
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

    def create_right_angle(self, moving_time=3.0):
        self.arm.set_joint_positions([0]*self.num_joints, moving_time=moving_time)

    def point_straight(self, moving_time=3.0):
        self.create_right_angle(moving_time)
        sleep(0.5)

        lower_jp = [0.0, -0.00020950394840690834, 0.9273922091469897, 0.0, -0.9271827051985841, 0.0]
        self.arm.set_joint_positions(lower_jp, moving_time=moving_time)
        sleep(0.5)

    def execute_trajectory(self, end_joint_targets, wp_time, record_joint_positions=False):
        recorded_joint_positions = [] if record_joint_positions else None

        for i, jt in enumerate(end_joint_targets):
            print("Executing waypoint {}".format(i+1))
            self.arm.set_joint_positions(jt, moving_time=wp_time,
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
    recorded_joint_positions = []
    for jt in joint_trajectories:
        rjp = arm.execute_trajectory(jt, wp_time=0.05,
                                         record_joint_positions=True)
        recorded_joint_positions.append(rjp)
        sleep(0.5)

    with open("data/hw_trajectories.pkl", "wb") as f:
        dill.dump(recorded_joint_positions, f)

    print("Successfully recorded joint positions.")

    arm.shutdown()


if __name__ == '__main__':
    main()
