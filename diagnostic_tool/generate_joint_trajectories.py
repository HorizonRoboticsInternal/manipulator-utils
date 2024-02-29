import dill
import numpy as np
from typing import List
from wx250s_kinematics import fwd_kin, inv_kin


def create_cartesian_traj(initial_joint_position: np.ndarray,
                          deltas: List[tuple[tuple, int]]) -> List[np.ndarray]:
    """
    Produces Cartesian joint trajectories for a series of provided gripper position deltas.

    :param initial_joint_position: The starting joint position.
    :param deltas: A list of tuples consisting of gripper Cartesian position (x, y, z) deltas
                   and number of discrete points along the respective waypoint.
    :return: A list of trajectories for each waypoint.
    """
    ee_tf = fwd_kin(initial_joint_position)  # Compute current gripper TF
    guess = initial_joint_position           # Always use starting position as initial guess
    trajectories = []

    for (dx, dy, dz), N in deltas:
        inc = 1. / N
        dx_inc = dx * inc
        dy_inc = dy * inc
        dz_inc = dz * inc
        curr_traj = []

        for i in range(N):
            ee_tf[0, 3] += dx_inc
            ee_tf[1, 3] += dy_inc
            ee_tf[2, 3] += dz_inc

            success, joint_positions = inv_kin(ee_tf, guess)
            if not success:
                raise RuntimeError("Inverse kinematics has failed to find a solution.")

            curr_traj.append(joint_positions)
            guess = joint_positions

        trajectories.append(np.asarray(curr_traj))

    return trajectories


def main():
    # A neutral starting position for WX250s
    starting_joint_position = np.asarray([0.0, -0.00020950394840690834, 0.9273922091469897,
                                          0.0, -0.9271827051985841, 0.0])

    # Equilateral triangle trajectory
    scale = 0.2
    delta_y = np.sin(np.deg2rad(30)) * scale
    delta_z = np.cos(np.deg2rad(30)) * scale

    # Gripper position deltas
    deltas = [(0, -delta_y, delta_z),
              (0, 2*delta_y, 0),
              (0, -delta_y, -delta_z)]

    # Add the number of discrete points along trajectory to each waypoint
    deltas = zip(deltas, [80]*3)

    trajectories = create_cartesian_traj(starting_joint_position, deltas)

    with open("data/joint_trajectories.pkl", "wb") as f:
        dill.dump(trajectories, f)

    print("Successfully saved joint trajectories.")


if __name__ == "__main__":
    main()
