import dill
import numpy as np
from wx250s_kinematics import fwd_kin, inv_kin


def create_cartesian_traj(starting_joint_pos: np.ndarray,
                          deltas: list[tuple],
                          disc: list[int]) -> list[np.ndarray]:
    """
    Produces Cartesian joint trajectories for a series of provided gripper position deltas.

    :param starting_joint_pos: The starting joint position.
    :param deltas: A list of gripper Cartesian position deltas.
    :param disc: The number of discretized points when traversing to each waypoint.
    :return: A list of trajectories for each waypoint.
    """
    ee_tf = fwd_kin(starting_joint_pos)  # Compute current gripper TF
    guess = starting_joint_pos           # Always use starting position as initial guess
    trajectories = []

    for N, (dx, dy, dz) in zip(disc, deltas):
        inc = 1. / N
        dx_inc = dx * inc
        dy_inc = dy * inc
        dz_inc = dz * inc
        curr_traj = []

        for i in range(N):
            ee_tf[0, 3] += dx_inc
            ee_tf[1, 3] += dy_inc
            ee_tf[2, 3] += dz_inc

            success, res = inv_kin(ee_tf, guess)
            if not success:
                raise RuntimeError("Inverse kinematics has failed to find a solution.")

            curr_traj.append(res)
            guess = res

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

    trajectories = create_cartesian_traj(starting_joint_position, deltas, [80]*3)

    with open("data/joint_trajectories.pkl", "wb") as f:
        dill.dump(trajectories, f)

    print("Successfully saved joint trajectories.")


if __name__ == "__main__":
    main()
