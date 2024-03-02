"""
    This file contains forward and inverse kinematics
    wrapper functions for the WX250s manipulator.
"""
import kincpp
import numpy as np
from typing import List
from dataclasses import dataclass, field


# Parameters for the WX250s arm.
# Treated as a frozen dataclass as these params should never change.
@dataclass(frozen=True)
class WX250sParams:
    S: np.ndarray = np.asarray([[0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0, -0.11065, 0.0, 0.0],
                                [0.0, 1.0, 0.0, -0.36065, 0.0, 0.04975],
                                [1.0, 0.0, 0.0, 0.0, 0.36065, 0.0],
                                [0.0, 1.0, 0.0, -0.36065, 0.0, 0.29975],
                                [1.0, 0.0, 0.0, 0.0, 0.36065, 0.0]]).T

    M: np.ndarray = np.asarray([[1.0, 0.0, 0.0, 0.458325],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 1.0, 0.36065],
                                [0.0, 0.0, 0.0, 1.0]])

    lower_joint_limits: np.ndarray = np.asarray([-3.141582727432251, -1.884955644607544,
                                                 -2.1467549800872803, -3.141582727432251,
                                                 -1.7453292608261108, -3.141582727432251])

    upper_joint_limits: np.ndarray = np.asarray([3.141582727432251, 1.9896754026412964,
                                                 1.6057028770446777, 3.141582727432251,
                                                 2.1467549800872803, 3.141582727432251])

    # Since a list is mutable, make sure to use an instance of the dataclass to retrieve this
    joint_names: List[str] = field(default_factory=lambda: ['waist', 'shoulder', 'elbow',
                                                            'forearm_roll', 'wrist_angle',
                                                            'wrist_rotate'])

    REV: float = 2 * np.pi


def wrap_joint_positions(joint_positions: np.ndarray) -> np.ndarray:
    """
    Wrap an array of joint commands to [-pi, pi) and between the joint limits.

    :param joint_positions: joint positions to wrap
    :return: array of joint positions wrapped between [-pi, pi)
    """
    joint_positions = (joint_positions + np.pi) % WX250sParams.REV - np.pi

    under_limit = joint_positions < WX250sParams.lower_joint_limits
    over_limit = joint_positions > WX250sParams.upper_joint_limits

    joint_positions[under_limit] += WX250sParams.REV
    joint_positions[over_limit] -= WX250sParams.REV

    return joint_positions


def fwd_kin(joint_positions: np.ndarray) -> np.ndarray:
    """
    Forward kinematics wrapper function for WX250s

    :param joint_positions: The current joint angles.
    :return: The current end effector TF.
    """
    return kincpp.forward(WX250sParams.M, WX250sParams.S, joint_positions)


def inv_kin(desired_ee_tf: np.ndarray,
            joint_position_guess: np.ndarray,
            position_tolerance: float = 1e-3,
            orientation_tolerance: float = 1e-3,
            max_iterations: int = 20
            ) -> (bool, np.ndarray):
    """
    Inverse kinematics wrapper function for WX250s

    :param desired_ee_tf: The desired end effector TF.
    :param joint_position_guess: The joint position initial guess for the IK solver.
    :param position_tolerance: The end effector Cartesian position tolerance.
    :param orientation_tolerance: The end effector orientation tolerance.
    :param max_iterations: The number of iterations before IK solver quits.
    :return: A tuple containing whether IK succeeded as well as the joint angles.
             Note if IK failed, the joint angle results are undefined.
    """

    success, joint_positions = kincpp.inverse(WX250sParams.M, WX250sParams.S,
                                              desired_ee_tf,
                                              joint_position_guess,
                                              position_tolerance,
                                              orientation_tolerance,
                                              max_iterations)

    joint_positions = wrap_joint_positions(joint_positions)

    return success, joint_positions
