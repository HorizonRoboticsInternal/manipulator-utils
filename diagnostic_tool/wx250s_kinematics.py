"""
    This file contains forward and inverse kinematics
    wrapper functions for the WX250s manipulator.
"""
import numpy as np
from dataclasses import dataclass, field
import kinematics_cpp.kincpp as kincpp


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
    joint_names: list[str] = field(default_factory=lambda: ['waist', 'shoulder', 'elbow',
                                                            'forearm_roll', 'wrist_angle',
                                                            'wrist_rotate'])

    REV: float = 2 * np.pi


def wrap_thetas(theta_list: np.ndarray) -> np.ndarray:
    """
    Wrap an array of joint commands to [-pi, pi) and between the joint limits.

    :param theta_list: array of floats to wrap
    :return: array of floats wrapped between [-pi, pi)
    """
    theta_list = (theta_list + np.pi) % WX250sParams.REV - np.pi

    under_limit = theta_list < WX250sParams.lower_joint_limits
    over_limit = theta_list > WX250sParams.upper_joint_limits

    theta_list[under_limit] += WX250sParams.REV
    theta_list[over_limit] -= WX250sParams.REV

    return theta_list


def fwd_kin(curr_thetas: np.ndarray) -> np.ndarray:
    """
    Forward kinematics wrapper function for WX250s

    :param curr_thetas: The current joint angles.
    :return: The current end effector TF.
    """
    return kincpp.forward(WX250sParams.M, WX250sParams.S, curr_thetas)


def inv_kin(curr_ee_tf: np.ndarray, theta_guess: np.ndarray,
            eomg: float = 1e-3, ev: float = 1e-3) -> (bool, np.ndarray):
    """
    Inverse kinematics wrapper function for WX250s

    :param curr_ee_tf: The current end effector TF.
    :param theta_guess: The joint angle initial guess for the IK solver.
    :param eomg: The end effector orientation tolerance.
    :param ev: The end effector Cartesian position tolerance.
    :return: A tuple containing whether IK succeeded as well as the joint angles.
             Note if IK failed, the joint angle results should not be used.
    """

    theta = kincpp.inverse(WX250sParams.S, WX250sParams.M,
                           curr_ee_tf, theta_guess, eomg, ev)

    # Our way of propagating the success bool from C++
    success = False if theta[0] == -99 else True

    theta = wrap_thetas(theta)

    return success, theta
