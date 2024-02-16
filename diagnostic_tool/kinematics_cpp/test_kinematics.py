"""
    Forward and inverse kinematics unit tests.
"""
import numpy as np
import kincpp


def fk_test():
    """
    Tests forward kinematics function.
    """
    M = np.array([[-1, 0, 0, 0],
                  [0, 1, 0, 6],
                  [0, 0, -1, 2],
                  [0, 0, 0, 1]])

    slist = np.array([[0, 0, 0],
                      [0, 0, 0],
                      [1, 0, -1],
                      [4, 0, -6],
                      [0, 1, 0],
                      [0, 0, -0.1]])

    theta_list = np.array([np.pi / 2.0, 3, np.pi])

    expected_res = np.array([[0, 1, 0, -5],
                             [1, 0, 0, 4],
                             [0, 0, -1, 1.68584073],
                             [0, 0, 0, 1]])

    fk_res = kincpp.forward(M, slist, theta_list)

    np.testing.assert_allclose(fk_res, expected_res, atol=1e-4)

    print("Forward Kinematics Test Passed.")


def ik_test():
    """
    Tests inverse kinematics function.
    """
    slist = np.array([[0, 0, 1, 4, 0, 0],
                      [0, 0, 0, 0, 1, 0],
                      [0, 0, -1, -6, 0, -0.1]]).T
    M = np.array([[-1, 0, 0, 0],
                  [0, 1, 0, 6],
                  [0, 0, -1, 2],
                  [0, 0, 0, 1]])
    T = np.array([[0, 1, 0, -5],
                  [1, 0, 0, 4],
                  [0, 0, -1, 1.6858],
                  [0, 0, 0, 1]])

    curr_theta = np.array([1.5, 2.5, 3])

    eomg = 0.01
    ev = 0.001

    expected_res = np.array([1.57073783, 2.99966384, 3.1415342], dtype=np.float64)

    ik_res = kincpp.inverse(slist, M, T, curr_theta, eomg, ev)
    success = ik_res[0] != -99

    assert success == True
    assert np.allclose(ik_res, expected_res, atol=1e-4)

    print("Inverse Kinematics Test Passed.")


if __name__ == '__main__':
    fk_test()
    ik_test()
