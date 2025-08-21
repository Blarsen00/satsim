import numpy as np


def skew(v):
    """
    Returns the skew-symmetric matrix of a 3-element vector v.
    """
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_multiply(q1, q2):
    """Multiply two quaternions q1 ⊗ q2, both in [x, y, z, w] format"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


# def quat_multiply(q1, q2):
#     """Multiply two quaternions q1 ⊗ q2, both in [x, y, z, w] format (scalar last)"""
#     x1, y1, z1, w1 = q1
#     x2, y2, z2, w2 = q2
#
#     x = w1*x2 + x1*w2 + y1*z2 - z1*y2
#     y = w1*y2 - x1*z2 + y1*w2 + z1*x2
#     z = w1*z2 + x1*y2 - y1*x2 + z1*w2
#     w = w1*w2 - x1*x2 - y1*y2 - z1*z2
#
#     return np.array([x, y, z, w])
