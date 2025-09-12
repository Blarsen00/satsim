import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt

from frames.base_frame import BaseParamFrame


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


def close_app(root):
    plt.close("all")
    root.destroy()


def test_page(root, page: BaseParamFrame):
    root.title("Test of frame")
    root.geometry("800x400")
    page.pack(fill="both", expand=True)
    root.protocol("WM_DELETE_WINDOW", lambda: close_app(root))
    root.mainloop()


