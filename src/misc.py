import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt


def skew(v: np.ndarray) -> np.ndarray:
    """
    Returns the skew-symmetric matrix :math:`\\mathbf{S}` of a 3-element vector :math:`\\mathbf{v}`.

    The skew-symmetric matrix :math:`\\mathbf{S}` allows the cross product to be
    represented as a matrix multiplication: :math:`\\mathbf{v} \\times \\mathbf{u} = \\mathbf{S}\\mathbf{u}`.

    Parameters
    ----------
    v : :class:`numpy.ndarray`
        A 3-element vector :math:`\\mathbf{v}`, shape (3,).

    Returns
    -------
    :class:`numpy.ndarray`
        The :math:`3 \\times 3` skew-symmetric matrix :math:`\\mathbf{S}`.
    """
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """
    Computes the quaternion conjugate :math:`\\mathbf{q}^*`.

    It negates the vector part (:math:`\\mathbf{i}`, :math:`\\mathbf{j}`, :math:`\\mathbf{k}`)
    while keeping the scalar part (:math:`\\mathbf{w}`) the same.
    The input and output format is assumed to be :math:`[x, y, z, w]`.

    Parameters
    ----------
    q : :class:`numpy.ndarray`
        The quaternion :math:`\\mathbf{q}` in :math:`[x, y, z, w]` format, shape (4,).

    Returns
    -------
    :class:`numpy.ndarray`
        The conjugate quaternion :math:`\\mathbf{q}^*` in :math:`[x, y, z, w]` format, shape (4,).
    """
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Performs the Hamilton product (multiplication) of two quaternions, :math:`\\mathbf{q}_1 \\otimes \\mathbf{q}_2`.

    Both input quaternions are assumed to be in the :math:`[x, y, z, w]` format.

    Parameters
    ----------
    q1 : :class:`numpy.ndarray`
        The first quaternion :math:`\\mathbf{q}_1` in :math:`[x, y, z, w]` format, shape (4,).
    q2 : :class:`numpy.ndarray`
        The second quaternion :math:`\\mathbf{q}_2` in :math:`[x, y, z, w]` format, shape (4,).

    Returns
    -------
    :class:`numpy.ndarray`
        The resulting quaternion product :math:`\\mathbf{q}_1 \\otimes \\mathbf{q}_2`
        in :math:`[x, y, z, w]` format, shape (4,).
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


def close_app(root: tk.Tk):
    """
    Closes all active Matplotlib figures and destroys the main Tkinter window.

    This is intended to be used as a cleanup routine when closing a Tkinter application.

    Parameters
    ----------
    root : :class:`tkinter.Tk`
        The main Tkinter root window instance.
    """
    plt.close("all")
    root.destroy()


def test_page(root: tk.Tk, page: tk.Frame):
    """
    Initializes a test Tkinter application window with a given frame.

    Sets up the window title, size, packs the frame, and assigns the
    :func:`close_app` function to the window's close protocol.

    Parameters
    ----------
    root : :class:`tkinter.Tk`
        The main Tkinter root window instance.
    page : :class:`tkinter.Frame`
        The Tkinter frame to be displayed within the window.
    """
    root.title("Test of frame")
    root.geometry("800x400")
    page.pack(fill="both", expand=True)
    root.protocol("WM_DELETE_WINDOW", lambda: close_app(root))
    root.mainloop()
