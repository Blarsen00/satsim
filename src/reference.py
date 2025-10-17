"""
Reference Frames Module

This module provides classes for defining and updating reference states (attitude
and angular velocity) for simulated bodies, such as spacecraft.

Classes
-------
BaseReference
    A generic base class for reference frames that updates dynamics using simple 
    forward integration.
EarthReference
    A specialized class that calculates a dynamic reference attitude based on 
    pointing to a primary target while attempting to align a secondary axis.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from dataclasses import is_dataclass, fields
from simulation import Simulate, PhysicalState


class BaseReference:
    """
    Base class for reference state generation.

    This class maintains and updates a reference state (attitude :math:`\\mathbf{R}` and 
    angular velocity :math:`\\mathbf{\\omega}`) using simple forward integration.

    Attributes
    ----------
    state : :class:`simulation.PhysicalState`
        The current reference physical state (attitude, angular velocity, etc.).
        Defaults to a new :class:`PhysicalState`.
    """
    def __init__(self, state=None) -> None:
        """
        Initializes the BaseReference with an optional initial state.

        Parameters
        ----------
        state : :class:`simulation.PhysicalState`, optional
            The initial physical state. If None, a new :class:`PhysicalState` is used.
        """
        self.state = PhysicalState() if state is None else state

    def update(self, dt: float, **kwargs):
        """
        Updates the reference state for a single time step.

        The update uses simple forward Euler integration for angular velocity 
        (:math:`\\mathbf{\\omega}`) and updates the attitude (:math:`\\mathbf{R}`) based on the 
        new angular velocity.

        Parameters
        ----------
        dt : float
            The time step duration in seconds.
        **kwargs : dict
            Placeholder for additional parameters not used in the base class update.

        Returns
        -------
        :class:`simulation.PhysicalState`
            The updated reference physical state.
        """
        self.state.w = Simulate.direct_euler(self.state.w, 
                                             self.state.w_dot,
                                             dt)
        self.state.rot = Simulate.calculate_attitude(self.state.rot,
                                                     self.state.w,
                                                     dt)
        return self.state

    def __str__(self) -> str:
        s = ''
        for field in fields(self.state):
            s += "{:<10}: {}\n".format(field.name, getattr(self.state, field.name))
        return s


class EarthReference(BaseReference):
    """
    A specialized reference frame class for Earth-centric targeting.

    This class dynamically calculates a reference attitude designed for pointing:
    The primary body x-axis points toward a primary target, and the secondary 
    body y-axis is oriented as closely as possible toward a secondary target 
    while remaining orthogonal to the primary axis. A typical setup will pass
    a coordinate on Earth to point at, and then point the y-axis at the sun as
    the secondary target. Both coordinates are expected to be given in ECI frame
    and the update step will only generate the attitude, not the angular velocity
    which will be initialized to zero.

    Inherits from
    ------------
    :class:`BaseReference`
    """
    def __init__(self, state=None) -> None:
        """
        Initializes the EarthReference.

        Parameters
        ----------
        state : :class:`simulation.PhysicalState`, optional
            The initial physical state. If None, a new :class:`PhysicalState` is used.
        """
        super().__init__(state)

    @staticmethod
    def project_to_plane(vec: np.ndarray, n: np.ndarray) -> np.ndarray:
        a = vec - (np.dot(vec, n)/(np.dot(n, n))) * n
        return a

    @staticmethod
    def orthonormalize(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> np.ndarray:
        """
        Orthonormalizes three vectors using the modified Gram-Schmidt process.

        Note: This method is present but not used by the current :meth:`update` 
        implementation, which relies on cross products for the final basis.

        Parameters
        ----------
        x : :class:`numpy.ndarray`
            The first vector (will be normalized), shape (3,).
        y : :class:`numpy.ndarray`
            The second vector (will be made orthogonal to x and normalized), shape (3,).
        z : :class:`numpy.ndarray`
            The third vector, shape (3,).

        Returns
        -------
        tuple of :class:`numpy.ndarray`
            The orthonormalized vectors :math:`(\\mathbf{x}, \\mathbf{y}, \\mathbf{z})`.
        """
        x /= np.linalg.norm(x, 2)
        y -= np.dot(y, x) * x  # Make y orthogonal to x
        y /= np.linalg.norm(y, 2)
        z = np.cross(x, y)  # Ensure orthogonality
        return x, y, z

    def update(self, dt: float, **kwargs):
        """
        Updates the reference attitude (:math:`\\mathbf{R}_{ref}`) for Earth-centric pointing.

        The new reference attitude is calculated such that the body x-axis (:math:`\\mathbf{x}`) 
        points from the satellite position (:math:`\\mathbf{p}_{sat}`) to the primary target 
        position (:math:`\\mathbf{p}_{target}`), and the body y-axis (:math:`\\mathbf{y}`) aligns 
        closest to the secondary target vector.

        Parameters
        ----------
        dt : float
            The time step duration. (Ignored in attitude calculation).
        **kwargs : dict
            Required keyword arguments:
            
            * **target_pos** (:class:`numpy.ndarray`): Primary target position vector (ECI frame).
            * **sat_pos** (:class:`numpy.ndarray`): Satellite position vector (ECI frame).
            * **second_target** (:class:`numpy.ndarray`): Vector defining the secondary target direction (ECI frame).
            * **sat_rot** (:class:`scipy.spatial.transform.Rotation`): Current satellite attitude (used for edge cases).

        Returns
        -------
        :class:`simulation.PhysicalState`
            The updated reference physical state. Only `self.state.rot` is modified.

        Notes
        -----
        If the secondary target is nearly collinear with the primary target vector 
        (:math:`\\mathbf{x}`), the norm of the projected vector :math:`\\mathbf{y}` will be near zero 
        (:math:`< 0.1`). In this case, the current satellite attitude's y- and z-axes are 
        projected onto the plane orthogonal to :math:`\\mathbf{x}`, and the one with the 
        largest norm is used to define the new :math:`\\mathbf{y}` axis to maintain a stable 
        reference.
        """
        try:
            # NOTE: ECI Reference frame
            target_pos: np.ndarray = kwargs["target_pos"]
            sat_pos: np.ndarray = kwargs["sat_pos"]
            second_target: np.ndarray = kwargs["second_target"]

            # NOTE: Satellite rotation matrix with respect to inertial frame (ECI)
            sat_rot: Rotation = kwargs["sat_rot"]
            R: np.ndarray = sat_rot.as_matrix() # Store the rotation matrix of sat

        except:
            # Default back to the basic reference if the wrong parameters are provided
            return super().update(dt)

        # Normalized vector from satellite to target position gives the x-axis
        x = target_pos - sat_pos
        x /= np.linalg.norm(x, 2)

        # NOTE: Project the second target vector onto the plane determined by the 
        # primary target, and align the y-axis with this projection. This will
        # correspond to the direction in which the y-axis points as much in the
        # direction of the second target as possible, while maintaining that the
        # x-axis points at the primary target.
        y = self.project_to_plane(second_target, x)

        # NOTE: The norm check is for the case where the second target is close to
        # exactly aligned with the primary target, meaning that it is impossible
        # to have the y-axis pointing in its direction, and there is no determined
        # solution for it. In that case the current attitude is used to maintain
        # the direction of the y-axis in order to minimize movements.

        # NOTE: The norm of y can be viewed as a measure as to how successful one 
        # can be in the endevour of pointing the y-axis in the direction of the
        # secondary target. The norm of y can be 1 at most because x is normalized.
        #        norm(y, 2) == 1 <=> y can perfectly point at secondary target.
        #        norm(y, 2) == 0 <=> y points exactly perpendicular to 2. target.
        if (np.linalg.norm(y,2) < 0.1):
            y_sat = R[:, 1]
            z_sat = R[:, 2]
            y = self.project_to_plane(y_sat, x)
            z = self.project_to_plane(z_sat, x)

            # NOTE: The current attitude is projected onto the plane created by
            # the primary target, and the axis with the strongest gets to be fixed,
            # while the other is determined by the cross product of the other two.
            # This is incase one of them happens to be perpendicular to the plane,
            # as that would give the zero vector, and break the program. Should
            # be noted that this might HAVE to be y, because of the check above,
            # but there is not real reason to remove this.

            if np.linalg.norm(y, 2) > np.linalg.norm(z, 2):
            # NOTE: Normalize before computing the cross product
                y /= np.linalg.norm(y, 2)
                z = np.cross(x, y)
            else:
                z /= np.linalg.norm(z, 2)
                y = np.cross(x, z)
        else:
            y /= np.linalg.norm(y, 2)
            z = np.cross(x, y)

        # NOTE: The reconstruction of the rotation matrix for the reference attitude
        # is handled automatically by scipy.
        self.state.rot = Rotation.from_matrix(np.column_stack((x,y,z)))

        return self.state
