from numpy.core.shape_base import hstack
import yaml
import numpy as np
import quaternion
import misc
from Orbit import Orbit
from Satellite import Satellite
from Target import Target, TargetEarth
import pdb


def project_to_plane(vec, n):
    a = vec - (np.dot(vec, n)/(np.dot(n, n))) * n
    return a

def orthonormalize(x, y, z):
    x /= np.linalg.norm(x, 2)
    y -= np.dot(y, x) * x  # Make y orthogonal to x
    y /= np.linalg.norm(y, 2)
    z = np.cross(x, y)  # Ensure orthogonality
    return x, y, z

def llh2ecef(l:float, mu:float, h:float):
    # [x,y,z] = LLH2ECEF(l,mu,h) computes the  ECEF positions (x,y,z)
    # from longitude l (rad), latitude mu (rad) and height h above the surface 
    # of the WGS-84 elipsoid.
    #
    # Author:    Thor I. Fossen
    # Date:      14th June 2001
    # Revisions: 27th January 2003, inputs l and mu are defined in rad
    #            30 Apr 2019, added decimals to r_e and r_p
    #
    # Converted to python 

    r_e = 6378137;              # WGS-84 data
    r_p = 6356752.3142

    e = 0.08181979099211
    N = r_e^2/np.sqrt( (r_e*np.cos(mu))^2 + (r_p*np.sin(mu))^2 )
    x = (N + h) * np.cos(mu) * np.cos(l)
    y = (N + h) * np.cos(mu) * np.sin(l)
    z = (N * (r_p/r_e)^2 + h) * np.sin(mu)
    
    return np.array([x, y, z])

def ssa(angle, unit="rad"):
    """
    Provide the smallest signed angle. It maps the angle to [-pi, pi]
    """
    if unit == "rad":
        return (angle + np.pi) % (2 * np.pi) - np.pi 
    if unit == "deg":
        return (angle + 180.0) % (360.0) - 180.0
    print(f"Could not recognise unit {unit}")


class Reference:
    def __init__(self, file="Yaml/Reference.yaml") -> None:
        with open(file, 'r') as file:
            self.param = yaml.safe_load(file)

        # Load parameters
        self.qc = np.quaternion(*self.param["qc"])
        try:
            self.wc = np.array(self.param["wc"])
            self.wc_dot = np.array(self.param["wc_dot"])
        except:
            self.wc = np.array([0.0, 0.0, 0.0])
            self.wc_dot = np.array([0.0, 0.0, 0.0])
            print(f"Only found reference for the attitude, assumes constant pointing")

    def print_config(self):
        print("#################### Reference parameters ####################")
        print(f"Starting reference attitude: {self.qc}")
        print(f"Starting reference angular velocity: {self.wc}")
        print(f"Starting reference angular rate: {self.wc_dot}")
        print("##############################################################")
        print()

    def get_data(self, data):
        if data == "velocity":
            return self.wc
        if data == "quaternion":
            return quaternion.as_float_array(self.qc)

    def update(self, dt):
        # (3.20) Update the attitude based on the angular velocity
        qw = np.quaternion(0, *self.wc)
        q_dot = 1/2 * qw * self.qc
        self.qc += dt * q_dot
        self.qc = misc.quat_norm(self.qc)


class TargetReference(Reference):
    def __init__(self, satellite:Satellite, orbit:Orbit, file="Yaml/Reference.yaml") -> None:
        super().__init__(file)
        self.orbit = orbit
        self.satellite = satellite

        try:
            self.theta = self.param["theta"]
            self.phi = self.param["phi"]
        except:
            self.theta = 0.0
            self.phi = 0.0
            print(f"Could not get a valid target position. Setting it to (0, 0)")

class EarthReferencePD(Reference):
    def __init__(self, satellite:Satellite, orbit:Orbit, file="Yaml/Reference.yaml") -> None:
        super().__init__(file)
        self.orbit = orbit
        self.satellite = satellite
        try:
            self.theta = self.param["theta"]
            self.phi = self.param["phi"]
        except:
            self.theta = 0.0
            self.phi = 0.0
            print(f"Could not get a valid target position. Setting it to (0, 0)")

        # Convert latitude and longitude from degrees to radians
        self.theta *= np.pi / 180.0
        self.phi *= np.pi / 180.0
        self.r = 6_791.0
        self.target_pos = np.array([0.0, 0.0, 0.0])
        self.target_pos[0] = self.r * np.sin(self.theta) * np.cos(self.phi)
        self.target_pos[1] = self.r * np.sin(self.theta) * np.sin(self.phi)
        self.target_pos[2] = self.r * np.cos(self.theta)
        self.R = np.eye(3)

        self.orbit_vector = np.array([0.0, 0.0, 0.0])

        # Testing some shit
        self.y = [0.0, 0.0, 0.0]

    def propagate(self, dt):
        """
        Update the target position.
        """
        self.phi += dt * 2*np.pi / (24 * 60 * 60)
        self.target_pos[0] = self.r * np.sin(self.theta) * np.cos(self.phi)
        self.target_pos[1] = self.r * np.sin(self.theta) * np.sin(self.phi)


    def update(self, dt):
        # self.orbit.update(dt)
        # self.propagate(dt)

        x = self.target_pos - self.orbit.pos
        x /= np.linalg.norm(x, 2)
        # print()
        # print(f"x: {x}")

        # Project the orbit vector onto the plane normal to the target vector
        y = project_to_plane(self.orbit.vector, x)

        # The orbit vector almost aligns with the target vector so we use 
        # the current position to determine the y- and z-axis insead.
        if (np.linalg.norm(y, 2) < 0.1):
            y_sat = quaternion.as_rotation_matrix(self.satellite.q)[:, 1]
            z_sat = quaternion.as_rotation_matrix(self.satellite.q)[:, 2]
            y = project_to_plane(y_sat, x)
            z = project_to_plane(z_sat, x)
            if (np.linalg.norm(y, 2) > np.linalg.norm(z, 2)):
                z = np.cross(x, y)
            else:
                y = np.cross(x, z)
        else:
            z = np.cross(x, y)

        # Normalize the axis vectors
        y /= np.linalg.norm(y, 2)
        z /= np.linalg.norm(z, 2)

        x, y, z = orthonormalize(x, y, z)
        self.R = np.vstack((x,y,z))
        self.R = self.R.T
        # print(f"R_x: {self.R[:,0]}")
        self.qc = quaternion.from_rotation_matrix(self.R)
        # self.qc = misc.quat_norm(self.qc)

        Q = quaternion.as_rotation_matrix(self.qc)

        # Shows that the x-axis changes after being converted to a quaternion
        # print(f"Q_x: {Q[:,0]}")
        # print(f"Error: {np.subtract(self.R[:,0],Q[:,0])}")
        # print(np.linalg.det(self.R))
        # print(np.linalg.det(Q))

        # print()
        # print("Original R:\n", self.R)
        # print("Reconstructed Q:\n", Q)
        # print("Difference (Q - R):\n", Q - self.R)
        # print("Error Norm:", np.linalg.norm(Q - self.R))
        # print(np.linalg.det(self.R))
        # print(Q @ self.R - np.eye(3))
        # print(self.R @ self.R.T)
        # print(np.linalg.norm(np.subtract(self.R[:, 0], x), 2))
        # print(np.linalg.norm(np.subtract(Q[:, 0], x), 2))
        # print(np.linalg.norm(np.subtract(Q[0, :], x), 2))
        # print()

class SimplePointing(Reference):
    """Class for testing the sliding mode controller"""

    def __init__(self, file="Yaml/Reference.yaml"):
        super().__init__(file)

    def updateAngularVelocity(self, dt: float):
        self.wc += self.wc_dot * dt
        return self.wc

    def update(self, dt: float):
        qw = np.quaternion(0, *self.wc)
        q_dot = 1/2 * qw * self.qc
        self.qc += dt * q_dot
        self.qc = misc.quat_norm(self.qc)

        # return self.qc, self.wc, self.wc_dot


class EarthPointing:
    """Class for generating the reference values for the sliding mode controller
    when pointing at a fixed point at earth at all times"""

    w_earth = 7.2921159e-10                 # Angular velocity of earth  (rad/s)
    qd = np.quaternion(1.0, 0.0, 0.0, 0.0)  # Desired attitude in orbit frame
    glos = np.array([63.41962246828129, 10.401924446004454]) # Logitude and latitude of NTNU Gløshaugen

    def __init__(self, latitude, longitude, height=0.0) -> None:
        self.target_pos = llh2ecef(latitude, longitude, height)

    def updateTragetPos(self, dt):
        psi = self.w_earth * dt 


    def update(self):
        pass


class WMAP(Reference):
    """
    Class for generating the WMAP trajectory from the example
    """

    # # The desired states for observing
    # phi_c_dot = 0.001745            # rad/s
    # theta_c  = 0.3927               # rad
    # psi_c_dot = 0.04859             # rad/s

    # The desired states for observing
    phi_c_dot = 0.745               # rad/s
    theta_c  = 0.3927               # rad
    psi_c_dot = 0.4859              # rad/s


    def __init__(self) -> None:
        self.qc = np.quaternion(1.0, 0.0, 0.0, 0.0)
        self.wc = np.array([1.0, 0.0, 0.0])
        self.wc_dot = np.array([0.0, 0.0, 0.0])

        self.phi_c = 0.0
        self.psi_c = 0.0

    def commandedQuaternion(self):
        qc = np.array([
            np.cos(self.theta_c/2) * np.cos((self.phi_c + self.psi_c)/2),
            np.sin(self.theta_c/2) * np.cos((self.phi_c - self.psi_c)/2),
            np.sin(self.theta_c/2) * np.sin((self.phi_c - self.psi_c)/2),
            np.cos(self.theta_c/2) * np.sin((self.phi_c + self.psi_c)/2)
        ])
        return quaternion.quaternion(*qc)

    def commandedAngularVelocity(self, theta_c: float):
        wc = np.array([
            self.psi_c_dot * np.sin(self.theta_c) * np.sin(theta_c),
            self.psi_c_dot * np.sin(self.theta_c) * np.cos(theta_c),
            self.phi_c_dot
        ])
        return wc

    def commandedAngularRate(self):
        self.wc_dot = self.psi_c_dot * self.phi_c_dot * np.array([
            np.sin(self.theta_c) * np.cos(self.psi_c),
            - np.sin(self.theta_c) * np.sin(self.psi_c),
            0
        ])
        # return wc_dot
        # return self.wc_dot

    def update(self, dt):
        # Update the commanded angular velocity
        # self.wc_dot = self.commandedAngularRate()
        self.commandedAngularRate()
        self.wc += dt * self.wc_dot

        # Integrate the commanded euler angles
        self.phi_c += dt * self.wc[0]
        self.psi_c += dt * self.wc[2]

        self.qc = self.commandedQuaternion()
        # return self.qc, self.wc, self.wc_dot
    
    def print(self):
        print()
        print(f"Commanded Quaternion: {self.qc}")
        print(f"Commanded Angular velocity: {self.wc}")
        print(f"Commanded Angular rate: {self.wc_dot}")
        print()


class ReferenceV2:
    def __init__(self, sat:Satellite, target:TargetEarth, file="Yaml/Reference.yaml") -> None:
        with open(file, 'r') as file:
            self.param = yaml.safe_load(file)
        file.close()

        self.satellite = sat

        try:
            self.qc = self.param["qc"]
            self.wc = self.param["wc"]
        except:
            print("Could not load reference parameters")

        print("#################### Reference parameters ####################")
        print(f"Starting reference attitude: {self.qc}")
        print(f"Starting reference angular velocity: {self.wc}")
        print("##############################################################")
        print()

    def angular_velocity(self, target:Target, secondary_target:Target, dt):
        q0 = self.attitude(self.satellite.pos, target.pos, secondary_target.pos)

        # TODO: Propagate the target and satellite position for 1 time step
        q1 = self.attitude(self.satellite.pos, target.pos, secondary_target.pos)

        qe = q1 * misc.quat_conjugate(q0)
        theta = 2 * np.arccos(qe.w)
        w = quaternion.as_float_array(qe)[1:] / np.linalg.norm(quaternion.as_float_array(qe)[1:], 2)
        return theta / dt * w

    def attitude(self, pos:np.ndarray, target_pos:np.ndarray, secondary_target_pos:np.ndarray):
        v1 = target_pos - pos
        v1 /= np.linalg.norm(v1, 2)

        v2 = secondary_target_pos - pos
        v2 /= np.linalg.norm(v2, 2)

        # Project the second target vector onto the normal plane
        y = project_to_plane(v2, v1)

        # If the secondary target is right above we need to use the current attitude instead
        if (np.linalg.norm(y, 2) < 0.1):
            y_sat = quaternion.as_rotation_matrix(self.satellite.q)[:, 1]
            z_sat = quaternion.as_rotation_matrix(self.satellite.q)[:, 2]
            y = project_to_plane(y_sat, v1)
            z = project_to_plane(z_sat, v1)
            if (np.linalg.norm(y, 2) > np.linalg.norm(z, 2)):
                z = np.cross(v1, y)
            else:
                y = np.cross(v1, z)
        else:
            z = np.cross(v1, y)

        # Normalize the axis vectors
        y /= np.linalg.norm(y, 2)
        z /= np.linalg.norm(z, 2)

        x, y, z = orthonormalize(v1, y, z)
        R = np.vstack((x,y,z))
        R = R.T
        return quaternion.from_rotation_matrix(R)


def test_attitude(pos:np.ndarray, q, target_pos:np.ndarray, secondary_target_pos:np.ndarray):
    v1 = target_pos - pos
    v1 /= np.linalg.norm(v1, 2)

    v2 = secondary_target_pos - pos
    v2 /= np.linalg.norm(v2, 2)

    # Project the second target vector onto the normal plane
    y = project_to_plane(v2, v1)

    # If the secondary target is right above we need to use the current attitude instead
    if (np.linalg.norm(y, 2) < 0.1):
        y_sat = quaternion.as_rotation_matrix(q)[:, 1]
        z_sat = quaternion.as_rotation_matrix(q)[:, 2]
        y = project_to_plane(y_sat, v1)
        z = project_to_plane(z_sat, v1)
        if (np.linalg.norm(y, 2) > np.linalg.norm(z, 2)):
            z = np.cross(v1, y)
        else:
            y = np.cross(v1, z)
    else:
        z = np.cross(v1, y)

    # Normalize the axis vectors
    y /= np.linalg.norm(y, 2)
    z /= np.linalg.norm(z, 2)

    x, y, z = orthonormalize(v1, y, z)
    R = np.vstack((x,y,z))
    R = R.T
    print(R.T)
    return quaternion.from_rotation_matrix(R)

if __name__ == '__main__':
    print(project_to_plane(np.array([1, 2, 3]), np.array([1, 0, 0])))
    pos = np.array([0.0, 0.0, 0.0])
    q = np.quaternion(1.0, 0.0, 0.0, 0.0)
    t1 = np.array([1.0, 2.0, 3.0])
    t2 = np.array([-1.0, 2.0, 0.0])
    print(test_attitude(pos, q, t1, t2))


