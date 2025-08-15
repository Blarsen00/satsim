import numpy as np
import yaml
import quaternion
from Actuators.Reactionwheel import ReactionWheel
from Actuators.Magnetorquer import Magnetorquer
import misc


class Satellite:
    """
    Class describing the dynamaics of a satellite. 
    Computes
        - Attitude
        - Orbit
    """
    def __init__(self, file="Yaml/Satellite.yaml") -> None:
        with open(file, 'r') as file:
            self.param = yaml.safe_load(file)

        # Load state parameters: attitude, angular velocity and position
        self.q = self.param["q"]
        self.q = np.quaternion(*self.q)
        self.w = np.array(self.param["w"])
        self.control = np.array(self.param["control actuators"])

        # Load constant parameters
        self.J = np.array(self.param["J"])
        self.J_inv = np.linalg.inv(self.J)
        self.H = np.array(self.param["H"])

        # Load queue size, it defines the max size of states to log at any given time
        self.queue_size = self.param["queue size"]

        # TODO: Fix this. The satellite should have a position
        self.pos = np.array([])

        # Tables for data
        self.torques = []
        self.commanded_torques = []
        self.angular_velocities = []
        self.attitudes = []
        self.references = []

        print("#################### Satellite parameters ####################")
        print(f"- Starting quaternion: {self.q}")
        print(f"- Starting angular velocity: {self.w}")
        print(f"- Queue size: {self.queue_size}")
        print(f"- H: {self.H}")
        print(f"- J: \n{self.J}")
        print(f"- Control: \n{self.control}")
        print("##############################################################")
        print()

    def log_state(self, table, state):
        if len(table) >= self.queue_size:
            table.pop(0)
            table.append(state)
        else:
            table.append(state)

    def get_data(self, data):
        if data == "velocity":
            return self.angular_velocities[-1]
        if data == "torque":
            return self.torques[-1]
        if data == "quaternion":
            return quaternion.as_float_array(self.attitudes[-1])
        print(f"Did not recognise the request of data of {data}")

    def calculate_angular_momentum(self, L, dt):
        H_dot = L - misc.skew_symmetric(self.w) * self.H
        return self.H + dt * H_dot

    def calculate_angular_velocity(self, L, dt):
        # (3.81) Update the angular velocity
        self.dw = self.J_inv @ (L - misc.skew_symmetric(self.w) @ self.J @ self.w)
        return self.w + dt * self.dw

    def calculate_attitude(self, L, dt):
        w = self.calculate_angular_velocity(L, dt)
        H = self.calculate_angular_momentum(L, dt)

        # (3.20) Update the attitude
        qw = np.quaternion(0, *self.w)
        q_dot = 1/2 * qw * self.q
        q = self.q + dt * q_dot
        return misc.quat_norm(q)

    def update_attitude(self, L, dt):
        """
        Updates the attitude of the satellite over a given period dt, using the equations
        for attitude given in the book. Uses Euler integration. 
        """
        # Actual torque applied
        LB = self.torque_translation(L, dt)

        self.H = self.calculate_angular_momentum(LB, dt)
        self.w = self.calculate_angular_velocity(LB, dt)
        self.q = self.calculate_attitude(LB, dt)

        # Update tables for plotting:
        self.log_state(self.torques, LB)
        self.log_state(self.commanded_torques, L)
        self.log_state(self.angular_velocities, self.w)
        if self.q.w < 0:
            self.log_state(self.attitudes, -self.q)
        else:
            self.log_state(self.attitudes, self.q)
        # self.log_state(self.references, self.q_ref)
    
    def torque_translation(self, L, dt):
        """
        Translate the commanded torque L, into realised torque given the force
        actuators of the satellite, and return that.  
        """
        return L
        # TODO: Fix this mess
        # We have no specified actuation, so optimal actuation is implied.
        # if self.actuators == None:
        # if self.control == None:
        #     return L

        # applied_torque = np.array([0.0, 0.0, 0.0])
        # for i, rw in enumerate(self.control):
        #     applied_torque[i] = rw.apply_torque(5*L[i], dt=dt)
        #     print(applied_torque[i])
        # print()
        # return applied_torque

