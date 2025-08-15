import numpy as np
import yaml

class Orbit:
    """
    Class for calculating the position of orbiting elements. 
    Assumes that a perfectly circular orbit.
    """

    def __init__(self, file="Yaml/Orbit.yaml"):
        with open(file, 'r') as file:
            self.param = yaml.safe_load(file)
        file.close()

        self.T = self.param["period"]
        self.r = self.param["altitude"] + 6_791
        self.theta = self.param["theta"]
        self.phi = self.param["phi"]
        self.incline = self.param["incline"]

        self.R_x = np.array([
            [1,     0,      0],
            [0, np.cos(self.incline), -np.sin(self.incline)],
            [0, np.sin(self.incline),  np.cos(self.incline)]
        ])

        # Spherical coordinates position
        self.pos = np.array([0.0, 0.0, 0.0])
        self.pos[0] = self.r * np.sin(self.theta) * np.cos(self.phi)
        self.pos[1] = self.r * np.sin(self.theta) * np.sin(self.phi)
        self.pos[2] = self.r * np.cos(self.theta)

        self.pos = self.R_x @ self.pos.T

        # The vector in the direction of the orbit
        self.vector = np.array([0.0, 0.0, 0.0])

        print("#################### Orbit parameters ####################")
        print(f"Starting position: {self.pos}")
        for key in self.param:
            print(f" - {key}: {self.param[key]}")
        print("##############################################################")
        print()

    def update(self, dt):
        # Update the angles 
        # self.theta += dt * 2 * np.pi / self.T
        self.phi += dt * 2 * np.pi / self.T
        pos = np.array([0.0, 0.0, 0.0])

        # Update the Spherical coordinates with the new angles
        pos[0] = self.r * np.sin(self.theta) * np.cos(self.phi)
        pos[1] = self.r * np.sin(self.theta) * np.sin(self.phi)
        pos[2] = self.r * np.cos(self.theta)

        self.pos = self.R_x @ pos.T
        self.vector = np.subtract(self.pos, pos)
        self.vector = np.linalg.norm(self.vector, 2)


        # # Update the Spherical coordinates with the new angles
        # self.pos[0] += (2 * np.pi * self.r / self.T) * (np.cos(self.theta)*np.sin(self.phi) - np.sin(self.theta)*np.sin(self.phi))
        # self.pos[1] += (2 * np.pi * self.r / self.T) * (np.cos(self.theta)*np.sin(self.phi) - np.sin(self.theta)*np.sin(self.phi))
        # self.pos[2] -= (2 * np.pi * self.r / self.T) * self.r * np.sin(self.theta)
