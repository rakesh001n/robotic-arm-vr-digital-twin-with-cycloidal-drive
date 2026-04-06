#!/usr/bin/env python3

import numpy as np
import math


class Kinematics:
    def __init__(self):
        # Robot parameters (mm)
        self.d1 = 52.0
        self.a2 = 150.0
        self.a3 = 113.5
        self.a4 = 80.0

    # =========================
    # DH MATRIX
    # =========================
    def dh_matrix(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),               np.cos(alpha),                d],
            [0, 0, 0, 1]
        ])

    # =========================
    # FORWARD KINEMATICS
    # =========================
    def forward_kinematics(self, q):
        t1, t2, t3, t4, t5 = q

        T1 = self.dh_matrix(t1, self.d1, 0, np.pi/2)
        T2 = self.dh_matrix(t2, 0, self.a2, 0)
        T3 = self.dh_matrix(t3, 0, self.a3, 0)
        T4 = self.dh_matrix(t4, 0, self.a4, 0)
        T5 = self.dh_matrix(t5, 0, 0, np.pi/2)

        T = T1 @ T2 @ T3 @ T4 @ T5

        position = T[0:3, 3]
        return position, T

    # =========================
    # INVERSE KINEMATICS
    # =========================
    def inverse_kinematics(self, target_pos, current_q):
        px, py, pz = target_pos

        # Base
        theta1 = math.atan2(py, px)

        # Planar conversion
        r = math.sqrt(px**2 + py**2)
        z = pz - self.d1

        # Maintain orientation
        theta234 = current_q[1] + current_q[2] + current_q[3]
        theta5 = current_q[4]

        # Wrist center
        wc_r = r - self.a4 * math.cos(theta234)
        wc_z = z - self.a4 * math.sin(theta234)

        # Law of cosines
        D = (wc_r**2 + wc_z**2 - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)

        if abs(D) > 1:
            print("⚠️ Target out of reach")
            return current_q

        t3_1 = math.acos(D)
        t3_2 = -math.acos(D)

        solutions = []

        for t3 in [t3_1, t3_2]:
            t2 = math.atan2(wc_z, wc_r) - math.atan2(
                self.a3 * math.sin(t3),
                self.a2 + self.a3 * math.cos(t3)
            )

            t4 = theta234 - (t2 + t3)

            solutions.append(np.array([theta1, t2, t3, t4, theta5]))

        # Choose closest solution
        best_q = min(solutions, key=lambda q: np.linalg.norm(q - current_q))

        return best_q


# =========================
# TEST (OPTIONAL)
# =========================
if __name__ == "__main__":
    kin = Kinematics()

    q = np.array([0, 0, 0, 0, 0], dtype=float)

    pos, _ = kin.forward_kinematics(q)
    print("FK:", pos)

    q_sol = kin.inverse_kinematics(pos, q)
    print("IK:", np.degrees(q_sol))

    