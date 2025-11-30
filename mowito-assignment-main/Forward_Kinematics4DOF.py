#!/usr/bin/env python3
"""
Task 2: Forward Kinematics for 4-Link Robot with Perpendicular Joints
Author: Nitheeshkumar S
Date: 30 November 2025
"""

import numpy as np
import math


class Kinematics4DOF:
    """
    Forward Kinematics calculator for a 4-link robot
    with perpendicular joints (α = 90°, 90°, 90°, 0°).
    """

    def __init__(self, link_length=1.0):
        self.L = link_length
        self.dh = [
            [0, 0, self.L, np.pi/2],
            [0, 0, self.L, np.pi/2],
            [0, 0, self.L, np.pi/2],
            [0, 0, self.L, 0]
        ]

    def dh_transform(self, theta, d, a, alpha):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,       sa,       ca,      d],
            [0,        0,        0,      1]
        ])

    def forward_kinematics(self, q1, q2, q3, q4, verbose=False):
        T01 = self.dh_transform(q1, *self.dh[0][1:])
        T12 = self.dh_transform(q2, *self.dh[1][1:])
        T23 = self.dh_transform(q3, *self.dh[2][1:])
        T34 = self.dh_transform(q4, *self.dh[3][1:])

        if verbose:
            print("\nIndividual Transformation Matrices:")
            print("T01:\n", T01)
            print("T12:\n", T12)
            print("T23:\n", T23)
            print("T34:\n", T34)

        T02 = T01 @ T12
        T03 = T02 @ T23
        T04 = T03 @ T34

        px, py, pz = T04[0, 3], T04[1, 3], T04[2, 3]
        return px, py, pz

    def full_matrix(self, q1, q2, q3, q4):
        T01 = self.dh_transform(q1, *self.dh[0][1:])
        T12 = self.dh_transform(q2, *self.dh[1][1:])
        T23 = self.dh_transform(q3, *self.dh[2][1:])
        T34 = self.dh_transform(q4, *self.dh[3][1:])
        return T01 @ T12 @ T23 @ T34


def test_forward_kinematics():
    print("=" * 70)
    print("TASK 2: FORWARD KINEMATICS TESTS")
    print("=" * 70)

    robot = Kinematics4DOF(link_length=1.0)

    # -------------------------------
    # Test 1: Zero configuration
    # -------------------------------
    print("\n[Test 1] Zero configuration (all joints = 0 rad)")
    q1 = q2 = q3 = q4 = 0
    x, y, z = robot.forward_kinematics(q1, q2, q3, q4, verbose=True)
    print(f"\nEnd-effector position:\n  x = {x:.4f} m\n  y = {y:.4f} m\n  z = {z:.4f} m")

    # -------------------------------
    # Test 2: Arbitrary configuration
    # -------------------------------
    print("\n" + "=" * 70)
    print("\n[Test 2] Arbitrary configuration (30°, 45°, -20°, 60°)")
    q1, q2, q3, q4 = np.radians([30, 45, -20, 60])
    x, y, z = robot.forward_kinematics(q1, q2, q3, q4)
    print(f"Joint angles (rad): {q1:.6f}, {q2:.6f}, {q3:.6f}, {q4:.6f}")
    print(f"End-effector position:\n  x = {x:.4f} m\n  y = {y:.4f} m\n  z = {z:.4f} m")

    # -------------------------------
    # Test 3: Transformation matrix
    # -------------------------------
    print("\n[Test 3] Full transformation matrix for Test 2:")
    T = robot.full_matrix(q1, q2, q3, q4)
    print(T)

    # -------------------------------
    # Test 5: Workspace exploration table
    # -------------------------------
    print("\n[Test 5] Workspace exploration (various configurations):\n")
    print(f"{'J1°':>6} {'J2°':>6} {'J3°':>6} {'J4°':>6} | {'X':>8} {'Y':>8} {'Z':>8} | Distance")
    print("-" * 70)

    configs = [
        (0, 0, 0, 0),
        (90, 0, 0, 0),
        (0, 90, 0, 0),
        (0, 0, 90, 0),
        (0, 0, 0, 90),
        (45, 45, 45, 45),
        (90, 90, 90, 90),
        (-45, 45, -45, 45),
    ]

    for (d1, d2, d3, d4) in configs:
        r1, r2, r3, r4 = np.radians([d1, d2, d3, d4])
        x, y, z = robot.forward_kinematics(r1, r2, r3, r4)
        dist = np.sqrt(x**2 + y**2 + z**2)

        print(f"{d1:>6.1f}° {d2:>6.1f}° {d3:>6.1f}° {d4:>6.1f}° | "
              f"{x:>8.4f} {y:>8.4f} {z:>8.4f} | {dist:>7.4f}")

    # -------------------------------
    # Test 6: Singularity
    # -------------------------------
    print("\n[Test 6] Checking for singularities:")
    q1, q2, q3, q4 = 0, np.pi, 0, 0
    x, y, z = robot.forward_kinematics(q1, q2, q3, q4)
    print("Singular configuration: J2 = 180° (folded back)")
    print(f"End-effector position: x={x:.4f}m, y={y:.4f}m, z={z:.4f}m")

    print("\n" + "=" * 70)
    print("✓ All forward kinematics tests completed successfully!")
    print("=" * 70)


if __name__ == "__main__":
    test_forward_kinematics()
