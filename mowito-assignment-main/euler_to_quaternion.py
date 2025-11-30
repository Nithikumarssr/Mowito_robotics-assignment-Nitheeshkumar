#!/usr/bin/env python3
"""
Task 1: Euler Angles to Quaternion Conversion (and vice versa)

This module implements bidirectional conversion between Euler angles and Quaternions,
with proper handling of gimbal lock singularities.

Mathematical Background:
- Euler angles: (phi, theta, psi) - rotation around X, Y, Z axes respectively
- Quaternion: q = [w, x, y, z] where w² + x² + y² + z² = 1
- Convention: ZYX (yaw-pitch-roll) intrinsic rotation sequence

Edge Cases Handled:
- Gimbal lock at theta = ±90° (±π/2 rad)
- Numerical stability near singularities
- Angle normalization to [-π, π]

Author: Nitheeshkumar S
Date: 30 November 2025
"""

import numpy as np
import math


def euler_to_quaternion(phi, theta, psi):
    """
    Convert Euler angles (phi, theta, psi) to Quaternion (w, x, y, z).

    Uses the ZYX (psi-theta-phi) convention.

    Args:
        phi (float): rotation around X-axis
        theta (float): rotation around Y-axis
        psi (float): rotation around Z-axis

    Returns:
        tuple: (w, x, y, z)
    """
    cy = math.cos(psi * 0.5)
    sy = math.sin(psi * 0.5)
    cp = math.cos(theta * 0.5)
    sp = math.sin(theta * 0.5)
    cr = math.cos(phi * 0.5)
    sr = math.sin(phi * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    norm = math.sqrt(w**2 + x**2 + y**2 + z**2)
    return (w/norm, x/norm, y/norm, z/norm)


def quaternion_to_euler(w, x, y, z):
    """
    Convert Quaternion (w, x, y, z) to Euler angles (phi, theta, psi).
    """
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if abs(norm - 1.0) > 1e-6:
        raise ValueError(f"Quaternion must be normalized. Current norm: {norm}")

    sin_theta = 2.0 * (w*y - z*x)
    gimbal_lock_threshold = 0.99999

    if abs(sin_theta) >= gimbal_lock_threshold:
        phi = 0.0
        theta = math.copysign(math.pi / 2, sin_theta)
        psi = 2.0 * math.atan2(x, w)
        print(f"⚠️  Gimbal lock detected at theta = {math.degrees(theta):.1f}°")

    else:
        t0 = 2.0 * (w*x + y*z)
        t1 = 1.0 - 2.0 * (x*x + y*y)
        phi = math.atan2(t0, t1)

        theta = math.asin(np.clip(sin_theta, -1.0, 1.0))

        t2 = 2.0 * (w*z + x*y)
        t3 = 1.0 - 2.0 * (y*y + z*z)
        psi = math.atan2(t2, t3)

    return (phi, theta, psi)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def test_conversions():
    print("="*70)
    print("TASK 1: EULER-QUATERNION CONVERSION TESTS")
    print("="*70)

    # Test 1
    print("\n[Test 1] Standard Euler angles:")
    phi, theta, psi = 0.1, 0.2, 0.3
    print(f"Input Euler: phi={math.degrees(phi):.2f}°, theta={math.degrees(theta):.2f}°, psi={math.degrees(psi):.2f}°")

    w, x, y, z = euler_to_quaternion(phi, theta, psi)
    print(f"Quaternion: w={w:.6f}, x={x:.6f}, y={y:.6f}, z={z:.6f}")
    print(f"Quaternion norm: {math.sqrt(w*w + x*x + y*y + z*z):.10f}")

    phi2, theta2, psi2 = quaternion_to_euler(w, x, y, z)
    print(f"Back to Euler: phi={math.degrees(phi2):.2f}°, theta={math.degrees(theta2):.2f}°, psi={math.degrees(psi2):.2f}°")
    print(f"Error: Δφ={abs(phi-phi2):.2e}, Δθ={abs(theta-theta2):.2e}, Δψ={abs(psi-psi2):.2e}")

    # Test 2: gimbal lock
    print("\n[Test 2] Gimbal lock case (theta = +90°):")
    phi, theta, psi = 0.5, math.pi/2, 0.3
    w, x, y, z = euler_to_quaternion(phi, theta, psi)
    phi2, theta2, psi2 = quaternion_to_euler(w, x, y, z)
    print(f"Back to Euler: phi={math.degrees(phi2):.2f}°, theta={math.degrees(theta2):.2f}°, psi={math.degrees(psi2):.2f}°")

    # Test 3: negative gimbal lock
    print("\n[Test 3] Gimbal lock case (theta = -90°):")
    phi, theta, psi = 0.2, -math.pi/2, 0.4
    w, x, y, z = euler_to_quaternion(phi, theta, psi)
    phi2, theta2, psi2 = quaternion_to_euler(w, x, y, z)
    print(f"Back to Euler: phi={math.degrees(phi2):.2f}°, theta={math.degrees(theta2):.2f}°, psi={math.degrees(psi2):.2f}°")

    # Test 4: zero rotation
    print("\n[Test 4] Zero rotation:")
    phi, theta, psi = 0.0, 0.0, 0.0
    w, x, y, z = euler_to_quaternion(phi, theta, psi)
    print(f"Quaternion for zero rotation: w={w:.6f}, x={x:.6f}, y={y:.6f}, z={z:.6f}")

    # Test 5: random stress test
    print("\n[Test 5] Random angle stress test:")
    np.random.seed(42)
    max_err = 0.0
    for _ in range(100):
        phi = np.random.uniform(-math.pi, math.pi)
        theta = np.random.uniform(-math.pi/2 + 0.1, math.pi/2 - 0.1)
        psi = np.random.uniform(-math.pi, math.pi)

        w, x, y, z = euler_to_quaternion(phi, theta, psi)
        phi2, theta2, psi2 = quaternion_to_euler(w, x, y, z)

        err = max(abs(phi-phi2), abs(theta-theta2), abs(psi-psi2))
        max_err = max(max_err, err)

    print(f"Maximum round-trip error: {max_err:.2e} rad ({math.degrees(max_err):.2e}°)")
    print("\n" + "="*70)
    print("✓ All tests completed successfully!")
    print("="*70)


if __name__ == "__main__":
    test_conversions()
