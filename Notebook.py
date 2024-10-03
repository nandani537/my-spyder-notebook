#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct  3 11:58:40 2024

@author: nandani
"""

import numpy as np
from scipy.linalg import expm

def skew_symmetric(v):
    """Creates a skew-symmetric matrix from a vector"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def se3_to_matrix(screw):
    """Convert an se(3) representation of a twist into a transformation matrix"""
    omega = screw[:3]
    v = screw[3:]
    skew_omega = skew_symmetric(omega)
    return np.block([
        [skew_omega, v.reshape(3, 1)],
        [0, 0, 0, 0]
    ])

def exp_twist(twist, theta):
    """Calculates the matrix exponential of a twist"""
    se3_matrix = se3_to_matrix(twist)
    return expm(se3_matrix * theta)

def forward_kinematics_spatial(M, screw_axes, joint_angles):
    """Computes the forward kinematics using the product of exponentials formula in the spatial frame."""
    T = np.eye(4)
    for i in range(screw_axes.shape[1]):
        T = T @ exp_twist(screw_axes[:, i], joint_angles[i])
    return T @ M

def forward_kinematics_body(M, screw_axes_body, joint_angles):
    """Computes the forward kinematics using the product of exponentials formula in the body frame."""
    T = np.eye(4)
    for i in range(screw_axes_body.shape[1]):
        T = T @ exp_twist(screw_axes_body[:, i], joint_angles[i])
    return M @ T

# Example definitions for a simple 3-DOF manipulator
M = np.array([
    [1, 0, 0, 1],
    [0, 1, 0, 2],
    [0, 0, 1, 3],
    [0, 0, 0, 1]
])  # Home configuration of the end-effector

screw_axes_spatial = np.array([
    [0, 0, 1, 0, 0, 0],
    [0, 1, 0, -2, 0, 0],
    [1, 0, 0, 0, 3, 0]
]).T  # Screw axes in the spatial frame

screw_axes_body = np.array([
    [0, 0, 1, 0, 2, 0],
    [0, 1, 0, 0, 0, 0],
    [1, 0, 0, 0, 0, -3]
]).T  # Screw axes in the body frame

# Generate random joint angles for testing
np.random.seed(42)
test_joint_angles = [
    np.random.uniform(-np.pi, np.pi, 3),
    np.random.uniform(-np.pi/2, np.pi/2, 3),
    [0, 0, 0],
    [np.pi/2, -np.pi/4, np.pi/6]
]

# Compare results for each set of joint angles
for i, angles in enumerate(test_joint_angles):
    T_spatial = forward_kinematics_spatial(M, screw_axes_spatial, angles)
    T_body = forward_kinematics_body(M, screw_axes_body, angles)
    
    print(f"Test {i + 1}: Joint angles = {angles}")
    print("Spatial frame FK:\n", T_spatial)
    print("Body frame FK:\n", T_body)
    print("Difference:\n", T_spatial - T_body)
    print("\n")
