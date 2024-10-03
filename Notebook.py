#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct  3 11:00:23 2024

@author: nandani
"""

import numpy as np
from scipy.linalg import expm

# Compute skew-symmetric matrix for a vector
def skew(vector):
    return np.array([[0, -vector[2], vector[1]],
                     [vector[2], 0, -vector[0]],
                     [-vector[1], vector[0], 0]])

#  Compute the adjoint transformation matrix for a homogeneous transformation matrix T
def adjoint(T):
    R = T[:3, :3]  # Rotation part
    p = T[:3, 3]   # Position part
    p_skew = skew(p)
    Ad_T = np.block([[R, np.zeros((3, 3))],
                     [p_skew @ R, R]])
    return Ad_T

#  Compute the transformation matrix using the twist (screw axis) and joint angle
def transformation_from_twist(S, theta):
    w = S[:3]   # Rotational part
    v = S[3:]   # Translational part
    skew_w = skew(w)
    
    # Calculate the exponential map of the twist
    mat_exp = expm(np.block([[skew_w, v.reshape(3, 1)], [0, 0, 0, 0]]) * theta)
    
    return mat_exp

#  Forward kinematics in spatial frame
def forward_kinematics_spatial(S_list, M, thetas):
    T = np.eye(4)  # Identity matrix to start with
    for i in range(len(thetas)):
        T = np.dot(T, transformation_from_twist(S_list[i], thetas[i]))  # Multiply transformations
    return np.dot(T, M)  # Multiply by home configuration matrix

#  Convert spatial screw axes to body screw axes using adjoint transformation
def compute_body_screw_axes(S_list, M):
    Ad_M_inv = np.linalg.inv(adjoint(M))  # Inverse of adjoint of M
    B_list = [Ad_M_inv @ S for S in S_list]  # Transform spatial screw axes into body screw axes
    return B_list

#  Forward kinematics in body frame
def forward_kinematics_body(B_list, M, thetas):
    T = M  # Start with the home configuration matrix
    for i in reversed(range(len(thetas))):
        T = np.dot(transformation_from_twist(B_list[i], thetas[i]), T)  # Notice the multiplication order
    return T

# Example of  screw axes (replace with actual values as per the robot's configuration)
S_list = [
    np.array([0, 0, 1, 0, 0, 0]),      # S1
    np.array([1, 0, 0, 0, 131.56, 0]), # S2
    np.array([0, 0, 1, 173.8, -131.56, 0]), # S3
    np.array([1, 0, 0, 0, 350, 0]),    # S4
    np.array([0, 0, 1, 350, -66.39, 0]), # S5
    np.array([1, 0, 0, 0, 436.6, 0])   # S6
]

# Home configuration matrix M
M = np.array([[1, 0, 0, 0.4366],
              [0, 1, 0, 0],
              [0, 0, 1, 0.280],
              [0, 0, 0, 1]])

# Generate 3-4 sets of random joint angles
random_joint_angles = [np.random.uniform(low=-np.pi, high=np.pi, size=6) for _ in range(4)]

# Convert spatial screw axes to body screw axes
B_list = compute_body_screw_axes(S_list, M)

# Test forward kinematics for each set of random joint angles
for i, thetas in enumerate(random_joint_angles):
    print(f"\nTest {i+1}:")
    print(f"Joint angles: {thetas}")

    # Compute forward kinematics in spatial frame
    T_spatial = forward_kinematics_spatial(S_list, M, thetas)
    
    # Compute forward kinematics in body frame
    T_body = forward_kinematics_body(B_list, M, thetas)

    # Print and check if both transformations are approximately equal
    print(f"Transformation in spatial frame:\n{T_spatial}")
    print(f"Transformation in body frame:\n{T_body}")
    print("Are both transformations approximately equal?", np.allclose(T_spatial, T_body))
