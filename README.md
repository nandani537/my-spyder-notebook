## **Forward Kinematics Questions with Python Code for questions number 5,6 and 7

Write python functions that computes the forward kinematics using product of exponential f5.	Write python functions that computes the forward kinematics using product of exponential formula in spatial and body frame.

Verify that the product of exponential formula in spatial and body frame provides the same results. Generate some (3-4) random joint angles and verify that both functions generate same output.

Verify using intuition that the forward kinematics formulas provide expected result. Test functions with 4 different sets of joint angles for which you can visualize what should be the solution and verify that the forward kinematics formulas provide the expected result.
    
## **Results
- 1.Test 1 (Random Angles: [-0.788, 2.832, 1.458]):
The forward kinematics matrices calculated using both the spatial and body frames show slight differences, especially in the position components.
Possible Reason: This discrepancy might be due to numerical precision errors in the computations, which is common when dealing with matrix exponentiation and successive multiplications in robotic kinematics.
Despite the differences, the rotational components of the matrices are similar that the orientation of the end-effector is consistent across both methods.

- 2. Test 2 (Random Angles: [0.310, -1.081, -1.081]):
Similar to Test 1, there are differences in the position components of the matrices computed using the spatial and body frames.
The rotational parts of the matrices, however, match closely, indicating consistency in orientation.

- 3. Test 3 (Joint Angles: [0, 0, 0]):
This test represents the home configuration of the robot.
Observation: Both the spatial and body frame forward kinematics correctly return the home configuration matrix M.
The difference between the matrices is zero.
Explanation: Since this is the home configuration, no joint rotations have been applied. This consistency confirms that the implementation of both FK methods correctly handles the base case where the robot is at its default pose.
- 4. Test 4 (Intuitive Angles: [π/2, -π/4, π/6]):
The rotational components of the resulting matrices are identical, indicating that the end-effector's orientation is the same in both the spatial and body frame calculations.
There are minor differences in the positional components of the matrices, which are again likely due to numerical precision errors.
Explanation: The end-effector's position depends on the order and nature of the joint rotations and translations. Small discrepancies can arise when performing multiple matrix operations, especially in systems with floating-point arithmetic.
