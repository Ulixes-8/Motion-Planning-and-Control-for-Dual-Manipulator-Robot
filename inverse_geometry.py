#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import pinocchio as pin 
import numpy as np
from numpy.linalg import pinv,inv,norm,svd,eig 
from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits
from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET


def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
    """
    Computes a grasp pose configuration for a robot to grip a cube at a specified target position.

    This function iteratively adjusts the robot's joint configuration to position its hands for 
    a stable grasp on the cube. The algorithm uses a gradient descent approach with forward 
    kinematics and Jacobian matrices to minimize the alignment error between the robot's 
    hand frames and the target cube frames.

    Args:
        robot: The robot object containing model and data structures required for kinematics and dynamics.
        qcurrent: Initial joint configuration (array-like) of the robot.
        cube: Object representing the cube to be grasped.
        cubetarget: Target frame for the cube's placement (position and orientation).
        viz (optional): Visualization object for rendering the robot's movements in a 3D environment.

    Returns:
        tuple: Final joint configuration (q) and a success flag indicating whether the grasp was successful.

    Example Usage:
        q, success = computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=visualizer)

    """
    
    # Place the cube at the target position
    setcubeplacement(robot, cube, cubetarget)
    
    # Get target placement for left and right hooks
    oMcubeL = getcubeplacement(cube, LEFT_HOOK)
    oMcubeR = getcubeplacement(cube, RIGHT_HOOK)
    
    # Initialize joint configuration from the current configuration
    q = qcurrent.copy()

    # Initialize success flag
    success = False
    max_iters = 1000  # Maximum number of iterations for convergence
    DT = 1e-2  # Step size for updating joint configuration

    for _ in range(max_iters):
        # Update robot's frame placements and joint Jacobians
        pin.framesForwardKinematics(robot.model, robot.data, q)
        pin.computeJointJacobians(robot.model, robot.data, q)

        # Get current placement of left and right hand frames
        oMhandL = robot.data.oMf[robot.model.getFrameId(LEFT_HAND)]
        oMhandR = robot.data.oMf[robot.model.getFrameId(RIGHT_HAND)]

        # Calculate error vectors for left and right hands
        error_L = pin.log(oMhandL.inverse() * oMcubeL).vector
        error_R = pin.log(oMhandR.inverse() * oMcubeR).vector

        # Check convergence criteria: if errors are below threshold and no collision, success
        if norm(error_L) < EPSILON and norm(error_R) < EPSILON and not collision(robot, q):
            success = True 
            break

        # Compute Jacobians for the left and right hands
        J_handL = pin.computeFrameJacobian(robot.model, robot.data, q, robot.model.getFrameId(LEFT_HAND))
        J_handR = pin.computeFrameJacobian(robot.model, robot.data, q, robot.model.getFrameId(RIGHT_HAND))
        
        # Stack error and Jacobian matrices
        error = np.hstack([error_L, error_R])
        J = np.vstack([J_handL, J_handR])

        # Compute joint update using the pseudoinverse of the Jacobian
        vq = np.linalg.pinv(J) @ error
        
        # Integrate joint updates into the configuration
        q = pin.integrate(robot.model, q, (vq) * DT)
        
        # Enforce joint limits to ensure valid configuration
        q = projecttojointlimits(robot, q)

        # Update visualization if enabled
        if viz is not None:
            from setup_meshcat import updatevisuals
            updatevisuals(viz, robot, cube, q)
    
    # Check for collision in the final configuration
    if collision(robot, q):
        success = False

    return q, success



# This is the original main method. 
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    print("q0: ", q0)
    print("successinit: ", successinit)
    print("qe: ", qe)
    print("successend: ", successend)
    
    updatevisuals(viz, robot, cube, q0)
    