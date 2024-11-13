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
    setcubeplacement(robot, cube, cubetarget)
    
    oMcubeL = getcubeplacement(cube, LEFT_HOOK)
    oMcubeR = getcubeplacement(cube, RIGHT_HOOK)
    
    q = qcurrent.copy()

    success = False
    max_iters = 1000  # Reduced max iterations
    DT = 1e-2  # Reduced step size for finer updates

    for _ in range(max_iters):
        pin.framesForwardKinematics(robot.model, robot.data, q)
        pin.computeJointJacobians(robot.model, robot.data, q)

        oMhandL = robot.data.oMf[robot.model.getFrameId(LEFT_HAND)]
        oMhandR = robot.data.oMf[robot.model.getFrameId(RIGHT_HAND)]

        error_L = pin.log(oMhandL.inverse() * oMcubeL).vector
        error_R = pin.log(oMhandR.inverse() * oMcubeR).vector

        if norm(error_L) < EPSILON and norm(error_R) < EPSILON and not collision(robot, q):
            success = True 
            # print("Successfully found a valid grasp pose.")
            break

        J_handL = pin.computeFrameJacobian(robot.model, robot.data, q, robot.model.getFrameId(LEFT_HAND))
        J_handR = pin.computeFrameJacobian(robot.model, robot.data, q, robot.model.getFrameId(RIGHT_HAND))
        
        error = np.hstack([error_L, error_R])
        J = np.vstack([J_handL, J_handR])

        # Compute the update using the pseudoinverse of J
        vq = np.linalg.pinv(J) @ error
        
        q = pin.integrate(robot.model, q, (vq) * DT)
        
        # Enforce joint limits
        q = projecttojointlimits(robot, q)

        if viz is not None:
            from setup_meshcat import updatevisuals
            updatevisuals(viz, robot, cube, q)
    
    if collision(robot, q):
        # print("Collision detected!")
        success = False

    # L_p_R = oMhandL.translation - oMhandR.translation
    # print("Distance between hands: ", np.linalg.norm(L_p_R))
    return q, success

# import matplotlib.pyplot as plt
# import pinocchio as pin
# import numpy as np
# from numpy.linalg import norm
# from tools import collision, getcubeplacement, setcubeplacement, projecttojointlimits, distanceToObstacle
# from config import LEFT_HOOK, RIGHT_HOOK, LEFT_HAND, RIGHT_HAND, EPSILON

# def computeqgrasppose(robot, qcurrent, cube, cubetarget, viz=None):
#     # Set the cube to the target placement
#     setcubeplacement(robot, cube, cubetarget)
    
#     # Retrieve placements for LEFT_HOOK and RIGHT_HOOK on the cube
#     oMcubeL = getcubeplacement(cube, LEFT_HOOK)
#     oMcubeR = getcubeplacement(cube, RIGHT_HOOK)
    
#     # Initialize robot configuration
#     q = qcurrent.copy()
#     success = False
#     max_iters = 30000  # Maximum iterations
#     DT = 1e-2  # Step size for integration
#     collision_threshold = 0.1  # Minimum acceptable distance to obstacles
#     collision_avoidance_gain = 0.02  # Scaling factor for minimal collision avoidance adjustments

#     # Store combined errors for plotting
#     combined_errors = []

#     for _ in range(max_iters):
#         # Forward kinematics and Jacobians
#         pin.framesForwardKinematics(robot.model, robot.data, q)
#         pin.computeJointJacobians(robot.model, robot.data, q)

#         # Get current placements of the hands
#         oMhandL = robot.data.oMf[robot.model.getFrameId(LEFT_HAND)]
#         oMhandR = robot.data.oMf[robot.model.getFrameId(RIGHT_HAND)]

#         # Calculate error in aligning hands with the hooks
#         error_L = pin.log(oMhandL.inverse() * oMcubeL).vector
#         error_R = pin.log(oMhandR.inverse() * oMcubeR).vector

#         # Combine the errors and store for plotting
#         total_error = np.hstack([error_L, error_R])
#         combined_errors.append(total_error)

#         # Check if the primary task is achieved and there are no collisions
#         if norm(error_L) < EPSILON and norm(error_R) < EPSILON and not collision(robot, q):
#             success = True
#             print("Primary objective achieved without collision.")
#             break

#         # Compute Jacobians for the hands
#         J_handL = pin.computeFrameJacobian(robot.model, robot.data, q, robot.model.getFrameId(LEFT_HAND))
#         J_handR = pin.computeFrameJacobian(robot.model, robot.data, q, robot.model.getFrameId(RIGHT_HAND))
        
#         # Stack errors and Jacobians for the primary task
#         error = np.hstack([error_L, error_R])
#         J = np.vstack([J_handL, J_handR])

#         # Compute primary task velocity using the pseudoinverse of J
#         vq_primary = np.linalg.pinv(J) @ error

#         # Null space projector for the primary task
#         P_primary = np.eye(robot.model.nv) - np.linalg.pinv(J) @ J

#         # Secondary task: Apply small adjustments if too close to an obstacle
#         distance_to_obstacle = distanceToObstacle(robot, q)
#         if distance_to_obstacle < collision_threshold:
#             # Apply a small random adjustment in the null space to avoid obstacles
#             random_adjustment = np.random.randn(robot.model.nv)
#             vq_secondary = P_primary @ random_adjustment * collision_avoidance_gain
#         else:
#             vq_secondary = np.zeros(robot.model.nv)

#         # Combine primary and secondary velocities
#         vq = vq_primary + vq_secondary

#         # Integrate and apply joint limits
#         q = pin.integrate(robot.model, q, vq * DT)
#         q = projecttojointlimits(robot, q)

#         # Update visualization if provided
#         if viz is not None:
#             from setup_meshcat import updatevisuals
#             updatevisuals(viz, robot, cube, q)

#     # Final collision check
#     if collision(robot, q):
#         print("Collision detected in final configuration.")
#         success = False

#     # Plot the combined error data
#     plot_combined_error_data(combined_errors)

#     return q, success


# def plot_combined_error_data(combined_errors):
#     """Plot the combined error data over time, with six clearly labeled lines for each chart."""
#     combined_errors = np.array(combined_errors)

#     plt.figure(figsize=(10, 8))

#     # Plot position errors for X, Y, Z (both left and right hands)
#     plt.subplot(211)
#     plt.plot(combined_errors[:, 0], label='Left X Error (m)')
#     plt.plot(combined_errors[:, 1], label='Left Y Error (m)')
#     plt.plot(combined_errors[:, 2], label='Left Z Error (m)')
#     plt.plot(combined_errors[:, 6], label='Right X Error (m)', linestyle='--')
#     plt.plot(combined_errors[:, 7], label='Right Y Error (m)', linestyle='--')
#     plt.plot(combined_errors[:, 8], label='Right Z Error (m)', linestyle='--')
#     plt.xlabel('Iteration')
#     plt.ylabel('Position Error')
#     plt.legend(loc='upper right')

#     # Plot orientation errors for Roll, Pitch, Yaw (both left and right hands)
#     plt.subplot(212)
#     plt.plot(combined_errors[:, 3], label='Left Roll Error (rad)')
#     plt.plot(combined_errors[:, 4], label='Left Pitch Error (rad)')
#     plt.plot(combined_errors[:, 5], label='Left Yaw Error (rad)')
#     plt.plot(combined_errors[:, 9], label='Right Roll Error (rad)', linestyle='--')
#     plt.plot(combined_errors[:, 10], label='Right Pitch Error (rad)', linestyle='--')
#     plt.plot(combined_errors[:, 11], label='Right Yaw Error (rad)', linestyle='--')
#     plt.xlabel('Iteration')
#     plt.ylabel('Orientation Error')
#     plt.legend(loc='upper right')

#     plt.tight_layout()
#     plt.show()






      
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    # q = robot.q0.copy() # The initial configuration of the robot. 
    
    # q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz) # Grasp the cube at the initial placement.
    # print("q0: ", q0)
    # print("successinit: ", successinit)
    # qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz) # Grasp the cube at the target placement.
    # print(f"qe: ", qe, "successend: ", successend)
    # updatevisuals(viz, robot, cube, q0) # You should see the robot at the initial configuration is correct. 
    # updatevisuals(viz, robot, cube, qe) # You should see the robot at the initial configuration is correct. 
    x_min, x_max = min(CUBE_PLACEMENT.translation[0], CUBE_PLACEMENT_TARGET.translation[0]) - 0.3, max(CUBE_PLACEMENT.translation[0], CUBE_PLACEMENT_TARGET.translation[0]) + 0.3
    y_min, y_max = min(CUBE_PLACEMENT.translation[1], CUBE_PLACEMENT_TARGET.translation[1]) - 0.3, max(CUBE_PLACEMENT.translation[1], CUBE_PLACEMENT_TARGET.translation[1]) + 0.3
    z_min, z_max = min(CUBE_PLACEMENT.translation[2], CUBE_PLACEMENT_TARGET.translation[2]) - 0.3, max(CUBE_PLACEMENT.translation[2], CUBE_PLACEMENT_TARGET.translation[2]) + 0.3

    while True:
        # Randomly sample x, y, z within bounds
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = np.random.uniform(z_min, z_max)
        
        # Define the cube placement as an SE3 transform with no rotation
        placement = pin.SE3(pin.SE3.Identity().rotation, np.array([x, y, z]))

        # Use setcubeplacement to update the cube's placement
        setcubeplacement(robot, cube, placement)
        
        # Check if the cube placement is in collision
        pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
        if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
            # Skip this placement if in collision and resample
            continue
        
        # Attempt to compute a robot configuration to grasp the cube at this placement
        print(f"Testing placement at {placement.translation}...")
        q, success = computeqgrasppose(robot, robot.q0.copy(), cube, placement, viz)
    

# from setup_pinocchio import loadobject
# from config import NEXTAGE_URDF, MESH_DIR, NEXTAGE_SRDF, ROBOT_PLACEMENT, TABLE_URDF, TABLE_MESH, TABLE_PLACEMENT, OBSTACLE_URDF, OBSTACLE_MESH, OBSTACLE_PLACEMENT, CUBE_URDF    ,CUBE_MESH    , CUBE_PLACEMENT

# if __name__ == "__main__":
#     from tools import setupwithmeshcat
#     from setup_pinocchio import addcollisiontorobot, finalisecollisionsetup
#     from setup_meshcat import updatevisuals
#     import pinocchio as pin
#     from numpy import array
#     from pinocchio.utils import rotate

#     # Set up robot and visualization
#     robot, cube, viz = setupwithmeshcat()

    # # Define parameters for the original obstacle
    # box_size = [0.25, 0.25, 0.005]  # Width, height, and depth
    # color = [1, 1, 0.78, 0.8]     # Color with transparency

    # # Original obstacle placement using OBSTACLE_PLACEMENT
    # OBSTACLE_PLACEMENT = pin.SE3(rotate('z', 0), array([0.43, -0.1, 0.94]))


    # # Define the z-offset to position the duplicate box above the original
    # z_offset = 0.2  # Distance above the original obstacle

    # # Position the duplicate obstacle directly above the original
    # duplicate_box_id = 'world/obstacle_above'
    # viz.addBox(duplicate_box_id, box_size, color)
    # viz.applyConfiguration(duplicate_box_id, [0.43, -0.1, 0.94 + z_offset, 0, 0, 0, 1])  # Place above the original

    # # Add the duplicate obstacle to the robot's collision model
    # duplicate_placement = OBSTACLE_PLACEMENT.copy()
    # duplicate_placement.translation[2] += z_offset
    # addcollisiontorobot(robot, loadobject(robot, OBSTACLE_URDF, OBSTACLE_MESH, duplicate_placement))

    # # Finalize the collision setup to incorporate both obstacles
    # finalisecollisionsetup(robot)

    # # Render the updated scene and display it in Meshcat
    # viz.display()  # Refresh display to show both obstacles

    # Run the placement tests
    # q = robot.q0.copy()  # Initial configuration of the robot

    # placements = [
    #     ("Original Placement", pin.SE3(rotate('z', 0.0), array([0.33, -0.3, 0.93]))),
    #     ("Offset Right", pin.SE3(rotate('z', 0.0), array([0.43, -0.3, 0.93]))),
    #     ("Offset Left", pin.SE3(rotate('z', 0.0), array([0.23, -0.3, 0.93]))),
    #     ("Raised Upwards", pin.SE3(rotate('z', 0.0), array([0.33, -0.3, 1.1]))),
    #     ("45° Rotated Upwards", pin.SE3(rotate('z', np.pi/4), array([0.33, -0.25, 1.05]))),
    # ]

    # # vector from tool to goal, in world frame
    # for name, placement in placements:
    #     print(f"Testing placement: {name}")
    #     setcubeplacement(robot, cube, placement)
        
    #     q0, success_init = computeqgrasppose(robot, q, cube, placement, viz)
    #     print(f"{name} - Initial configuration success: {success_init}")
    #     print("q0:", q0)
        
    #     if success_init:
    #         updatevisuals(viz, robot, cube, q0)
    #     else:
    #         print(f"{name} - Initial configuration failed.")

    #     input(f"Press Enter to continue to the next placement...")



# This is the original main method. 
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from setup_meshcat import updatevisuals
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    updatevisuals(viz, robot, cube, q0)
    