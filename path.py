#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import time
from tools import setcubeplacement, distanceToObstacle
import pinocchio as pin
import numpy as np
from inverse_geometry import computeqgrasppose
from scipy.spatial import KDTree


def displaypath(robot,path,dt,viz):
    for q in path:
        viz.display(q)
        time.sleep(dt)

import numpy as np
import pinocchio as pin


## Uniform sampler. 
def sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None):
    """
    Samples a feasible placement for the cube within specified bounds based on
    the initial and goal placements, with a fixed range for the z-axis.
    """
    min_obstacle_distance = 0.04  # Minimum allowable distance from obstacles

    # Define bounds based on cube placement and target
    x_min, x_max = min(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]), max(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0])
    y_min, y_max = min(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]), max(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1])
    z_min, z_max = 1.05, 1.4  # Fixed bounds for z-axis

    while True:
        # Sample x, y, z within specified bounds
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = np.random.uniform(z_min, z_max)
        sampled_point = np.array([x, y, z])

        # Define the cube placement as an SE3 transform with no rotation
        placement = pin.SE3(pin.SE3.Identity().rotation, sampled_point)
        setcubeplacement(robot, cube, placement)

        # Check for collisions
        pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
        if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
            print("Cube placement in collision, resampling...")
            continue  # Resample if in collision

        # Attempt to compute a robot configuration to grasp the cube at this placement
        q, success = computeqgrasppose(robot, robot.q0.copy(), cube, placement, viz)
        
        if success:
            # Check distance to obstacle for the current configuration
            dist_to_obstacle = distanceToObstacle(robot, q)
            if dist_to_obstacle >= min_obstacle_distance:
                print("Sampled placement and configuration are valid under all constraints.")
                print("Placement:", placement)
                return q, placement
            else:
                print("Sampled placement too close to obstacle, resampling...")

## Gaussian Sampler 
# def sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None):
#     """
#     Samples a feasible placement for the cube using Gaussian sampling centered 
#     above the fixed obstacle placement with dynamically defined bounds based on
#     the initial and goal placements.
#     """
#     min_obstacle_distance = 0.04  # Minimum allowable distance from obstacles

#     # Fixed obstacle location
#     obstacle_position = np.array([0.43, -0.1, 0.94])
#     gaussian_center = obstacle_position + np.array([0.0, 0.0, 0.3])  # Center above the obstacle
#     # Experiment with the x and y centers... 
#     # gaussian_center = obstacle_position + np.array([0.0, 0.0, 0.3])  

#     # Define dynamic bounds based on current cube placement and target, plus margin
#     margin = 0.12 # Test further 
#     x_min, x_max = min(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) - margin, max(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) + margin
#     y_min, y_max = min(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) - margin, max(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) + margin
#     z_min, z_max = min(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) - margin, max(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) + margin

#     # Standard deviations for Gaussian sampling (adjustable based on workspace and clearance needs)

#     std_dev_x = .3  # Spread in x-direction around obstacle
#     std_dev_y = .3 # Spread in y-direction around obstacle
#     std_dev_z = .3  # Smaller spread in z to focus samples slightly above obstacle
#     while True:
#         # Sample x, y, z from a Gaussian distribution centered above the obstacle
#         x = np.clip(np.random.normal(gaussian_center[0], std_dev_x), x_min, x_max)
#         y = np.clip(np.random.normal(gaussian_center[1], std_dev_y), y_min, y_max)
#         z = np.clip(np.random.normal(gaussian_center[2], std_dev_z), z_min, z_max)
#         sampled_point = np.array([x, y, z])

#         # Define the cube placement as an SE3 transform with no rotation
#         placement = pin.SE3(pin.SE3.Identity().rotation, sampled_point)
#         setcubeplacement(robot, cube, placement)

#         # Check for collisions
#         pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
#         if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
#             print("Cube placement in collision, resampling...")
#             continue  # Resample if in collision

#         # Attempt to compute a robot configuration to grasp the cube at this placement
#         q, success = computeqgrasppose(robot, robot.q0.copy(), cube, placement, viz)
        
#         if success:
#             # Check distance to obstacle for the current configuration
#             dist_to_obstacle = distanceToObstacle(robot, q)
#             if dist_to_obstacle >= min_obstacle_distance:
#                 print("Sampled placement and configuration are valid under all constraints.")
#                 print("Placement:", placement)
#                 return q, placement
#             else:
#                 print("Sampled placement too close to obstacle, resampling...")

def project_path(robot, cube, q_curr, cube_curr, cube_rand, step_size=0.025, viz=None):
    # step_size = 0.1 # Test further

    # Calculate the number of interpolation steps based on step_size
    distance = np.linalg.norm(cube_curr.translation - cube_rand.translation)
    num_steps = int(distance / step_size) + 1
    
    # Initialize paths with the starting configurations
    robot_path = [q_curr]
    cube_path = [cube_curr]
    
    # Linear interpolation between cube placements
    for step in range(1, num_steps + 1):
        alpha = step / num_steps
        
        # Interpolate the cube's SE3 placement
        interpolated_cube_pos = pin.SE3.Interpolate(cube_curr, cube_rand, alpha)
        
        # Set the cube at the interpolated position and check for collisions
        setcubeplacement(robot, cube, interpolated_cube_pos)
        pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
        if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
            # Cube placement is invalid; return the valid portion of the path
            # print("Cube collision detected in interpolated path. Returning valid portion.")
            return robot_path, cube_path
        
        # Use computeqgrasppose to find a valid robot configuration for this cube placement
        q, success = computeqgrasppose(robot, robot_path[-1].copy(), cube, interpolated_cube_pos, viz)
        if not success:
            # Robot configuration is invalid; return the valid portion of the path
            # print("Failed to compute valid robot configuration at interpolated position. Returning valid portion.")
            return robot_path, cube_path
        
        # Append the valid configurations to the paths
        robot_path.append(q)
        cube_path.append(interpolated_cube_pos)
    
    # If we reach here, the full path is valid
    return robot_path, cube_path

def distance(q1, q2):
    '''Compute Euclidean distance between two configurations.'''
    return np.linalg.norm(q2 - q1)

def add_edge_and_vertex(G_robot, G_cube, robot_points, parent_index, q, cube_placement, tree_needs_update):
    '''Add a new node to the robot and cube graphs without immediately updating the KDTree.'''
    G_robot.append((parent_index, q))
    G_cube.append((parent_index, cube_placement))
    robot_points.append(q)  # Add to points list
    tree_needs_update[0] = True  # Flag that KDTree needs update

def get_path(G):
    '''Reconstruct the path from the goal back to the initial configuration.'''
    path = []
    node = G[-1]
    while node[0] is not None:
        path.insert(0, node[1])  # Insert at the beginning to reverse the order
        node = G[node[0]]
    path.insert(0, G[0][1])  # Add the initial configuration
    return path

def lazy_kd_tree_query(robot_points, kd_tree, target, tree_needs_update):
    '''Only rebuild and query KDTree if new points were added.'''
    if tree_needs_update[0]:  # Check if rebuild is needed
        kd_tree = KDTree(robot_points)  # Rebuild KDTree
        tree_needs_update[0] = False  # Reset the flag after rebuilding
    distance, nearest_index = kd_tree.query(target)
    return distance, nearest_index, kd_tree

def computepath(qinit, qgoal, cubeplacementq0, cubeplacementqgoal, robot=None, cube=None):
    if robot is None or cube is None:
        print("Error: Robot and cube objects must be provided.")
        return []
    
    max_iterations = 250
    max_retries = 10
    step_size = 0.025
    goal_tolerance = 0.5
    if __name__ == "__main__":
        step_size = 0.1
    else:
        step_size = 0.025

    for retry in range(max_retries):
        G_start = [(None, qinit)]
        G_goal = [(None, qgoal)]
        G_cube_start = [(None, cubeplacementq0)]
        G_cube_goal = [(None, cubeplacementqgoal)]

        robot_points_start = [qinit]
        robot_points_goal = [qgoal]

        kd_tree_robot_start = KDTree([qinit])
        kd_tree_robot_goal = KDTree([qgoal])

        tree_needs_update_start = [False]
        tree_needs_update_goal = [False]

        for i in range(max_iterations):
            if np.random.rand() < 0.1:
                q_rand = qgoal
                cube_rand = cubeplacementqgoal
                print("Goal biasing applied.")
            else:
                q_rand, cube_rand = sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None)

            _, nearest_index_start, kd_tree_robot_start = lazy_kd_tree_query(
                robot_points_start, kd_tree_robot_start, q_rand, tree_needs_update_start
            )
            q_near_start = G_start[nearest_index_start][1]
            cube_near_start = G_cube_start[nearest_index_start][1]

            robot_path_segment, cube_path_segment = project_path(
                robot, cube, q_near_start, cube_near_start, cube_rand, step_size=step_size, viz=None
            )

            for j in range(len(robot_path_segment)):
                parent_index = len(G_start) - 1 if j > 0 else nearest_index_start
                add_edge_and_vertex(G_start, G_cube_start, robot_points_start, parent_index, 
                                    robot_path_segment[j], cube_path_segment[j], tree_needs_update_start)

            q_new_start = robot_path_segment[-1]
            _, nearest_index_goal, kd_tree_robot_goal = lazy_kd_tree_query(
                robot_points_goal, kd_tree_robot_goal, q_new_start, tree_needs_update_goal
            )
            q_near_goal = G_goal[nearest_index_goal][1]
            cube_near_goal = G_cube_goal[nearest_index_goal][1]

            robot_path_segment_goal, cube_path_segment_goal = project_path(
                robot, cube, q_near_goal, cube_near_goal, cubeplacementqgoal, step_size=step_size, viz=None
            )

            for j in range(len(robot_path_segment_goal)):
                parent_index = len(G_goal) - 1 if j > 0 else nearest_index_goal
                add_edge_and_vertex(G_goal, G_cube_goal, robot_points_goal, parent_index, 
                                    robot_path_segment_goal[j], cube_path_segment_goal[j], tree_needs_update_goal)

            if distance(q_new_start, robot_path_segment_goal[-1]) < goal_tolerance:
                print("Trees connected! Path found.")
                path_start_robot = get_path(G_start)
                path_goal_robot = get_path(G_goal)
                final_robot_path = path_start_robot + path_goal_robot[::-1]
                return final_robot_path

            if i % 10 == 0:
                _, nearest_goal_index, kd_tree_robot_start = lazy_kd_tree_query(
                    robot_points_start, kd_tree_robot_start, qgoal, tree_needs_update_start
                )
                print(f"Iteration {i + (retry * max_iterations)}: Nearest vertex to goal: {distance(G_start[nearest_goal_index][1], qgoal)}. Will succeed when this value is less than {goal_tolerance}.")

        print(f"Retry {retry + 1}/{max_retries}: Max iterations reached without connecting the trees. Restarting...")

    print("Failed to find a path after maximum retries. This is rare, so try again as is and DO NOT change ANY hyperparameters.")
    return []


#Original Main Method. 
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    
    robot, cube, viz = setupwithmeshcat()
    
    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz=None)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz=None)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, robot=robot, cube=cube)
    # Please make sure that robot and cube are passed to compute path. 
    # Note: we were allowed to make this API change according to Prof. Tonneau.
    
    if viz is not None:
        displaypath(robot, path, dt=0.1, viz=viz)  # pass the actual viz object if initialized
    else:
        print("Visualization is not enabled.")
