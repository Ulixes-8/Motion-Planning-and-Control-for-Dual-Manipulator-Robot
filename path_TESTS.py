#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from numpy.linalg import pinv

from config import LEFT_HAND, RIGHT_HAND
import time


#This was the original code 
#returns a collision free path from qinit to qgoal under grasping constraints
#the path is expressed as a list of configurations
# def computepath(qinit,qgoal,cubeplacementq0, cubeplacementqgoal):
#     #TODO
#     return [qinit, qgoal]
#     pass


# def displaypath(robot,path,dt,viz):
#     for q in path:
#         viz.display(q)
#         time.sleep(dt)


def displaypath(robot, robot_path, cube_path, dt, viz):
    '''
    Display the path for both the robot and the cube in the visualizer.
    
    Parameters:
        robot: The robot object with its kinematic model.
        robot_path: List of robot configurations to display.
        cube_path: List of cube placements corresponding to the robot_path.
        dt: Time delay between displaying each configuration.
        viz: The visualization object.
    '''
    for q, cube_pos in zip(robot_path, cube_path):
        # Update the robot configuration in the visualizer
        viz.display(q)
        
        # Update the cube's position in the visualizer
        setcubeplacement(robot, cube, cube_pos)
        pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
        
        #Print distance between hands
        oMhandL = robot.data.oMf[robot.model.getFrameId(LEFT_HAND)]
        oMhandR = robot.data.oMf[robot.model.getFrameId(RIGHT_HAND)]
        L_p_R = oMhandL.translation - oMhandR.translation
        print("Distance between hands: ", np.linalg.norm(L_p_R))
        
        # Pause briefly to visualize each step
        time.sleep(dt)


import pinocchio as pin
import numpy as np
from inverse_geometry import computeqgrasppose
from tools import collision

import pinocchio as pin
import numpy as np
from inverse_geometry import computeqgrasppose
from tools import collision, setcubeplacement

# def sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None):
#     '''
#     Sample a random cube placement within the robot's reachable area with no rotation,
#     and solve for a valid robot configuration that can grasp the cube at this placement.
    
#     Parameters:
#         robot: The robot object with its kinematic model.
#         cube: The cube object representing the object to be grasped.
#         cubeplacementq0: Initial placement of the cube as a reference point.
#         cubeplacementqgoal: Goal placement of the cube as a reference point.
#         viz: Optional visualization object for displaying sampled configurations.
        
#     Returns:
#         A tuple (q, placement) where:
#             - q is the robot configuration to grasp the cube at `placement`
#             - placement is the SE3 position of the cube.
#         Returns (None, None) if no valid placement is found.
#     '''
    
#     # Define workspace bounds based on initial and goal placements, plus a margin
#     x_min, x_max = min(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) - 0.3, max(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) + 0.3
#     y_min, y_max = min(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) - 0.3, max(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) + 0.3
#     z_min, z_max = min(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) - 0.3, max(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) + 0.3
#     # x_min, x_max = min(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]), max(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) + 0.5
#     # y_min, y_max = min(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]), max(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) + 0.5
#     # z_min, z_max = min(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]), max(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) + 0.5

#     while True:
#         # Randomly sample x, y, z within bounds
#         x = np.random.uniform(x_min, x_max)
#         y = np.random.uniform(y_min, y_max)
#         z = np.random.uniform(z_min, z_max)
        
#         # Define the cube placement as an SE3 transform with no rotation
#         placement = pin.SE3(pin.SE3.Identity().rotation, np.array([x, y, z]))

#         # Use setcubeplacement to update the cube's placement
#         setcubeplacement(robot, cube, placement)
        
#         # Check if the cube placement is in collision
#         pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
#         if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
#             # Skip this placement if in collision and resample
#             continue
        
#         # Attempt to compute a robot configuration to grasp the cube at this placement
#         q, success = computeqgrasppose(robot, robot.q0.copy(), cube, placement, viz)
        
#         # If a valid configuration is found, return it along with the cube placement
#         if success:
#             return q, placement
        
#         # Otherwise, resample if the configuration is invalid

from tools import distanceToObstacle

def sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None):

    # Define workspace bounds based on initial and goal placements, plus a margin
    x_min, x_max = min(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) - 0.3, max(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) + 0.3
    y_min, y_max = min(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) - 0.3, max(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) + 0.3
    z_min, z_max = min(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) - 0.3, max(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) + 0.3

    min_obstacle_distance = 0.035 # Define minimum allowable distance from obstacle

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
        q, success = computeqgrasppose(robot, robot.q0.copy(), cube, placement, viz)
        
        # Check distance to obstacle for the current configuration
        if success:
            dist_to_obstacle = distanceToObstacle(robot, q)
            print(f"Distance to obstacle for sampled configuration: {dist_to_obstacle}")

            # Verify if the configuration meets the minimum distance requirement
            if dist_to_obstacle >= min_obstacle_distance:
                return q, placement
            else:
                print("Sampled placement too close to obstacle, resampling...")
        
        # Otherwise, resample if configuration is invalid or too close to the obstacle



#This was the original code
# if __name__ == "__main__":
#     from tools import setupwithmeshcat
#     from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
#     from inverse_geometry import computeqgrasppose
    
#     robot, cube, viz = setupwithmeshcat()
    
    
#     q = robot.q0.copy()
#     q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
#     # qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
#     for _ in range (100):
#         sample_cube_placement(robot, cube, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, viz)
    
#     # if not(successinit and successend):
#     #     print ("error: invalid initial or end configuration")
    
#     # path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
    
#     # displaypath(robot,path,dt=0.5,viz=None) #you ll probably want to lower dt
    


import numpy as np
import pinocchio as pin
from inverse_geometry import computeqgrasppose

import numpy as np
import pinocchio as pin
from inverse_geometry import computeqgrasppose

import numpy as np
import pinocchio as pin
from inverse_geometry import computeqgrasppose

def project_path(robot, cube, q_curr, q_rand, cube_curr, cube_rand, step_size=0.01, viz=None):
    '''
    Project a path from q_curr to q_rand and cube_curr to cube_rand such that every configuration
    in the path maintains the grasp constraint and remains collision-free. Returns the furthest
    valid configuration along the path.
    
    Parameters:
        robot: The robot object with its kinematic model.
        cube: The cube object representing the object to be grasped.
        q_curr: Initial robot configuration.
        q_rand: Target robot configuration.
        cube_curr: Initial cube placement.
        cube_rand: Target cube placement.
        step_size: Discretization step for interpolation between configurations.
        viz: Optional visualization object.
    
    Returns:
        A tuple (robot_path, cube_path, success) where:
            - robot_path is a list of valid robot configurations up to the furthest valid point.
            - cube_path is a list of corresponding cube placements up to the furthest valid point.
            - success is a boolean indicating if the full path was successfully projected.
    '''
    
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
            return robot_path, cube_path, False
        
        # Use computeqgrasppose to find a valid robot configuration for this cube placement
        q, success = computeqgrasppose(robot, robot_path[-1].copy(), cube, interpolated_cube_pos, viz)
        if not success:
            # Robot configuration is invalid; return the valid portion of the path
            # print("Failed to compute valid robot configuration at interpolated position. Returning valid portion.")
            return robot_path, cube_path, False
        
        # Append the valid configurations to the paths
        robot_path.append(q)
        cube_path.append(interpolated_cube_pos)
    
    # If we reach here, the full path is valid
    return robot_path, cube_path, True




# #Use this main method to show that sample function works correctly..
# if __name__ == "__main__":
#     from tools import setupwithmeshcat
#     from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
#     from inverse_geometry import computeqgrasppose
    
#     # Set up the robot, cube, and visualization
#     robot, cube, viz = setupwithmeshcat()
    
#     # Get the initial configuration q0 and the goal configuration qe
#     q = robot.q0.copy()
#     q0, success_init = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz=None)
#     qe, success_end = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET, viz=None)

#     if not (success_init and success_end):
#         print("Error: Invalid initial or end configuration.")
#     else:
#         print("Initial and goal configurations successfully computed.")
    
#     for _ in range(100):
#         # Sample a random valid cube placement and corresponding robot configuration
#         q_rand, cube_rand = sample_cube_placement(robot, cube, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, viz=None)
        
#         # Project a path from the initial configuration (q0, CUBE_PLACEMENT) to the sampled configuration
#         robot_path, cube_path, success = project_path(robot, cube, q0, q_rand, CUBE_PLACEMENT, cube_rand, step_size=0.05, viz=None)
        
#         # Display the path if it was successfully projected
#         if success:
#             print("Displaying successfully projected path.")
#             displaypath(robot, robot_path, cube_path, dt=0.1, viz=None)  # Lowered dt for smoother visualization
#         else:
#             print("Partial path projected; displaying the longest valid portion.")
#             displaypath(robot, robot_path, cube_path, dt=0.1, viz=None)



def distance(q1, q2):
    '''Compute Euclidean distance between two configurations.'''
    return np.linalg.norm(q2 - q1)

def nearest_vertex(G, q_rand):
    '''Return the index of the Node in G with the configuration closest to q_rand.'''
    min_dist = float('inf')
    nearest_index = -1
    for i, (_, q) in enumerate(G):
        dist = distance(q, q_rand)
        if dist < min_dist:
            min_dist = dist
            nearest_index = i
    return nearest_index

def add_edge_and_vertex(G_robot, G_cube, parent_index, q, cube_placement):
    '''Add a new node to both the robot and cube graphs.'''
    G_robot.append((parent_index, q))
    G_cube.append((parent_index, cube_placement))

def get_path(G):
    '''Reconstruct the path from the goal back to the initial configuration.'''
    path = []
    node = G[-1]
    while node[0] is not None:
        path.insert(0, node[1])  # Insert at the beginning to reverse the order
        node = G[node[0]]
    path.insert(0, G[0][1])  # Add the initial configuration
    return path

import numpy as np

# def computepath(robot, cube, qinit, qgoal, cubeplacementq0, cubeplacementqgoal, max_iterations=2000, step_size=0.1, goal_tolerance=0.5, viz=None):
#     '''
#     RRT-based planner to compute a collision-free path from qinit to qgoal under grasping constraints with goal biasing.
    
#     Parameters:
#         robot: The robot object with its kinematic model.
#         cube: The cube object representing the object to be grasped.
#         qinit: Initial configuration of the robot.
#         qgoal: Goal configuration of the robot.
#         cubeplacementq0: Initial cube placement.
#         cubeplacementqgoal: Target cube placement.
#         max_iterations: Maximum number of RRT iterations.
#         step_size: Step size for path projection.
#         goal_tolerance: Distance tolerance for reaching the goal.
#         viz: Optional visualization object.
    
#     Returns:
#         robot_path: List of robot configurations from qinit to qgoal.
#         cube_path: Corresponding list of cube placements.
#     '''
    
#     # Initialize the RRT graphs with the starting configuration and cube placement
#     G_robot = [(None, qinit)]
#     G_cube = [(None, cubeplacementq0)]
    
#     for i in range(max_iterations):
#         # Step 1: Sample a random configuration and cube placement with 10% goal bias
#         if np.random.rand() < 0.1:
#             q_rand = qgoal
#             cube_rand = cubeplacementqgoal
#             print("Goal biasing applied.")
#         else:
#             q_rand, cube_rand = sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None)
        
#         # Step 2: Find the nearest vertex in both graphs
#         nearest_index = nearest_vertex(G_robot, q_rand)
#         q_near = G_robot[nearest_index][1]
#         cube_near = G_cube[nearest_index][1]
        
#         # Step 3: Project a path from q_near and cube_near to q_rand and cube_rand under grasping constraints
#         robot_path_segment, cube_path_segment, _ = project_path(robot, cube, q_near, q_rand, cube_near, cube_rand, step_size=step_size, viz=None)
        
#         # Add the longest valid segment found to the graphs, even if success is False
#         if robot_path_segment and cube_path_segment:
#             # Add each configuration pair from the path segment to the graphs with appropriate parent indexing
#             for j in range(len(robot_path_segment)):
#                 parent_index = len(G_robot) - 1 if j > 0 else nearest_index  # Link to previous node or q_near
#                 add_edge_and_vertex(G_robot, G_cube, parent_index, robot_path_segment[j], cube_path_segment[j])
#                 # print("Added edge and vertex.")
#                 # print(f"Robot path length: {len(G_robot)}, Cube path length: {len(G_cube)}")
                
#             # Check if the last configuration in the segment has reached the goal
#             if distance(robot_path_segment[-1], qgoal) < goal_tolerance:
#                 print("Goal reached!")
                
#                 # Add the goal configuration to both graphs and retrieve the path
#                 add_edge_and_vertex(G_robot, G_cube, len(G_robot) - 1, qgoal, cubeplacementqgoal)
                
#                 # Reconstruct the final paths
#                 final_robot_path = get_path(G_robot)
#                 final_cube_path = get_path(G_cube)
#                 # print(f"Is the first element the same? {final_robot_path[0] == qinit}")
#                 # print(f"Is the last element the same? {final_robot_path[-1] == qgoal}")
#                 #What about for the cube?
#                 # print(f"Is the first element the same? {final_cube_path[0] == cubeplacementq0}")
#                 # print(f"Is the last element the same? {final_cube_path[-1] == cubeplacementqgoal}")
                
#                 return final_robot_path, final_cube_path  # Path found
            
#         # Print how close we are to the goal every 10 iterations using nearest vertex to goal
#         if i % 10 == 0:
#             nearest_goal_index = nearest_vertex(G_robot, qgoal)
#             print(f"Iteration {i}: Nearest to goal: {distance(G_robot[nearest_goal_index][1], qgoal)}")            
    
#     print("Max iterations reached without finding a good path. This is rare, so try again and/or increase the goal tolerance.")
#     return [], []  # No valid path found

import numpy as np

def computepath(qinit,qgoal,cubeplacementq0, cubeplacementqgoal, robot=None, cube=None):

    # Check if robot and cube objects are provided
    if robot is None or cube is None:
        print("Error: Robot and cube objects must be provided.")
        return [], []
    
    # Best paramters
    max_iterations=1000
    step_size=0.025
    goal_tolerance=0.5
    
    # Initialize two trees: one starting from qinit, the other from qgoal
    G_start = [(None, qinit)]
    G_goal = [(None, qgoal)]
    G_cube_start = [(None, cubeplacementq0)]
    G_cube_goal = [(None, cubeplacementqgoal)]
    
    for i in range(max_iterations):
        # Step 1: Sample a random configuration and cube placement with 10% goal bias
        if np.random.rand() < 0.1:
            q_rand = qgoal
            cube_rand = cubeplacementqgoal
            print("Goal biasing applied.")
        else:
            q_rand, cube_rand = sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None)
        
        # Step 2: Extend the start tree towards the sample
        nearest_index_start = nearest_vertex(G_start, q_rand)
        q_near_start = G_start[nearest_index_start][1]
        cube_near_start = G_cube_start[nearest_index_start][1]
        
        # Project path from start tree towards random sample
        robot_path_segment, cube_path_segment, success = project_path(robot, cube, q_near_start, q_rand, cube_near_start, cube_rand, step_size=step_size, viz=None)
        
        # Add the path segment to the start tree
        for j in range(len(robot_path_segment)):
            parent_index = len(G_start) - 1 if j > 0 else nearest_index_start
            add_edge_and_vertex(G_start, G_cube_start, parent_index, robot_path_segment[j], cube_path_segment[j])
        
        # Step 3: Attempt to connect the goal tree to the new node in the start tree
        q_new_start = robot_path_segment[-1]
        nearest_index_goal = nearest_vertex(G_goal, q_new_start)
        q_near_goal = G_goal[nearest_index_goal][1]
        cube_near_goal = G_cube_goal[nearest_index_goal][1]
        
        # Project path from goal tree towards the new node in the start tree
        robot_path_segment_goal, cube_path_segment_goal, success = project_path(robot, cube, q_near_goal, q_new_start, cube_near_goal, cubeplacementqgoal, step_size=step_size, viz=None)
        
        # Add the path segment to the goal tree
        for j in range(len(robot_path_segment_goal)):
            parent_index = len(G_goal) - 1 if j > 0 else nearest_index_goal
            add_edge_and_vertex(G_goal, G_cube_goal, parent_index, robot_path_segment_goal[j], cube_path_segment_goal[j])
        
        # Check if trees meet within goal tolerance
        if distance(q_new_start, robot_path_segment_goal[-1]) < goal_tolerance:
            print("Trees connected! Path found.")
            
            # Reconstruct paths from both trees and merge them
            path_start_robot = get_path(G_start)
            path_goal_robot = get_path(G_goal)
            path_start_cube = get_path(G_cube_start)
            path_goal_cube = get_path(G_cube_goal)
            
            # Combine paths, reversing the goal path
            final_robot_path = path_start_robot + path_goal_robot[::-1]
            final_cube_path = path_start_cube + path_goal_cube[::-1]
            
            return final_robot_path, final_cube_path
    
        # Optional: Print progress every 10 iterations
        if i % 10 == 0:
            nearest_goal_index = nearest_vertex(G_start, qgoal)
            print(f"Iteration {i}: Nearest to goal: {distance(G_start[nearest_goal_index][1], qgoal)}")
        
        if i > 300: 
            print("We are likely stuck in a local minimum. Ctrl+C and run again. This is very rare.")
    
    print("Max iterations reached without connecting the trees. This is rare, so try again and/or increase the goal tolerance.")
    return [], []  # No valid path found


# # Proof that it works for the initial configuration
# if __name__ == "__main__":
#     from tools import setupwithmeshcat
#     from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
#     from inverse_geometry import computeqgrasppose
#     import time
    
#     # Set up the robot, cube, and visualization
#     robot, cube, viz = setupwithmeshcat()
    
#     # Get the initial configuration q0 and the goal configuration qe
#     q = robot.q0.copy()
#     q0, success_init = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz=None)
#     qe, success_end = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET, viz=None)

#     # Ensure that both initial and goal configurations are valid
#     if not (success_init and success_end):
#         print("Error: Invalid initial or goal configuration.")
#     else:
#         print("Initial and goal configurations successfully computed.")
        
#         # Run RRT to find a path
#         # robot_path, cube_path = computepath(robot, cube, q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, max_iterations=1000, step_size=0.025, goal_tolerance=0.5, viz=None)
#         robot_path, cube_path = computepath(q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, viz=None)
        
#         # Display the path if found
#         if robot_path and cube_path:
#             print("Displaying RRT path.")
#             for i in range(10):
#                 displaypath(robot, robot_path, cube_path, dt=1, viz=None)
#         else:
#             print("No valid path found.")




# def add_target_marker(viz, target_position, marker_size=0.05):
#     '''
#     Adds a green static marker in MeshCat to indicate the target position for the cube.
    
#     Parameters:
#         viz: The MeshCat visualizer instance.
#         target_position: SE3 object representing the target position.
#         marker_size: Size of the marker in meters (default 0.05).
#     '''
#     marker_name = "goal_marker"
#     marker_geom = "sphere"  # Can also use "box" for a cube marker
#     color = [0, 1, 0, 1]  # Green with full opacity

#     # Add a sphere marker
#     viz.addSphere(marker_name, marker_size, color)
#     viz.applyConfiguration(marker_name, list(target_position.translation) + [0, 0, 0, 1])
#     print(f"Target marker added at position: {target_position.translation}")


# if __name__ == "__main__":
#     from tools import setupwithmeshcat
#     from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
#     from inverse_geometry import computeqgrasppose
#     import pinocchio as pin
#     import numpy as np

#     # Set up the robot, cube, and visualization
#     robot, cube, viz = setupwithmeshcat()

#     # Define the bounds for random cube placements
#     x_min, x_max = min(CUBE_PLACEMENT.translation[0], CUBE_PLACEMENT_TARGET.translation[0]) - 0.1, max(CUBE_PLACEMENT.translation[0], CUBE_PLACEMENT_TARGET.translation[0]) + 0.1
#     y_min, y_max = min(CUBE_PLACEMENT.translation[1], CUBE_PLACEMENT_TARGET.translation[1]) - 0.1, max(CUBE_PLACEMENT.translation[1], CUBE_PLACEMENT_TARGET.translation[1]) + 0.1
#     z_min, z_max = min(CUBE_PLACEMENT.translation[2], CUBE_PLACEMENT_TARGET.translation[2]) - 0.1, max(CUBE_PLACEMENT.translation[2], CUBE_PLACEMENT_TARGET.translation[2]) + 0.1

#     # Define a number of test cases
#     num_tests = 1
#     test_count = 0

#     while test_count < num_tests:
#         # Step 1: Randomly sample start and goal cube placements within bounds
#         start_cube_placement = pin.SE3(pin.SE3.Identity().rotation, np.array([
#             np.random.uniform(x_min, x_max),
#             np.random.uniform(y_min, y_max),
#             np.random.uniform(z_min, z_max)
#         ]))
        

        
#         # Check if both start and goal placements are collision-free
#         setcubeplacement(robot, cube, start_cube_placement)
#         pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
#         if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
#             print(f"Start placement at {start_cube_placement.translation} is in collision. Resampling...")
#             continue

#         setcubeplacement(robot, cube, CUBE_PLACEMENT)
#         pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
#         if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
#             print(f"Goal placement at {CUBE_PLACEMENT.translation} is in collision. Resampling...")
#             continue

#         # Step 2: Compute the initial and goal robot configurations
#         print(f"\nRunning test case {test_count + 1}...")
#         q = robot.q0.copy()
#         q0, success_init = computeqgrasppose(robot, q, cube, start_cube_placement, viz=None)
#         qe, success_end = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz=None)

#         # Ensure both initial and goal configurations are valid
#         if not (success_init and success_end):
#             print(f"Test case {test_count + 1}: Error - Invalid initial or goal configuration.")
#             continue
#         else:
#             print(f"Test case {test_count + 1}: Initial and goal configurations successfully computed.")

#         # Step 3: Run RRT to find a path
#         robot_path, cube_path = computepath(
#             robot, cube, q0, qe, start_cube_placement, CUBE_PLACEMENT,
#             max_iterations=1000, step_size=0.1, goal_tolerance=0.5, viz=None
#         )

#         # Display the path if found
#         if robot_path and cube_path:
#             print(f"Test case {test_count + 1}: Displaying RRT path.")
#             print(f"Difference between last element and goal: {np.linalg.norm(robot_path[-1] - qe)}")
#             print(f"Difference between last cube placement and goal: {np.linalg.norm(cube_path[-1].translation - CUBE_PLACEMENT.translation)}")
                        
#             for i in range(10):
#                 displaypath(robot, robot_path, cube_path, dt=1, viz=None)
#         else:
#             print(f"Test case {test_count + 1}: No valid path found.")
        
#         # Increment test count if a valid test case was executed
#         test_count += 1


# def distance(q1, q2):
#     '''Compute Euclidean distance between two configurations.'''
#     return np.linalg.norm(q2 - q1)

# def nearest_vertex(G, q_rand):
#     '''Return the index of the Node in G with the configuration closest to q_rand.'''
#     min_dist = float('inf')
#     nearest_index = -1
#     for i, (_, q) in enumerate(G):
#         dist = distance(q, q_rand)
#         if dist < min_dist:
#             min_dist = dist
#             nearest_index = i
#     return nearest_index

# def add_edge_and_vertex(G_robot, G_cube, parent_index, q, cube_placement):
#     '''Add a new node to both the robot and cube graphs.'''
#     G_robot.append((parent_index, q))
#     G_cube.append((parent_index, cube_placement))

# def get_path(G):
#     '''Reconstruct the path from the goal back to the initial configuration.'''
#     path = []
#     node = G[-1]
#     while node[0] is not None:
#         path.insert(0, node[1])  # Insert at the beginning to reverse the order
#         node = G[node[0]]
#     path.insert(0, G[0][1])  # Add the initial configuration
#     return path

## Compute Path With No retries.
# def computepath(qinit,qgoal,cubeplacementq0, cubeplacementqgoal, robot=None, cube=None):

#     # Check if robot and cube objects are provided
#     if robot is None or cube is None:
#         print("Error: Robot and cube objects must be provided.")
#         return []
    
#     # DO NOT CHANGE THESE PARAMETERS OR IT WILL MOST LIKELY RESULT IN THE CONTROL PART FAILING. 
#     max_iterations=1000
#     step_size=0.025
#     goal_tolerance=0.5
    
#     # Initialize two trees: one starting from qinit, the other from qgoal
#     G_start = [(None, qinit)]
#     G_goal = [(None, qgoal)]
#     G_cube_start = [(None, cubeplacementq0)]
#     G_cube_goal = [(None, cubeplacementqgoal)]
    
#     for i in range(max_iterations):
#         # Step 1: Sample a random configuration and cube placement with 10% goal bias
#         if np.random.rand() < 0.1:
#             q_rand = qgoal
#             cube_rand = cubeplacementqgoal
#             print("Goal biasing applied.")
#         else:
#             q_rand, cube_rand = sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None)
        
#         # Step 2: Extend the start tree towards the sample
#         nearest_index_start = nearest_vertex(G_start, q_rand)
#         q_near_start = G_start[nearest_index_start][1]
#         cube_near_start = G_cube_start[nearest_index_start][1]
        
#         # Project path from start tree towards random sample
#         robot_path_segment, cube_path_segment = project_path(robot, cube, q_near_start, cube_near_start, cube_rand, step_size=step_size, viz=None)
        
#         # Add the path segment to the start tree
#         for j in range(len(robot_path_segment)):
#             parent_index = len(G_start) - 1 if j > 0 else nearest_index_start
#             add_edge_and_vertex(G_start, G_cube_start, parent_index, robot_path_segment[j], cube_path_segment[j])
        
#         # Step 3: Attempt to connect the goal tree to the new node in the start tree
#         q_new_start = robot_path_segment[-1]
#         nearest_index_goal = nearest_vertex(G_goal, q_new_start)
#         q_near_goal = G_goal[nearest_index_goal][1]
#         cube_near_goal = G_cube_goal[nearest_index_goal][1]
        
#         # Project path from goal tree towards the new node in the start tree
#         robot_path_segment_goal, cube_path_segment_goal = project_path(robot, cube, q_near_goal, cube_near_goal, cubeplacementqgoal, step_size=step_size, viz=None)
        
#         # Add the path segment to the goal tree
#         for j in range(len(robot_path_segment_goal)):
#             parent_index = len(G_goal) - 1 if j > 0 else nearest_index_goal
#             add_edge_and_vertex(G_goal, G_cube_goal, parent_index, robot_path_segment_goal[j], cube_path_segment_goal[j])
        
#         # Check if trees meet within goal tolerance
#         if distance(q_new_start, robot_path_segment_goal[-1]) < goal_tolerance:
#             print("Trees connected! Path found.")
            
#             # Reconstruct paths from both trees and merge them
#             path_start_robot = get_path(G_start)
#             path_goal_robot = get_path(G_goal)
            
#             # Combine paths, reversing the goal path
#             final_robot_path = path_start_robot + path_goal_robot[::-1]
            
#             return final_robot_path
    
#         # Optional: Print progress every 10 iterations
#         if i % 10 == 0:
#             nearest_goal_index = nearest_vertex(G_start, qgoal)
#             print(f"Iteration {i}: Nearest vertex to goal: {distance(G_start[nearest_goal_index][1], qgoal)}. Will succeed when this value is less than {goal_tolerance}.")
        
#         if i > 300: 
#             print("We are likely stuck in a local minimum. Ctrl+C and run again. This is rare, so try again as is and DO NOT change ANY hyperparameters.")
    
#     print("Max iterations reached without connecting the trees. This is rare, so try again as is and DO NOT change ANY hyperparameters.")
#     return [] # No valid path found

# # Vanilla compute path with retries. 
# def computepath(qinit, qgoal, cubeplacementq0, cubeplacementqgoal, robot=None, cube=None): 

#     # Check if robot and cube objects are provided
#     if robot is None or cube is None:
#         print("Error: Robot and cube objects must be provided.")
#         return []
    
#     # DO NOT CHANGE THESE PARAMETERS OR IT WILL MOST LIKELY RESULT IN THE CONTROL PART FAILING. 
#     max_iterations=100
#     max_retries = 10  # Number of times to retry after 100 iterations if path not found
#     step_size=0.025
#     goal_tolerance=0.5
    
#     for retry in range(max_retries):
#         # Initialize two trees: one starting from qinit, the other from qgoal
#         G_start = [(None, qinit)]
#         G_goal = [(None, qgoal)]
#         G_cube_start = [(None, cubeplacementq0)]
#         G_cube_goal = [(None, cubeplacementqgoal)]
        
#         for i in range(max_iterations):
#             # Step 1: Sample a random configuration and cube placement with 10% goal bias
#             if np.random.rand() < 0.1:
#                 q_rand = qgoal
#                 cube_rand = cubeplacementqgoal
#                 print("Goal biasing applied.")
#             else:
#                 q_rand, cube_rand = sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None)
            
#             # Step 2: Extend the start tree towards the sample
#             nearest_index_start = nearest_vertex(G_start, q_rand)
#             q_near_start = G_start[nearest_index_start][1]
#             cube_near_start = G_cube_start[nearest_index_start][1]
            
#             # Project path from start tree towards random sample
#             robot_path_segment, cube_path_segment = project_path(robot, cube, q_near_start, cube_near_start, cube_rand, step_size=step_size, viz=None)
            
#             # Add the path segment to the start tree
#             for j in range(len(robot_path_segment)):
#                 parent_index = len(G_start) - 1 if j > 0 else nearest_index_start
#                 add_edge_and_vertex(G_start, G_cube_start, parent_index, robot_path_segment[j], cube_path_segment[j])
            
#             # Step 3: Attempt to connect the goal tree to the new node in the start tree
#             q_new_start = robot_path_segment[-1]
#             nearest_index_goal = nearest_vertex(G_goal, q_new_start)
#             q_near_goal = G_goal[nearest_index_goal][1]
#             cube_near_goal = G_cube_goal[nearest_index_goal][1]
            
#             # Project path from goal tree towards the new node in the start tree
#             robot_path_segment_goal, cube_path_segment_goal = project_path(robot, cube, q_near_goal, cube_near_goal, cubeplacementqgoal, step_size=step_size, viz=None)
            
#             # Add the path segment to the goal tree
#             for j in range(len(robot_path_segment_goal)):
#                 parent_index = len(G_goal) - 1 if j > 0 else nearest_index_goal
#                 add_edge_and_vertex(G_goal, G_cube_goal, parent_index, robot_path_segment_goal[j], cube_path_segment_goal[j])
            
#             # Check if trees meet within goal tolerance
#             if distance(q_new_start, robot_path_segment_goal[-1]) < goal_tolerance:
#                 print("Trees connected! Path found.")
                
#                 # Reconstruct paths from both trees and merge them
#                 path_start_robot = get_path(G_start)
#                 path_goal_robot = get_path(G_goal)
                
#                 # Combine paths, reversing the goal path
#                 final_robot_path = path_start_robot + path_goal_robot[::-1]
                
#                 return final_robot_path
        
#             # Optional: Print progress every 10 iterations
#             if i % 10 == 0:
#                 nearest_goal_index = nearest_vertex(G_start, qgoal)
#                 print(f"Iteration {i + (retry * max_iterations)}: Nearest vertex to goal: {distance(G_start[nearest_goal_index][1], qgoal)}. Will succeed when this value is less than {goal_tolerance}.")
        
#         print(f"Retry {retry + 1}/{max_retries}: Max iterations reached without connecting the trees. Restarting...")

#     print("Failed to find a path after maximum retries. This is rare, so try again as is and DO NOT change ANY hyperparameters.")
#     return []  # No valid path found




# KD TREE IMPLEMENTATION 1
# import numpy as np
# from scipy.spatial import KDTree

# def distance(q1, q2):
#     '''Compute Euclidean distance between two configurations.'''
#     return np.linalg.norm(q2 - q1)

# def add_edge_and_vertex(G_robot, G_cube, kd_tree_robot, parent_index, q, cube_placement):
#     '''Add a new node to the robot and cube graphs, updating the robot's KDTree.'''
#     G_robot.append((parent_index, q))
#     G_cube.append((parent_index, cube_placement))
    
#     # Rebuild the KDTree with the updated robot graph
#     kd_tree_robot = KDTree([node[1] for node in G_robot])
#     return kd_tree_robot  # Return the updated KDTree

# def get_path(G):
#     '''Reconstruct the path from the goal back to the initial configuration.'''
#     path = []
#     node = G[-1]
#     while node[0] is not None:
#         path.insert(0, node[1])  # Insert at the beginning to reverse the order
#         node = G[node[0]]
#     path.insert(0, G[0][1])  # Add the initial configuration
#     return path

# def computepath(qinit, qgoal, cubeplacementq0, cubeplacementqgoal, robot=None, cube=None):
#     # Check if robot and cube objects are provided
#     if robot is None or cube is None:
#         print("Error: Robot and cube objects must be provided.")
#         return []
    
#     # DO NOT CHANGE THESE PARAMETERS OR IT WILL MOST LIKELY RESULT IN THE CONTROL PART FAILING. 
#     max_iterations = 100
#     max_retries = 10  # Number of times to retry after 100 iterations if path not found
#     step_size = 0.025
#     goal_tolerance = 0.5
    
#     for retry in range(max_retries):
#         # Initialize two trees: one starting from qinit, the other from qgoal
#         G_start = [(None, qinit)]
#         G_goal = [(None, qgoal)]
#         G_cube_start = [(None, cubeplacementq0)]
#         G_cube_goal = [(None, cubeplacementqgoal)]
        
#         # Initialize KDTree only for the robot configurations
#         kd_tree_robot_start = KDTree([qinit])
#         kd_tree_robot_goal = KDTree([qgoal])

#         for i in range(max_iterations):
#             # Step 1: Sample a random configuration and cube placement with 10% goal bias
#             if np.random.rand() < 0.1:
#                 q_rand = qgoal
#                 cube_rand = cubeplacementqgoal
#                 print("Goal biasing applied.")
#             else:
#                 q_rand, cube_rand = sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None)
            
#             # Step 2: Extend the start tree towards the sampled configuration
#             _, nearest_index_start = kd_tree_robot_start.query(q_rand)
#             q_near_start = G_start[nearest_index_start][1]
#             cube_near_start = G_cube_start[nearest_index_start][1]
            
#             # Project path from the start tree towards the random sample
#             robot_path_segment, cube_path_segment = project_path(robot, cube, q_near_start, cube_near_start, cube_rand, step_size=step_size, viz=None)
            
#             # Add the path segment to the start tree
#             for j in range(len(robot_path_segment)):
#                 parent_index = len(G_start) - 1 if j > 0 else nearest_index_start
#                 kd_tree_robot_start = add_edge_and_vertex(G_start, G_cube_start, kd_tree_robot_start, parent_index, robot_path_segment[j], cube_path_segment[j])
        
#             # Step 3: Attempt to connect the goal tree to the new node in the start tree
#             q_new_start = robot_path_segment[-1]
#             _, nearest_index_goal = kd_tree_robot_goal.query(q_new_start)
#             q_near_goal = G_goal[nearest_index_goal][1]
#             cube_near_goal = G_cube_goal[nearest_index_goal][1]
            
#             # Project path from goal tree towards the new node in the start tree
#             robot_path_segment_goal, cube_path_segment_goal = project_path(robot, cube, q_near_goal, cube_near_goal, cubeplacementqgoal, step_size=step_size, viz=None)
            
#             # Add the path segment to the goal tree
#             for j in range(len(robot_path_segment_goal)):
#                 parent_index = len(G_goal) - 1 if j > 0 else nearest_index_goal
#                 kd_tree_robot_goal = add_edge_and_vertex(G_goal, G_cube_goal, kd_tree_robot_goal, parent_index, robot_path_segment_goal[j], cube_path_segment_goal[j])
            
#             # Check if trees meet within goal tolerance
#             if distance(q_new_start, robot_path_segment_goal[-1]) < goal_tolerance:
#                 print("Trees connected! Path found.")
                
#                 # Reconstruct paths from both trees and merge them
#                 path_start_robot = get_path(G_start)
#                 path_goal_robot = get_path(G_goal)
                
#                 # Combine paths, reversing the goal path
#                 final_robot_path = path_start_robot + path_goal_robot[::-1]
                
#                 return final_robot_path
        
#             # Optional: Print progress every 10 iterations
#             if i % 10 == 0:
#                 _, nearest_goal_index = kd_tree_robot_start.query(qgoal)
#                 print(f"Iteration {i + (retry * max_iterations)}: Nearest vertex to goal: {distance(G_start[nearest_goal_index][1], qgoal)}. Will succeed when this value is less than {goal_tolerance}.")
        
#         print(f"Retry {retry + 1}/{max_retries}: Max iterations reached without connecting the trees. Restarting...")

#     print("Failed to find a path after maximum retries. This is rare, so try again as is and DO NOT change ANY hyperparameters.")
#     return []  # No valid path found

#Original Main Method. 
if __name__ == "__main__":
    from tools import setupwithmeshcat
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    
    robot, cube, viz = setupwithmeshcat()
    
    
    q = robot.q0.copy()
    q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz)
    qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz)
    
    if not(successinit and successend):
        print ("error: invalid initial or end configuration")
    
    path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, robot=robot, cube=cube)
    
    displaypath(robot,path,dt=0.5,viz=None) #you ll probably want to lower dt