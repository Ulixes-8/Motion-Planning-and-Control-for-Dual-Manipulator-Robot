#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 21 11:44:32 2023

@author: stonneau
"""

import pinocchio as pin
import numpy as np
from config import LEFT_HAND, RIGHT_HAND
import time
from inverse_geometry import computeqgrasppose


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



# def project_path2(robot, cube, q_curr, q_rand, cube_curr, cube_rand, step_size=0.01, viz=None):
#     '''
#     Project a path from q_curr to q_rand and cube_curr to cube_rand such that every configuration
#     in the path maintains the grasp constraint and remains collision-free. Returns the furthest
#     valid configuration along the path.
    
#     Parameters:
#         robot: The robot object with its kinematic model.
#         cube: The cube object representing the object to be grasped.
#         q_curr: Initial robot configuration.
#         q_rand: Target robot configuration.
#         cube_curr: Initial cube placement.
#         cube_rand: Target cube placement.
#         step_size: Discretization step for interpolation between configurations.
#         viz: Optional visualization object.
    
#     Returns:
#         A tuple (robot_path, cube_path, success) where:
#             - robot_path is a list of valid robot configurations up to the furthest valid point.
#             - cube_path is a list of corresponding cube placements up to the furthest valid point.
#             - success is a boolean indicating if the full path was successfully projected.
#     '''
    
#     # Calculate the number of interpolation steps based on step_size
#     distance = np.linalg.norm(cube_curr.translation - cube_rand.translation)
#     num_steps = int(distance / step_size) + 1
    
#     # Initialize paths with the starting configurations
#     robot_path = [q_curr]
#     cube_path = [cube_curr]
    
#     # Linear interpolation between cube placements
#     for step in range(1, num_steps + 1):
#         alpha = step / num_steps
        
#         # Interpolate the cube's SE3 placement
#         interpolated_cube_pos = pin.SE3.Interpolate(cube_curr, cube_rand, alpha)
        
#         # Set the cube at the interpolated position and check for collisions
#         setcubeplacement(robot, cube, interpolated_cube_pos)
#         pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
#         if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
#             # Cube placement is invalid; return the valid portion of the path
#             # print("Cube collision detected in interpolated path. Returning valid portion.")
#             return robot_path, cube_path, False
        
#         # Use computeqgrasppose to find a valid robot configuration for this cube placement
#         q, success = computeqgrasppose(robot, robot_path[-1].copy(), cube, interpolated_cube_pos, viz)
#         if not success:
#             # Robot configuration is invalid; return the valid portion of the path
#             # print("Failed to compute valid robot configuration at interpolated position. Returning valid portion.")
#             return robot_path, cube_path, False
        
#         # Append the valid configurations to the paths
#         robot_path.append(q)
#         cube_path.append(interpolated_cube_pos)
    
#     # If we reach here, the full path is valid
#     return robot_path, cube_path, True



# def distance2(q1, q2):
#     '''Compute Euclidean distance between two configurations.'''
#     return np.linalg.norm(q2 - q1)

# def nearest_vertex2(G, q_rand):
#     '''Return the index of the Node in G with the configuration closest to q_rand.'''
#     min_dist = float('inf')
#     nearest_index = -1
#     for i, (_, q) in enumerate(G):
#         dist = distance(q, q_rand)
#         if dist < min_dist:
#             min_dist = dist
#             nearest_index = i
#     return nearest_index

# def add_edge_and_vertex2(G_robot, G_cube, parent_index, q, cube_placement):
#     '''Add a new node to both the robot and cube graphs.'''
#     G_robot.append((parent_index, q))
#     G_cube.append((parent_index, cube_placement))

# def get_path2(G):
#     '''Reconstruct the path from the goal back to the initial configuration.'''
#     path = []
#     node = G[-1]
#     while node[0] is not None:
#         path.insert(0, node[1])  # Insert at the beginning to reverse the order
#         node = G[node[0]]
#     path.insert(0, G[0][1])  # Add the initial configuration
#     return path

# import numpy as np

# def computepath2(robot, cube, qinit, qgoal, cubeplacementq0, cubeplacementqgoal, max_iterations=2000, step_size=0.025, goal_tolerance=0.5, viz=None):
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
#         nearest_index = nearest_vertex2(G_robot, q_rand)
#         q_near = G_robot[nearest_index][1]
#         cube_near = G_cube[nearest_index][1]
        
#         # Step 3: Project a path from q_near and cube_near to q_rand and cube_rand under grasping constraints
#         robot_path_segment, cube_path_segment, _ = project_path2(robot, cube, q_near, q_rand, cube_near, cube_rand, step_size=step_size, viz=None)
        
#         # Add the longest valid segment found to the graphs, even if success is False
#         if robot_path_segment and cube_path_segment:
#             # Add each configuration pair from the path segment to the graphs with appropriate parent indexing
#             for j in range(len(robot_path_segment)):
#                 parent_index = len(G_robot) - 1 if j > 0 else nearest_index  # Link to previous node or q_near
#                 add_edge_and_vertex2(G_robot, G_cube, parent_index, robot_path_segment[j], cube_path_segment[j])
#                 # print("Added edge and vertex.")
#                 # print(f"Robot path length: {len(G_robot)}, Cube path length: {len(G_cube)}")
                
#             # Check if the last configuration in the segment has reached the goal
#             if distance2(robot_path_segment[-1], qgoal) < goal_tolerance:
#                 print("Goal reached!")
                
#                 # Add the goal configuration to both graphs and retrieve the path
#                 add_edge_and_vertex2(G_robot, G_cube, len(G_robot) - 1, qgoal, cubeplacementqgoal)
                
#                 # Reconstruct the final paths
#                 final_robot_path = get_path2(G_robot)
#                 final_cube_path = get_path2(G_cube)
#                 # print(f"Is the first element the same? {final_robot_path[0] == qinit}")
#                 # print(f"Is the last element the same? {final_robot_path[-1] == qgoal}")
#                 #What about for the cube?
#                 # print(f"Is the first element the same? {final_cube_path[0] == cubeplacementq0}")
#                 # print(f"Is the last element the same? {final_cube_path[-1] == cubeplacementqgoal}")
                
#                 return final_robot_path  # Path found
            
#         # Print how close we are to the goal every 10 iterations using nearest vertex to goal
#         if i % 10 == 0:
#             nearest_goal_index = nearest_vertex2(G_robot, qgoal)
#             print(f"Iteration {i}: Nearest to goal: {distance(G_robot[nearest_goal_index][1], qgoal)}")            
    
#     print("Max iterations reached without finding a good path. This is rare, so try again and/or increase the goal tolerance.")
#     return []# No valid path found






# """
# Created on Thu Sep 21 11:44:32 2023

# @author: stonneau
# """

# from tools import setcubeplacement, distanceToObstacle
# from inverse_geometry import computeqgrasppose
# from scipy.spatial import KDTree




# def sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None):
#     """
#     Samples a feasible placement for the cube using Gaussian sampling centered 
#     above the fixed obstacle placement with dynamically defined bounds based on
#     the initial and goal placements.
#     """
#     min_obstacle_distance = 0.04  # Minimum allowable distance from obstacles

#     # Fixed obstacle location
#     obstacle_position = np.array([0.43, -0.1, 0.94])
#     gaussian_center = obstacle_position + np.array([0.0, 0.0, 0.3])  # 15 cm above obstacle
#     # Experiment with the x and y centers... 
#     # gaussian_center = obstacle_position + np.array([0.0, 0.0, 0.3])  # 15 cm above obstacle

#     # Define dynamic bounds based on current cube placement and target, plus margin
#     margin = 0.14
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

# def project_path(robot, cube, q_curr, cube_curr, cube_rand, step_size=0.1, viz=None):
#     step_size = 0.1

#     # Calculate the number of interpolation steps based on step_size
#     distance = np.linalg.norm(cube_curr.translation - cube_rand.translation)
#     num_steps = int(distance / step_size) + 1
    
#     # Initialize paths with the starting configurations
#     robot_path = [q_curr]
#     cube_path = [cube_curr]
    
#     # Linear interpolation between cube placements
#     for step in range(1, num_steps + 1):
#         alpha = step / num_steps
        
#         # Interpolate the cube's SE3 placement
#         interpolated_cube_pos = pin.SE3.Interpolate(cube_curr, cube_rand, alpha)
        
#         # Set the cube at the interpolated position and check for collisions
#         setcubeplacement(robot, cube, interpolated_cube_pos)
#         pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
#         if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
#             # Cube placement is invalid; return the valid portion of the path
#             # print("Cube collision detected in interpolated path. Returning valid portion.")
#             return robot_path, cube_path
        
#         # Use computeqgrasppose to find a valid robot configuration for this cube placement
#         q, success = computeqgrasppose(robot, robot_path[-1].copy(), cube, interpolated_cube_pos, viz)
#         if not success:
#             # Robot configuration is invalid; return the valid portion of the path
#             # print("Failed to compute valid robot configuration at interpolated position. Returning valid portion.")
#             return robot_path, cube_path
        
#         # Append the valid configurations to the paths
#         robot_path.append(q)
#         cube_path.append(interpolated_cube_pos)
    
#     # If we reach here, the full path is valid
#     return robot_path, cube_path

# def distance(q1, q2):
#     '''Compute Euclidean distance between two configurations.'''
#     return np.linalg.norm(q2 - q1)

# def add_edge_and_vertex(G_robot, G_cube, robot_points, parent_index, q, cube_placement, tree_needs_update):
#     '''Add a new node to the robot and cube graphs without immediately updating the KDTree.'''
#     G_robot.append((parent_index, q))
#     G_cube.append((parent_index, cube_placement))
#     robot_points.append(q)  # Add to points list
#     tree_needs_update[0] = True  # Flag that KDTree needs update

# def get_path(G):
#     '''Reconstruct the path from the goal back to the initial configuration.'''
#     path = []
#     node = G[-1]
#     while node[0] is not None:
#         path.insert(0, node[1])  # Insert at the beginning to reverse the order
#         node = G[node[0]]
#     path.insert(0, G[0][1])  # Add the initial configuration
#     return path

# def lazy_kd_tree_query(robot_points, kd_tree, target, tree_needs_update):
#     '''Only rebuild and query KDTree if new points were added.'''
#     if tree_needs_update[0]:  # Check if rebuild is needed
#         kd_tree = KDTree(robot_points)  # Rebuild KDTree
#         tree_needs_update[0] = False  # Reset the flag after rebuilding
#     distance, nearest_index = kd_tree.query(target)
#     return distance, nearest_index, kd_tree

# def computepath(qinit, qgoal, cubeplacementq0, cubeplacementqgoal, robot=None, cube=None):
#     if robot is None or cube is None:
#         print("Error: Robot and cube objects must be provided.")
#         return []
    
#     max_iterations = 100
#     max_retries = 10
#     step_size = 0.025
#     goal_tolerance = 0.5

#     for retry in range(max_retries):
#         G_start = [(None, qinit)]
#         G_goal = [(None, qgoal)]
#         G_cube_start = [(None, cubeplacementq0)]
#         G_cube_goal = [(None, cubeplacementqgoal)]

#         robot_points_start = [qinit]
#         robot_points_goal = [qgoal]

#         kd_tree_robot_start = KDTree([qinit])
#         kd_tree_robot_goal = KDTree([qgoal])

#         tree_needs_update_start = [False]
#         tree_needs_update_goal = [False]

#         for i in range(max_iterations):
#             if np.random.rand() < 0.1:
#                 q_rand = qgoal
#                 cube_rand = cubeplacementqgoal
#                 print("Goal biasing applied.")
#             else:
#                 q_rand, cube_rand = sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None)

#             _, nearest_index_start, kd_tree_robot_start = lazy_kd_tree_query(
#                 robot_points_start, kd_tree_robot_start, q_rand, tree_needs_update_start
#             )
#             q_near_start = G_start[nearest_index_start][1]
#             cube_near_start = G_cube_start[nearest_index_start][1]

#             robot_path_segment, cube_path_segment = project_path(
#                 robot, cube, q_near_start, cube_near_start, cube_rand, step_size=step_size, viz=None
#             )

#             for j in range(len(robot_path_segment)):
#                 parent_index = len(G_start) - 1 if j > 0 else nearest_index_start
#                 add_edge_and_vertex(G_start, G_cube_start, robot_points_start, parent_index, 
#                                     robot_path_segment[j], cube_path_segment[j], tree_needs_update_start)

#             q_new_start = robot_path_segment[-1]
#             _, nearest_index_goal, kd_tree_robot_goal = lazy_kd_tree_query(
#                 robot_points_goal, kd_tree_robot_goal, q_new_start, tree_needs_update_goal
#             )
#             q_near_goal = G_goal[nearest_index_goal][1]
#             cube_near_goal = G_cube_goal[nearest_index_goal][1]

#             robot_path_segment_goal, cube_path_segment_goal = project_path(
#                 robot, cube, q_near_goal, cube_near_goal, cubeplacementqgoal, step_size=step_size, viz=None
#             )

#             for j in range(len(robot_path_segment_goal)):
#                 parent_index = len(G_goal) - 1 if j > 0 else nearest_index_goal
#                 add_edge_and_vertex(G_goal, G_cube_goal, robot_points_goal, parent_index, 
#                                     robot_path_segment_goal[j], cube_path_segment_goal[j], tree_needs_update_goal)

#             if distance(q_new_start, robot_path_segment_goal[-1]) < goal_tolerance:
#                 print("Trees connected! Path found.")
#                 path_start_robot = get_path(G_start)
#                 path_goal_robot = get_path(G_goal)
#                 final_robot_path = path_start_robot + path_goal_robot[::-1]
#                 return final_robot_path

#             if i % 10 == 0:
#                 _, nearest_goal_index, kd_tree_robot_start = lazy_kd_tree_query(
#                     robot_points_start, kd_tree_robot_start, qgoal, tree_needs_update_start
#                 )
#                 print(f"Iteration {i + (retry * max_iterations)}: Nearest vertex to goal: {distance(G_start[nearest_goal_index][1], qgoal)}. Will succeed when this value is less than {goal_tolerance}.")

#         print(f"Retry {retry + 1}/{max_retries}: Max iterations reached without connecting the trees. Restarting...")

#     print("Failed to find a path after maximum retries. This is rare, so try again as is and DO NOT change ANY hyperparameters.")
#     return []





# # #Original Main Method. 
# # if __name__ == "__main__":
# #     from tools import setupwithmeshcat
# #     from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
# #     from inverse_geometry import computeqgrasppose
    
# #     robot, cube, viz = setupwithmeshcat()
    
    
# #     q = robot.q0.copy()
# #     q0,successinit = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT, viz=None)
# #     qe,successend = computeqgrasppose(robot, q, cube, CUBE_PLACEMENT_TARGET,  viz=None)
    
# #     if not(successinit and successend):
# #         print ("error: invalid initial or end configuration")
    
# #     path1, path2 = computepath2(robot, cube, q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)
# #     for i in range(10):
# #         displaypath(robot, path1, path2, .5, viz)
# #         time.sleep(1)


# # import numpy as np
# # import time
# # from tools import setupwithmeshcat, setcubeplacement, distanceToObstacle
# # from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
# # from inverse_geometry import computeqgrasppose
# # from rrt_implementations import computepath, computepath2  # Assuming `computepath` is RRT vanilla and `computepath2` is RRT-Connect with KD Tree

# # def run_rrt_test(robot, cube, viz, std_devs):
# #     results = []

# #     for std_dev in std_devs:
# #         std_dev_x, std_dev_y, std_dev_z = std_dev
# #         print(f"Testing std dev (X, Y, Z): {std_dev}")
        
# #         rrt_success = 0
# #         rrt_time = 0
# #         rrt_connect_success = 0
# #         rrt_connect_time = 0
# #         trials = 100

# #         for _ in range(trials):
# #             # Randomly sample CUBE_PLACEMENT and CUBE_PLACEMENT_TARGET
# #             q_init, cubeplacement_q0 = sample_cube_placement(robot, cube, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, std_dev_x, std_dev_y, std_dev_z, viz=None)
# #             q_goal, cubeplacement_qgoal = sample_cube_placement(robot, cube, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, std_dev_x, std_dev_y, std_dev_z, viz=None)

# #             # Check that initial and goal configurations are valid
# #             if q_init is None or q_goal is None:
# #                 print("Error: Invalid initial or end configuration. Skipping this trial.")
# #                 continue
            
# #             # Run RRT Vanilla
# #             start_time = time.time()
# #             path_vanilla = computepath(q_init, q_goal, cubeplacement_q0, cubeplacement_qgoal, robot=robot, cube=cube)
# #             rrt_time += time.time() - start_time
# #             if path_vanilla:
# #                 rrt_success += 1

# #             # Run RRT-Connect with KD Tree
# #             start_time = time.time()
# #             path_rrt_connect = computepath2(robot, cube, q_init, q_goal, cubeplacement_q0, cubeplacement_qgoal)
# #             rrt_connect_time += time.time() - start_time
# #             if path_rrt_connect:
# #                 rrt_connect_success += 1

# #         # Calculate average times and success rates for each std_dev setting
# #         rrt_success_rate = (rrt_success / trials) * 100
# #         rrt_connect_success_rate = (rrt_connect_success / trials) * 100
# #         avg_rrt_time = rrt_time / trials
# #         avg_rrt_connect_time = rrt_connect_time / trials

# #         # Store results
# #         results.append({
# #             "std_dev": std_dev,
# #             "RRT_success_rate": rrt_success_rate,
# #             "RRT_avg_time": avg_rrt_time,
# #             "RRT_Connect_success_rate": rrt_connect_success_rate,
# #             "RRT_Connect_avg_time": avg_rrt_connect_time
# #         })

# #     return results

# # if __name__ == "__main__":
# #     # Set up robot, cube, and visualizer
# #     robot, cube, viz = setupwithmeshcat()
# #     std_devs = [(0.1, 0.1, 0.1), (0.2, 0.2, 0.2), (0.3, 0.3, 0.3), (0.4, 0.4, 0.4), (0.5, 0.5, 0.5), (0.6, 0.6, 0.6), (0.7, 0.7, 0.7)]

# #     # Run the RRT comparative test
# #     results = run_rrt_test(robot, cube, viz, std_devs)

# #     # Print results in a formatted table
# #     print(f"{'Std Dev (X, Y, Z)':<20} {'RRT Success Rate (%)':<20} {'RRT Avg Time (s)':<20} {'RRT-Connect Success Rate (%)':<25} {'RRT-Connect Avg Time (s)':<25}")
# #     print("=" * 110)
# #     for result in results:
# #         std_dev = result['std_dev']
# #         rrt_success_rate = result['RRT_success_rate']
# #         rrt_avg_time = result['RRT_avg_time']
# #         rrt_connect_success_rate = result['RRT_Connect_success_rate']
# #         rrt_connect_avg_time = result['RRT_Connect_avg_time']
# #         print(f"{std_dev:<20} {rrt_success_rate:<20.2f} {rrt_avg_time:<20.2f} {rrt_connect_success_rate:<25.2f} {rrt_connect_avg_time:<25.2f}")



# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# Created on Thu Sep 21 11:44:32 2023

# @author: stonneau
# """

# import time
# from tools import setcubeplacement, distanceToObstacle
# import pinocchio as pin
# import numpy as np
# from inverse_geometry import computeqgrasppose
# from scipy.spatial import KDTree


# def displaypath(robot,path,dt,viz):
#     for q in path:
#         viz.display(q)
#         time.sleep(dt)

# import numpy as np
# import pinocchio as pin



# # def sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None):

# #     margin = 0.3

# #     # Define workspace bounds based on initial and goal placements, plus a margin
# #     x_min, x_max = min(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) - margin, max(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) + margin
# #     y_min, y_max = min(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) - margin, max(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) + margin
# #     z_min, z_max = min(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) - margin, max(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) + margin

# #     # DO NOT CHANGE THIS VALUE OR IT WILL MOST LIKELY RESULT IN THE CONTROL PART FAILING.
# #     min_obstacle_distance = 0.035 # Define minimum allowable distance from obstacle

# #     while True:
# #         # Randomly sample x, y, z within bounds
# #         x = np.random.uniform(x_min, x_max)
# #         y = np.random.uniform(y_min, y_max)
# #         z = np.random.uniform(z_min, z_max)
        
# #         # Define the cube placement as an SE3 transform with no rotation
# #         placement = pin.SE3(pin.SE3.Identity().rotation, np.array([x, y, z]))

# #         # Use setcubeplacement to update the cube's placement
# #         setcubeplacement(robot, cube, placement)
        
# #         # Check if the cube placement is in collision
# #         pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
# #         if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
# #             # Skip this placement if in collision and resample
# #             continue

# #         # Attempt to compute a robot configuration to grasp the cube at this placement
# #         q, success = computeqgrasppose(robot, robot.q0.copy(), cube, placement, viz)
        
# #         # Check distance to obstacle for the current configuration
# #         if success:
# #             dist_to_obstacle = distanceToObstacle(robot, q)

# #             # Verify if the configuration meets the minimum distance requirement
# #             if dist_to_obstacle >= min_obstacle_distance:
# #                 print("Sampled placement and configuration are valid under all constraints, including distance to obstacle.")
# #                 print("Placement:", placement)
# #                 return q, placement
# #             else:
# #                 print("Sampled placement too close to obstacle, resampling...")
        
# #         # Otherwise, resample if configuration is invalid or too close to the obstacle

import numpy as np
import time
from tools import setupwithmeshcat, setcubeplacement
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
from inverse_geometry import computeqgrasppose
from scipy.spatial import KDTree
from tools import distanceToObstacle

def sample_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, viz=None):
    """
    Samples a feasible placement for the cube using Gaussian sampling centered 
    above the fixed obstacle placement with dynamically defined bounds based on
    the initial and goal placements.
    """
    min_obstacle_distance = 0.04  # Minimum allowable distance from obstacles

    # Fixed obstacle location
    obstacle_position = np.array([0.43, -0.1, 0.94])
    gaussian_center = obstacle_position + np.array([0.0, 0.0, 0.3])  # 15 cm above obstacle
    # Experiment with the x and y centers... 
    # gaussian_center = obstacle_position + np.array([0.0, 0.0, 0.3])  # 15 cm above obstacle

    # Define dynamic bounds based on current cube placement and target, plus margin
    margin = 0.12
    x_min, x_max = min(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) - margin, max(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) + margin
    y_min, y_max = min(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) - margin, max(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) + margin
    z_min, z_max = min(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) - margin, max(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) + margin

    # Standard deviations for Gaussian sampling (adjustable based on workspace and clearance needs)

    std_dev_x = .3  # Spread in x-direction around obstacle
    std_dev_y = .3 # Spread in y-direction around obstacle
    std_dev_z = .3  # Smaller spread in z to focus samples slightly above obstacle
    while True:
        # Sample x, y, z from a Gaussian distribution centered above the obstacle
        x = np.clip(np.random.normal(gaussian_center[0], std_dev_x), x_min, x_max)
        y = np.clip(np.random.normal(gaussian_center[1], std_dev_y), y_min, y_max)
        z = np.clip(np.random.normal(gaussian_center[2], std_dev_z), z_min, z_max)
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

def project_path(robot, cube, q_curr, cube_curr, cube_rand, step_size=0.025, viz=None):
    step_size = 0.1

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
    
    max_iterations = 100
    max_retries = 1
    step_size = 0.025
    goal_tolerance = 0.5

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
                return final_robot_path, i

            if i % 10 == 0:
                _, nearest_goal_index, kd_tree_robot_start = lazy_kd_tree_query(
                    robot_points_start, kd_tree_robot_start, qgoal, tree_needs_update_start
                )
                print(f"Iteration {i + (retry * max_iterations)}: Nearest vertex to goal: {distance(G_start[nearest_goal_index][1], qgoal)}. Will succeed when this value is less than {goal_tolerance}.")

        print(f"Retry {retry + 1}/{max_retries}: Max iterations reached without connecting the trees. Restarting...")

    print("Failed to find a path after maximum retries. This is rare, so try again as is and DO NOT change ANY hyperparameters.")
    return [], 100



import numpy as np
import time
from tools import setupwithmeshcat, setcubeplacement, distanceToObstacle
from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
from inverse_geometry import computeqgrasppose
from scipy.spatial import KDTree

import numpy as np
import pinocchio as pin
from tools import setcubeplacement, distanceToObstacle
from inverse_geometry import computeqgrasppose

def sample_random_cube_placement(robot, cube, cubeplacementq0, cubeplacementqgoal, std_dev_x, std_dev_y, std_dev_z, viz=None):
    """
    Samples a random feasible placement for the cube using Gaussian sampling centered above a fixed obstacle, with configurable margins and clipping.
    Ensures no collision and validates the robot configuration.
    
    Args:
        robot: The robot object containing model and data structures required for kinematics and dynamics.
        cube: The cube object representing the object to be grasped.
        cubeplacementq0: Initial placement of the cube.
        cubeplacementqgoal: Goal placement for the cube.
        std_dev_x, std_dev_y, std_dev_z: Standard deviations for sampling the placement.
        viz (optional): Visualization object for rendering the robot's movements in a 3D environment.
        
    Returns:
        tuple: A valid joint configuration and cube placement.
    """
    min_obstacle_distance = 0.04  # Minimum allowable distance from obstacles

    # Fixed obstacle location
    obstacle_position = np.array([0.43, -0.1, 0.94])
    gaussian_center = obstacle_position + np.array([0.0, 0.0, 0.3])  # 15 cm above obstacle
    
    # Define dynamic bounds based on cube placements and margin
    margin = 0.13
    x_min, x_max = min(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) - margin, max(cubeplacementq0.translation[0], cubeplacementqgoal.translation[0]) + margin
    y_min, y_max = min(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) - margin, max(cubeplacementq0.translation[1], cubeplacementqgoal.translation[1]) + margin
    z_min, z_max = min(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) - margin, max(cubeplacementq0.translation[2], cubeplacementqgoal.translation[2]) + margin

    # Start sampling loop
    while True:
        # Sample x, y, z from Gaussian distribution centered above obstacle
        x = np.clip(np.random.normal(gaussian_center[0], std_dev_x), x_min, x_max)
        y = np.clip(np.random.normal(gaussian_center[1], std_dev_y), y_min, y_max)
        z = np.clip(np.random.normal(gaussian_center[2], std_dev_z), z_min, z_max)
        sampled_point = np.array([x, y, z])

        # Define the cube placement with no rotation
        placement = pin.SE3(pin.SE3.Identity().rotation, sampled_point)
        setcubeplacement(robot, cube, placement)

        # Check for collisions
        pin.updateGeometryPlacements(cube.model, cube.data, cube.collision_model, cube.collision_data)
        if pin.computeCollisions(cube.collision_model, cube.collision_data, False):
            print("BAD CUBE PLACEMENT")
            continue  # Resample if in collision

        # Attempt to compute grasp pose for this placement
        q, success = computeqgrasppose(robot, robot.q0.copy(), cube, placement, viz)
        
        if success:
            # Check distance to obstacle for this configuration
            dist_to_obstacle = distanceToObstacle(robot, q)
            if dist_to_obstacle >= min_obstacle_distance:
                print("GOOD CUBE PLACEMENT")
                print("Placement:", placement)
                return q, placement
            else:
                print("BAD Q FOR CUBE PLACEMENT")


def run_rrt_connect_test(robot, cube, viz, std_devs):
    results = []

    for std_dev in std_devs:
        std_dev_x, std_dev_y, std_dev_z = std_dev
        print(f"Testing std dev (X, Y, Z): {std_dev}")
        
        success_count = 0
        times = []
        iterations_counts = []
        trials = 100

        for _ in range(trials):
            # Randomly sample initial and goal placements with specified standard deviations
            q_init, cubeplacement_q0 = sample_random_cube_placement(robot, cube, cubeplacementq0=CUBE_PLACEMENT, cubeplacementqgoal=CUBE_PLACEMENT_TARGET, std_dev_x=std_dev_x, std_dev_y=std_dev_y, std_dev_z=std_dev_z, viz=None)
            q_goal, cubeplacement_qgoal = sample_random_cube_placement(robot, cube, cubeplacementq0=CUBE_PLACEMENT, cubeplacementqgoal=CUBE_PLACEMENT_TARGET, std_dev_x=std_dev_x, std_dev_y=std_dev_y, std_dev_z=std_dev_z, viz=None)

            # Run RRT-Connect with KD-Tree
            start_time = time.time()
            path_rrt_connect, iteration_count = computepath(q_init, q_goal, cubeplacement_q0, cubeplacement_qgoal, robot=robot, cube=cube)
            elapsed_time = time.time() - start_time
            times.append(elapsed_time)
            iterations_counts.append(iteration_count)

            # Count successful pathfinding attempts
            if path_rrt_connect:
                success_count += 1

        # Calculate success rate and time metrics
        success_rate = (success_count / trials) * 100
        avg_time = np.mean(times) if times else 0
        max_time = np.max(times) if times else 0
        min_time = np.min(times) if times else 0
        std_time = np.std(times) if times else 0

        # Calculate iteration metrics
        avg_iterations = np.mean(iterations_counts) if iterations_counts else 0
        max_iterations = np.max(iterations_counts) if iterations_counts else 0
        min_iterations = np.min(iterations_counts) if iterations_counts else 0
        std_iterations = np.std(iterations_counts) if iterations_counts else 0

        # Store results
        results.append({
            "std_dev": std_dev,
            "success_rate": success_rate,
            "avg_time": avg_time,
            "max_time": max_time,
            "min_time": min_time,
            "std_time": std_time,
            "avg_iterations": avg_iterations,
            "max_iterations": max_iterations,
            "min_iterations": min_iterations,
            "std_iterations": std_iterations
        })

    return results



if __name__ == "__main__":
    # Set up robot, cube, and visualizer
    robot, cube, viz = setupwithmeshcat()
    std_devs = [(0.1, 0.1, 0.1), (0.2, 0.2, 0.2), (0.3, 0.3, 0.3), (0.4, 0.4, 0.4)]

    # Run the RRT-Connect test
    results = run_rrt_connect_test(robot, cube, None, std_devs)

    print(f"{'Std Dev (X, Y, Z)':<20} {'Success Rate (%)':<20} {'Avg Time (s)':<15} {'Max Time (s)':<15} {'Min Time (s)':<15} {'Std Dev Time (s)':<15} {'Avg Iter':<10} {'Max Iter':<10} {'Min Iter':<10} {'Std Iter':<10}")
    print("=" * 140)
    for result in results:
        std_dev = str(result['std_dev'])  # Convert tuple to string
        success_rate = result['success_rate']
        avg_time = result['avg_time']
        max_time = result['max_time']
        min_time = result['min_time']
        std_time = result['std_time']
        avg_iterations = result['avg_iterations']
        max_iterations = result['max_iterations']
        min_iterations = result['min_iterations']
        std_iterations = result['std_iterations']
        print(f"{std_dev:<20} {success_rate:<20.2f} {avg_time:<15.2f} {max_time:<15.2f} {min_time:<15.2f} {std_time:<15.2f} {avg_iterations:<10.2f} {max_iterations:<10} {min_iterations:<10} {std_iterations:<10.2f}")
