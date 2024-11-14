#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  6 15:32:51 2023

@author: stonneau
"""

import numpy as np

from bezier import Bezier
    
# in my solution these gains were good enough for all joints but you might want to tune this.
Kp = 300.               # proportional gain (P of PD)
Kv = 2 * np.sqrt(Kp)   # derivative gain (D of PD)

def controllaw(sim, robot, trajs, tcurrent, cube):
    q, vq = sim.getpybulletstate()
    #TODO 
    torques = [0.0 for _ in sim.bulletCtrlJointsInPinOrder]
    sim.step(torques)

# if __name__ == "__main__":
        
#     from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
#     from config import DT
    
#     robot, sim, cube = setupwithpybullet()
    
    
#     from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
#     from inverse_geometry import computeqgrasppose
#     from path import computepath
    
#     q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
#     qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
#     # path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET) # This was the original. Fix the signature before submitting.
#     # path = computepath(robot, cube, q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    
#     #setting initial configuration
#     sim.setqsim(q0)
    
    
#     #TODO this is just an example, you are free to do as you please.
#     #In any case this trajectory does not follow the path 
#     #0 init and end velocities
#     def maketraj(q0,q1,T): #TODO compute a real trajectory !
#         q_of_t = Bezier([q0,q0,q1,q1],t_max=T)
#         vq_of_t = q_of_t.derivative(1)
#         vvq_of_t = vq_of_t.derivative(1)
#         return q_of_t, vq_of_t, vvq_of_t
    
    
#     #TODO this is just a random trajectory, you need to do this yourself
#     total_time=4.
#     trajs = maketraj(q0, qe, total_time)   
    
#     tcur = 0.
    
    
#     while tcur < total_time:
#         rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
#         tcur += DT

import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt




class Bezier:   
    def __init__(self, pointlist, t_min=0.0, t_max=1.0, mult_t=1.0):
        self.dim_ = pointlist[0].shape[0]
        self.T_min_ = t_min
        self.T_max_ = t_max
        self.mult_T_ = mult_t
        self.size_ = len(pointlist) - 1
        self.degree_ = self.size_
        self.control_points_ = pointlist    
        if (self.size_ < 1 or self.T_max_ <= self.T_min_):
            raise ValueError("Can't create Bezier curve; min bound is higher than max bound.")
                
    def __call__(self, t):
        if not (self.T_min_ <= t <= self.T_max_):
            raise ValueError("Can't evaluate Bezier curve, time t is out of range")        
        if self.size_ == 1:
            return self.mult_T_ * self.control_points_[0]
        else:
            return self.eval_horner(t)
            
    def eval_horner(self, t):
        u = (t - self.T_min_) / (self.T_max_ - self.T_min_)
        u_op, bc, tn = 1.0 - u, 1, 1
        tmp = self.control_points_[0] * u_op
        for i in range(1, self.degree_):
            tn *= u
            bc *= (self.degree_ - i + 1) / i
            tmp = (tmp + tn * bc * self.control_points_[i]) * u_op
        return (tmp + tn * u * self.control_points_[-1]) * self.mult_T_
                    
    def derivative(self, order):
        if order == 0:
            return self

        derived_wp = []
        for point1, point2 in zip(self.control_points_[:-1], self.control_points_[1:]):
            derived_wp.append(self.degree_ * (point2 - point1))

        if not derived_wp:
            derived_wp.append([0] * self.dim_)

        deriv = Bezier(derived_wp, self.T_min_, self.T_max_, self.mult_T_ * (1.0 / (self.T_max_ - self.T_min_)))
        return deriv.derivative(order - 1)

# Best performing maketraj 
def maketraj(robot, cube, q0, q1, path_points, total_time):
    """
    Compute a Bézier trajectory that fits the given path using SLSQP and your Bezier class.

    Parameters:
    - robot: Robot model (if needed for additional constraints).
    - cube: Cube object (if needed).
    - q0: Initial joint configuration (numpy array).
    - q1: Final joint configuration (numpy array).
    - path_points: Numpy array of joint configurations along the path (n_points x dim).
    - total_time: Total time duration.

    Returns:
    - q_of_t: Bezier instance representing the trajectory.
    - vq_of_t: First derivative of the Bezier curve.
    - vvq_of_t: Second derivative of the Bezier curve.
    """
    n_points = len(path_points)
    dim = q0.shape[0]  # Number of joints

    # Degree of the Bezier curve (ensure it's at least 3 for acceleration constraints)
    degree = min(n_points + 3, 15)
    n_ctrl_pts = degree + 1  # Number of control points

    # Initial guess for control points (linear interpolation between q0 and q1)
    P0 = np.linspace(q0, q1, n_ctrl_pts)  # Shape: (n_ctrl_pts, dim)
    x0 = P0.flatten()  # Flatten to 1D array for the optimizer

    # Time samples corresponding to path points
    times = np.linspace(0, total_time, n_points)

    # Define the cost function
    def cost_function(x):
        P = x.reshape((n_ctrl_pts, dim))
        control_points = [P[i] for i in range(n_ctrl_pts)]
        bezier_curve = Bezier(control_points, t_min=0.0, t_max=total_time)
        cost = 0.0
        for t, q_desired in zip(times, path_points):
            q_t = bezier_curve(t)
            cost += np.sum((q_t - q_desired) ** 2)
        return cost

    # Constraints list
    constraints = []

    # Initial and final position constraints
    def initial_position_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        return P[0] - q0

    def final_position_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        return P[-1] - q1

    constraints.append({'type': 'eq', 'fun': initial_position_constraint})
    constraints.append({'type': 'eq', 'fun': final_position_constraint})

    # Initial and final velocity constraints
    def initial_velocity_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        control_points = [P[i] for i in range(n_ctrl_pts)]
        bezier_curve = Bezier(control_points, t_min=0.0, t_max=total_time)
        v0 = bezier_curve.derivative(1)(0.0)
        return v0.flatten()

    def final_velocity_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        control_points = [P[i] for i in range(n_ctrl_pts)]
        bezier_curve = Bezier(control_points, t_min=0.0, t_max=total_time)
        vT = bezier_curve.derivative(1)(total_time)
        return vT.flatten()

    constraints.append({'type': 'eq', 'fun': initial_velocity_constraint})
    constraints.append({'type': 'eq', 'fun': final_velocity_constraint})

    # Initial and final acceleration constraints
    def initial_acceleration_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        control_points = [P[i] for i in range(n_ctrl_pts)]
        bezier_curve = Bezier(control_points, t_min=0.0, t_max=total_time)
        a0 = bezier_curve.derivative(2)(0.0)
        return a0.flatten()

    def final_acceleration_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        control_points = [P[i] for i in range(n_ctrl_pts)]
        bezier_curve = Bezier(control_points, t_min=0.0, t_max=total_time)
        aT = bezier_curve.derivative(2)(total_time)
        return aT.flatten()

    constraints.append({'type': 'eq', 'fun': initial_acceleration_constraint})
    constraints.append({'type': 'eq', 'fun': final_acceleration_constraint})

    bounds = None  # Set bounds if needed


    print("Optimization started... This may take a few mintues.") 
    result = minimize(
        cost_function,
        x0,
        method='SLSQP',
        constraints=constraints,
        bounds=bounds,
        options={'ftol': 1e-9, 'disp': True, 'maxiter': 1000}
    )
    


    if not result.success:
        raise ValueError(f"Optimization failed: {result.message}")

    # Extract the optimized control points
    P_opt = result.x.reshape((n_ctrl_pts, dim))
    control_points_opt = [P_opt[i] for i in range(n_ctrl_pts)]

    # Create the Bezier curve using the optimized control points
    q_of_t = Bezier(control_points_opt, t_min=0.0, t_max=total_time)
    vq_of_t = q_of_t.derivative(1)
    vvq_of_t = q_of_t.derivative(2)
    
    # Save the trajectory to a JSON file so we can de-bug controllaw quickly. 
    if result.fun < 0.1:
        save_trajectory_to_json('trajectory.json', q_of_t, vq_of_t, vvq_of_t)
    
    trajs = [q_of_t, vq_of_t, vvq_of_t]
    
    if result.fun < 0.1:
        print("Optimization highly likely to have succeeded... If it fails, please run it one more time! :D It works 99 percent of the time.")
        return trajs, True
    print("Optimization likely to have failed. Re-running.")
    return trajs, False




import json
import numpy as np

# Function to save trajectory control points to a JSON file
def save_trajectory_to_json(filename, q_of_t, vq_of_t, vvq_of_t):
    trajectory_data = {
        'q_control_points': [point.tolist() for point in q_of_t.control_points_],
        'vq_control_points': [point.tolist() for point in vq_of_t.control_points_],
        'vvq_control_points': [point.tolist() for point in vvq_of_t.control_points_],
        't_min': q_of_t.T_min_,
        't_max': q_of_t.T_max_,
        'mult_t': q_of_t.mult_T_
    }

    with open(filename, 'w') as f:
        json.dump(trajectory_data, f, indent=4)
    print(f"Trajectory saved to {filename}")

# Example usage

# Function to load trajectory control points from a JSON file and reconstruct Bezier objects
def load_trajectory_from_json(filename):
    with open(filename, 'r') as f:
        trajectory_data = json.load(f)

    # Reconstruct the Bezier curves for position, velocity, and acceleration
    q_of_t = Bezier([np.array(point) for point in trajectory_data['q_control_points']],
                    t_min=trajectory_data['t_min'],
                    t_max=trajectory_data['t_max'],
                    mult_t=trajectory_data['mult_t'])

    vq_of_t = Bezier([np.array(point) for point in trajectory_data['vq_control_points']],
                     t_min=trajectory_data['t_min'],
                     t_max=trajectory_data['t_max'],
                     mult_t=trajectory_data['mult_t'])

    vvq_of_t = Bezier([np.array(point) for point in trajectory_data['vvq_control_points']],
                      t_min=trajectory_data['t_min'],
                      t_max=trajectory_data['t_max'],
                      mult_t=trajectory_data['mult_t'])

    print(f"Trajectory loaded from {filename}")
    return q_of_t, vq_of_t, vvq_of_t

def controllaw(sim, robot, trajs, tcurrent, cube):
    import numpy as np
    import pinocchio as pin
    import quadprog  # Import quadprog for QP solving
    from tools import distanceToObstacle

    # Kp = 7000.0               # Proportional gain
    # Kv = 2 * np.sqrt(Kp)     # Derivative gain
    # Kf = 30             # Force gain
    # Kt = 2 * np.sqrt(Kf)                # Torque gain
    
    Kp = 7000.0               # Proportional gain
    Kv = 2 * np.sqrt(Kp)     # Derivative gain
    Kf = 30            # Force gain
    Kt = 2 * np.sqrt(Kf)                # Torque gain

    # Get current joint positions and velocities from the simulator
    q, vq = sim.getpybulletstate()
    q = np.array(q)
    vq = np.array(vq)
    
    
    
    # # TEST ##########################################
    # dist = distanceToObstacle(robot, q)
    # # print(f"Distance to obstacle: {dist}")
    # d_term = 1300.0
    # # Threshold distance for obstacle avoidance
    # dist_threshold = 0.2  # Define a reasonable distance threshold
    # avoidance_gain = 1.0 / max(dist, dist_threshold)  # Increase influence as distance decreases
    
    

    # Desired joint positions, velocities, and accelerations from trajectory
    q_des = trajs[0](tcurrent)
    vq_des = trajs[1](tcurrent)

    # Get the end-effector frame indices
    LEFT_HAND = 'LARM_EFF'
    RIGHT_HAND = 'RARM_EFF'
    ee_frame_names = [LEFT_HAND, RIGHT_HAND]
    ee_frame_ids = [robot.model.getFrameId(name) for name in ee_frame_names]

    # Check if frame IDs are valid
    for name, frame_id in zip(ee_frame_names, ee_frame_ids):
        if frame_id < 0:
            raise ValueError(f"Frame '{name}' not found in robot model.")

    # Update robot model with current state
    pin.computeAllTerms(robot.model, robot.data, q, vq)
    M = robot.data.M        # Joint space inertia matrix
    b = pin.nonLinearEffects(robot.model, robot.data, q, vq)
    pin.updateFramePlacements(robot.model, robot.data)
    pin.computeJointJacobians(robot.model, robot.data, q)
    pin.computeJointJacobiansTimeVariation(robot.model, robot.data, q, vq)

    # Create a separate data for desired state
    data_des = robot.model.createData()
    pin.forwardKinematics(robot.model, data_des, q_des, vq_des)
    pin.updateFramePlacements(robot.model, data_des)

    # Initialize variables for stacking
    num_ee = len(ee_frame_ids)
    task_dim = 6 * num_ee  # Control in XYZ and orientation for each end-effector
    J_stack = []
    J_dot_v_stack = []
    x_ddot_des_stack = []
    f_c_stack = []

    for i, ee_frame_id in enumerate(ee_frame_ids):
        # Current end-effector pose and velocity
        oMee = robot.data.oMf[ee_frame_id]
        x_current = oMee.translation.copy()
        R_current = oMee.rotation.copy()

        v_frame = pin.getFrameVelocity(
            robot.model, robot.data, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        x_dot_current = v_frame.linear.copy()
        omega_current = v_frame.angular.copy()

        # Desired end-effector pose and velocity
        oMee_des = data_des.oMf[ee_frame_id]
        x_desired = oMee_des.translation.copy()
        R_desired = oMee_des.rotation.copy()

        v_frame_des = pin.getFrameVelocity(
            robot.model, data_des, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        x_dot_desired = v_frame_des.linear.copy()
        omega_desired = v_frame_des.angular.copy()

        # Compute position and orientation errors
        e_translation = x_desired - x_current 
        e_rotation = pin.log3(R_desired @ R_current.T)  # 3D orientation error

        # Compute velocity errors
        e_dot_translation = x_dot_desired - x_dot_current
        e_dot_rotation = omega_desired - omega_current

        # Combined errors
        e = np.hstack((e_translation, e_rotation))
        e_dot = np.hstack((e_dot_translation, e_dot_rotation))

        # Desired end-effector acceleration (PD control)
        x_ddot_desired = Kp * e + Kv * e_dot  # 6D vector 
        
        # # TEST ##########################################
        #     # Calculate the repulsive force based on the distance to obstacle
        # if dist < dist_threshold:
        #     repulsive_force = avoidance_gain * e_translation  # Direction away from obstacle
        #     # Optionally scale repulsive force with distance if needed
        #     repulsive_force = d_term * repulsive_force  # Adjust based on proportional gain
        #     print(f"Repulsive force: {repulsive_force}")

        #     # Add the repulsive force to the desired end-effector acceleration
        #     x_ddot_desired[:3] -= repulsive_force  # Reduces acceleration toward the obstacle

        # Compute Jacobian and its time derivative at the current state
        J = pin.computeFrameJacobian(
            robot.model, robot.data, q, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_dot = pin.getFrameJacobianTimeVariation(
            robot.model, robot.data, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        J_dot_v = J_dot @ vq

        # Compute operational space inertia matrix Lambda
        Lambda_inv = J @ np.linalg.inv(M) @ J.T
        Lambda = np.linalg.inv(Lambda_inv + 1e-6 * np.eye(Lambda_inv.shape[0]))  # Regularize for stability

        # Compute desired force and torque
        f_force = Kf * e_translation
        f_torque = Kt * e_rotation
        f_c = np.hstack((f_force, f_torque))

        # Contribution due to desired force
        x_ddot_force = Lambda @ f_c  # 6D vector

        # Total desired end-effector acceleration
        x_ddot_total = x_ddot_desired + x_ddot_force

        # Stack the Jacobians and desired accelerations
        J_stack.append(J)
        J_dot_v_stack.append(J_dot_v)
        x_ddot_des_stack.append(x_ddot_total)
        f_c_stack.append(f_c)

    # Stack the matrices
    J_total = np.vstack(J_stack)
    J_dot_v_total = np.hstack(J_dot_v_stack)
    x_ddot_des_total = np.hstack(x_ddot_des_stack)
    f_c_total = np.hstack(f_c_stack)
    
    # Bias for f_c_total in linear y axis to make the robot move in the y direction.
    f_c_total[1] -= 125
    

    # Formulate the QP problem
    # Minimize 0.5 * || J_total @ q_ddot + J_dot_v_total - x_ddot_des_total ||^2
    # This can be written as: 0.5 * q_ddot.T @ H_qp @ q_ddot + d_qp.T @ q_ddot
    H_qp = J_total.T @ J_total
    d_qp = J_total.T @ (J_dot_v_total - x_ddot_des_total)
    d_qp = -d_qp  # quadprog uses -d in the cost function

    # Ensure H_qp is positive definite
    reg = 1e-6
    H_qp += reg * np.eye(H_qp.shape[0])

    try:
        # Convert to double precision and ensure contiguity
        H_qp = np.ascontiguousarray(H_qp, dtype=np.float64)
        d_qp = np.ascontiguousarray(d_qp, dtype=np.float64)

        # Solve the QP
        q_ddot_desired = quadprog.solve_qp(H_qp, d_qp)[0]
    except ValueError as e:
        print("QP solver failed: ", e)
        q_ddot_desired = np.zeros(robot.model.nv)

    # Compute control torques including force feedback
    tau = M @ q_ddot_desired + b + J_total.T @ f_c_total
    
    # Postural bias. 
    zero_torque_joints = [1, 2]
    for joint_index in zero_torque_joints:
        tau[joint_index] = 0.0


    # Send torques to the simulator
    sim.step(tau.tolist())






# # Main code
# if __name__ == "__main__":

#     # Import necessary modules and functions
#     from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
#     from config import DT
    

#     robot, sim, cube, viz = setupwithpybulletandmeshcat()
    

#     from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
#     from inverse_geometry import computeqgrasppose
#     from path import computepath

#     flag = False
#     while flag == False:    
#         q0, successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
#         qe, successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET, None)
#         path = computepath(q0, qe, CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET, robot=robot, cube=cube)

#         # # # # # Extract the robot's path
#         robot_path = path  # Assuming path[0] is the robot's path
#         path_points = np.array(robot_path)  # Should have shape (n_points, dim)
#         n_points = len(path_points)
#         dim = q0.shape[0]

#         # # Setting initial configuration
#         sim.setqsim(q0)

#         total_time = 15.0
        
#         tcur = 0.0
#         DT = 0.01  # Time step, adjust as needed

#         # # # # # # Compute the trajectory
#         trajs, flag = maketraj(robot, cube, q0, qe, path_points, total_time)
    
#     q_of_t, vq_of_t, vvq_of_t = trajs
#     trajs = (q_of_t, vq_of_t, vvq_of_t)

#     # Evaluate the trajectory
#     t_sample = np.linspace(0, total_time, 100)
#     q_samples = np.array([q_of_t(t) for t in t_sample])  # Shape: (100, dim)
#     vq_samples = np.array([vq_of_t(t) for t in t_sample])
#     vvq_samples = np.array([vvq_of_t(t) for t in t_sample])

#     # # Times corresponding to path points
#     times = np.linspace(0, total_time, n_points)


#     # Plot positions
#     for joint_idx in range(dim):
#         plt.figure()
#         plt.plot(times, path_points[:, joint_idx], 'rx', label='Path Points')
#         plt.plot(t_sample, q_samples[:, joint_idx], 'b-', label='Bezier Trajectory')
#         plt.title(f'Joint {joint_idx + 1} Position')
#         plt.xlabel('Time (s)')
#         plt.ylabel('Joint Angle (rad)')
#         plt.legend()
#         plt.grid(True)
#         plt.show()

#     # # Plot velocities
#     # for joint_idx in range(dim):
#     #     plt.figure()
#     #     plt.plot(t_sample, vq_samples[:, joint_idx], 'g-', label='Velocity')
#     #     plt.title(f'Joint {joint_idx + 1} Velocity')
#     #     plt.xlabel('Time (s)')
#     #     plt.ylabel('Velocity (rad/s)')
#     #     plt.grid(True)
#     #     plt.show()

#     # # Plot accelerations
#     # for joint_idx in range(dim):
#     #     plt.figure()
#     #     plt.plot(t_sample, vvq_samples[:, joint_idx], 'm-', label='Acceleration')
#     #     plt.title(f'Joint {joint_idx + 1} Acceleration')
#     #     plt.xlabel('Time (s)')
#     #     plt.ylabel('Acceleration (rad/s²)')
#     #     plt.grid(True)
#     #     plt.show()


#     while tcur < total_time:
#         rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
#         tcur += DT
        
        

# Control Law Testing
if __name__ == "__main__":

    # Import necessary modules and functions
    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    

    robot, sim, cube, viz = setupwithpybulletandmeshcat()
    

    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET
    from inverse_geometry import computeqgrasppose
    from path import computepath

    q0, successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe, successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET, None)

    sim.setqsim(q0)

    total_time = 15.0 # DO NOT ADJUST
    
    tcur = 0.0 
    DT = 0.01  # DO NOT ADJUST


    q_of_t, vq_of_t, vvq_of_t = load_trajectory_from_json('trajectory.json') # NEar perfect trajectory
    trajs = (q_of_t, vq_of_t, vvq_of_t)



    while tcur < total_time:
        rununtil(controllaw, DT, sim, robot, trajs, tcur, cube)
        tcur += DT
        

    
    