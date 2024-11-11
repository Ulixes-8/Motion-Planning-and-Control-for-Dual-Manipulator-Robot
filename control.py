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
#     path = computepath(robot, cube, q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)

    
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
from scipy.special import comb
import matplotlib.pyplot as plt

def bernstein_poly(i, n, t):
    """
    Compute the Bernstein polynomial of degree n and index i at time t.
    """
    return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

def bezier_curve(P, t):
    """
    Evaluate the Bézier curve at time t.

    Parameters:
    - P: Control points (n_ctrl_pts x dim).
    - t: Time value(s) in [0, 1].

    Returns:
    - Curve point(s) at time t.
    """
    n = len(P) - 1  # Degree of the curve
    dim = P.shape[1]
    B = np.array([bernstein_poly(i, n, t) for i in range(n + 1)])  # Shape: (n+1,)
    point = B @ P  # Shape: (dim,)
    return point

def bezier_first_derivative(P, t, T):
    """
    Evaluate the first derivative of the Bézier curve at time t.

    Parameters:
    - P: Control points (n_ctrl_pts x dim).
    - t: Time value(s) in [0, 1].
    - T: Total time duration.

    Returns:
    - First derivative at time t.
    """
    n = len(P) - 1  # Degree of the curve
    dim = P.shape[1]
    derivative = np.zeros(dim)
    for i in range(n):
        coeff = n * (bernstein_poly(i, n - 1, t))
        derivative += coeff * (P[i + 1] - P[i])
    derivative /= T  # Adjust for total time
    return derivative

def bezier_second_derivative(P, t, T):
    """
    Evaluate the second derivative of the Bézier curve at time t.

    Parameters:
    - P: Control points (n_ctrl_pts x dim).
    - t: Time value(s) in [0, 1].
    - T: Total time duration.

    Returns:
    - Second derivative at time t.
    """
    n = len(P) - 1  # Degree of the curve
    dim = P.shape[1]
    second_derivative = np.zeros(dim)
    for i in range(n - 1):
        coeff = n * (n - 1) * (bernstein_poly(i, n - 2, t))
        second_derivative += coeff * (P[i + 2] - 2 * P[i + 1] + P[i])
    second_derivative /= T ** 2  # Adjust for total time squared
    return second_derivative

def maketraj(q0, q1, T, path_points, times):
    """
    Compute a Bézier trajectory that fits the given path using SLSQP.

    Parameters:
    - q0: Initial joint configuration (numpy array).
    - q1: Final joint configuration (numpy array).
    - T: Total time duration.
    - path_points: Numpy array of joint configurations along the path (n_points x dim).
    - times: Corresponding time points for each configuration in path_points (numpy array).

    Returns:
    - q_of_t: Function to evaluate the Bézier curve.
    - vq_of_t: Function to evaluate the first derivative.
    - vvq_of_t: Function to evaluate the second derivative.
    """
    n_points = len(path_points)
    dim = q0.shape[0]  # Number of joints

    # Degree of the Bézier curve (adjust as needed)
    degree = min(n_points + 3, 10)  # Limit degree to prevent overfitting
    n_ctrl_pts = degree + 1  # Number of control points

    # Normalize time to [0, 1]
    t_normalized = times / T

    # Initial guess for control points (linear interpolation)
    P0 = np.linspace(q0, q1, n_ctrl_pts)  # Shape: (n_ctrl_pts, dim)
    x0 = P0.flatten()  # Flatten to 1D array for the optimizer

    # Cost function
    def cost_function(x):
        P = x.reshape((n_ctrl_pts, dim))
        cost = 0.0
        for t, q_desired in zip(t_normalized, path_points):
            q_t = bezier_curve(P, t)
            cost += np.sum((q_t - q_desired) ** 2)
        return cost

    # Constraints list
    constraints = []

    # Initial and final position constraints
    def initial_position_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        return P[0] - q0  # Should be zero

    def final_position_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        return P[-1] - q1  # Should be zero

    constraints.append({'type': 'eq', 'fun': initial_position_constraint})
    constraints.append({'type': 'eq', 'fun': final_position_constraint})

    # Initial and final velocity constraints
    def initial_velocity_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        v0 = bezier_first_derivative(P, t=0.0, T=T)
        return v0  # Should be zero

    def final_velocity_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        vT = bezier_first_derivative(P, t=1.0, T=T)
        return vT  # Should be zero

    constraints.append({'type': 'eq', 'fun': initial_velocity_constraint})
    constraints.append({'type': 'eq', 'fun': final_velocity_constraint})

    # Initial and final acceleration constraints
    def initial_acceleration_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        a0 = bezier_second_derivative(P, t=0.0, T=T)
        return a0  # Should be zero

    def final_acceleration_constraint(x):
        P = x.reshape((n_ctrl_pts, dim))
        aT = bezier_second_derivative(P, t=1.0, T=T)
        return aT  # Should be zero

    constraints.append({'type': 'eq', 'fun': initial_acceleration_constraint})
    constraints.append({'type': 'eq', 'fun': final_acceleration_constraint})

    # Optional: Add joint limit constraints (if needed)
    # q_min = np.full(dim, -np.pi)
    # q_max = np.full(dim, np.pi)
    # bounds = [(q_min[i], q_max[i]) for _ in range(n_ctrl_pts) for i in range(dim)]
    bounds = None  # No bounds in this example

    # Solve the optimization problem
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

    # Define the trajectory functions
    def q_of_t(t):
        t_norm = t / T
        return bezier_curve(P_opt, t_norm)

    def vq_of_t(t):
        t_norm = t / T
        return bezier_first_derivative(P_opt, t_norm, T)

    def vvq_of_t(t):
        t_norm = t / T
        return bezier_second_derivative(P_opt, t_norm, T)

    return q_of_t, vq_of_t, vvq_of_t

    
    
    
if __name__ == "__main__":
    # Example usage

    from tools import setupwithpybullet, setupwithpybulletandmeshcat, rununtil
    from config import DT
    
    robot, sim, cube = setupwithpybullet()
    
    
    from config import CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET    
    from inverse_geometry import computeqgrasppose
    from path import computepath
    
    q0,successinit = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT, None)
    qe,successend = computeqgrasppose(robot, robot.q0, cube, CUBE_PLACEMENT_TARGET,  None)
    # path = computepath(q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET) # This was the original. Fix the signature before submitting.
    path = computepath(robot, cube, q0,qe,CUBE_PLACEMENT, CUBE_PLACEMENT_TARGET)


    # Generate a simple path (e.g., linear interpolation between q0 and q1)
    total_time = 4.0  # Total duration
    n_points = 20  # Number of path points
    times = np.linspace(0, total_time, n_points)
    path_points = np.linspace(q0, q1, n_points)  # Linear path

    # Compute the trajectory
    q_of_t, vq_of_t, vvq_of_t = maketraj(q0, q1, total_time, path_points, times)

    # Evaluate the trajectory at sample times
    t_sample = np.linspace(0, total_time, 100)
    q_samples = np.array([q_of_t(t) for t in t_sample])  # Shape: (100, dim)

    # Plot the results
    dim = q0.shape[0]
    for joint_idx in range(dim):
        plt.figure()
        plt.plot(times, path_points[:, joint_idx], 'rx', label='Path Points')
        plt.plot(t_sample, q_samples[:, joint_idx], 'b-', label='Bezier Trajectory')
        plt.title(f'Joint {joint_idx+1} Trajectory')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Angle (rad)')
        plt.legend()
        plt.grid(True)
        plt.show()

    # Optionally, plot velocities and accelerations
    vq_samples = np.array([vq_of_t(t) for t in t_sample])
    vvq_samples = np.array([vvq_of_t(t) for t in t_sample])

    for joint_idx in range(dim):
        plt.figure()
        plt.plot(t_sample, vq_samples[:, joint_idx], 'g-', label='Velocity')
        plt.title(f'Joint {joint_idx+1} Velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (rad/s)')
        plt.grid(True)
        plt.show()

        plt.figure()
        plt.plot(t_sample, vvq_samples[:, joint_idx], 'm-', label='Acceleration')
        plt.title(f'Joint {joint_idx+1} Acceleration')
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (rad/s^2)')
        plt.grid(True)
        plt.show()
