# Safe Quadrotor Navigation using RRT* and CLF-CBF-QP Control

This project presents an obstacle-aware quadrotor navigation system that combines **3D RRT*** path planning with a **CLF-CBF-QP safety-critical controller**.

The system was developed in **MATLAB** and integrated with **CoppeliaSim** through the ZMQ Remote API. A collision-free global path is generated with RRT*, smoothed with cubic splines, and then tracked by a controller that enforces both trajectory stability and obstacle avoidance.

## Overview

Quadrotor navigation in cluttered environments requires both:
- a global planner to generate feasible paths, and
- a controller that can track those paths safely in real time.

In this project, I combined:
- **RRT*** for 3D path planning,
- **cubic spline interpolation** for smooth trajectory generation,
- a **PD controller** as baseline,
- and a **CLF-CBF-QP controller** for safe trajectory tracking.

The CLF term stabilizes the tracking error, while the CBF constraints act as a safety filter to keep the quadrotor away from obstacles.

## Key Contributions

- Designed a complete quadrotor navigation pipeline in cluttered indoor environments
- Implemented 3D RRT* path planning with obstacle-aware collision checking
- Generated smooth reference trajectories using cubic splines
- Implemented a CLF-CBF-QP controller for stable and safe tracking
- Integrated MATLAB control logic with CoppeliaSim using the ZMQ Remote API
- Compared the proposed controller against a PD baseline

## Methods

The pipeline consists of:
1. Generating a collision-free 3D path with RRT*
2. Smoothing waypoints into continuous reference trajectories
3. Tracking the trajectory using either:
   - a PD controller, or
   - a CLF-CBF-QP controller
4. Enforcing obstacle avoidance using Higher-Order Control Barrier Functions (HOCBF)
5. Evaluating safety and tracking performance in simulation

## Results

The CLF-CBF-QP controller improved safety and tracking robustness compared to the PD baseline.

Reported results include:
- **Total RMSE:** 0.218 m (CLF-CBF-QP) vs 0.296 m (PD)
- **Minimum obstacle distance:** 0.808 m vs 0.375 m
- **Safety violations:** 0 vs 121

These results show that the optimization-based controller preserves tracking quality while providing strong safety guarantees in cluttered environments.

## Files in this Repository

- `docs/project_report.pdf` – full project report
- `assets/demo_video.mp4` – simulation demo
- `assets/coppelia_scene.ttt` – CoppeliaSim scene
- `code/matlab_final_project.m` – summary of the MATLAB controller architecture and implementation

## Tools and Technologies

- MATLAB
- CoppeliaSim
- ZMQ Remote API
- RRT*
- Control Lyapunov Functions (CLF)
- Control Barrier Functions (CBF)
- Quadratic Programming

## Notes

The original controller implementation was developed in MATLAB.  
This repository currently includes the simulation assets, report, and demo materials. 

## Author

**Luisa Chavez**  
M.S. in Electrical Engineering, NC State University
