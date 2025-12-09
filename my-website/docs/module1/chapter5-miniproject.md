---
sidebar_position: 5
---

# Module 1: The Robotic Nervous System (ROS 2) - Chapter 5: Mini Project

## 1. Concepts

This mini-project will challenge you to integrate various ROS 2 concepts and tools to implement basic mobile robot navigation. You will apply your knowledge of nodes, topics, and potentially services to control a robot's movement.

## 2. Tooling

We will use `rclpy`, `ros2 topic` for control, and potentially `rviz` for visualization of the robot's movement in a simulated environment.

## 3. Implementation walkthrough

This section will provide high-level guidance for the mini-project, assuming you will fill in the details based on previous chapters.

## 4. Case study / example

A case study on using odometry and simple command velocity topics for robot movement.

## 5. Mini project: Basic Mobile Robot Navigation

The goal of this mini-project is to develop a ROS 2 application that enables a simulated mobile robot to navigate a simple environment.

### Project Objective

-   Create a ROS 2 node to publish velocity commands (`Twist` messages) to control a simulated robot.
-   Implement a simple logic (e.g., driving straight, turning at intervals, following a wall) to make the robot move.
-   Visualize the robot's movement in a simulation (e.g., Gazebo or `rviz`).

### Validation

-   The robot moves as expected according to the implemented logic.
-   No collisions occur (in a simple, open environment).

## 6. Debugging & common failures

Common issues encountered in mobile robot navigation projects, such as incorrect velocity commands, coordinate frame mismatches, or controller instability.
