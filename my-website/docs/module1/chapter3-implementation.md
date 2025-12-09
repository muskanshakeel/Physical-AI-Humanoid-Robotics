---
sidebar_position: 3
---

# Module 1: The Robotic Nervous System (ROS 2) - Chapter 3: Implementation Walkthrough

## 1. Concepts

This chapter will guide you through the step-by-step process of implementing a basic ROS 2 node in Python, combining the theoretical knowledge from Chapter 1 and the tooling skills from Chapter 2.

## 2. Tooling

We will primarily use a text editor (e.g., VS Code), a terminal for executing ROS 2 commands, and Python for coding the node.

## 3. Implementation walkthrough: Building a Simple Publisher-Subscriber System

This section provides a detailed walkthrough to create a ROS 2 package containing a publisher node and a subscriber node.

### Step 1: Create a ROS 2 Package

First, create a new ROS 2 package using `ros2 pkg create`.

### Step 2: Implement the Publisher Node

Write a Python script for a publisher node that periodically sends "Hello ROS 2!" messages on a topic.

### Step 3: Implement the Subscriber Node

Write a Python script for a subscriber node that receives and prints messages from the topic.

### Step 4: Build and Run

Instructions on how to build the ROS 2 package and run both the publisher and subscriber nodes.

## 4. Case study / example

A case study on implementing a ROS 2 node that controls a simulated LED based on subscribed data.

## 5. Mini project

A hands-on project to create a ROS 2 package for a simple robot arm that publishes its joint states and subscribes to velocity commands.

## 6. Debugging & common failures

Common issues encountered during ROS 2 node development, such as package building errors, node not starting, or topic communication failures.
