---
sidebar_position: 5
---

# Module 4: Vision-Language-Action (VLA) - Chapter 5: Mini Project

## 1. Concepts

This mini-project integrates speech recognition (Whisper), LLM-based planning, and ROS 2 action execution to create a simple end-to-end Vision-Language-Action pipeline. You will enable a robot to respond to basic voice commands.

<h2> 2. Tooling</h2>

We will utilize Whisper for speech-to-text, an LLM API for planning, and ROS 2 for defining and executing robot actions.

<h2> 3. Implementation Walkthrough</h2>

This section will provide high-level guidance for the mini-project, assuming you will fill in the details based on previous chapters.

<h2> 4. Case study / example</h2>

A case study on developing a simplified voice interface for a smart home assistant that controls lights and appliances.

<h2> 5. Mini Project: Simple Voice Command to Robot Action</h2>

The goal of this mini-project is to build a basic VLA system where a robot (simulated or real) can perform a simple action based on a spoken command.

<h3> Project Objective</h3>

-   **Voice Command Input**: Use Whisper to transcribe a spoken command (e.g., "robot, move forward").
-   **LLM Interpretation**: Feed the transcribed text to an LLM to generate a corresponding robot action (e.g., `move_forward(distance=1.0)`).
-   **ROS 2 Action Execution**: Convert the LLM's output into a ROS 2 action goal and send it to a robot action server (e.g., a simple robot controller that moves forward).

<h3> Validation</h3>

-   The robot correctly interprets the voice command and executes the corresponding action.

<h2> 6. Debugging & common failures</h2>

Common issues in this integrated pipeline, such as misinterpretations by Whisper, LLM planning errors, or failures in translating LLM output to valid ROS 2 actions.
