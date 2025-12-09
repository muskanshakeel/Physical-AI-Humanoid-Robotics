# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-physical-ai-textbook` | **Date**: 2025-12-06 | **Spec**: [./spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-textbook/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the high-level architecture and content for the "Physical AI & Humanoid Robotics — An AI-Native Textbook." The book will follow a 4-module backbone: Digital Brain (ROS2), Simulation (Gazebo/Unity), Perception (NVIDIA Isaac), and Autonomy (VLA). Each module will have a consistent chapter structure. The research approach will be tied to real robotics SDKs, and quality validation methods will ensure reproducibility and prevent hallucination. The core pipeline architecture is Digital Brain → Simulation → Perception → Autonomy. The book will be output as Docusaurus markdown and deployed to GitHub Pages via CI.

## Technical Context

**Language/Version**: Python (ROS2, FastAPI), C++ (ROS2), JavaScript/TypeScript (Docusaurus)
**Primary Dependencies**: ROS2 Humble/Iron, Gazebo Fortress/Garden, Unity HDRP, Isaac Sim (Omniverse), OpenAI Agents/ChatKit SDK, FastAPI, Neon Serverless Postgres, Qdrant Cloud, Docusaurus
**Storage**: Neon Serverless Postgres, Qdrant Cloud
**Testing**: ROS node command validation, physics simulation stability, VSLAM map/Nav2 path generation, VLA sequence validation, Docusaurus build/deployment
**Target Platform**: Ubuntu 22.04, NVIDIA Jetson Orin Nano/NX, Web (GitHub Pages)
**Project Type**: Web (Docusaurus static site), Robotic Control (ROS2, Isaac Sim)
**Performance Goals**: Simulation stability (60s error-free), VLA pipeline 95% accuracy, RAG chatbot 90% accuracy
**Constraints**: Exact 4-module structure, Docusaurus markdown, GitHub Pages deployment, specific SDK versions (ROS2 Humble/Iron), hardware requirements (RTX 4070 Ti+, Jetson Orin Nano/NX)
**Scale/Scope**: Complete 4-module textbook with integrated RAG chatbot and capstone project

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[Gates determined based on constitution file]

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-textbook/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# For Docusaurus
docs/ # Docusaurus content root
├── intro.md
├── module1/ # Module 1: The Robotic Nervous System (ROS 2)
│   ├── chapter1-concepts.md
│   ├── chapter2-tooling.md
│   ├── chapter3-implementation.md
│   ├── chapter4-casestudy.md
│   ├── chapter5-miniproject.md
│   └── chapter6-debugging.md
├── module2/ # Module 2: The Digital Twin (Gazebo & Unity)
│   ├── ... (similar structure)
├── module3/ # Module 3: The AI-Robot Brain (NVIDIA Isaac)
│   ├── ... (similar structure)
├── module4/ # Module 4: Vision-Language-Action (VLA)
│   ├── ... (similar structure, including capstone project)
└── deployment.md # Docusaurus to GitHub Pages deployment guide

# For RAG Chatbot
rag-chatbot/
├── backend/ # FastAPI application
│   ├── app/
│   │   ├── api/ # API routes
│   │   ├── services/ # OpenAI Agents/ChatKit SDK integration, Qdrant/Neon interaction
│   │   └── models/ # Data models
│   └── main.py
├── frontend/ # (Could be integrated into Docusaurus or a separate lightweight UI)
└── data/ # Placeholder for book content embeddings/indices
```

**Structure Decision**: The project will use a Docusaurus-based documentation structure for the main textbook content, located under a `docs/` directory. The RAG chatbot will be a separate application, potentially with a FastAPI backend and a lightweight frontend, residing in a `rag-chatbot/` directory.

## Core Pipeline Architecture

**Digital Brain → Simulation → Perception → Autonomy**

**Technical path:**
- ROS2 (Control)
- Gazebo / Unity (Physics)
- NVIDIA Isaac (Perception + Navigation)
- VLA (Task Planning)

## Book Output Format

- Docusaurus markdown
- GitHub Pages deployment
- CI pipeline auto-builds Docusaurus and publishes to the `gh-pages` branch
- Output is compiled into a Docusaurus v3 site and deployed automatically to GitHub Pages using CI. ✔️

## Core Technical Anchors

- ROS2: rclpy, Nodes, URDF, Actions
- Gazebo/Unity: URDF load, physics engine, sensors
- NVIDIA Isaac: Omniverse USD, Isaac Sim, Isaac ROS, VSLAM, Nav2
- VLA: Whisper ➜ LLM task decomposition ➜ ROS action graph

## Research Approach

- Research-concurrent writing
- Consult official docs at module boundaries
- Use simulation logs, SDK specs, hardware constraints
- Minimize theory → maximize executable examples
- Hardware-informed pedagogy
- GPU requirements (RTX, VRAM specifics)
- Edge inference (Jetson Orin family)
- Humanoid / proxy robot capability constraints

## Quality Validation Methods

- Code must execute on Ubuntu 22.04 + ROS2 Humble/Iron
- Simulation reproducibility:
    - Gazebo worlds load reliably
    - Isaac pipelines yield consistent results
- VLA task outputs deterministic enough for action execution
- Every code example runnable from scratch

**Validation questions:**
- Does the book enable a student to build a humanoid pipeline without guessing?
- Do instructions match actual SDK versions?
- Are hardware requirements realistic?
- Is every example executable?

## Decisions Requiring Documentation

- **Simulation vs. Real Robots**: What is the primary focus for examples and projects? Will real robot examples be provided as extensions, or will they be a core part of the curriculum? How will the book address the gap between simulation and reality (e.g., sensor noise, actuator dynamics)?
- **RTX Workstation vs. Cloud Isaac Sim**: What are the recommended and minimum hardware specifications for running Isaac Sim? Will the book provide guidance on using cloud-based instances of Isaac Sim as an alternative to a local RTX workstation?
- **Humanoid Body Complexity (URDF/SDF Depth)**: How complex will the reference humanoid robot model be? Will it be a simple biped with basic kinematics, or a more complex model with a full range of motion and sensor suite?
- **Jetson Deployment Limitations**: What are the specific performance limitations of running the book's examples on Jetson Orin Nano vs. Orin NX? How will the book guide users in optimizing their code for these devices?
- **Voice Autonomy vs. Button-Trigger Autonomy**: Will the capstone project rely exclusively on voice commands for triggering robot actions, or will there be alternative methods (e.g., a GUI button) for users who may not have a microphone array?
- **Nav2 Limitations for Bipeds**: How will the book address the challenges of using Nav2, which is primarily designed for wheeled robots, with bipedal platforms? Will it provide custom configurations or alternative navigation strategies?

## Tradeoff Examples

- **Gazebo vs. Isaac Sim**: Gazebo offers a faster, more lightweight simulation environment that is well-suited for basic robotics tasks and ROS integration. Isaac Sim, on the other hand, provides photorealistic rendering and advanced sensor simulation, but at the cost of requiring a powerful GPU. The choice between them will depend on the specific learning objectives of each chapter.
- **Jetson Orin Nano vs. Orin NX**: The Jetson Orin Nano is a more affordable option for edge AI, but the Orin NX provides significantly more processing power. The book will need to provide clear guidance on which device is suitable for which examples, and how to optimize code for each.
- **Quadruped Proxy vs. Humanoid Model**: A quadruped model (like Unitree Go2) is more stable and easier to control than a humanoid model, making it a good proxy for teaching many robotics concepts. However, a humanoid model is more relevant to the book's title and long-term goals. The book may use a combination of both, starting with the quadruped and progressing to the humanoid.
- **Local Control Loops vs. Cloud Inference**: Running control loops locally on the robot provides lower latency and higher reliability. However, offloading complex AI inference tasks to the cloud can enable more sophisticated capabilities. The book will explore the tradeoffs between these two approaches and provide examples of each.
- **End-to-End VLA vs. Modular ROS Pipelines**: An end-to-end Vision-Language-Action (VLA) model can offer a more seamless and integrated approach to robot control. However, a modular ROS pipeline, with separate nodes for perception, planning, and action, is more flexible and easier to debug. The book will likely teach the modular approach first, then introduce the end-to-end VLA as a more advanced topic.

## Testing Strategy

- Module 1: ROS nodes send/receive commands correctly
- Module 2: Physics sim stays stable for 60 seconds
- Module 3: VSLAM map + Nav2 generates valid path
- Module 4: Speech → Plan → Action sequence validated

## Technical details:
- Use research-concurrent approach (learn Isaac pipelines + write implementation side-by-side)
- Reference official robotics documentation (ROS2, Gazebo, Isaac, Whisper, Nav2)
- Use inline citations to original SDK documentation

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
