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

- Simulation vs real robots
- RTX workstation vs cloud Isaac Sim
- Humanoid body complexity (URDF/SDF depth)
- Jetson deployment limitations
- Voice autonomy vs button-trigger autonomy
- Nav2 limitations for bipeds

## Tradeoff Examples

- Gazebo (fast) vs Isaac (photorealistic, GPU heavy)
- Jetson Orin Nano vs Orin NX
- Quadruped proxy vs humanoid model
- Local control loops vs cloud inference
- End-to-end VLA vs modular ROS pipelines

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
