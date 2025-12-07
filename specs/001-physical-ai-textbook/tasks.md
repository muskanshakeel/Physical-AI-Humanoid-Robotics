---

description: "Task list for Physical AI & Humanoid Robotics — An AI-Native Textbook implementation"
---

# Tasks: Physical AI & Humanoid Robotics — An AI-Native Textbook

**Input**: Design documents from `/specs/001-physical-ai-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: The feature specification (spec.md) requests independent tests for each user story, therefore, test tasks will be included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus**: `docs/`
- **RAG Chatbot Backend**: `rag-chatbot/backend/app/`
- **RAG Chatbot Frontend**: `rag-chatbot/frontend/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic Docusaurus and RAG chatbot structure (FR-013)

- [ ] T001 Create Docusaurus project skeleton in `docs/`
- [ ] T002 Initialize `rag-chatbot/backend/` as a FastAPI project
- [ ] T003 Initialize `rag-chatbot/frontend/` for a lightweight UI (optional, can be integrated into Docusaurus later)
- [ ] T004 [P] Configure Docusaurus `docusaurus.config.js` for GitHub Pages deployment (FR-007, SC-006)
- [ ] T005 [P] Create initial `docs/intro.md` and `docs/deployment.md` (FR-007, SC-009)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure for RAG chatbot and Docusaurus module structure that MUST be complete before ANY user story can be implemented.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create Docusaurus module directories: `docs/module1/`, `docs/module2/`, `docs/module3/`, `docs/module4/` (FR-001, FR-014, SC-001)
- [ ] T007 Implement base RAG chatbot backend structure in `rag-chatbot/backend/app/` with `api/`, `services/`, `models/` directories (FR-006)
- [ ] T008 Configure Neon Serverless Postgres and Qdrant Cloud client connections in `rag-chatbot/backend/app/services/` (FR-006)
- [ ] T009 Create base data models for RAG chatbot (e.g., `Query`, `Answer`, `Context`) in `rag-chatbot/backend/app/models/`
- [ ] T010 Implement FastAPI `main.py` in `rag-chatbot/backend/` with basic routing setup

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Learning Core Robotics Concepts (Priority: P1) 🎯 MVP

**Goal**: Users understand fundamental ROS 2 concepts and apply them in simple examples within Module 1.

**Independent Test**: Successfully run the provided ROS 2 mini-project in `docs/module1/chapter5-miniproject.md`, demonstrating comprehension of core concepts (SC-002, SC-010).

### Implementation for User Story 1

- [ ] T011 [US1] Create `docs/module1/chapter1-concepts.md` (ROS 2 concepts: rclpy, Nodes, Topics, Services, Actions, URDF) (FR-002, FR-014)
- [ ] T012 [US1] Create `docs/module1/chapter2-tooling.md` (ROS 2 tooling, e.g., `ros2 run`, `ros2 topic`, `rviz`) (FR-002, FR-014)
- [ ] T013 [US1] Create `docs/module1/chapter3-implementation.md` (ROS 2 basic node implementation walkthrough) (FR-002, FR-003, FR-014)
- [ ] T014 [US1] Create `docs/module1/chapter4-casestudy.md` (ROS 2 simple robotic arm control example) (FR-002, FR-003, FR-014)
- [ ] T015 [US1] Create `docs/module1/chapter5-miniproject.md` (ROS 2 mini-project: basic mobile robot navigation) (FR-002, FR-003, FR-014)
- [ ] T016 [US1] Create `docs/module1/chapter6-debugging.md` (ROS 2 debugging & common failures) (FR-002, FR-014)
- [ ] T017 [US1] Ensure all code examples in `docs/module1/` are executable on Ubuntu 22.04 with ROS2 Humble/Iron (FR-003, SC-002)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Exploring Digital Twins (Priority: P1)

**Goal**: Users learn about robotic simulation using Gazebo and Unity as digital twins.

**Independent Test**: Successfully load and interact with a simulated robot in both Gazebo and Unity, following Module 2's "Implementation walkthrough" (`docs/module2/chapter3-implementation.md`) (SC-003, SC-010).

### Implementation for User Story 2

- [ ] T018 [US2] Create `docs/module2/chapter1-concepts.md` (Digital twin concepts, Gazebo/Unity basics, physics simulation) (FR-002, FR-014)
- [ ] T019 [US2] Create `docs/module2/chapter2-tooling.md` (Gazebo/Unity simulation tooling, URDF/SDF integration) (FR-002, FR-014)
- [ ] T020 [US2] Create `docs/module2/chapter3-implementation.md` (Gazebo/Unity robot model loading and interaction walkthrough) (FR-002, FR-003, FR-014)
- [ ] T021 [US2] Create `docs/module2/chapter4-casestudy.md` (Gazebo/Unity sensor simulation and data publishing example) (FR-002, FR-003, FR-014)
- [ ] T022 [US2] Create `docs/module2/chapter5-miniproject.md` (Gazebo/Unity mini-project: environment interaction) (FR-002, FR-003, FR-014)
- [ ] T023 [US2] Create `docs/module2/chapter6-debugging.md` (Gazebo/Unity simulation debugging & common failures) (FR-002, FR-014)
- [ ] T024 [US2] Ensure all code examples and simulation worlds in `docs/module2/` are executable/loadable on Ubuntu 22.04 (FR-003, SC-002, SC-003)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Integrating AI with Robotics (Priority: P1)

**Goal**: Users understand how NVIDIA Isaac enables advanced AI-robot interactions and apply it to tasks like VSLAM.

**Independent Test**: Successfully deploy and run an Isaac ROS application for VSLAM on an NVIDIA Jetson, following Module 3's "Implementation walkthrough" (`docs/module3/chapter3-implementation.md`) (SC-002, SC-007, SC-010).

### Implementation for User Story 3

- [ ] T025 [US3] Create `docs/module3/chapter1-concepts.md` (NVIDIA Isaac Sim/ROS concepts, Omniverse USD, VSLAM, Nav2 basics) (FR-002, FR-014)
- [ ] T026 [US3] Create `docs/module3/chapter2-tooling.md` (NVIDIA Isaac tooling, Jetson setup) (FR-002, FR-014)
- [ ] T027 [US3] Create `docs/module3/chapter3-implementation.md` (Isaac ROS VSLAM implementation walkthrough on Jetson) (FR-002, FR-003, FR-014)
- [ ] T028 [US3] Create `docs/module3/chapter4-casestudy.md` (Isaac Sim + Nav2 integration for path planning example) (FR-002, FR-003, FR-014)
- [ ] T029 [US3] Create `docs/module3/chapter5-miniproject.md` (Isaac Sim mini-project: object detection and manipulation) (FR-002, FR-003, FR-014)
- [ ] T030 [US3] Create `docs/module3/chapter6-debugging.md` (NVIDIA Isaac debugging & common failures) (FR-002, FR-014)
- [ ] T031 [US3] Ensure all code examples and simulation scenes in `docs/module3/` are executable/loadable (FR-003, SC-002, SC-003)

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Building a Vision-Language-Action Pipeline (Priority: P1)

**Goal**: Users build a complete Vision-Language-Action (VLA) pipeline for autonomous humanoid robotics, from voice command to robot manipulation.

**Independent Test**: Successfully implement the Module 4 "Capstone Project" (`docs/module4/chapter6-casestudy.md` as capstone) to enable an autonomous humanoid robot to respond to voice commands by planning and executing ROS 2 actions (SC-005, SC-008, SC-010).

### Implementation for User Story 4

- [ ] T032 [US4] Create `docs/module4/chapter1-concepts.md` (VLA pipeline: Whisper, LLM planning, ROS2 action graph) (FR-002, FR-014)
- [ ] T033 [US4] Create `docs/module4/chapter2-tooling.md` (VLA specific tooling: Whisper setup, LLM API integration) (FR-002, FR-014)
- [ ] T034 [US4] Create `docs/module4/chapter3-implementation.md` (Whisper to text processing walkthrough) (FR-002, FR-003, FR-014)
- [ ] T035 [US4] Create `docs/module4/chapter4-casestudy.md` (LLM-based task decomposition and ROS2 action graph generation) (FR-002, FR-003, FR-014)
- [ ] T036 [US4] Create `docs/module4/chapter5-miniproject.md` (VLA mini-project: simple voice command to robot action) (FR-002, FR-003, FR-014)
- [ ] T037 [US4] Create `docs/module4/chapter6-casestudy.md` (Capstone Project: Autonomous Humanoid (Voice → Plan → Navigate → Detect → Manipulate)) (FR-008, FR-002, FR-003, FR-014)
- [ ] T038 [US4] Create `docs/module4/chapter7-debugging.md` (VLA pipeline debugging & common failures) (FR-002, FR-014)
- [ ] T039 [US4] Ensure all code examples and VLA sequences in `docs/module4/` are executable/validatable (FR-003, SC-002, SC-005, SC-008)

---

## Phase 7: User Story 5 - Interacting with the RAG Chatbot (Priority: P1)

**Goal**: Users ask questions about the textbook content and receive accurate, context-aware answers via an integrated RAG chatbot.

**Independent Test**: Interact with the chatbot, asking questions about both the entire book and selected text, and verify the accuracy and relevance of the responses (SC-004).

### Implementation for User Story 5

- [ ] T040 [US5] Implement FastAPI endpoint for RAG chatbot query (`rag-chatbot/backend/app/api/rag.py`) (FR-006)
- [ ] T041 [US5] Implement service to embed book content and store in Qdrant (`rag-chatbot/backend/app/services/embedding_service.py`) (FR-004)
- [ ] T042 [US5] Implement service to retrieve relevant content from Qdrant based on query (`rag-chatbot/backend/app/services/retrieval_service.py`) (FR-004, FR-005)
- [ ] T043 [US5] Integrate OpenAI Agents/ChatKit SDK for RAG generation logic (`rag-chatbot/backend/app/services/rag_service.py`) (FR-004, FR-005)
- [ ] T044 [US5] Define data models for RAG requests/responses in `rag-chatbot/backend/app/models/rag_models.py`
- [ ] T045 [US5] Implement a lightweight frontend in `rag-chatbot/frontend/` to interact with the RAG backend (display chat interface)
- [ ] T046 [US5] Develop a process to load all `docs/` content into the RAG chatbot's knowledge base (`rag-chatbot/data/ingest.py`) (FR-004)
- [ ] T047 [US5] Implement logic for context-aware queries based on selected text (FR-005)
- [ ] T048 [US5] Ensure RAG chatbot returns accurate and relevant answers with citations (SC-004)

---

## Phase 8: Bonus Features (Optional, if requested)

**Purpose**: Implementation of optional features as defined in the spec.

- [ ] T049 [P] [Optional] Implement Claude Code subagents and reusable Agent Skills (FR-009)
- [ ] T050 [P] [Optional] Implement Signup/Signin using BetterAuth (FR-010)
- [ ] T051 [P] [Optional] Implement per-chapter personalization button (FR-011)
- [ ] T052 [P] [Optional] Implement Urdu translation toggle per chapter (FR-012)

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and final validation.

- [ ] T053 [P] Review and standardize Docusaurus navigation and table of contents across all modules.
- [ ] T054 [P] Ensure all inline citations to SDK documentation are present and correctly formatted (SC-010).
- [ ] T055 Code cleanup and refactoring across Docusaurus and RAG chatbot codebases.
- [ ] T056 Performance optimization for Docusaurus build and RAG chatbot response times.
- [ ] T057 Security hardening for RAG chatbot API.
- [ ] T058 Final validation of all Docusaurus site deployments to GitHub Pages (SC-006).
- [ ] T059 Final reproducibility checks for all simulation environments (Gazebo, Isaac Sim) (SC-003).

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
- **User Stories (Phase 3-7)**: All depend on Foundational phase completion.
  - User stories can then proceed in parallel (if staffed).
  - Or sequentially in priority order (P1 → P2 → P3).
- **Bonus Features (Phase 8)**: Depend on relevant core user story implementations.
- **Polish (Phase 9)**: Depends on all desired user stories and bonus features being complete.

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 3 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories.
- **User Story 4 (P1)**: Can start after Foundational (Phase 2) - Integrates concepts from US1, US2, US3 but should be independently testable.
- **User Story 5 (P1)**: Can start after Foundational (Phase 2) - Dependent on book content from US1-US4 for its knowledge base.

### Within Each User Story

- Content creation (chapters) before integration tasks.
- Models before services for RAG chatbot.
- Services before API endpoints for RAG chatbot.
- Core implementation before integration.
- Story complete before moving to next priority.

### Parallel Opportunities

- All Setup tasks (Phase 1) marked [P] can run in parallel.
- Once Foundational phase (Phase 2) completes, all user stories (Phases 3-7) can start in parallel (if team capacity allows).
- Within each user story, chapter creation can be parallelized.
- For the RAG chatbot (US5), embedding, retrieval, and RAG generation services can be developed in parallel initially.
- Bonus features (Phase 8) are largely independent and can be parallelized.
- Polish tasks (Phase 9) marked [P] can run in parallel.

---

## Parallel Example: User Story 1 (Phase 3)

```bash
# Example for Module 1 chapter creation
Task: "Create docs/module1/chapter1-concepts.md"
Task: "Create docs/module1/chapter2-tooling.md"
Task: "Create docs/module1/chapter3-implementation.md"
Task: "Create docs/module1/chapter4-casestudy.md"
Task: "Create docs/module1/chapter5-miniproject.md"
Task: "Create docs/module1/chapter6-debugging.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 1
4.  **STOP and VALIDATE**: Test User Story 1 independently
5.  Deploy/demo if ready

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3.  Add User Story 2 → Test independently → Deploy/Demo
4.  Add User Story 3 → Test independently → Deploy/Demo
5.  Add User Story 4 → Test independently → Deploy/Demo (Capstone!)
6.  Add User Story 5 → Test independently → Deploy/Demo (RAG Chatbot!)
7.  Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together.
2.  Once Foundational is done:
    -   Developer A: User Story 1 (Module 1 content + examples)
    -   Developer B: User Story 2 (Module 2 content + examples)
    -   Developer C: User Story 3 (Module 3 content + examples)
    -   Developer D: User Story 5 (RAG Chatbot backend/frontend)
3.  Developer E (or A/B/C): User Story 4 (Module 4 content + capstone, after initial modules are in progress)
4.  Stories complete and integrate independently.

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
