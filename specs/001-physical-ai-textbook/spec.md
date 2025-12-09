# Feature Specification: Physical AI & Humanoid Robotics — An AI-Native Textbook

**Feature Branch**: `001-physical-ai-textbook`
**Created**: 2025-12-06
**Status**: Ready
**Input**: User description: "/sp.specify

Title: Physical AI & Humanoid Robotics — An AI-Native Textbook

Goal:
Generate a complete, accurate, reproducible, 4-module specification for the Physical AI & Humanoid Robotics textbook. This book must be written in Docusaurus markdown and deployed to GitHub Pages. It must include integrated RAG chatbot features and optional personalization/translation extensions.

Deliverables Required by Spec:
1. Full book architecture with EXACTLY 4 Modules:
   - Module 1: The Robotic Nervous System (ROS 2)
   - Module 2: The Digital Twin (Gazebo & Unity)
   - Module 3: The AI-Robot Brain (NVIDIA Isaac)
   - Module 4: Vision-Language-Action (VLA)

2. Each module must follow a consistent structure:
   1. Concepts
   2. Tooling
   3. Implementation walkthrough
   4. Case study / example
   5. Mini project
   6. Debugging & common failures

3. Technical Anchors:
   - ROS2: rclpy, Nodes, Topics, Services, Actions, URDF
   - Gazebo/Unity: physics simulation, sensor simulation. **Guidance**: Gazebo should be used for examples focusing on core ROS 2 integration and simple physics. Unity should be used for examples requiring advanced rendering, complex sensor simulation, or benefiting from the NVIDIA Isaac Sim integration.
   - NVIDIA Isaac: Isaac Sim, Isaac ROS, VSLAM, Nav2, USD assets
   - VLA: Whisper → LLM planning → ROS2 action graph

4. Actual SDKs used:
   - ROS 2 Humble / Iron (Ubuntu 22.04)
   - Gazebo Fortress / Garden
   - Unity HDRP
   - Isaac Sim (Omniverse)
   - NVIDIA Jetson Orin Nano/NX for edge deployment

5. Hardware References:
   - Workstation: RTX 4070 Ti+ (ideal 3090/4090)
   - Jetson Orin Nano/NX (8GB–16GB) with JetPack 5.1+
   - RealSense D435i/D455 + IMU sensors (Firmware v5.14.0.0 recommended)
   - Microphone array for Whisper (e.g., ReSpeaker Mic Array v2.0)
   - Robot options: Unitree Go2 (Firmware v1.1+), OP3, Unitree G1

### Software Bill of Materials
All code examples must be compatible with the following core software versions. A more detailed package-level dependency list will be maintained in `requirements.txt` (Python) and `package.json` (Node.js) files within the repository.
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill (or Iron Irwini, examples must specify)
- **Gazebo**: Fortress (or Garden, examples must specify)
- **Unity**: 2022.3 LTS with HDRP
- **NVIDIA Isaac Sim**: 2023.1.1+
- **NVIDIA JetPack**: 5.1+
- **Python**: 3.10
- **Node.js**: 18.x

6. RAG Chatbot Requirements:
   - Architecture: OpenAI Agents/ChatKit SDK + FastAPI
   - Storage: Neon Serverless Postgres + Qdrant Cloud (Free Tier)
   - Must answer questions based on:
       a) whole book
       b) selected text only
   - **Clarification on "Selected Text"**: When a user highlights text, the frontend should pass the highlighted text block to the API. The RAG service should use this text as the primary, high-priority context for answering the query.
   - **Clarification on "Relevant and Cited"**: A "relevant" answer directly addresses the user's query. A "cited" answer must include a direct quote or a summary from the source text and a clear link or reference to the source chapter or section.
   - **API Routes and "Minimal Implementation Outline"**: The implementation outline must include:
       - A FastAPI backend with a `/query` endpoint that accepts a JSON payload with `query_text` and optional `context_text` (for selected text).
       - A service layer that orchestrates calls to Qdrant (for retrieval) and OpenAI (for generation).
       - Pydantic models for request and response validation.
       - A basic `data_ingestion` script to process the book's markdown files, create embeddings, and store them in Qdrant.

7. Bonus Features (include in spec but mark optional):
   - Claude Code subagents and reusable Agent Skills (+50 pts)
   - Signup/Signin using BetterAuth (+50 pts)
   - Per-chapter personalization button (+50 pts)
   - Urdu translation toggle per chapter (+50 pts)

8. Quality Validation:
   - Every code example must be executable on Ubuntu 22.04 (ROS2 Humble/Iron).
   - Reproducibility checks for Gazebo and Isaac Sim scenes.
   - VSLAM + Nav2 validations.
   - VLA pipeline validated with deterministic text → plan → ROS action graph.
   - All simulation worlds must load without errors for at least 60 seconds.

9. Research Approach:
   - Research-concurrent writing: Check official SDK docs at boundaries.
   - Hardware-informed pedagogy.
   -   **Inline citations to SDK docs**: Citations should follow the format `[Source Name, Year]` and link to the official documentation where possible. A bibliography should be included at the end of each module.
      -   **Minimize theory; maximize runnable examples**: The principle of "minimize theory, maximize runnable examples" should be interpreted as a target of at least one runnable code example for every major theoretical concept introduced.

10. Output Requirements:
   - Provide a complete book skeleton organized into Docusaurus folders.
   - Each module must contain a minimum of 4–7 chapters.
   - Include capstone project in Module 4: Autonomous Humanoid (Voice → Plan → Navigate → Detect → Manipulate).
   - Include deployment section: Docusaurus → GitHub Pages.

11. Final Output Format:
   - Produce a full Spec-Kit Plus specification file.
   - Use clean headings, no filler, no hallucination.
   - Make all instructions executable and verifiable.
   - Ensure accuracy using a standard similar to the Claude Constitution.

Instructions to Spec-Kit:
Use this prompt to generate:
- The full book specification
- Module-level specs
- Chapter-level specs
- System architecture diagrams in text form
- Validation checklists
- Docusaurus-ready directory structure
- RAG architecture spec

No deviations from the 4-module structure allowed."

## User Scenarios & Testing *(mandatory)*

### User Personas
While the primary target is a developer with some programming experience, the content should be accessible to multiple user levels.
- **Beginner**: A student or hobbyist new to robotics but with basic Python knowledge. The "Concepts" and "Walkthrough" sections are primarily for them.
- **Intermediate**: A software developer looking to move into the robotics field. The "Case Studies" and "Mini Projects" are designed to challenge them.
- **Expert**: An experienced robotics professional looking to quickly learn a new SDK. The runnable examples and clear API documentation are for them.

### User Story 1 - Learning Core Robotics Concepts (Priority: P1)

Users want to understand fundamental ROS 2 concepts and apply them in simple examples within Module 1.

**Why this priority**: Foundational knowledge for physical AI and robotics.

**Independent Test**: Can be fully tested by following Module 1's "Implementation walkthrough" and successfully running the provided ROS 2 mini-project, demonstrating comprehension of core concepts.

**Acceptance Scenarios**:

1.  **Given** a new learner, **When** they read Module 1 "Concepts" and "Implementation walkthrough", **Then** they can successfully run the provided ROS 2 mini-project.
2.  **Given** a learner encounters a common issue in Module 1, **When** they consult "Debugging & common failures", **Then** they can resolve the issue.

---

### User Story 2 - Exploring Digital Twins (Priority: P1)

Users want to learn about robotic simulation using Gazebo and Unity as digital twins.

**Why this priority**: Essential for safe and efficient development of physical AI systems.

**Independent Test**: Can be fully tested by following Module 2's "Implementation walkthrough" to successfully load and interact with a simulated robot in both Gazebo and Unity.

**Acceptance Scenarios**:

1.  **Given** a user with a workstation, **When** they follow Module 2 "Implementation walkthrough", **Then** they can successfully load and interact with a simulated robot in Gazebo and Unity.
2.  **Given** a user wants to explore sensor simulation, **When** they review Module 2 "Tooling", **Then** they can configure and use simulated sensors.

---

### User Story 3 - Integrating AI with Robotics (Priority: P1)

Users want to understand how NVIDIA Isaac enables advanced AI-robot interactions and apply it to tasks like VSLAM.

**Why this priority**: Core to the "AI-Robot Brain" concept and practical application of AI in robotics.

**Independent Test**: Can be fully tested by following Module 3's "Implementation walkthrough" to deploy and run an Isaac ROS application for VSLAM on an NVIDIA Jetson.

**Acceptance Scenarios**:

1.  **Given** a user with an NVIDIA Jetson, **When** they follow Module 3 "Implementation walkthrough", **Then** they can deploy and run an Isaac ROS application for VSLAM.
2.  **Given** a user needs to understand USD assets, **When** they refer to Module 3 "Concepts", **Then" they can grasp the basics of USD in Isaac Sim.

---

### User Story 4 - Building a Vision-Language-Action Pipeline (Priority: P1)

Users want to build a complete Vision-Language-Action (VLA) pipeline for autonomous humanoid robotics, from voice command to robot manipulation.

**Why this priority**: The capstone project and ultimate goal of integrating the concepts from previous modules.

**Independent Test**: Can be fully tested by successfully implementing the Module 4 "Capstone Project" to enable an autonomous humanoid robot to respond to voice commands by planning and executing ROS 2 actions.

**Acceptance Scenarios**:

1.  **Given** a user with a microphone array and a robot (simulated or real), **When** they complete the Module 4 "Capstone Project", **Then** the robot can respond to voice commands by planning and executing ROS 2 actions (Navigate, Detect, Manipulate).
2.  **Given** a user wants to understand the VLA flow, **When** they read Module 4 "Concepts", **Then** they can explain the Whisper → LLM planning → ROS2 action graph.

---

### User Story 5 - Interacting with the RAG Chatbot (Priority: P1)

Users want to ask questions about the textbook content and receive accurate, context-aware answers via an integrated RAG chatbot.

**Why this priority**: Enhances the learning experience by providing immediate answers and clarification.

**Independent Test**: Can be fully tested by interacting with the chatbot, asking questions about both the entire book and selected text, and verifying the accuracy and relevance of the responses.

**Acceptance Scenarios**:

1.  **Given** a user viewing any chapter, **When** they ask a question about the whole book via the RAG chatbot, **Then** they receive a relevant and cited answer.
2.  **Given** a user highlights a section of text, **When** they ask a question about the selected text via the RAG chatbot, **Then** they receive a relevant and cited answer.

---

### Edge Cases

-   **Hardware Incompatibility**: What happens when a user's workstation or Jetson hardware does not meet the specified minimum requirements? (The book should provide guidance on scaled-down examples or alternative approaches for lower-spec hardware, or clearly state minimums and potential limitations.)
-   **Network Latency/Connectivity**: How does the RAG chatbot handle issues with network latency or intermittent connectivity to cloud services (e.g., OpenAI, Neon, Qdrant)? (The chatbot should implement graceful degradation, retry mechanisms, and informative error messages.)
-   **SDK Versioning**: What happens if an SDK (e.g., ROS 2, Gazebo, Isaac Sim) receives a major update or becomes deprecated after publication? (The book should specify exact versions used and provide guidance or links to resources for updating examples to newer versions.)
-   **Out-of-Scope and Unanswerable Chatbot Queries**: How does the RAG chatbot respond to questions that are not directly answerable from the textbook content? (The chatbot should clearly state that it can only answer questions based on the provided book content. If a query returns no relevant context from the knowledge base, the chatbot must respond with a message like "I could not find a relevant answer in the textbook for your question.")
-   **Reproducibility Across Environments**: How is reproducibility ensured for complex simulation setups (Gazebo, Isaac Sim scenes) across different Ubuntu 22.04 environments? (The book should provide Dockerfiles, environment setup scripts, or detailed installation instructions to maximize reproducibility. A `Dockerfile` defining the 'Clean Ubuntu Environment' is a mandatory deliverable.)
-   **Poor Performance on Minimum-Spec Hardware**: What happens if a user meets the minimum hardware requirements but experiences poor performance (e.g., low simulation frame rate)? (The introduction to hardware-intensive modules (e.g., Module 3) must include a warning note explaining that minimum hardware may result in slow performance and should recommend closing other applications.)

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The textbook MUST be structured into exactly 4 modules: "Module 1: The Robotic Nervous System (ROS 2)", "Module 2: The Digital Twin (Gazebo & Unity)", "Module 3: The AI-Robot Brain (NVIDIA Isaac)", and "Module 4: Vision-Language-Action (VLA)".
-   **FR-002**: Each module MUST follow a consistent structure: Concepts, Tooling, Implementation walkthrough, Case study / example, Mini project, Debugging & common failures.

### Module Section Details
To ensure consistency, each of the 6 sections within a module must adhere to the following content requirements:
- **1. Concepts**: Introduce the core theoretical concepts for the chapter's topic. Should be concise and linked to practical application.
- **2. Tooling**: Describe the key software and hardware tools relevant to the chapter. Include installation and basic configuration instructions.
- **3. Implementation Walkthrough**: A step-by-step guide to implementing a small-scale example of the concept. Code should be fully explained.
- **4. Case Study / Example**: A slightly more advanced, real-world-adjacent example that builds on the walkthrough.
- **5. Mini Project**: A hands-on project for the reader to complete, with clear objectives and validation steps, but less step-by-step guidance than the walkthrough.
- **6. Debugging & Common Failures**: Must list at least 3 common errors or failure modes a user might encounter for the chapter's topic. For each, it must provide clear steps to diagnose and resolve the issue.

-   **FR-003**: Every code example in the textbook MUST be executable on Ubuntu 22.04 (ROS2 Humble/Iron).
-   **FR-004**: The integrated RAG chatbot MUST answer questions based on the entire book content.
-   **FR-005**: The integrated RAG chatbot MUST answer questions based on selected text within a chapter.
-   **FR-006**: The RAG chatbot functionality MUST include clearly defined API routes and a minimal implementation outline.
-   **FR-007**: The book content and structure MUST be deployable as a static site using Docusaurus to GitHub Pages.
-   **FR-008**: Module 4 MUST include a capstone project titled "Autonomous Humanoid (Voice → Plan → Navigate → Detect → Manipulate)".
-   **FR-009 (Optional)**: The system MAY include Claude Code subagents and reusable Agent Skills.
-   **FR-010 (Optional)**: The system MAY include Signup/Signin using BetterAuth.
-   **FR-011 (Optional)**: The system MAY include a per-chapter personalization button.
-   **FR-012 (Optional)**: The system MAY include an Urdu translation toggle per chapter.
-   **FR-013**: The book MUST provide a complete Docusaurus-ready directory structure skeleton.
-   **FR-014**: Each module MUST contain a minimum of 4–7 chapters.
-   **FR-015**: A glossary of common terms and acronyms MUST be provided as a dedicated page in the Docusaurus site.
-   **FR-016 (Optional)**: The RAG chatbot backend SHOULD log anonymized user queries and the chatbot's responses to a dedicated table for analysis and improvement purposes. No personally identifiable information should be stored.
-   **FR-017**: A documented process and associated scripts MUST be provided to allow for updating the RAG chatbot's knowledge base when the textbook content changes.
-   **FR-018**: To facilitate navigation between theory and practice, every code example or simulation mentioned MUST include a direct hyperlink to the corresponding code or asset in the repository.

### Definitions
- **Chapter**: A "chapter" is a single markdown file within a module directory. It should be between 500 and 3000 words. Each chapter should focus on a specific, narrow topic.
- **Minimize Theory Principle**: The principle of "minimize theory, maximize runnable examples" should be interpreted as a target of at least one runnable code example for every major theoretical concept introduced.
- **Clean Ubuntu 22.04 Environment**: This refers to a base installation of Ubuntu 22.04 Desktop, with `git`, `python3`, and `pip` installed, plus the specific SDKs mentioned in this spec (ROS 2 Humble/Iron, etc.). To ensure reproducibility, a `Dockerfile` will be provided in the repository root to define this environment.
- **Simulation Stability**: Beyond loading for 60 seconds, "simulation stability" means that the simulation runs without physics-related errors (e.g., model explosions, joint failures) and that sensor data streams are continuous and within expected ranges for the duration of a given test or example.

### Key Entities *(include if feature involves data)*

-   **Module**: A top-level organizational unit of the textbook.
    -   Key Attributes: Title (e.g., "The Robotic Nervous System"), Ordered Sub-sections (Concepts, Tooling, Implementation walkthrough, Case study / example, Mini project, Debugging & common failures), Associated Technical Anchors (e.g., ROS2: rclpy, Nodes), Number of Chapters (4-7 minimum).
-   **Chapter**: A discrete learning unit within a Module.
    -   Key Attributes: Title, Markdown Content, Code Examples (executable), Inline Citations to SDK docs, Optional Personalization Data, Optional Translation Data.
-   **RAG Chatbot**: An AI-powered conversational interface for the textbook.
    -   Key Attributes: Query (user input text), Context (scope: whole book or selected text), Answer (chatbot's response), Sources (citations/references from the book), API Routes, Implementation Outline (FastAPI, OpenAI Agents/ChatKit SDK, Neon Serverless Postgres, Qdrant Cloud).
-   **User**: An individual interacting with the textbook and RAG chatbot.
    -   Key Attributes: Input Queries, Selected Text, Personalization Preferences, Language Preference.
-   **Code Example**: A runnable code snippet demonstrating a concept.
    -   Key Attributes: Source Code, Executability (on Ubuntu 22.04), Reproducibility Checks.
-   **Simulation World**: A virtual environment for robotics.
    -   Key Attributes: Platform (Gazebo/Unity/Isaac Sim), Scene Assets (USD), Reproducibility Checks, Load Stability (60 seconds error-free).

### Non-Functional Requirements

-   **NFR-001 (Performance)**:
    -   **Site Load Speed**: The Docusaurus site must achieve a Google PageSpeed Insights score of 85 or higher for desktop on its main landing pages.
    -   **Build Time**: The Docusaurus production build (`npm run build`) must complete in under 5 minutes on a standard GitHub Actions runner.
    -   **RAG Chatbot Response Time**: The RAG chatbot API must provide a response (p95) in under 3.0 seconds for typical queries.
-   **NFR-002 (Accessibility)**:
    -   **WCAG Compliance**: All web-based content, including the Docusaurus site and any frontend components for the RAG chatbot, must meet Web Content Accessibility Guidelines (WCAG) 2.1 Level AA.
    -   **Keyboard Navigation**: All interactive UI elements must be fully navigable and operable using only a keyboard, in a logical order.
-   **NFR-003 (Security)**:
    -   **API Security**: The FastAPI backend for the RAG chatbot must implement the following:
        -   **Rate Limiting**: A limit of 60 requests per minute per IP address to prevent abuse.
        -   **Input Sanitization**: All user-provided input must be sanitized to prevent common injection attacks (e.g., XSS).
    -   **Secret Management**: No secrets (API keys, database credentials) shall be hardcoded in the source code; they must be loaded from environment variables.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: All 4 modules are present, and each contains the specified 6 sub-sections, along with a minimum of 4 chapters, organized within the Docusaurus directory structure.
-   **SC-002**: 100% of all code examples provided in the textbook successfully compile and execute on a clean Ubuntu 22.04 environment configured for ROS 2 Humble/Iron, Gazebo Fortress/Garden, Unity HDRP, and NVIDIA Isaac (Sim, ROS).
-   **SC-003**: All Gazebo and Isaac Sim scenes described in the book load without errors for at least 60 seconds and pass their respective reproducibility checks.
-   **SC-004**: The RAG chatbot, when queried with a test set of 100 questions derived from the book content, returns accurate and relevant answers with accompanying citations for at least 90% of queries. **Measurability**: This will be tested against a predefined, version-controlled test suite of 100 questions (`/tests/rag_qa_suite.json`). An answer is "accurate" if it correctly reflects the textbook content and "relevant" if it addresses the user's query. A human evaluator will make the final judgment for this test.
-   **SC-005**: The VLA pipeline, as demonstrated in the Module 4 capstone project, consistently translates a predefined set of voice commands into the correct, deterministic ROS action graphs with 95% accuracy in a simulated environment. **Measurability**: This will be tested against a version-controlled set of 20 voice commands. A "correct" graph is one that matches the pre-defined, expected graph for that command.
-   **SC-006**: The Docusaurus site successfully deploys to GitHub Pages via the documented deployment process, and all content is accessible and rendered correctly.
-   **SC-007**: VSLAM and Nav2 validations, as described in Module 3, pass with expected performance metrics.
-   **SC-008**: The VLA pipeline demonstrates validated text-to-plan-to-ROS action graph conversions with deterministic outcomes for defined inputs.
-   **SC-009**: The book skeleton includes a deployment section detailing Docusaurus to GitHub Pages.
-   **SC-010**: All technical anchors (ROS2, Gazebo/Unity, NVIDIA Isaac, VLA specifics) are addressed and exemplified within the respective modules.

### Success Criteria for Optional Features
- **SC-OPT-001 (Claude Code Subagents)**: If implemented, the system must demonstrate at least one reusable Agent Skill being successfully used to generate a code snippet for a mini-project.
- **SC-OPT-002 (BetterAuth)**: If implemented, a user must be able to sign up, sign in, and have their session persist across the site.
- **SC-OPT-003 (Personalization)**: If implemented, clicking the personalization button must demonstrably alter the content on the page (e.g., switching between a beginner and expert explanation).
- **SC-OPT-004 (Urdu Translation)**: If implemented, toggling the translation must translate at least 95% of the textual content of a chapter into legible Urdu.