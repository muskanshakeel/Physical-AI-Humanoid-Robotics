---
id: 1
title: Update RAG Chatbot Constitution
stage: constitution
date: 2025-12-14
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "rag", "chatbot", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
Project: Integrated RAG Chatbot for Published Book

Core Principles:
- Accuracy: All responses must be strictly based on the book content or user-selected text. No hallucination.
- Clarity: Answers must be understandable for a technically literate audience (AI/Robotics enthusiasts or researchers).
- Consistency: Chatbot behavior must be consistent across similar questions.
- Security: All user data (queries, selections) must be stored securely in Neon Serverless Postgres.
- Scalability: Design for efficient vector search and retrieval using Qdrant Cloud.
- Professionalism: Maintain clean code, modular architecture, and proper API design.

Key Standards:
- Data sources: Only book content (including user-selected text), no external information unless explicitly allowed.
- Embedding & Retrieval: Use Qdrant Cloud Free Tier for vector embeddings; ensure efficient similarity search.
- Backend: FastAPI must handle queries, context retrieval, and AI response generation reliably.
- AI Model: Use Qwen via Qwen CLI or OpenAI Agents/ChatKit SDK for response generation.
- Logging: All user queries and responses must be logged with timestamps.
- Testing: End-to-end testing of chatbot interaction must be done before deployment.

Constraints:
- Response latency: ≤ 2 seconds per query under normal load.
- Data storage: Use Neon Serverless Postgres for chat history and session data.
- Compliance: Follow privacy standards for storing user-selected text.
- Deployment: Must be fully embedded into the published book (e.g., Docusaurus integration).

Success Criteria:
- Chatbot only answers based on book content or user-selected text.
- Stable, fast responses with minimal errors.
- Fully functional embedded interface in the book.
- Secure storage of user interactions.
- Proper vector search accuracy (retrieved content matches query context).
```

## Response snapshot

Updated the constitution file with the specific requirements for the Integrated RAG Chatbot for Published Book project, incorporating all the core principles and standards into the template.

## Outcome

- ✅ Impact: Updated project constitution with 6 core principles and technical standards
- 🧪 Tests: none
- 📁 Files: Updated .specify/memory/constitution.md with new project constitution
- 🔁 Next prompts: Implementation planning and feature specification
- 🧠 Reflection: Constitution now properly reflects the RAG chatbot project requirements

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - Constitution properly updated with all required principles
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Begin feature specification for the RAG chatbot