<!-- SYNC IMPACT REPORT
Version change: 1.0.0 → 1.1.0
Modified principles:
- PRINCIPLE_1_NAME: Placeholder → Accuracy
- PRINCIPLE_2_NAME: Placeholder → Clarity
- PRINCIPLE_3_NAME: Placeholder → Consistency
- PRINCIPLE_4_NAME: Placeholder → Security
- PRINCIPLE_5_NAME: Placeholder → Scalability
- PRINCIPLE_6_NAME: Placeholder → Professionalism
Added sections: None
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ⚠ pending
- .specify/templates/spec-template.md ⚠ pending
- .specify/templates/tasks-template.md ⚠ pending
Follow-up TODOs: None
-->

# Integrated RAG Chatbot for Published Book Constitution

## Core Principles

### Accuracy
All responses must be strictly based on the book content or user-selected text. No hallucination is allowed in any generated responses.

### Clarity
Answers must be understandable for a technically literate audience (AI/Robotics enthusiasts or researchers) while maintaining accessibility.

### Consistency
Chatbot behavior must be consistent across similar questions, ensuring predictable and reliable user experience.

### Security
All user data (queries, selections) must be stored securely in Neon Serverless Postgres with proper access controls and encryption.

### Scalability
Design for efficient vector search and retrieval using Qdrant Cloud to handle increasing loads and maintain performance standards.

### Professionalism
Maintain clean code, modular architecture, and proper API design throughout the entire development lifecycle.

## Data and Technical Standards
Data sources: Only book content (including user-selected text), no external information unless explicitly allowed. Embedding & Retrieval: Use Qdrant Cloud Free Tier for vector embeddings; ensure efficient similarity search. Backend: FastAPI must handle queries, context retrieval, and AI response generation reliably. AI Model: Use Qwen via Qwen CLI or OpenAI Agents/ChatKit SDK for response generation. Logging: All user queries and responses must be logged with timestamps for audit and analysis.

## Development and Testing Standards
Testing: End-to-end testing of chatbot interaction must be done before deployment. Response latency: ≤ 2 seconds per query under normal load. Data storage: Use Neon Serverless Postgres for chat history and session data. Compliance: Follow privacy standards for storing user-selected text. Deployment: Must be fully embedded into the published book (e.g., Docusaurus integration).

## Governance
This constitution governs all development decisions for the Integrated RAG Chatbot project. All implementations must comply with the stated principles. Any deviation requires explicit documentation and approval. Code reviews must verify adherence to all principles. Performance benchmarks and security standards must be maintained throughout the development lifecycle.

**Version**: 1.1.0 | **Ratified**: 2025-12-14 | **Last Amended**: 2025-12-14
