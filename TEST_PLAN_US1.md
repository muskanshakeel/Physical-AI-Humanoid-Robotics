# Test Plan for User Story 1 - Authenticated Chatbot Interaction

## Overview
This test plan covers validation of the RAG chatbot feature with BetterAuth authentication. The goal is to ensure that users can authenticate with the system and interact with the RAG chatbot to ask questions about book content and receive accurate, contextually relevant answers.

## Test Objectives
- Verify authentication system (registration, login, logout)
- Validate chat functionality and response quality
- Confirm RAG responses are accurate and based on book content
- Ensure response time meets ≤2 seconds requirement
- Test error handling and edge cases

## Test Environment
- Backend: FastAPI server running on localhost:8000
- Frontend: React development server
- Database: Neon Serverless Postgres
- Vector DB: Qdrant Cloud
- AI Model: Qwen or OpenAI (configured via environment)

## Test Scenarios

### 1. Authentication Flow
#### 1.1 User Registration
- **Test ID**: TST-001
- **Scenario**: New user registration
- **Steps**:
  1. Navigate to registration form
  2. Enter valid email, password, and name
  3. Submit registration
- **Expected**: Successful registration with JWT token, user stored in DB
- **Pass Criteria**: User created, token returned, no errors

#### 1.2 User Login
- **Test ID**: TST-002
- **Scenario**: Existing user login
- **Steps**:
  1. Navigate to login form
  2. Enter valid credentials
  3. Submit login
- **Expected**: Successful login with JWT token, session created
- **Pass Criteria**: Token returned, session stored, no errors

#### 1.3 User Logout
- **Test ID**: TST-003
- **Scenario**: User logout
- **Steps**:
  1. Authenticate user
  2. Click logout button
  3. Submit logout request
- **Expected**: Session invalidated, token cleared
- **Pass Criteria**: Session marked inactive, frontend state cleared

#### 1.4 Authentication Error Cases
- **Test ID**: TST-004
- **Scenario**: Invalid credentials
- **Steps**:
  1. Attempt login with invalid email/password
- **Expected**: Error message, no token returned
- **Pass Criteria**: 401 status, error message returned

### 2. Chat Functionality
#### 2.1 Basic Query Processing
- **Test ID**: TST-005
- **Scenario**: User asks a question about book content
- **Steps**:
  1. Authenticate user
  2. Enter a question in chat interface
  3. Submit query
- **Expected**: Relevant response based on book content
- **Pass Criteria**: Response generated within 2 seconds, relevant to query

#### 2.2 Query History
- **Test ID**: TST-006
- **Scenario**: Multiple queries in session
- **Steps**:
  1. Authenticate user
  2. Submit multiple queries
  3. Verify conversation flow
- **Expected**: All queries and responses displayed
- **Pass Criteria**: Messages persisted in UI, chronological order

#### 2.3 Source Attribution
- **Test ID**: TST-007
- **Scenario**: Response with source references
- **Steps**:
  1. Submit query that requires book content
  2. Check response for source attribution
- **Expected**: Sources listed with response
- **Pass Criteria**: Sources returned with response, properly formatted

### 3. RAG Response Quality
#### 3.1 Accuracy Validation
- **Test ID**: TST-008
- **Scenario**: Verify responses based on book content
- **Steps**:
  1. Submit specific questions about book sections
  2. Compare response to actual book content
- **Expected**: Responses directly related to book content
- **Pass Criteria**: Response accuracy >85% to source material

#### 3.2 Context Relevance
- **Test ID**: TST-009
- **Scenario**: Questions about specific book topics
- **Steps**:
  1. Submit questions about specific book concepts
  2. Verify context relevance
- **Expected**: Responses focused on requested topic
- **Pass Criteria**: Contextually relevant responses >90% of time

#### 3.3 Vector Search Effectiveness
- **Test ID**: TST-010
- **Scenario**: Semantic similarity matching
- **Steps**:
  1. Submit queries with synonyms/related terms
  2. Verify relevant content retrieval
- **Expected**: Relevant book content retrieved
- **Pass Criteria**: Top 3 results relevant to query >80% of time

### 4. Performance Requirements
#### 4.1 Response Time
- **Test ID**: TST-011
- **Scenario**: Query response time measurement
- **Steps**:
  1. Submit multiple queries
  2. Measure response time for each
- **Expected**: Responses generated ≤2 seconds
- **Pass Criteria**: 95% of responses under 2 seconds

#### 4.2 Concurrent Users
- **Test ID**: TST-012
- **Scenario**: Multiple simultaneous users
- **Steps**:
  1. Simulate multiple concurrent users
  2. Submit queries simultaneously
- **Expected**: System handles load without degradation
- **Pass Criteria**: Response time <2 seconds for 95% of requests under load

### 5. Error Handling
#### 5.1 Invalid Input
- **Test ID**: TST-013
- **Scenario**: Malformed queries
- **Steps**:
  1. Submit empty queries
  2. Submit queries with special characters
- **Expected**: Proper error handling, no crashes
- **Pass Criteria**: Graceful error messages, system stability

#### 5.2 API Failures
- **Test ID**: TST-014
- **Scenario**: Service unavailability
- **Steps**:
  1. Simulate vector DB failure
  2. Simulate AI service failure
- **Expected**: Graceful degradation
- **Pass Criteria**: User-friendly error messages, no system crashes

## Test Data
- Valid user credentials for testing
- Sample book content for query validation
- Expected responses for accuracy testing

## Success Criteria
- All authentication tests pass (TST-001 to TST-004)
- Chat functionality tests pass (TST-005 to TST-007)
- RAG response quality meets standards (TST-008 to TST-010)
- Performance requirements met (TST-011 to TST-012)
- Error handling works properly (TST-013 to TST-014)
- Overall system reliability >95%

## Test Execution Order
1. Authentication tests (TST-001 to TST-004)
2. Basic chat functionality (TST-005 to TST-007)
3. RAG response validation (TST-008 to TST-010)
4. Performance testing (TST-011 to TST-012)
5. Error handling (TST-013 to TST-014)