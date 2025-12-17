# Data Model: Integrated RAG Chatbot for Published Book

## Entity Definitions

### User
- **Description**: Represents authenticated users of the system
- **Fields**:
  - `id` (UUID, Primary Key): Unique identifier for the user
  - `email` (String, Unique): User's email address for authentication
  - `name` (String): User's display name
  - `created_at` (DateTime): Timestamp when the user account was created
  - `updated_at` (DateTime): Timestamp when the user account was last updated
  - `is_active` (Boolean): Whether the user account is active
- **Relationships**: One-to-many with Session, Query, and Response
- **Validation**: Email must be valid format, unique constraint on email

### Session
- **Description**: Represents authenticated user sessions with associated query history
- **Fields**:
  - `id` (UUID, Primary Key): Unique identifier for the session
  - `user_id` (UUID, Foreign Key): Reference to the user who owns this session
  - `session_token` (String, Unique): Secure session token
  - `created_at` (DateTime): Timestamp when the session was created
  - `expires_at` (DateTime): Timestamp when the session expires
  - `is_active` (Boolean): Whether the session is currently active
- **Relationships**: Belongs to User, one-to-many with Query
- **Validation**: Session token must be cryptographically secure, proper expiration handling

### Query
- **Description**: Represents user questions about book content
- **Fields**:
  - `id` (UUID, Primary Key): Unique identifier for the query
  - `user_id` (UUID, Foreign Key): Reference to the user who made the query
  - `session_id` (UUID, Foreign Key): Reference to the session during which the query was made
  - `content` (Text): The actual question/query text
  - `book_selection` (Text, Optional): Specific text from the book that the query relates to
  - `timestamp` (DateTime): When the query was made
  - `query_vector` (JSONB, Optional): Vector embedding of the query for similarity search
- **Relationships**: Belongs to User and Session, one-to-one with Response
- **Validation**: Content must not be empty, length limits to prevent abuse

### Response
- **Description**: Represents system-generated answers based on book content
- **Fields**:
  - `id` (UUID, Primary Key): Unique identifier for the response
  - `query_id` (UUID, Foreign Key): Reference to the query this responds to
  - `content` (Text): The AI-generated response text
  - `sources` (JSONB): List of book content sources used to generate the response
  - `confidence_score` (Float): Confidence level of the response (0.0 to 1.0)
  - `timestamp` (DateTime): When the response was generated
  - `response_time_ms` (Integer): Time taken to generate the response in milliseconds
- **Relationships**: Belongs to Query
- **Validation**: Content must not be empty, confidence score between 0.0 and 1.0

### BookContent
- **Description**: Represents the published book material that serves as the knowledge base
- **Fields**:
  - `id` (UUID, Primary Key): Unique identifier for the book content chunk
  - `title` (String): Title or heading of the content chunk
  - `content` (Text): The actual book content text
  - `section` (String): Book section or chapter identifier
  - `page_number` (Integer, Optional): Page number in the original book
  - `vector_embedding` (JSONB): Vector representation for similarity search
  - `created_at` (DateTime): When this content was indexed
  - `updated_at` (DateTime): When this content was last updated
- **Relationships**: Many-to-many with Response through source references
- **Validation**: Content must not be empty, vector embedding must be properly formatted

### ChatHistory
- **Description**: Aggregated view of user interactions for history retrieval
- **Fields**:
  - `id` (UUID, Primary Key): Unique identifier for the history entry
  - `user_id` (UUID, Foreign Key): Reference to the user
  - `query_id` (UUID, Foreign Key): Reference to the original query
  - `response_id` (UUID, Foreign Key): Reference to the response
  - `session_id` (UUID, Foreign Key): Reference to the session
  - `timestamp` (DateTime): When the interaction occurred
  - `is_starred` (Boolean): Whether the user has starred this interaction
- **Relationships**: Belongs to User, Query, Response, and Session
- **Validation**: Proper foreign key relationships maintained

## State Transitions

### Session States
- `ACTIVE`: Session is valid and user can make queries
- `EXPIRED`: Session has timed out and needs re-authentication
- `REVOKED`: Session was manually invalidated by user or system

### Query Processing States
- `PENDING`: Query received but not yet processed
- `PROCESSING`: Query is being processed by the RAG system
- `COMPLETED`: Response has been generated and returned
- `FAILED`: Query processing failed due to error

## Database Indexes

### Critical Indexes for Performance
- User.email (unique, for authentication)
- Session.session_token (unique, for session validation)
- Query.user_id (for user query history)
- Query.timestamp (for chronological query retrieval)
- BookContent.vector_embedding (for similarity search - potentially using pgvector)
- ChatHistory.user_id and timestamp (for user history retrieval)

## Security Considerations

### Data Protection
- User PII (email, name) stored encrypted at rest
- Session tokens stored with secure hashing
- Query and response logs include only necessary information
- Book content is stored as provided, with no user modifications

### Access Control
- Users can only access their own sessions, queries, and responses
- Session validation required for all API endpoints
- Rate limiting to prevent data scraping
- Audit logging for security monitoring