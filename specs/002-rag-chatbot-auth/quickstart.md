# Quickstart Guide: Integrated RAG Chatbot for Published Book

## Prerequisites

- Python 3.11+
- Node.js 18+ (for Docusaurus integration)
- Docker (for local development)
- Access to Qdrant Cloud (with provided API key)
- Neon Serverless Postgres database access

## Environment Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <project-directory>
   ```

2. **Set up Python virtual environment**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r backend/requirements.txt
   ```

3. **Set up environment variables**
   Create a `.env` file in the backend directory:
   ```env
   QDRANT_URL=https://bde9168d-2fca-400c-9e83-3e097447d7d6.us-east4-0.gcp.cloud.qdrant.io:6333
   QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.2ugMB2Att2IJf8iHocFg56QT2Zb0b3V2dtDk7
   DATABASE_URL='postgresql://neondb_owner:npg_2WnI7wqztRaN@ep-steep-mode-ahcpn4zl-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require'
   BETTER_AUTH_SECRET=your-super-secret-auth-key-here
   QWEN_API_KEY=your-qwen-api-key-here
   ```

4. **Install frontend dependencies**
   ```bash
   cd frontend
   npm install
   ```

## Running the Application

### Backend (FastAPI Server)

1. **Navigate to backend directory**
   ```bash
   cd backend
   ```

2. **Run database migrations**
   ```bash
   python -m alembic upgrade head
   ```

3. **Start the backend server**
   ```bash
   uvicorn src.main:app --reload --port 8000
   ```

### Frontend Development Server

1. **Navigate to frontend directory**
   ```bash
   cd frontend
   ```

2. **Start the development server**
   ```bash
   npm run dev
   ```

### Docusaurus Integration

1. **Navigate to docs directory**
   ```bash
   cd docs
   ```

2. **Start Docusaurus server**
   ```bash
   npm run start
   ```

## Book Content Indexing

To index your book content for the RAG system:

1. **Prepare your book content** in a structured format (JSON, Markdown, or text files)

2. **Run the indexing script**
   ```bash
   cd backend
   python -m src.scripts.index_book_content --source path/to/book/content
   ```

3. **Verify indexing completion**
   ```bash
   curl -X GET http://localhost:8000/api/book/toc -H "Authorization: Bearer your-session-token"
   ```

## Using the Chat Interface

### Authentication
1. Register a new user:
   ```bash
   curl -X POST http://localhost:8000/api/auth/register \
     -H "Content-Type: application/json" \
     -d '{"email": "user@example.com", "password": "password123", "name": "User Name"}'
   ```

2. Login to get a session token:
   ```bash
   curl -X POST http://localhost:8000/api/auth/login \
     -H "Content-Type: application/json" \
     -d '{"email": "user@example.com", "password": "password123"}'
   ```

### Making Queries
1. Submit a query to the RAG system:
   ```bash
   curl -X POST http://localhost:8000/api/chat/query \
     -H "Authorization: Bearer your-session-token" \
     -H "Content-Type: application/json" \
     -d '{"query": "What is the main concept of reinforcement learning?", "book_selection": ""}'
   ```

2. View your chat history:
   ```bash
   curl -X GET http://localhost:8000/api/chat/history \
     -H "Authorization: Bearer your-session-token"
   ```

## Configuration Options

### Backend Configuration
- `BACKEND_HOST`: Host for the backend server (default: localhost)
- `BACKEND_PORT`: Port for the backend server (default: 8000)
- `MAX_QUERY_LENGTH`: Maximum length of user queries (default: 1000 characters)
- `RESPONSE_TIMEOUT`: Timeout for AI responses in seconds (default: 30)

### Vector Search Configuration
- `VECTOR_TOP_K`: Number of similar content pieces to retrieve (default: 5)
- `VECTOR_THRESHOLD`: Minimum similarity threshold (default: 0.7)
- `VECTOR_EMBEDDING_MODEL`: Model used for embeddings (default: sentence-transformers/all-MiniLM-L6-v2)

### AI Model Configuration
- `AI_MODEL_PROVIDER`: Either 'qwen' or 'openai' (default: qwen)
- `AI_TEMPERATURE`: Creativity parameter (0.0 to 1.0, default: 0.7)
- `AI_MAX_TOKENS`: Maximum tokens in response (default: 1000)

## Testing

### Running Backend Tests
```bash
cd backend
pytest tests/unit/ -v
pytest tests/integration/ -v
```

### Running Frontend Tests
```bash
cd frontend
npm run test
```

### End-to-End Tests
```bash
cd backend
pytest tests/e2e/ -v
```

## Deployment

### Production Build
1. **Build the frontend**
   ```bash
   cd frontend
   npm run build
   ```

2. **Build the backend**
   ```bash
   cd backend
   pip install -r requirements-prod.txt
   ```

3. **Deploy to your preferred platform** (AWS, GCP, Azure, or containerized environment)

### Environment Variables for Production
Ensure the following environment variables are set in production:
- `DATABASE_URL`: Production database URL
- `QDRANT_URL` and `QDRANT_API_KEY`: Production Qdrant credentials
- `BETTER_AUTH_SECRET`: Strong secret for authentication
- `QWEN_API_KEY` or `OPENAI_API_KEY`: AI model provider key
- `ALLOWED_ORIGINS`: Comma-separated list of allowed origins

## Troubleshooting

### Common Issues

1. **Database Connection Issues**
   - Verify your Neon Postgres connection string
   - Check that your database credentials are correct
   - Ensure your database is running and accessible

2. **Vector Search Not Returning Results**
   - Verify that book content has been properly indexed
   - Check that Qdrant service is accessible
   - Confirm that the similarity threshold isn't too high

3. **Authentication Not Working**
   - Ensure `BETTER_AUTH_SECRET` is set and secure
   - Verify that session tokens are being properly stored and sent with requests

4. **AI Responses Taking Too Long**
   - Check your AI model provider's API status
   - Verify that your API keys are valid and have sufficient quota
   - Consider implementing response caching for common queries

### Performance Monitoring
Monitor these key metrics:
- Average response time for queries (should be <2 seconds)
- Vector search accuracy rate
- API error rates
- Database connection pool usage
- Cache hit ratios