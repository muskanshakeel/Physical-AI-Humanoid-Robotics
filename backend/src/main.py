from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from datetime import datetime
from sqlalchemy.ext.asyncio import AsyncSession

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for Retrieval-Augmented Generation Chatbot with Book Content",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for local development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running!"}

@app.get("/health")
async def health_check():
    return {"status": "healthy", "timestamp": datetime.utcnow().isoformat()}

# Include API routes
from .api.chat import router as chat_router
from .api.book import router as book_router
from .api.frontend_compat import router as frontend_router

app.include_router(chat_router, prefix="/api/chat")
app.include_router(book_router, prefix="/api/book")
app.include_router(frontend_router)  # No prefix for frontend compatibility endpoints

# Test endpoint to ensure routing is working
@app.get("/test")
async def test_endpoint():
    return {"message": "Test endpoint working!"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)