from fastapi import FastAPI
from .app.api import rag_api

app = FastAPI()

app.include_router(rag_api.router, prefix="/rag", tags=["rag"])

@app.get("/")
async def root():
    return {"message": "Welcome to the RAG Chatbot Backend!"}
