# rag-chatbot/backend/app/api/rag_api.py

from fastapi import APIRouter
from ..models.rag_models import Query, Answer
from ..services.rag_service import generate_rag_answer

router = APIRouter()

@router.post("/query", response_model=Answer)
async def query_rag_chatbot(query: Query):
    """
    Queries the RAG chatbot with the provided text and optional context.
    """
    return await generate_rag_answer(query)
