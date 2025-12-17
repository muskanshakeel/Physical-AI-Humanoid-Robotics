from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from ..db.connection import get_db_session
from ..services.rag_service import RAGService
from ..models.request import ChatQueryRequest, ChatHistoryRequest, FrontendChatRequest
from ..models.response import ChatResponse, ChatHistoryResponse, ChatHistoryItem, FrontendChatResponse
from typing import Dict
import uuid

router = APIRouter(tags=["Chat"])

@router.post("/query", response_model=ChatResponse)
async def chat_query(
    request: ChatQueryRequest,
    db: AsyncSession = Depends(get_db_session)
):
    """
    Submit a query to the RAG system
    """
    try:
        # Validate query content
        is_valid = await RAGService.validate_query_content(request.query)
        if not is_valid:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid query content"
            )

        # Process the query using RAG service (without user-specific tracking)
        result = await RAGService.process_query(
            db=db,
            user_id="anonymous",  # Use anonymous user ID
            session_id=str(uuid.uuid4()),  # Generate a new session ID
            query_text=request.query,
            book_selection=request.book_selection
        )

        return ChatResponse(
            success=True,
            query_id=result["query_id"],
            response=result["response"]
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Query processing failed: {str(e)}"
        )

@router.get("/history", response_model=ChatHistoryResponse)
async def get_chat_history(
    request: ChatHistoryRequest = Depends(),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Retrieve chat history (placeholder - no persistent history without auth)
    """
    try:
        # For now, return an empty history since we're not tracking user sessions
        # In a real implementation, you might track by session ID in cookies or headers
        return ChatHistoryResponse(
            success=True,
            history=[],
            pagination={
                "total": 0,
                "limit": request.limit,
                "offset": request.offset,
                "has_more": False
            }
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"History retrieval failed: {str(e)}"
        )

@router.get("/history/{query_id}", response_model=Dict)
async def get_specific_chat(
    query_id: str,
    db: AsyncSession = Depends(get_db_session)
):
    """
    Retrieve a specific query-response pair
    """
    try:
        result = await RAGService.get_query_response(db, query_id)

        if not result:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Query not found"
            )

        return {
            "success": True,
            "query": result["query"],
            "response": result["response"]
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Chat retrieval failed: {str(e)}"
        )

