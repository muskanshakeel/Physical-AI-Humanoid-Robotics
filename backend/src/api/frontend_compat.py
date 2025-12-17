from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from ..db.connection import get_db_session
from ..models.request import FrontendChatRequest
from ..models.response import FrontendChatResponse
from ..services.rag_service import RAGService
import logging

# Create a new router for frontend compatibility endpoints
router = APIRouter(tags=["Frontend Compatibility"])

@router.post("/chat", response_model=FrontendChatResponse)
async def frontend_chat_endpoint(
    request: FrontendChatRequest,
    db: AsyncSession = Depends(get_db_session)
):
    """
    Chat endpoint that matches frontend expectations
    """
    try:
        # Prepare the query for the RAG service
        query_text = request.message
        if request.selected_text and request.context_only:
            # If context_only is True, focus on the selected text
            query_text = f"Based on the following text: '{request.selected_text}', {request.message}"
        elif request.selected_text:
            # If there's selected text, include it in the context
            query_text = f"Considering this text: '{request.selected_text}'. {request.message}"

        # Validate query content
        is_valid = await RAGService.validate_query_content(query_text)
        if not is_valid:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Invalid query content"
            )

        # Process the query using RAG service
        result = await RAGService.process_query(
            db=db,
            user_id="anonymous",  # Use anonymous user ID
            session_id=request.session_id,
            query_text=query_text,
            book_selection=None  # No specific book selection
        )

        # Extract sources from result if available
        sources = []
        if result.get("response"):
            response_obj = result["response"]
            if hasattr(response_obj, "sources"):
                # If response object has sources attribute (Pydantic model)
                sources = [source.section for source in response_obj.sources if hasattr(source, 'section')]
            elif isinstance(response_obj, dict) and "sources" in response_obj:
                # If response is a dict with sources
                response_sources = response_obj["sources"]
                if isinstance(response_sources, list):
                    sources = []
                    for source in response_sources:
                        if hasattr(source, 'section'):
                            sources.append(source.section)
                        elif isinstance(source, dict) and 'section' in source:
                            sources.append(source['section'])
                        elif hasattr(source, '__dict__') and 'section' in source.__dict__:
                            sources.append(getattr(source, 'section', ''))

        # Get response content
        response_content = ""
        if result.get("response"):
            response_obj = result["response"]
            if hasattr(response_obj, "content"):
                response_content = response_obj.content
            elif isinstance(response_obj, dict) and "content" in response_obj:
                response_content = response_obj["content"]
            else:
                response_content = str(response_obj)

        return FrontendChatResponse(
            session_id=request.session_id,
            response=response_content,
            sources=sources
        )
    except HTTPException:
        raise
    except Exception as e:
        logging.error(f"Chat processing error: {str(e)}")
        # Return a user-friendly error message
        error_response = f"Sorry, I encountered an error processing your request: {str(e)[:100]}..."
        return FrontendChatResponse(
            session_id=request.session_id,
            response=error_response,
            sources=[]
        )

@router.get("/history/{session_id}", response_model=dict)
async def get_chat_history_by_session(
    session_id: str,
    db: AsyncSession = Depends(get_db_session)
):
    """
    Get chat history for a specific session
    """
    try:
        # For now, return an empty history since we're not storing session-based history
        # In a full implementation, you would retrieve from database based on session_id
        return {
            "session_id": session_id,
            "history": []
        }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"History retrieval failed: {str(e)}"
        )