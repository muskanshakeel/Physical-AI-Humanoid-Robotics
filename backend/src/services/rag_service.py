from typing import List, Dict, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from datetime import datetime, timedelta
import time
import asyncio
from ..models.query import Query
from ..models.response import Response
from ..models.user import User
from ..models.session import Session
from ..services.ai_service import get_ai_service
from ..services.vector_index_service import VectorIndexService
from ..models.response import SourceResponse
from fastapi import HTTPException, status
import uuid

class RAGService:
    """
    RAG (Retrieval-Augmented Generation) service for processing user queries
    """

    @staticmethod
    async def process_query(db: AsyncSession, user_id: str, session_id: str, query_text: str, book_selection: Optional[str] = None) -> Dict:
        """
        Process a user query using RAG approach
        """
        start_time = time.time()

        # Create query record
        db_query = Query(
            user_id=user_id,
            session_id=session_id,
            content=query_text,
            book_selection=book_selection
        )

        db.add(db_query)
        await db.commit()
        await db.refresh(db_query)

        # Perform vector similarity search to find relevant book content
        search_results = await VectorIndexService.search_similar_content(query_text)

        # Format sources from search results
        sources = []
        for result in search_results:
            payload = result.get("payload", {})
            sources.append({
                "section": payload.get("section", ""),
                "title": payload.get("title", ""),
                "page_number": None,  # Would come from the payload if available
                "relevance_score": result.get("score", 0.0)
            })

        # Prepare context for AI model
        context_parts = []
        for result in search_results:
            payload = result.get("payload", {})
            context_parts.append(payload.get("content_preview", ""))

        context = "\n\n".join(context_parts)

        # Generate response using AI model
        ai_service = get_ai_service()
        ai_response = await ai_service.generate_response(query_text, context)

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Create response record
        db_response = Response(
            query_id=db_query.id,
            content=ai_response,
            sources=sources,
            confidence_score=0.85,  # Placeholder confidence score
            response_time_ms=response_time_ms
        )

        db.add(db_response)
        await db.commit()
        await db.refresh(db_response)

        # Format and return response
        response_data = {
            "id": db_response.id,
            "content": db_response.content,
            "sources": [
                SourceResponse(
                    section=source["section"],
                    title=source["title"],
                    page_number=source["page_number"],
                    relevance_score=source["relevance_score"]
                )
                for source in sources
            ],
            "confidence_score": db_response.confidence_score,
            "response_time_ms": db_response.response_time_ms
        }

        return {
            "query_id": db_query.id,
            "response": response_data
        }

    @staticmethod
    async def get_query_response(db: AsyncSession, query_id: str) -> Optional[Dict]:
        """
        Get a specific query-response pair
        """
        # Get the query
        query_result = await db.execute(select(Query).filter(Query.id == query_id))
        query = query_result.scalars().first()

        if not query:
            return None

        # Get the associated response
        response_result = await db.execute(select(Response).filter(Response.query_id == query_id))
        response = response_result.scalars().first()

        if not response:
            return None

        # Format the response
        return {
            "query": {
                "id": query.id,
                "content": query.content,
                "timestamp": query.timestamp
            },
            "response": {
                "id": response.id,
                "content": response.content,
                "sources": response.sources,
                "confidence_score": response.confidence_score,
                "timestamp": response.timestamp,
                "response_time_ms": response.response_time_ms
            }
        }

    @staticmethod
    async def get_user_history(db: AsyncSession, user_id: str, limit: int = 20, offset: int = 0) -> List[Dict]:
        """
        Get user's query history
        """
        # Get queries with their responses
        result = await db.execute(
            select(Query, Response)
            .join(Response, Query.id == Response.query_id)
            .filter(Query.user_id == user_id)
            .order_by(Query.timestamp.desc())
            .offset(offset)
            .limit(limit)
        )

        history = []
        for query, response in result.all():
            history.append({
                "id": query.id,
                "query": {
                    "id": query.id,
                    "content": query.content,
                    "timestamp": query.timestamp
                },
                "response": {
                    "id": response.id,
                    "content": response.content,
                    "timestamp": response.timestamp
                }
            })

        return history

    @staticmethod
    async def validate_query_content(query_text: str) -> bool:
        """
        Validate query content meets requirements
        """
        if not query_text or len(query_text.strip()) == 0:
            return False

        if len(query_text) > 1000:  # Max query length from settings
            return False

        return True