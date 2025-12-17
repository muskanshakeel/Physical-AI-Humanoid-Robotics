from typing import List, Dict, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
import asyncio
from ..models.book_content import BookContent
from ..vector_db.client import qdrant_client
from ..services.ai_service import get_ai_service
from fastapi import HTTPException, status

class VectorIndexService:
    """
    Service for handling vector indexing and search operations
    """

    @staticmethod
    async def index_book_content(content_id: str, content: str, title: str, section: str) -> bool:
        """
        Index a piece of book content in the vector database
        """
        try:
            # Get the AI service to generate embeddings
            ai_service = get_ai_service()

            # Generate embedding for the content
            embedding = await ai_service.embed_text(content)

            # Prepare payload with content metadata
            payload = {
                "content_id": content_id,
                "title": title,
                "section": section,
                "content_preview": content[:200]  # Store a preview of the content
            }

            # Add to vector database
            success = await qdrant_client.add_vectors(
                texts=[embedding],
                payloads=[payload],
                ids=[content_id]
            )

            return success
        except Exception as e:
            print(f"Error indexing content: {e}")
            return False

    @staticmethod
    async def index_all_book_content(db: AsyncSession) -> bool:
        """
        Index all book content in the database
        """
        try:
            # Get all book content
            result = await db.execute(select(BookContent))
            contents = result.scalars().all()

            # Process each content item
            for content in contents:
                await VectorIndexService.index_book_content(
                    content.id,
                    content.content,
                    content.title,
                    content.section
                )

            return True
        except Exception as e:
            print(f"Error indexing all content: {e}")
            return False

    @staticmethod
    async def search_similar_content(query: str, limit: int = 5) -> List[Dict]:
        """
        Search for similar content in the vector database
        """
        try:
            # Get the AI service to generate embeddings for the query
            ai_service = get_ai_service()

            # Generate embedding for the query
            query_embedding = await ai_service.embed_text(query)

            # Search in vector database
            results = await qdrant_client.search_vectors(query_embedding, limit=limit)

            return results
        except Exception as e:
            print(f"Error searching content: {e}")
            return []

    @staticmethod
    async def update_content_index(content_id: str, new_content: str, title: str, section: str) -> bool:
        """
        Update the index for a specific content item
        """
        try:
            # Delete the old vector
            await qdrant_client.delete_vectors([content_id])

            # Add the updated content
            success = await VectorIndexService.index_book_content(
                content_id,
                new_content,
                title,
                section
            )

            return success
        except Exception as e:
            print(f"Error updating content index: {e}")
            return False

    @staticmethod
    async def delete_content_from_index(content_id: str) -> bool:
        """
        Remove content from the vector index
        """
        try:
            success = await qdrant_client.delete_vectors([content_id])
            return success
        except Exception as e:
            print(f"Error deleting content from index: {e}")
            return False