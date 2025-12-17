from fastapi import APIRouter, Depends, HTTPException, status, Query
from sqlalchemy.ext.asyncio import AsyncSession
from ..db.connection import get_db_session
from ..services.book_content_service import BookContentService
from ..models.response import BookContentListResponse, TableOfContentsListResponse
from typing import Optional

router = APIRouter(tags=["Book Content"])

@router.get("/content", response_model=BookContentListResponse)
async def get_book_content(
    section: Optional[str] = Query(None, description="Filter by section"),
    search: Optional[str] = Query(None, description="Search term for content"),
    db: AsyncSession = Depends(get_db_session)
):
    """
    Retrieve book content by section or search
    """
    try:
        if section:
            contents = await BookContentService.get_by_section(db, section)
        elif search:
            contents = await BookContentService.search_content(db, search)
        else:
            # If no filter is provided, return an empty list or all content (limited)
            contents = []

        return BookContentListResponse(
            success=True,
            content=contents
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Content retrieval failed: {str(e)}"
        )

@router.get("/toc", response_model=TableOfContentsListResponse)
async def get_table_of_contents(
    db: AsyncSession = Depends(get_db_session)
):
    """
    Retrieve table of contents
    """
    try:
        toc = await BookContentService.get_table_of_contents(db)

        return TableOfContentsListResponse(
            success=True,
            table_of_contents=toc
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"TOC retrieval failed: {str(e)}"
        )

# Additional endpoints that could be implemented:
# - GET /api/book/content/{id} - Get specific content by ID
# - POST /api/book/content - Add new content (admin only)
# - PUT /api/book/content/{id} - Update content (admin only)
# - DELETE /api/book/content/{id} - Delete content (admin only)