from typing import List, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import and_, or_
from ..models.book_content import BookContent
from ..models.response import BookContentResponse, TableOfContentsResponse
from fastapi import HTTPException, status
import uuid

class BookContentService:
    """
    Service for handling book content operations
    """

    @staticmethod
    async def get_by_id(db: AsyncSession, content_id: str) -> Optional[BookContentResponse]:
        """
        Get book content by ID
        """
        result = await db.execute(select(BookContent).filter(BookContent.id == content_id))
        content = result.scalars().first()

        if not content:
            return None

        return BookContentResponse(
            id=content.id,
            title=content.title,
            content=content.content,
            section=content.section,
            page_number=content.page_number
        )

    @staticmethod
    async def get_by_section(db: AsyncSession, section: str) -> List[BookContentResponse]:
        """
        Get book content by section
        """
        result = await db.execute(select(BookContent).filter(BookContent.section == section))
        contents = result.scalars().all()

        return [
            BookContentResponse(
                id=content.id,
                title=content.title,
                content=content.content,
                section=content.section,
                page_number=content.page_number
            )
            for content in contents
        ]

    @staticmethod
    async def search_content(db: AsyncSession, search_term: str) -> List[BookContentResponse]:
        """
        Search book content by text
        """
        result = await db.execute(
            select(BookContent).filter(
                or_(
                    BookContent.content.contains(search_term),
                    BookContent.title.contains(search_term),
                    BookContent.section.contains(search_term)
                )
            )
        )
        contents = result.scalars().all()

        return [
            BookContentResponse(
                id=content.id,
                title=content.title,
                content=content.content,
                section=content.section,
                page_number=content.page_number
            )
            for content in contents
        ]

    @staticmethod
    async def get_table_of_contents(db: AsyncSession) -> List[TableOfContentsResponse]:
        """
        Get the table of contents for the book
        """
        # Get unique sections ordered by page number
        result = await db.execute(
            select(BookContent.section, BookContent.page_number)
            .distinct()
            .order_by(BookContent.page_number)
        )
        sections = result.all()

        toc = []
        for section, page_number in sections:
            toc.append(
                TableOfContentsResponse(
                    section=section,
                    title=section,  # In a real implementation, this might come from a separate TOC table
                    page_number=page_number,
                    children=[]
                )
            )

        return toc

    @staticmethod
    async def create_content(db: AsyncSession, title: str, content: str, section: str, page_number: Optional[int] = None) -> BookContentResponse:
        """
        Create new book content
        """
        db_content = BookContent(
            title=title,
            content=content,
            section=section,
            page_number=page_number
        )

        db.add(db_content)
        await db.commit()
        await db.refresh(db_content)

        return BookContentResponse(
            id=db_content.id,
            title=db_content.title,
            content=db_content.content,
            section=db_content.section,
            page_number=db_content.page_number
        )

    @staticmethod
    async def update_content(db: AsyncSession, content_id: str, title: Optional[str] = None, content: Optional[str] = None,
                           section: Optional[str] = None, page_number: Optional[int] = None) -> Optional[BookContentResponse]:
        """
        Update book content
        """
        result = await db.execute(select(BookContent).filter(BookContent.id == content_id))
        db_content = result.scalars().first()

        if not db_content:
            return None

        # Update fields if provided
        if title is not None:
            db_content.title = title
        if content is not None:
            db_content.content = content
        if section is not None:
            db_content.section = section
        if page_number is not None:
            db_content.page_number = page_number

        await db.commit()
        await db.refresh(db_content)

        return BookContentResponse(
            id=db_content.id,
            title=db_content.title,
            content=db_content.content,
            section=db_content.section,
            page_number=db_content.page_number
        )

    @staticmethod
    async def delete_content(db: AsyncSession, content_id: str) -> bool:
        """
        Delete book content
        """
        result = await db.execute(select(BookContent).filter(BookContent.id == content_id))
        db_content = result.scalars().first()

        if not db_content:
            return False

        await db.delete(db_content)
        await db.commit()

        return True