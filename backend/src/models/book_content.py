from sqlalchemy import Column, String, Text, Integer, DateTime, JSON
from sqlalchemy.sql import func
from . import Base
import uuid

class BookContent(Base):
    """
    BookContent model representing the published book material that serves as the knowledge base
    """
    __tablename__ = "book_content"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    title = Column(String, nullable=False)
    content = Column(Text, nullable=False)
    section = Column(String, nullable=False)
    page_number = Column(Integer, nullable=True)
    vector_embedding = Column(JSON, nullable=True)  # Store as JSON for flexibility
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())