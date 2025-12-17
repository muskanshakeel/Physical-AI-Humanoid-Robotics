from sqlalchemy import Column, String, Text, DateTime, JSON, ForeignKey
from sqlalchemy.sql import func
from . import Base
import uuid

class Query(Base):
    """
    Query model representing user questions about book content
    """
    __tablename__ = "queries"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, ForeignKey("users.id"), nullable=False)
    session_id = Column(String, ForeignKey("sessions.id"), nullable=False)
    content = Column(Text, nullable=False)
    book_selection = Column(Text, nullable=True)  # Optional selected text from the book
    timestamp = Column(DateTime(timezone=True), server_default=func.now())
    query_vector = Column(JSON, nullable=True)  # Vector embedding of the query for similarity search
    # response_id = Column(String, ForeignKey("responses.id"), nullable=True)  # One-to-one with Response