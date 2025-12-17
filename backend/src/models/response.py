from sqlalchemy import Column, String, Text, Integer, DateTime, JSON, Float, ForeignKey
from sqlalchemy.sql import func
from . import Base
from pydantic import BaseModel
from typing import Optional, List, Dict
import uuid
from datetime import datetime


# SQLAlchemy Models
class Response(Base):
    """
    Response model representing system-generated answers based on book content
    """
    __tablename__ = "responses"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    query_id = Column(String, ForeignKey("queries.id"), nullable=False)  # Foreign key to Query
    content = Column(Text, nullable=False)
    sources = Column(JSON, nullable=False)  # List of book content sources used to generate the response
    confidence_score = Column(Float, nullable=False)  # Confidence level of the response (0.0 to 1.0)
    timestamp = Column(DateTime(timezone=True), server_default=func.now())
    response_time_ms = Column(Integer, nullable=False)  # Time taken to generate the response in milliseconds


# Pydantic Response Models
class UserResponse(BaseModel):
    id: str
    email: str
    name: str
    created_at: Optional[datetime] = None


class AuthResponse(BaseModel):
    success: bool
    user: UserResponse
    session_token: str
    expires_at: datetime


class LoginResponse(BaseModel):
    success: bool
    user: UserResponse
    session_token: str
    expires_at: datetime


class LogoutResponse(BaseModel):
    success: bool
    message: str


class SourceResponse(BaseModel):
    section: str
    title: str
    page_number: Optional[int] = None
    relevance_score: float


class ChatResponseItem(BaseModel):
    id: str
    content: str
    sources: List[SourceResponse]
    confidence_score: float
    response_time_ms: int


class ChatResponse(BaseModel):
    success: bool
    query_id: str
    response: ChatResponseItem

class FrontendChatResponse(BaseModel):
    session_id: str
    response: str
    sources: List[str] = []


class ChatHistoryItem(BaseModel):
    id: str
    query: str
    response: str


class ChatHistoryResponse(BaseModel):
    success: bool
    history: List[ChatHistoryItem]
    pagination: Dict[str, int]


class BookContentResponse(BaseModel):
    id: str
    title: str
    content: str
    section: str
    page_number: Optional[int] = None
    created_at: Optional[datetime] = None


class BookContentListResponse(BaseModel):
    success: bool
    content: List[BookContentResponse]


class TableOfContentsResponse(BaseModel):
    section: str
    title: str
    page_number: int


class TableOfContentsListResponse(BaseModel):
    success: bool
    table_of_contents: List[TableOfContentsResponse]


class ErrorResponse(BaseModel):
    success: bool = False
    error: str
    details: Optional[str] = None