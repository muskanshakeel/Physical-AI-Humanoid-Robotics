from sqlalchemy import Column, Integer, String, DateTime, Boolean, Text
from sqlalchemy.sql import func
from sqlalchemy.orm import relationship
from . import Base
from datetime import datetime
import uuid

class User(Base):
    """
    User model representing authenticated users of the system
    """
    __tablename__ = "users"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    email = Column(String, unique=True, nullable=False, index=True)
    name = Column(String, nullable=False)
    hashed_password = Column(String, nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())
    is_active = Column(Boolean, default=True)

    # Relationships - will be defined once other models are created
    # sessions = relationship("Session", back_populates="user")
    # queries = relationship("Query", back_populates="user")
    # responses = relationship("Response", back_populates="user")