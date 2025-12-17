from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime

# Authentication requests
class UserRegisterRequest(BaseModel):
    email: str
    password: str
    name: str

class UserLoginRequest(BaseModel):
    email: str
    password: str

# Chat requests
class ChatQueryRequest(BaseModel):
    query: str
    book_selection: Optional[str] = None
    context: Optional[Dict[str, Any]] = None

class FrontendChatRequest(BaseModel):
    message: str
    session_id: str
    selected_text: Optional[str] = None
    context_only: bool = False

class ChatHistoryRequest(BaseModel):
    limit: int = 20
    offset: int = 0
    sort: str = "desc"

# Book content requests
class BookContentRequest(BaseModel):
    section: Optional[str] = None
    search: Optional[str] = None