# rag-chatbot/backend/app/models/rag_models.py

from pydantic import BaseModel, Field
from typing import List, Optional

class Query(BaseModel):
    """
    Represents a user's query to the RAG chatbot.
    """
    query_text: str = Field(..., description="The main text of the user's question.")
    context_text: Optional[str] = Field(None, description="Optional: Selected text from the document to provide additional context for the query.")

class Source(BaseModel):
    """
    Represents a source document or section from which an answer was retrieved.
    """
    chapter_title: str = Field(..., description="The title of the chapter the source came from.")
    section_title: Optional[str] = Field(None, description="The title of the section within the chapter.")
    page_number: Optional[int] = Field(None, description="The page number in the original document (if applicable).")
    text_snippet: str = Field(..., description="A snippet of text from the source that supports the answer.")
    url: Optional[str] = Field(None, description="URL to the source document or section.")

class Answer(BaseModel):
    """
    Represents the RAG chatbot's response to a query.
    """
    answer_text: str = Field(..., description="The generated answer from the chatbot.")
    sources: List[Source] = Field([], description="A list of source documents used to generate the answer.")
    is_cited: bool = Field(False, description="True if the answer includes direct citations from the source text.")
    is_relevant: bool = Field(False, description="True if the answer directly addresses the user's query.")
