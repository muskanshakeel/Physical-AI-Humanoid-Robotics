import os
from abc import ABC, abstractmethod
from typing import List, Dict, Optional
import asyncio

class BaseAIService(ABC):
    """
    Abstract base class for AI services
    """

    @abstractmethod
    async def generate_response(self, prompt: str, context: str = "") -> str:
        """
        Generate a response based on the prompt and context
        """
        pass

    @abstractmethod
    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embeddings for the given text
        """
        pass

class QwenAIService(BaseAIService):
    """
    Qwen AI service implementation
    """

    def __init__(self):
        self.api_key = os.getenv("QWEN_API_KEY")
        self.model = "qwen-max"  # or whatever model is appropriate

    async def generate_response(self, prompt: str, context: str = "") -> str:
        """
        Generate response using Qwen API
        """
        # This is a placeholder implementation
        # In a real implementation, we would call the Qwen API
        import time
        time.sleep(0.1)  # Simulate API call delay

        # For now, return a mock response
        return f"Mock response to: {prompt}. Context: {context[:50]}..."

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embeddings using Qwen API
        """
        # This is a placeholder implementation
        # In a real implementation, we would call the Qwen embedding API
        import random
        # Return a mock embedding vector of 384 dimensions (typical for sentence transformers)
        return [random.random() for _ in range(384)]

class OpenAIService(BaseAIService):
    """
    OpenAI service implementation
    """

    def __init__(self):
        self.api_key = os.getenv("OPENAI_API_KEY")
        self.model = "gpt-3.5-turbo"  # or gpt-4 if available

    async def generate_response(self, prompt: str, context: str = "") -> str:
        """
        Generate response using OpenAI API
        """
        # This is a placeholder implementation
        # In a real implementation, we would call the OpenAI API
        import time
        time.sleep(0.1)  # Simulate API call delay

        # For now, return a mock response
        return f"OpenAI mock response to: {prompt}. Context: {context[:50]}..."

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embeddings using OpenAI API
        """
        # This is a placeholder implementation
        # In a real implementation, we would call the OpenAI embedding API
        import random
        # Return a mock embedding vector of 1536 dimensions (for text-embedding-ada-002)
        return [random.random() for _ in range(1536)]

# Factory function to get the appropriate AI service
def get_ai_service() -> BaseAIService:
    provider = os.getenv("AI_MODEL_PROVIDER", "qwen").lower()

    if provider == "openai":
        return OpenAIService()
    else:  # default to qwen
        return QwenAIService()