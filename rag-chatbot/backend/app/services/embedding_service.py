# rag-chatbot/backend/app/services/embedding_service.py

import os
from typing import List
from qdrant_client.http.models import PointStruct
from .qdrant_service import get_qdrant_client, COLLECTION_NAME

# Assuming a simple text embedding model for now
# In a real scenario, this would integrate with a model like OpenAI's text-embedding-ada-002
# or a local SentenceTransformer model.

def get_dummy_embedding(text: str) -> List[float]:
    """Generates a dummy embedding for a given text.
    In a real application, this would use a proper embedding model.
    """
    # Simple hash-based embedding for demonstration
    embedding = [float(ord(char)) / 128.0 for char in text[:768].ljust(768, ' ')]
    return embedding

async def embed_and_store_content(content_id: str, text_content: str, metadata: dict):
    """
    Generates an embedding for the given text content and stores it in Qdrant.
    """
    client = get_qdrant_client()
    if not client:
        print("Failed to get Qdrant client.")
        return False

    embedding = get_dummy_embedding(text_content)
    
    point = PointStruct(
        id=content_id,
        vector=embedding,
        payload={**metadata, "text_content": text_content}
    )

    try:
        operation_info = client.upsert(
            collection_name=COLLECTION_NAME,
            wait=True,
            points=[point]
        )
        print(f"Content '{content_id}' embedded and stored. Status: {operation_info.status}")
        return True
    except Exception as e:
        print(f"Error embedding and storing content: {e}")
        return False

if __name__ == "__main__":
    # Example Usage
    content_id = "doc_123"
    text = "This is a sample text about physical AI and humanoid robotics."
    meta = {"source": "chapter1", "title": "Introduction"}
    
    # This example requires a running Qdrant instance
    print("Running embedding_service example. Make sure Qdrant is accessible.")
    # Assuming get_qdrant_client() and COLLECTION_NAME are correctly configured for testing.
    # For a real test, ensure QDRANT_HOST is set or a local instance is running.
    # Ensure qdrant_client is installed: pip install qdrant-client
    # For dummy embedding, no extra model is needed.
    
    # Example: run this from an async context or modify for sync testing
    # import asyncio
    # asyncio.run(embed_and_store_content(content_id, text, meta))
    print("Embedding and storage example finished (requires async run for full test).")
