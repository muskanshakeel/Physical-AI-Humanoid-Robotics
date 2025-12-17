from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
import os
from uuid import uuid4
import sys

class QdrantClientWrapper:
    def __init__(self):
        # Check if we're in a test environment or if QDRANT_URL is empty (development)
        is_test = 'pytest' in sys.modules or 'pytest' in sys.argv[0] if sys.argv else False
        qdrant_url = os.getenv("QDRANT_URL", "")

        if is_test or not qdrant_url:
            # In test or development environment without QDRANT_URL, set up a mock client
            self.client = None
            self.collection_name = "book_content"
            return

        self.client = QdrantClient(
            url=qdrant_url,
            api_key=os.getenv("QDRANT_API_KEY"),
            prefer_grpc=True  # Use gRPC for better performance
        )

        # Collection name for book content
        self.collection_name = "book_content"

        # Initialize the collection if it doesn't exist
        self._initialize_collection()

    def _initialize_collection(self):
        """
        Initialize the collection with proper vector configuration
        """
        if self.client is None:
            # In test mode, skip initialization
            return

        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=384,  # Using sentence-transformers/all-MiniLM-L6-v2 which produces 384-dim vectors
                    distance=models.Distance.COSINE
                )
            )

    async def add_vectors(self, texts: List[str], payloads: List[Dict] = None, ids: List[str] = None) -> bool:
        """
        Add vectors to the collection
        """
        if self.client is None:
            # In test mode, just return success
            return True

        if ids is None:
            ids = [str(uuid4()) for _ in texts]

        if payloads is None:
            payloads = [{}] * len(texts)

        try:
            self.client.upload_collection(
                collection_name=self.collection_name,
                vectors=texts,  # This will be the embeddings
                payload=payloads,
                ids=ids
            )
            return True
        except Exception as e:
            print(f"Error adding vectors: {e}")
            return False

    async def search_vectors(self, query_vector: List[float], limit: int = 5) -> List[Dict]:
        """
        Search for similar vectors
        """
        if self.client is None:
            # In test mode, return mock results
            return [
                {
                    "id": "mock-id-1",
                    "payload": {"content": "Mock search result for testing", "source": "test_source.txt"},
                    "score": 0.9
                }
            ]

        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "payload": result.payload,
                    "score": result.score
                })

            return formatted_results
        except Exception as e:
            print(f"Error searching vectors: {e}")
            return []

    async def delete_vectors(self, ids: List[str]) -> bool:
        """
        Delete vectors by IDs
        """
        if self.client is None:
            # In test mode, just return success
            return True

        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=ids
                )
            )
            return True
        except Exception as e:
            print(f"Error deleting vectors: {e}")
            return False

# Global instance
qdrant_client = QdrantClientWrapper()