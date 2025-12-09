# rag-chatbot/backend/app/services/qdrant_service.py

import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models

load_dotenv()

# TODO: Replace with actual Qdrant Cloud details or environment variables
QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_PORT = int(os.getenv("QDRANT_PORT", "6333"))
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY", None) # Required for Qdrant Cloud

COLLECTION_NAME = "physical_ai_textbook"

def get_qdrant_client():
    """Initializes and returns a Qdrant client."""
    try:
        if QDRANT_API_KEY:
            # Connect to Qdrant Cloud
            client = QdrantClient(
                url=f"https://{QDRANT_HOST}", # Use https for cloud
                api_key=QDRANT_API_KEY,
            )
        else:
            # Connect to local Qdrant instance
            client = QdrantClient(host=QDRANT_HOST, port=QDRANT_PORT)
        return client
    except Exception as e:
        print(f"Error initializing Qdrant client: {e}")
        return None

def initialize_qdrant_collection(client: QdrantClient):
    """Ensures the Qdrant collection exists and is configured."""
    try:
        if not client.collection_exists(collection_name=COLLECTION_NAME):
            client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE), # Example size, adjust as needed
            )
            print(f"Collection '{COLLECTION_NAME}' created.")
        else:
            print(f"Collection '{COLLECTION_NAME}' already exists.")
    except Exception as e:
        print(f"Error initializing Qdrant collection: {e}")

if __name__ == "__main__":
    # Example usage:
    client = get_qdrant_client()
    if client:
        print("Successfully initialized Qdrant client (placeholder).")
        initialize_qdrant_collection(client)
        print("Qdrant client operations complete.")
