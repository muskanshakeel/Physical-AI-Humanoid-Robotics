# rag-chatbot/backend/app/services/retrieval_service.py

import os
from typing import List, Dict, Any
from qdrant_client.http.models import Filter, FieldCondition, MatchValue
from .qdrant_service import get_qdrant_client, COLLECTION_NAME
from .embedding_service import get_dummy_embedding # Reusing dummy for consistency

async def retrieve_relevant_content(query_text: str, top_k: int = 3, context_filter: Dict[str, Any] = None) -> List[Dict[str, Any]]:
    """
    Retrieves relevant content from Qdrant based on the query text.
    Optionally, a context_filter can be applied to narrow down the search.
    """
    client = get_qdrant_client()
    if not client:
        print("Failed to get Qdrant client.")
        return []

    query_embedding = get_dummy_embedding(query_text)

    qdrant_filter = None
    if context_filter:
        # Example: if context_filter is {'chapter': 'chapter1'}
        qdrant_filter = Filter(
            must=[FieldCondition(key=k, match=MatchValue(value=v)) for k, v in context_filter.items()]
        )

    try:
        search_result = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            query_filter=qdrant_filter,
            limit=top_k,
            with_payload=True # Return the full payload
        )

        relevant_content = []
        for hit in search_result:
            relevant_content.append({
                "score": hit.score,
                "text_content": hit.payload.get("text_content"),
                "metadata": {k: v for k, v in hit.payload.items() if k != "text_content"}
            })
        print(f"Retrieved {len(relevant_content)} relevant content items.")
        return relevant_content

    except Exception as e:
        print(f"Error retrieving content from Qdrant: {e}")
        return []

if __name__ == "__main__":
    # Example Usage
    query = "What is ROS 2?"
    
    # This example requires a running Qdrant instance with some embedded content
    print("Running retrieval_service example. Make sure Qdrant is accessible and has data.")
    # For a real test, ensure QDRANT_HOST is set and data has been embedded via embedding_service.py.
    # import asyncio
    # async def test_retrieval():
    #     results = await retrieve_relevant_content(query)
    #     for r in results:
    #         print(f"Score: {r['score']}, Content: {r['text_content'][:100]}...")
    # asyncio.run(test_retrieval())
    print("Retrieval example finished (requires async run and Qdrant data for full test).")
