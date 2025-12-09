# rag-chatbot/backend/app/services/rag_service.py

import os
from typing import List
from dotenv import load_dotenv

from ..models.rag_models import Query, Answer, Source
from .retrieval_service import retrieve_relevant_content
from .embedding_service import get_dummy_embedding # for query embedding if not using LLM API for it

load_dotenv()

# TODO: Replace with actual OpenAI/LLM API client setup
# For demonstration, we'll use a simple static response
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY", "your_openai_api_key")

async def generate_rag_answer(query: Query) -> Answer:
    """
    Generates an answer using RAG logic: retrieve relevant content and then query LLM.
    """
    relevant_chunks = []
    if query.context_text:
        # Prioritize selected text as primary context
        relevant_chunks.append({"text_content": query.context_text, "metadata": {"source": "selected_text"}})
        print(f"Using selected text as primary context: {query.context_text[:50]}...")
    else:
        # Retrieve content from Qdrant if no explicit context is provided
        retrieved_content = await retrieve_relevant_content(query.query_text, top_k=5)
        relevant_chunks.extend(retrieved_content)
        print(f"Retrieved {len(retrieved_content)} chunks from Qdrant.")

    if not relevant_chunks:
        return Answer(
            answer_text="I could not find a relevant answer in the textbook for your question.",
            sources=[],
            is_cited=False,
            is_relevant=False
        )

    # Concatenate relevant text for the LLM prompt
    context_for_llm = "\n".join([chunk["text_content"] for chunk in relevant_chunks])
    
    # TODO: Integrate with actual LLM API (e.g., OpenAI, Anthropic, Gemini)
    # For now, a dummy LLM response
    llm_prompt = f"Given the following context from the textbook:\n\n{context_for_llm}\n\nAnswer the question: {query.query_text}"
    print(f"LLM Prompt (first 200 chars): {llm_prompt[:200]}...")

    # Dummy LLM response based on keywords
    answer_text = "This is a dummy answer based on the retrieved context. "
    if "ROS 2" in query.query_text:
        answer_text += "ROS 2 is a key concept discussed in the provided text."
    elif "Isaac Sim" in query.query_text:
        answer_text += "Isaac Sim is a relevant topic from the provided context."
    elif "robot" in query.query_text:
        answer_text += "Robots are central to the content you provided."
    else:
        answer_text += "The provided context contains information related to your query."

    # Simulate citation generation
    sources = []
    is_cited = False
    for chunk in relevant_chunks:
        meta = chunk.get("metadata", {})
        source = Source(
            chapter_title=meta.get("title", "Unknown Chapter"), # Assuming 'title' is chapter title
            section_title=meta.get("section", None),
            text_snippet=chunk["text_content"][:50] + "...",
            url=meta.get("url", None)
        )
        sources.append(source)
        if meta.get("source") != "selected_text": # Assume retrieved content from Qdrant is cited
            is_cited = True

    return Answer(
        answer_text=answer_text,
        sources=sources,
        is_cited=is_cited,
        is_relevant=True # Assume it's relevant if context was found
    )

if __name__ == "__main__":
    # Example Usage
    query_example = Query(query_text="What are the main components of ROS 2?")
    # or with context
    # query_example = Query(query_text="What are nodes?", context_text="Nodes are the basic unit of computation in ROS 2.")
    
    # This example requires async run and ideally a populated Qdrant
    print("Running rag_service example. Make sure Qdrant is accessible and has data.")
    # import asyncio
    # async def test_rag_generation():
    #     answer = await generate_rag_answer(query_example)
    #     print(f"Generated Answer: {answer.answer_text}")
    #     for src in answer.sources:
    #         print(f"  Source: {src.chapter_title} - {src.text_snippet}")
    #     print(f"Is Cited: {answer.is_cited}, Is Relevant: {answer.is_relevant}")
    # asyncio.run(test_rag_generation())
    print("RAG generation example finished (requires async run for full test).")
