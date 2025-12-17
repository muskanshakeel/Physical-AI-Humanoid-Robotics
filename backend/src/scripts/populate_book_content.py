#!/usr/bin/env python3
"""
Script to populate the database with book content from the Docusaurus docs directory.
"""

import asyncio
import os
import sys
from pathlib import Path
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker

# Add the backend/src directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.config.settings import settings
from src.models.book_content import BookContent
from src.services.book_content_service import BookContentService
from src.services.vector_index_service import VectorIndexService

async def read_markdown_files(docs_path: str):
    """Read all markdown files from the docs directory."""
    content_list = []

    docs_dir = Path(docs_path)

    # Walk through all directories and subdirectories
    for file_path in docs_dir.rglob("*.md"):
        # Skip tutorial-basics and tutorial-extras which are standard Docusaurus tutorials
        if 'tutorial-basics' in str(file_path) or 'tutorial-extras' in str(file_path):
            continue

        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

            # Extract title from the first line if it's a markdown header
            lines = content.split('\n')
            title = "Untitled"
            for line in lines:
                if line.startswith('# '):
                    title = line[2:].strip()
                    break

            # Use the directory structure to determine the section
            relative_path = file_path.relative_to(docs_dir)
            section = str(relative_path.parent)

            content_list.append({
                'title': title,
                'content': content,
                'section': section,
                'file_path': str(file_path)
            })

    return content_list

async def main():
    """Main function to populate the database with book content."""
    # Create async engine
    engine = create_async_engine(settings.database_url)

    # Create async session maker
    async_session = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)

    # Read content from docs directory
    docs_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'my-website', 'docs')

    if not os.path.exists(docs_path):
        print(f"Docs directory not found: {docs_path}")
        return

    print("Reading markdown files from docs directory...")
    content_list = await read_markdown_files(docs_path)

    print(f"Found {len(content_list)} markdown files")

    # Insert content into the database
    async with async_session() as session:
        for i, content_data in enumerate(content_list):
            print(f"Processing {content_data['file_path']} ({i+1}/{len(content_list)})")

            # Create book content entry
            book_content = await BookContentService.create_content(
                db=session,
                title=content_data['title'],
                content=content_data['content'],
                section=content_data['section']
            )

            # Index the content for vector search
            await VectorIndexService.index_book_content(
                content_id=book_content.id,
                content=content_data['content'],
                title=content_data['title'],
                section=content_data['section']
            )

    print("Database has been populated with book content and vector index created.")

    # Close the engine
    await engine.dispose()

if __name__ == "__main__":
    asyncio.run(main())