# rag-chatbot/data/ingest.py

import os
import glob
import re
from pathlib import Path
import asyncio

# Assuming these imports are available in the project's Python environment
# For demonstration, we use relative imports, adjust for actual deployment
from ..backend.app.services.embedding_service import embed_and_store_content
from ..backend.app.services.qdrant_service import get_qdrant_client, COLLECTION_NAME

async def ingest_markdown_content(docs_path: Path):
    """
    Reads all markdown files from the specified docs_path, extracts content,
    generates embeddings, and stores them in Qdrant.
    """
    print(f"Starting ingestion from {docs_path}...")
    
    md_files = glob.glob(str(docs_path / "**/*.md"), recursive=True)
    md_files.extend(glob.glob(str(docs_path / "**/*.mdx"), recursive=True))

    client = get_qdrant_client()
    if not client:
        print("Qdrant client not available. Ingestion aborted.")
        return

    # Clear existing collection for fresh ingest (optional, for testing)
    try:
        if client.collection_exists(collection_name=COLLECTION_NAME):
            client.delete_collection(collection_name=COLLECTION_NAME)
            print(f"Deleted existing collection '{COLLECTION_NAME}'.")
        # Re-initialize after deletion
        from ..backend.app.services.qdrant_service import initialize_qdrant_collection
        initialize_qdrant_collection(client)
    except Exception as e:
        print(f"Error during collection reset: {e}")
        return

    for file_path in md_files:
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Extract title (H1) and clean content
            title_match = re.search(r'^#\s*(.*)', content, re.MULTILINE)
            title = title_match.group(1).strip() if title_match else Path(file_path).stem.replace('-', ' ').title()
            
            # Remove frontmatter if present (common in Docusaurus)
            content = re.sub(r'---\s*[\s\S]*?---\s*', '', content, 1)
            
            # Remove markdown syntax for cleaner text (simplified)
            clean_content = re.sub(r'#+\s*', '', content) # remove headers
            clean_content = re.sub(r'\[(.*?)\]\(.*?\)', r'\1', clean_content) # remove links
            clean_content = re.sub(r'\*\*(.*?)\*\*', r'\1', clean_content) # remove bold
            clean_content = re.sub(r'__(.*?)__', r'\1', clean_content) # remove bold
            clean_content = re.sub(r'_(.*?)_', r'\1', clean_content) # remove italics
            clean_content = re.sub(r'`(.*?)`', r'\1', clean_content) # remove inline code
            clean_content = re.sub(r'```[\s\S]*?```', '', clean_content) # remove code blocks
            clean_content = re.sub(r'\s+', ' ', clean_content).strip() # normalize whitespace


            # Create a unique ID for the content chunk
            content_id = str(Path(file_path).relative_to(docs_path)).replace(os.sep, "__")

            # Simple chunking: take the whole document for now.
            # For large documents, proper chunking would be necessary.
            
            metadata = {
                "source_file": str(Path(file_path).relative_to(docs_path)),
                "title": title,
                "url": f"/docs/{Path(file_path).relative_to(docs_path).with_suffix('')}" # Docusaurus URL style
            }

            success = await embed_and_store_content(content_id, clean_content, metadata)
            if success:
                print(f"Successfully ingested: {file_path}")
            else:
                print(f"Failed to ingest: {file_path}")

        except Exception as e:
            print(f"Error processing file {file_path}: {e}")

    print("Ingestion process completed.")

if __name__ == "__main__":
    # Adjust this path to your Docusaurus docs directory
    # For this example, assuming 'my-website/docs' relative to script's dir
    current_dir = Path(__file__).parent
    docs_root = current_dir.parent.parent / "my-website" / "docs"
    
    # Run the async ingestion
    asyncio.run(ingest_markdown_content(docs_root))
