"""
Auto-Reindex Script for RAG Chatbot
Reads markdown files from physical-robotics-ai-book/docs/, chunks, embeds, and indexes to Qdrant
Supports --force (wipe and reindex) and --incremental (skip unchanged files)
"""

import os
import glob
import uuid
import argparse
import hashlib
import json
from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance
import cohere
from dotenv import load_dotenv
import logging
import time

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Load environment variables from .env file
load_dotenv()

# Initialize clients
cohere_client = cohere.Client(os.getenv('COHERE_API_KEY'))
qdrant = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

COLLECTION_NAME = 'physical-robotics-ai-book'
CHUNK_SIZE = 512  # Characters per chunk
CHUNK_OVERLAP = 128  # Overlap between chunks
# Cohere's embed-english-v3.0 model returns 1024-dimensional vectors for search_document input type
VECTOR_SIZE = 1024
CACHE_FILE = Path(__file__).parent / '.reindex_cache.json'

# Exponential backoff configuration
MAX_RETRIES = 3
INITIAL_BACKOFF = 2  # seconds


def load_cache():
    """Load file hash cache from disk"""
    if CACHE_FILE.exists():
        try:
            with open(CACHE_FILE, 'r') as f:
                return json.load(f)
        except Exception as e:
            logger.warning(f"Failed to load cache: {e}")
    return {}


def save_cache(cache):
    """Save file hash cache to disk"""
    try:
        with open(CACHE_FILE, 'w') as f:
            json.dump(cache, f, indent=2)
    except Exception as e:
        logger.warning(f"Failed to save cache: {e}")


def compute_file_hash(file_path):
    """Compute MD5 hash of file content"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    return hashlib.md5(content.encode('utf-8')).hexdigest()


def chunk_text(text, chunk_size=CHUNK_SIZE, overlap=CHUNK_OVERLAP):
    """Split text into overlapping chunks"""
    chunks = []
    start = 0
    text_length = len(text)

    while start < text_length:
        end = start + chunk_size
        chunk = text[start:end]

        # Only add non-empty chunks
        if chunk.strip():
            chunks.append(chunk.strip())

        start += chunk_size - overlap

    return chunks


def chunk_markdown(file_path: str, docs_dir: Path):
    """
    Split Markdown file into overlapping chunks with metadata.

    Args:
        file_path: Path to the markdown file
        docs_dir: Base documentation directory for relative paths

    Returns:
        List of chunk dictionaries with content, chunk_id, and source_file
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except Exception as e:
        logger.error(f"Error reading {file_path}: {e}")
        return []

    # Get relative path for source reference
    try:
        relative_path = Path(file_path).relative_to(docs_dir)
        source_file = str(relative_path)
    except ValueError:
        source_file = os.path.basename(file_path)

    # Create chunks with overlap
    text_chunks = chunk_text(content)

    # Add metadata to each chunk
    chunks_with_metadata = []
    for idx, chunk_text in enumerate(text_chunks):
        chunk_data = {
            'content': chunk_text,
            'source_file': source_file,
            'chunk_id': f"{source_file}#chunk{idx}",
        }
        chunks_with_metadata.append(chunk_data)

    return chunks_with_metadata


def embed_with_retry(texts, input_type='search_document'):
    """Embed text chunks with exponential backoff retry"""
    for attempt in range(MAX_RETRIES):
        try:
            response = cohere_client.embed(
                texts=texts,
                model='embed-english-v3.0',
                input_type=input_type
            )
            return response.embeddings

        except cohere.CohereAPIError as e:
            if "quota" in str(e).lower() or "credit" in str(e).lower() or "rate" in str(e).lower():
                if attempt < MAX_RETRIES - 1:
                    backoff_time = INITIAL_BACKOFF * (2 ** attempt)
                    logger.warning(f"Rate limit hit, retrying in {backoff_time}s (attempt {attempt + 1}/{MAX_RETRIES})")
                    time.sleep(backoff_time)
                else:
                    logger.error(f"Max retries reached. Quota/rate limit error: {e}")
                    raise
            else:
                logger.error(f"Cohere API error: {e}")
                raise
        except Exception as e:
            logger.error(f"Unexpected error during embedding: {e}")
            raise

    return []


def main():
    """Main reindexing function"""
    parser = argparse.ArgumentParser(description='Reindex documentation for RAG chatbot')
    parser.add_argument('--force', action='store_true', help='Wipe and reindex all documents')
    parser.add_argument('--incremental', action='store_true', help='Only reindex changed files (default)')
    args = parser.parse_args()

    # Default to incremental if neither flag specified
    if not args.force and not args.incremental:
        args.incremental = True

    logger.info("=" * 60)
    logger.info("RAG Chatbot Documentation Reindexer")
    logger.info(f"Mode: {'FORCE (wipe all)' if args.force else 'INCREMENTAL (changed files only)'}")
    logger.info("=" * 60)

    try:
        # Setup Qdrant collection
        collections = qdrant.get_collections().collections
        collection_exists = any(col.name == COLLECTION_NAME for col in collections)

        if collection_exists:
            if args.force:
                logger.info(f"Force mode: Deleting existing collection '{COLLECTION_NAME}'")
                qdrant.delete_collection(collection_name=COLLECTION_NAME)
                collection_exists = False
            else:
                logger.info(f"Collection '{COLLECTION_NAME}' already exists, using incremental mode")

        if not collection_exists:
            logger.info(f"Creating new collection '{COLLECTION_NAME}'")
            qdrant.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=VECTOR_SIZE,
                    distance=Distance.COSINE
                )
            )

        # Find markdown files
        docs_dir = Path(__file__).parent.parent.parent / 'physical-robotics-ai-book' / 'docs'
        if not docs_dir.exists():
            logger.error(f"Documentation directory not found: {docs_dir}")
            return

        md_files = sorted(docs_dir.rglob('*.md'))
        md_files = [f for f in md_files if not f.name.startswith('.')]
        logger.info(f"Found {len(md_files)} markdown files")

        if not md_files:
            logger.warning("No markdown files found to index")
            return

        # Load cache for incremental mode
        cache = {}
        if args.incremental and not args.force:
            cache = load_cache()

        # Process files
        files_to_process = []
        for file_path in md_files:
            file_hash = compute_file_hash(file_path)

            # Skip unchanged files in incremental mode
            if args.incremental and not args.force:
                if str(file_path) in cache and cache[str(file_path)] == file_hash:
                    logger.info(f"Skipping unchanged file: {file_path.name}")
                    continue

            files_to_process.append((file_path, file_hash))

        if not files_to_process:
            logger.info("No files need reindexing")
            return

        logger.info(f"Processing {len(files_to_process)} files")

        # Process each file
        all_chunks = []
        new_cache = cache.copy()
        for file_path, file_hash in files_to_process:
            logger.info(f"Processing: {file_path.name}")
            chunks = chunk_markdown(str(file_path), docs_dir)
            all_chunks.extend(chunks)
            new_cache[str(file_path)] = file_hash
            logger.info(f"  -> Generated {len(chunks)} chunks")

        if not all_chunks:
            logger.warning("No chunks created")
            return

        # Embed chunks in batches with retry
        logger.info(f"Embedding {len(all_chunks)} chunks")
        texts = [chunk['content'] for chunk in all_chunks]
        all_embeddings = []

        embed_batch_size = 50
        for i in range(0, len(texts), embed_batch_size):
            batch = texts[i:i + embed_batch_size]
            logger.info(f"Embedding batch {i // embed_batch_size + 1}/{(len(texts) + embed_batch_size - 1) // embed_batch_size} ({len(batch)} chunks)")
            embeddings = embed_with_retry(batch, input_type='search_document')
            all_embeddings.extend(embeddings)

        # Create points for Qdrant
        points = []
        for idx, (chunk, embedding) in enumerate(zip(all_chunks, all_embeddings)):
            point = PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload=chunk
            )
            points.append(point)

        # Upsert to Qdrant in batches
        batch_size = 100
        total_upserted = 0
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            qdrant.upsert(
                collection_name=COLLECTION_NAME,
                points=batch
            )
            total_upserted += len(batch)
            logger.info(f"Upserted batch {i // batch_size + 1} ({len(batch)} points) - Total: {total_upserted}/{len(points)}")

        # Save cache
        if args.incremental or args.force:
            save_cache(new_cache)
            logger.info("Cache updated")

        # Validate
        collection_info = qdrant.get_collection(COLLECTION_NAME)
        logger.info("=" * 60)
        logger.info("Reindexing complete!")
        logger.info(f"Total files processed: {len(files_to_process)}")
        logger.info(f"Total chunks indexed: {len(all_chunks)}")
        logger.info(f"Points in collection: {collection_info.points_count}")
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"Error during reindexing: {e}")
        raise


if __name__ == '__main__':
    main()
