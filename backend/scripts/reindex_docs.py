import os
import glob
import uuid
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, VectorParams, Distance
import cohere
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Initialize clients
cohere_client = cohere.Client(os.getenv('COHERE_API_KEY'))
qdrant = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

COLLECTION_NAME = 'physical-robotics-ai-book'
MAX_TOKENS = 384
# Cohere's embed-english-v3.0 model returns 1024-dimensional vectors for search_document input type
VECTOR_SIZE = 1024


def chunk_markdown(file_path: str, max_tokens: int = MAX_TOKENS):
    """
    Split Markdown file into chunks at heading boundaries.

    Args:
        file_path: Path to the markdown file
        max_tokens: Maximum tokens per chunk (default: 384)

    Returns:
        List of chunk dictionaries with content, chunk_id, and source_file
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return []

    # Simple chunking by ## headings
    sections = content.split('\n## ')
    chunks = []

    for i, section in enumerate(sections):
        if i > 0:
            section = '## ' + section

        # Rough token limit approximation (1 token ≈ 4 characters)
        max_chars = max_tokens * 4
        chunk_content = section[:max_chars]

        chunks.append({
            'content': chunk_content,
            'chunk_id': f"{file_path}:{i}",
            'source_file': file_path,
        })

    return chunks


def main():
    """Main reindexing function"""
    try:
        # Explicitly create collection with optimal settings
        print(f"Checking/creating Qdrant collection: {COLLECTION_NAME}")
        try:
            # Try to get collection info to see if it exists
            collection_info = qdrant.get_collection(COLLECTION_NAME)
            print(f"Collection '{COLLECTION_NAME}' already exists")
        except Exception:
            # Collection doesn't exist, create it with optimal settings
            print(f"Creating collection '{COLLECTION_NAME}' with optimal settings...")
            qdrant.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=VECTOR_SIZE,  # Cohere embed-english-v3.0 returns 1024-dim vectors
                    distance=Distance.COSINE  # Cosine distance is optimal for text embeddings
                )
            )
            print(f"Collection '{COLLECTION_NAME}' created successfully")

        # Find all Markdown files in docs/
        # Note: Looking in physical-robotics-ai-book/docs/ which is the actual docs directory
        docs_pattern = os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', 'physical-robotics-ai-book', 'docs', '**', '*.md')
        md_files = glob.glob(docs_pattern, recursive=True)

        if not md_files:
            print("No markdown files found. Checking alternative location...")
            # Fallback to project root docs/ if physical-robotics-ai-book doesn't exist
            docs_pattern = os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', 'docs', '**', '*.md')
            md_files = glob.glob(docs_pattern, recursive=True)

        if not md_files:
            print("No markdown files found in either location. Exiting.")
            return

        print(f"Found {len(md_files)} markdown files to process")

        all_chunks = []
        for file_path in md_files:
            chunks = chunk_markdown(file_path)
            all_chunks.extend(chunks)

        if not all_chunks:
            print("No chunks created. Exiting.")
            return

        print(f"Created {len(all_chunks)} chunks from {len(md_files)} files")

        # Batch embed chunks to avoid rate limits
        texts = [chunk['content'] for chunk in all_chunks]
        print(f"Embedding {len(texts)} text chunks...")

        import time
        all_embeddings = []

        # Process in smaller batches to avoid rate limits
        embed_batch_size = 10  # Reduced from default to avoid rate limiting
        for i in range(0, len(texts), embed_batch_size):
            text_batch = texts[i:i + embed_batch_size]
            print(f"  Embedding batch {i // embed_batch_size + 1} ({len(text_batch)} items)...")

            batch_response = cohere_client.embed(
                texts=text_batch,
                model='embed-english-v3.0',
                input_type='search_document',
            )
            all_embeddings.extend(batch_response.embeddings)

            # Add small delay between batches to respect rate limits
            if i + embed_batch_size < len(texts):
                time.sleep(1)  # 1 second delay between batches

        embeddings_response = type('obj', (object,), {'embeddings': all_embeddings})()

        # Prepare points for upsert
        print(f"Preparing points for Qdrant collection '{COLLECTION_NAME}'...")
        points = []
        for chunk, embedding in zip(all_chunks, embeddings_response.embeddings):
            # Use UUID to avoid ID collisions
            point_id = str(uuid.uuid4())

            points.append(PointStruct(
                id=point_id,
                vector=embedding,
                payload=chunk,
            ))

        # Batch upsert (Qdrant recommends batches of 100-1000)
        batch_size = 100
        total_upserted = 0
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            qdrant.upsert(
                collection_name=COLLECTION_NAME,
                points=batch,
            )
            total_upserted += len(batch)
            print(f"Upserted batch {i // batch_size + 1} ({len(batch)} points) - Total: {total_upserted}/{len(points)}")

        # Validate that all points were indexed
        collection_info = qdrant.get_collection(COLLECTION_NAME)
        indexed_count = collection_info.points_count
        expected_count = len(points)

        print(f"\n✅ Indexing complete!")
        print(f"   Files processed: {len(md_files)}")
        print(f"   Chunks created: {len(all_chunks)}")
        print(f"   Points upserted: {total_upserted}")
        print(f"   Points in collection: {indexed_count}")
        print(f"   Collection: {COLLECTION_NAME}")

        if indexed_count == expected_count:
            print(f"   ✅ All {indexed_count} points successfully indexed")
        else:
            print(f"   ⚠️  Expected {expected_count} points, but {indexed_count} found in collection")

    except Exception as e:
        print(f"X Error during reindexing: {str(e)}")
        raise


if __name__ == '__main__':
    main()
