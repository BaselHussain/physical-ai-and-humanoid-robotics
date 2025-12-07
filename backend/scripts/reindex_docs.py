import os
import glob
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct
import cohere

# Initialize clients
cohere_client = cohere.Client(os.getenv('COHERE_API_KEY', 'hANpicZ327eXmRUe1lvMGiKSRSgoSiwyPBAb4KF7'))
qdrant = QdrantClient(
    url=os.getenv('QDRANT_URL', 'https://343a987c-7b19-4609-bf87-557078db2bd9.europe-west3-0.gcp.cloud.qdrant.io:6333'),
    api_key=os.getenv('QDRANT_API_KEY', 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Qm3w_29EVGXWz_VpjQ9d1sGlX8wvPIpYfsIQv_dIAYc')
)

COLLECTION_NAME = 'hackathon-api-cluster'
MAX_TOKENS = 384


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
    # Find all Markdown files in docs/
    # Note: Looking in physical-robotics-ai-book/docs/ which is the actual docs directory
    docs_pattern = os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', 'physical-robotics-ai-book', 'docs', '**', '*.md')
    md_files = glob.glob(docs_pattern, recursive=True)

    if not md_files:
        print("No markdown files found. Checking alternative location...")
        # Fallback to project root docs/ if physical-robotics-ai-book doesn't exist
        docs_pattern = os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', 'docs', '**', '*.md')
        md_files = glob.glob(docs_pattern, recursive=True)

    print(f"Found {len(md_files)} markdown files to process")

    all_chunks = []
    for file_path in md_files:
        chunks = chunk_markdown(file_path)
        all_chunks.extend(chunks)

    if not all_chunks:
        print("No chunks created. Exiting.")
        return

    print(f"Created {len(all_chunks)} chunks from {len(md_files)} files")

    # Batch embed chunks
    texts = [chunk['content'] for chunk in all_chunks]
    print(f"Embedding {len(texts)} text chunks...")

    embeddings_response = cohere_client.embed(
        texts=texts,
        model='embed-english-v3.0',
        input_type='search_document',
    )

    # Upsert to Qdrant
    print(f"Upserting to Qdrant collection '{COLLECTION_NAME}'...")
    points = []
    for chunk, embedding in zip(all_chunks, embeddings_response.embeddings):
        # Convert chunk_id to a stable integer ID
        point_id = abs(hash(chunk['chunk_id'])) % (10 ** 9)

        points.append(PointStruct(
            id=point_id,
            vector=embedding,
            payload=chunk,
        ))

    # Batch upsert (Qdrant recommends batches of 100-1000)
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=batch,
        )
        print(f"Upserted batch {i // batch_size + 1} ({len(batch)} points)")

    print(f"\n✅ Indexing complete!")
    print(f"   Files processed: {len(md_files)}")
    print(f"   Chunks created: {len(all_chunks)}")
    print(f"   Collection: {COLLECTION_NAME}")


if __name__ == '__main__':
    main()
