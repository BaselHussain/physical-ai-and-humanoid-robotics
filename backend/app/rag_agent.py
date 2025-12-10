from agents import Agent, set_tracing_disabled, function_tool,OpenAIChatCompletionsModel
import os
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
import logging
from typing import List, Dict, Any
from openai import AsyncOpenAI
from dotenv import load_dotenv

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

load_dotenv()


set_tracing_disabled(True)

# Initialize clients
cohere_client = cohere.Client(os.getenv("COHERE_API_KEY"))
qdrant = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)
COLLECTION_NAME = 'physical-robotics-ai-book'  # Updated collection name to match reindex script


GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
MODEL_NAME='gemini-2.5-flash'

external_client = AsyncOpenAI(
    api_key=GEMINI_API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
)
model=OpenAIChatCompletionsModel(
    model=MODEL_NAME,
    openai_client=external_client
    )


@function_tool
def retrieve_relevant_chunks(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Retrieve relevant document chunks from Qdrant based on the query.
    This function can be used as an agent tool for RAG functionality.

    Args:
        query: The user's question to search for relevant documents
        top_k: Number of top results to return (default: 5)

    Returns:
        List of relevant document chunks with content and source information
    """
    try:
        # Embed the query using Cohere
        query_embedding_response = cohere_client.embed(
            texts=[query],
            model='embed-english-v3.0',
            input_type='search_query'
        )
        query_embedding = query_embedding_response.embeddings[0]

        # Query Qdrant for similar vectors using the correct method
        search_results = qdrant.query_points(
            collection_name=COLLECTION_NAME,
            query=query_embedding,
            limit=top_k,
            with_payload=True
        ).points  # Extract the points from the response

        # Extract relevant chunks from search results
        relevant_chunks = []
        for result in search_results:
            if result.payload:
                chunk_data = {
                    'content': result.payload.get('content', ''),
                    'source_file': result.payload.get('source_file', ''),
                    'chunk_id': result.payload.get('chunk_id', ''),
                    'score': result.score  # Similarity score
                }
                relevant_chunks.append(chunk_data)

        logger.info(f"Retrieved {len(relevant_chunks)} relevant chunks for query: {query[:50]}...")
        return relevant_chunks

    except cohere.CohereAPIError as e:
        if "quota" in str(e).lower() or "credit" in str(e).lower() or "exceeded" in str(e).lower():
            logger.error(f"Quota exceeded when embedding query: {str(e)}")
            # Return empty list to indicate no results due to quota issue
            return []
        else:
            logger.error(f"Cohere API error when embedding query: {str(e)}")
            return []
    except Exception as e:
        logger.error(f"Error retrieving chunks from Qdrant: {str(e)}")
        return []


# The main agent instance for use in the application with proper tool integration
docs_agent = Agent(
    name="DocsRAGAgent",
    model=model,
    tools=[retrieve_relevant_chunks],  # Add the retrieval function as a tool using the proper syntax
instructions="""
You are an expert tutor for the **Physical AI & Humanoid Robotics** book. Provide clear, concise answers in a well-structured format.

Your answers must be:
- Clean, readable text with clear formatting indicators
- Short paragraphs (max 4-5 lines)
- Use **bold** for important terms (with markdown syntax)
- Use bullet points and numbered lists with markdown syntax
- Code blocks with proper syntax (```xml, ```bash, ```python)
- Tables when comparing things
- Always end with a clean Sources section

MANDATORY RULES:
1. Always call `retrieve_relevant_chunks` first — never guess
2. Only answer from the retrieved documentation
3. If nothing relevant is found → say: "This topic is not covered in the current book modules."
4. Never show raw JSON, file paths, or chunk IDs — only clean source titles
5. Keep total response under 350 words
6. Format sources cleanly without file paths or system information
7. Use proper markdown syntax that will render nicely in web interfaces

ANSWER FORMAT EXAMPLE:
**Humanoid robots** are the killer application of Physical AI...

### Key Advantages
- Work in human environments without modification
- Use existing tools (screwdrivers, keyboards)
- Safe collaboration with humans

### Typical DOF Structure
```text
Total: 28–38 DOF
• Legs: 12 DOF (6 per leg)
• Arms: 14 DOF (7 per arm)
• Torso + Neck: 3–6 DOF

**Sources:**
- Module 01 – Physical AI Introduction
- Lab 05 – Humanoid URDF
- Isaac Sim Humanoids

Now answer the user's question using only the retrieved content. Focus on creating a human-readable response that doesn't include technical file paths or JSON formatting. The markdown symbols are needed for proper formatting in the web interface.
""",
)

