import cohere
from qdrant_client import QdrantClient
from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from agents import Runner
from app.rag_agent import docs_agent
from app.session_manager import create_session, get_session, add_message
from models.chat import ChatRequest
import json

# Existing configuration
SITE_MAP_URL = 'https://baselhussain.github.io/physical-ai-and-humanoid-robotics/sitemap.xml'
COLLECTION_NAME = 'hackathon-api-cluster'

cohere_client = cohere.Client('hANpicZ327eXmRUe1lvMGiKSRSgoSiwyPBAb4KF7')
embed_model = 'embed-english-v3.0'

qdrant = QdrantClient(
    url="https://343a987c-7b19-4609-bf87-557078db2bd9.europe-west3-0.gcp.cloud.qdrant.io:6333",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.Qm3w_29EVGXWz_VpjQ9d1sGlX8wvPIpYfsIQv_dIAYc"
)

# Initialize FastAPI app
app = FastAPI()

# Enable CORS for local development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3001"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "rag-chatbot-api"}


@app.post("/api/chatkit/session")
async def create_chatkit_session():
    """Create a new ChatKit session"""
    session = create_session()
    # In production, generate JWT client_secret here
    client_secret = f"mock_token_{session.session_id}"
    return {
        "client_secret": client_secret,
        "session_id": session.session_id,
    }


@app.post("/api/chatkit/refresh")
async def refresh_chatkit_session(request: dict):
    """Refresh an expired ChatKit token"""
    token = request.get("token", "")
    # Extract session_id from mock token
    if token.startswith("mock_token_"):
        session_id = token.replace("mock_token_", "")
        session = get_session(session_id)
        if session:
            return {"client_secret": token}
    raise HTTPException(status_code=401, detail="Invalid or expired token")


@app.post("/api/chat")
async def chat(request: ChatRequest):
    """Handle chat requests with streaming SSE response"""
    session = get_session(request.session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    # Add user message
    add_message(request.session_id, "user", request.message)

    async def generate():
        try:
            # Use streaming runner - this returns a RunResultStreaming object
            result = Runner.run_streamed(docs_agent, input=request.message)

            assistant_content = ""
            has_content = False

            # Stream events from the result
            async for event in result.stream_events():
                if hasattr(event, 'data'):
                    # Handle different event types appropriately
                    if hasattr(event, 'type') and event.type == "raw_response_event":
                        # Handle raw response events
                        if hasattr(event.data, 'delta'):
                            delta = getattr(event.data, 'delta', '')
                            if delta:
                                assistant_content += delta
                                has_content = True
                                yield f"data: {json.dumps({'type': 'token', 'delta': delta})}\n\n"
                    elif hasattr(event, 'type') and event.type == "error":
                        # Handle error events
                        error_msg = getattr(event.data, 'error', 'An error occurred during processing')
                        yield f"data: {json.dumps({'type': 'error', 'message': str(error_msg)})}\n\n"
                        break

            # Add assistant message (simplified - in production, extract sources from Qdrant)
            if has_content:
                add_message(request.session_id, "assistant", assistant_content, source_refs=[])

            # Always send complete event to signal end of stream
            yield f"data: {json.dumps({'type': 'complete'})}\n\n"

        except Exception as e:
            yield f"data: {json.dumps({'type': 'error', 'message': str(e)})}\n\n"
        finally:
            # Ensure the stream is properly closed
            yield f"data: [DONE]\n\n"

    return StreamingResponse(generate(), media_type="text/event-stream")