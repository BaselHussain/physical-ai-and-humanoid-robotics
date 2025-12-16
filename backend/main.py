import cohere
from qdrant_client import QdrantClient
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import StreamingResponse, Response
from fastapi.middleware.cors import CORSMiddleware
from agents import Runner
from app.rag_agent import docs_agent
from app.session_manager import create_session, get_session, add_message
from models.chat import ChatRequest, SessionResponse
import json
from typing import Any, AsyncIterator
import re
import logging

# ChatKit imports
from app.chatkit_server import RAGChatKitServer
from app.chatkit_store import MemoryStore
from chatkit.server import StreamingResult

# Database imports for Neon Postgres persistence
from app import database
from app import chat_history

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# Configuration
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

# Enable CORS for local development and production
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3001", "https://baselhussain.github.io"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Database lifecycle management
@app.on_event("startup")
async def startup_event():
    """Initialize database connection pool on startup"""
    try:
        logger.info("🚀 Starting RAG Chatbot API...")
        # Initialize Neon Postgres connection pool
        await database.get_pool()
        logger.info("✅ Database connection pool initialized")
    except ValueError as e:
        # NEON_DATABASE_URL not set - log warning but don't crash (optional feature)
        logger.warning(f"⚠️ Database persistence disabled: {e}")
        logger.warning("Chat history will not be saved. Add NEON_DATABASE_URL to .env to enable persistence.")
    except Exception as e:
        # Database connection failed - log error but continue (graceful degradation)
        logger.error(f"❌ Database initialization failed: {e}")
        logger.error("Chat history persistence will not work. Fix database connection to enable it.")


@app.on_event("shutdown")
async def shutdown_event():
    """Close database connection pool on shutdown"""
    try:
        logger.info("👋 Shutting down RAG Chatbot API...")
        await database.close_pool()
        logger.info("✅ Database connection pool closed")
    except Exception as e:
        logger.error(f"Error closing database pool: {e}")

# Define health and API routes FIRST (before mounting ChatKit)
# This ensures these routes take precedence

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "rag-chatbot-api"}


@app.post("/api/session", response_model=SessionResponse)
async def create_session_endpoint():
    """Create a new chat session and return session_id"""
    # Create session in memory (existing functionality)
    session = create_session()

    # ALSO create session in Neon database for persistence (new feature)
    try:
        # Use session_id as user_identifier (could be enhanced with browser fingerprint later)
        await chat_history.create_session(user_identifier=session.session_id)
        logger.info(f"✅ Session {session.session_id} persisted to database")
    except Exception as e:
        # Log error but don't fail - graceful degradation if database is unavailable
        logger.error(f"⚠️ Failed to persist session to database: {e}")
        logger.info("Session will work normally, but history won't be saved")

    return SessionResponse(session_id=session.session_id)


@app.post("/api/chat")
async def chat_endpoint(request: ChatRequest):
    """Handle chat requests with streaming SSE response - enhanced with proper RAG"""
    session = get_session(request.session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    # Add user message to in-memory session (existing functionality)
    add_message(request.session_id, "user", request.message)

    # ALSO save user message to database (new feature)
    try:
        await chat_history.save_message(
            session_id=request.session_id,
            message_text=request.message,
            role="user",
            source_references=[]
        )
        logger.debug(f"User message saved to database for session {request.session_id}")
    except Exception as e:
        logger.error(f"⚠️ Failed to save user message to database: {e}")

    async def generate():
        try:
            # Use the streaming runner with our RAG-enabled docs_agent
            # The agent will automatically retrieve relevant documentation before responding
            result = Runner.run_streamed(docs_agent, input=request.message)

            assistant_content = ""
            has_content = False

            # Stream events from the result
            async for event in result.stream_events():
                if hasattr(event, 'data'):
                    # Handle different event types appropriately
                    if hasattr(event, 'type') and event.type == "raw_response_event":
                        # Handle raw response events (tokens)
                        if hasattr(event.data, 'delta'):
                            delta = getattr(event.data, 'delta', '')
                            if delta:
                                # Check if this delta starts with query information and filter it out if needed
                                if delta.startswith('{') and ('"query"' in delta or '"input"' in delta):
                                    # Skip query JSON that might be included in the response
                                    continue
                                assistant_content += delta
                                has_content = True
                                yield f"data: {json.dumps({'type': 'token', 'delta': delta})}\n\n"
                    elif hasattr(event, 'type') and event.type == "tool_call":
                        # Handle tool call events (like search_documentation)
                        tool_name = getattr(event.data, 'name', 'unknown')
                        tool_args = getattr(event.data, 'arguments', {})
                        yield f"data: {json.dumps({'type': 'tool_call', 'tool_name': tool_name, 'arguments': tool_args})}\n\n"
                    elif hasattr(event, 'type') and event.type == "tool_result":
                        # Handle tool result events (search results)
                        tool_result = getattr(event.data, 'content', '')
                        yield f"data: {json.dumps({'type': 'tool_result', 'content': tool_result})}\n\n"
                    elif hasattr(event, 'type') and event.type == "error":
                        # Handle error events
                        error_msg = getattr(event.data, 'error', 'An error occurred during processing')
                        # Check if the error is related to quota exceeded (Google API specific)
                        error_str = str(error_msg).lower()
                        if ("quota" in error_str or "credit" in error_str or "exceeded" in error_str or
                            "rate limit" in error_str or "429" in error_str or
                            "billing" in error_str or "usage limit" in error_str):
                            yield f"data: {json.dumps({'type': 'error', 'message': 'Quota exceeded. Please try again later.'})}\n\n"
                        else:
                            yield f"data: {json.dumps({'type': 'error', 'message': str(error_msg)})}\n\n"
                        break

            # Add assistant message with source references if content was generated
            if has_content:
                # Extract source references from the agent's response (this would require
                # capturing tool calls or having the agent include citations in its output)
                # For now, we'll add without explicit source_refs, but the agent includes
                # source citations in its response based on retrieved documents

                # Add to in-memory session (existing functionality)
                add_message(request.session_id, "assistant", assistant_content, source_refs=[])

                # ALSO save assistant message to database (new feature)
                try:
                    await chat_history.save_message(
                        session_id=request.session_id,
                        message_text=assistant_content,
                        role="assistant",
                        source_references=[]  # TODO: Extract actual source references from agent response
                    )
                    logger.debug(f"Assistant message saved to database for session {request.session_id}")
                except Exception as e:
                    logger.error(f"⚠️ Failed to save assistant message to database: {e}")

            # Send complete event to signal end of stream
            yield f"data: {json.dumps({'type': 'complete'})}\n\n"

        except Exception as e:
            # Check if the exception is related to quota exceeded (Google API specific)
            error_str = str(e).lower()
            if ("quota" in error_str or "credit" in error_str or "exceeded" in error_str or
                "rate limit" in error_str or "429" in error_str or
                "billing" in error_str or "usage limit" in error_str):
                yield f"data: {json.dumps({'type': 'error', 'message': 'Quota exceeded. Please try again later.'})}\n\n"
            else:
                yield f"data: {json.dumps({'type': 'error', 'message': str(e)})}\n\n"
        finally:
            # Ensure the stream is properly closed
            yield f"data: [DONE]\n\n"

    return StreamingResponse(generate(), media_type="text/event-stream")


@app.get("/api/sessions/{session_id}/restore")
async def restore_session_endpoint(session_id: str):
    """
    Restore chat history for a session from the database.
    This endpoint enables chat history persistence across browser sessions.

    Args:
        session_id: UUID of the session to restore

    Returns:
        dict: Session info and message history

    Raises:
        HTTPException: 404 if session not found, 503 if database unavailable
    """
    try:
        # Get session metadata
        session = await chat_history.get_session(session_id)

        if session is None:
            raise HTTPException(
                status_code=404,
                detail=f"Session {session_id} not found in database"
            )

        # Get all messages for this session
        messages = await chat_history.get_session_history(session_id)

        # Convert messages to dict format for JSON response
        messages_dict = [msg.to_dict() for msg in messages]

        logger.info(f"✅ Restored {len(messages)} messages for session {session_id}")

        return {
            "session": session.to_dict(),
            "messages": messages_dict,
            "message_count": len(messages_dict)
        }

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except ValueError as e:
        # Database not configured
        raise HTTPException(
            status_code=503,
            detail="Chat history persistence is not enabled. Database connection not configured."
        )
    except Exception as e:
        logger.error(f"❌ Failed to restore session {session_id}: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to restore session: {str(e)}"
        )


@app.get("/api/debug/sessions")
async def debug_view_sessions():
    """
    Debug endpoint to view all chat sessions in the database.
    Access in browser: http://localhost:8000/api/debug/sessions
    """
    try:
        pool = await database.get_pool()
        async with pool.acquire() as conn:
            # Get all sessions with message counts
            rows = await conn.fetch("""
                SELECT
                    cs.session_id,
                    cs.user_identifier,
                    cs.created_at,
                    cs.last_active_at,
                    COUNT(cm.message_id) as message_count
                FROM chat_sessions cs
                LEFT JOIN chat_messages cm ON cs.session_id = cm.session_id
                GROUP BY cs.session_id, cs.user_identifier, cs.created_at, cs.last_active_at
                ORDER BY cs.last_active_at DESC
                LIMIT 50
            """)

            sessions = []
            for row in rows:
                sessions.append({
                    "session_id": str(row['session_id']),
                    "user_identifier": row['user_identifier'],
                    "created_at": row['created_at'].isoformat(),
                    "last_active_at": row['last_active_at'].isoformat(),
                    "message_count": row['message_count']
                })

            return {
                "total_sessions": len(sessions),
                "sessions": sessions
            }

    except ValueError as e:
        raise HTTPException(
            status_code=503,
            detail="Database not configured"
        )
    except Exception as e:
        logger.error(f"Failed to fetch sessions: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to fetch sessions: {str(e)}"
        )


@app.get("/api/debug/messages")
async def debug_view_messages(session_id: str = None, limit: int = 50):
    """
    Debug endpoint to view all chat messages in the database.
    Access in browser:
    - All messages: http://localhost:8000/api/debug/messages
    - Specific session: http://localhost:8000/api/debug/messages?session_id=YOUR_SESSION_ID
    - Custom limit: http://localhost:8000/api/debug/messages?limit=100
    """
    try:
        import uuid
        pool = await database.get_pool()
        async with pool.acquire() as conn:
            if session_id:
                # Get messages for specific session
                # Convert session_id string to UUID
                session_uuid = uuid.UUID(session_id)
                rows = await conn.fetch("""
                    SELECT
                        cm.message_id,
                        cm.session_id,
                        cm.role,
                        cm.message_text,
                        cm.timestamp,
                        cm.source_references,
                        cs.user_identifier
                    FROM chat_messages cm
                    JOIN chat_sessions cs ON cm.session_id = cs.session_id
                    WHERE cm.session_id = $1
                    ORDER BY cm.timestamp ASC
                    LIMIT $2
                """, session_uuid, limit)
            else:
                # Get all messages across all sessions
                rows = await conn.fetch("""
                    SELECT
                        cm.message_id,
                        cm.session_id,
                        cm.role,
                        cm.message_text,
                        cm.timestamp,
                        cm.source_references,
                        cs.user_identifier
                    FROM chat_messages cm
                    JOIN chat_sessions cs ON cm.session_id = cs.session_id
                    ORDER BY cm.timestamp DESC
                    LIMIT $1
                """, limit)

            messages = []
            for row in rows:
                messages.append({
                    "message_id": str(row['message_id']),
                    "session_id": str(row['session_id']),
                    "user_identifier": row['user_identifier'],
                    "role": row['role'],
                    "message_text": row['message_text'][:200] + "..." if len(row['message_text']) > 200 else row['message_text'],
                    "full_message_text": row['message_text'],
                    "timestamp": row['timestamp'].isoformat(),
                    "source_references": row['source_references']
                })

            return {
                "total_messages": len(messages),
                "session_id_filter": session_id,
                "messages": messages
            }

    except ValueError as e:
        raise HTTPException(
            status_code=503,
            detail="Database not configured"
        )
    except Exception as e:
        logger.error(f"Failed to fetch messages: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Failed to fetch messages: {str(e)}"
        )


def convert_simple_markdown_to_html(text):
    """
    Convert simple markdown syntax to HTML for better frontend rendering.
    Handles: **bold**, *italic*, ### headers, - lists, and | tables
    """
    if not text:
        return text

    # Convert headers: ### Header -> <h3>Header</h3>
    text = re.sub(r'^### (.*?)(\n|$)', r'<h3>\1</h3>\2', text, flags=re.MULTILINE)
    text = re.sub(r'^## (.*?)(\n|$)', r'<h2>\1</h2>\2', text, flags=re.MULTILINE)
    text = re.sub(r'^# (.*?)(\n|$)', r'<h1>\1</h1>\2', text, flags=re.MULTILINE)

    # Convert bold: **text** -> <strong>text</strong>
    text = re.sub(r'\*\*(.*?)\*\*', r'<strong>\1</strong>', text)

    # Convert italic: *text* -> <em>text</em>
    text = re.sub(r'\*(.*?)\*', r'<em>\1</em>', text)

    # Convert lists: - item -> <ul><li>item</li></ul>
    # This is more complex as we need to group consecutive list items
    lines = text.split('\n')
    result_lines = []
    in_list = False

    for line in lines:
        if line.strip().startswith('- '):
            if not in_list:
                result_lines.append('<ul>')
                in_list = True
            # Extract the list item content
            item_content = line[2:]  # Remove '- ' prefix
            # Process markdown within the list item
            item_content = re.sub(r'\*\*(.*?)\*\*', r'<strong>\1</strong>', item_content)
            item_content = re.sub(r'\*(.*?)\*', r'<em>\1</em>', item_content)
            result_lines.append(f'<li>{item_content}</li>')
        else:
            if in_list:
                result_lines.append('</ul>')
                in_list = False
            result_lines.append(line)

    if in_list:
        result_lines.append('</ul>')

    text = '\n'.join(result_lines)

    # Convert simple tables (this is a basic implementation)
    # Look for markdown table patterns and convert to HTML table
    lines = text.split('\n')
    result_lines = []
    in_table = False

    for i, line in enumerate(lines):
        if re.match(r'^\|.*\|$', line) and (i == 0 or re.match(r'^\|.*\|$', lines[i-1])):
            if not in_table:
                result_lines.append('<table>')
                in_table = True
            # Convert table row
            cells = re.findall(r'\|(.*?)\|', line)
            cell_html = ''.join([f'<td>{cell.strip()}</td>' for cell in cells])
            result_lines.append(f'<tr>{cell_html}</tr>')
        else:
            if in_table:
                result_lines.append('</table>')
                in_table = False
            result_lines.append(line)

    if in_table:
        result_lines.append('</table>')

    text = '\n'.join(result_lines)

    return text


# Initialize ChatKit server
chatkit_store = MemoryStore()
chatkit_server = RAGChatKitServer(chatkit_store)


# ChatKit OPTIONS handler for CORS preflight
@app.options("/chatkit")
async def chatkit_options():
    """Handle CORS preflight requests for ChatKit endpoint"""
    return Response(status_code=200)


# ChatKit endpoint following official examples
@app.post("/chatkit")
async def chatkit_endpoint(request: Request):
    """
    ChatKit endpoint that processes all ChatKit requests and returns streaming responses.
    Follows the official openai-chatkit-advanced-samples pattern.
    """
    try:
        # Log request details for debugging
        print(f"ChatKit request - Method: {request.method}, Headers: {dict(request.headers)}")

        payload = await request.body()
        print(f"ChatKit request body length: {len(payload)} bytes")

        # Handle empty body gracefully
        if not payload or payload == b'':
            print("WARNING: Received empty request body")
            raise HTTPException(status_code=400, detail="Request body cannot be empty. ChatKit requires a valid JSON payload.")

        result = await chatkit_server.process(payload, {"request": request})

        if isinstance(result, StreamingResult):
            return StreamingResponse(result, media_type="text/event-stream")

        if hasattr(result, "json"):
            return Response(content=result.json, media_type="application/json")

        return result

    except HTTPException:
        raise
    except Exception as e:
        # Log the error for debugging
        print(f"ChatKit endpoint error: {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"ChatKit processing error: {str(e)}")
    