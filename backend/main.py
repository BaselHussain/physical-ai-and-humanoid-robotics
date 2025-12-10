import cohere
from qdrant_client import QdrantClient
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from agents import Runner
from app.rag_agent import docs_agent
from app.session_manager import create_session, get_session, add_message
from models.chat import ChatRequest, SessionResponse
import json
from typing import Any, AsyncIterator
import re


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


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "service": "rag-chatbot-api"}


@app.post("/api/session", response_model=SessionResponse)
async def create_session_endpoint():
    """Create a new chat session and return session_id"""
    session = create_session()
    return SessionResponse(session_id=session.session_id)


@app.post("/api/chat")
async def chat_endpoint(request: ChatRequest):
    """Handle chat requests with streaming SSE response - enhanced with proper RAG"""
    session = get_session(request.session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    # Add user message
    add_message(request.session_id, "user", request.message)

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
                add_message(request.session_id, "assistant", assistant_content, source_refs=[])

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