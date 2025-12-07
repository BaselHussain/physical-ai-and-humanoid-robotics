# Quickstart: RAG Chatbot Development & Deployment

**Feature**: RAG Chatbot for Physical AI & Humanoid Robotics Book
**Date**: 2025-12-07
**Audience**: Developers implementing the feature

---

## Prerequisites

### Required Tools
- **Node.js**: 18+ LTS
- **Python**: 3.11+
- **uv**: Python package manager (already installed in `backend/`)
- **Git**: Version control
- **GitHub Account**: For Actions and Pages deployment
- **Render Account**: Free tier for backend deployment

### Required API Keys (Already Configured)
- ✅ **Gemini API Key**: In `backend/.env` as `GEMINI_API_KEY`
- ✅ **Cohere API Key**: Hardcoded in `backend/main.py`
- ✅ **Qdrant Cloud URL + API Key**: Hardcoded in `backend/main.py`

---

## Local Development Setup

### 1. Install Frontend Dependencies

```bash
# From project root
npm install

# Install ChatKit React library
npm install @openai/chatkit-react

# Verify installation
npm list @openai/chatkit-react
# Expected: @openai/chatkit-react@<version>
```

### 2. Swizzle Docusaurus Layout Component

```bash
# Wrap the Layout component to inject ChatKit
npm run swizzle @docusaurus/theme-classic Layout -- --wrap

# This creates: src/theme/Layout/index.js
```

### 3. Create ChatKit Widget Component

Create `src/components/ChatKitWidget.tsx`:

```tsx
import React, { useEffect } from 'react';
import { ChatKit, useChatKit } from '@openai/chatkit-react';
import '@openai/chatkit-react/styles.css';

export function ChatKitWidget() {
  const { control, setComposerValue, focusComposer } = useChatKit({
    api: {
      async getClientSecret(existing) {
        if (existing) {
          // Refresh expired token
          const res = await fetch('http://localhost:8000/api/chatkit/refresh', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ token: existing }),
          });
          return (await res.json()).client_secret;
        }

        // Create new session
        const res = await fetch('http://localhost:8000/api/chatkit/session', {
          method: 'POST',
        });
        return (await res.json()).client_secret;
      },
    },
    theme: {
      colorScheme: 'light',
      color: { accent: { primary: '#2D8CFF' } },
      radius: 'round',
      density: 'compact',
    },
    onError: ({ error }) => {
      console.error('ChatKit error:', error);
    },
  });

  // Text highlighting listener
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 3) {
        setComposerValue(`${selectedText}\n\n`);
        focusComposer();
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, [setComposerValue, focusComposer]);

  return (
    <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 1000 }}>
      <ChatKit control={control} className="h-[600px] w-[400px]" />
    </div>
  );
}
```

### 4. Update Swizzled Layout

Edit `src/theme/Layout/index.js`:

```jsx
import React from 'react';
import Layout from '@theme-original/Layout';
import { ChatKitWidget } from '@site/src/components/ChatKitWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <ChatKitWidget />
    </>
  );
}
```

### 5. Setup Backend (OpenAI Agents SDK Integration)

#### Install Dependencies

```bash
cd backend

# Install OpenAI Agents SDK and dependencies
uv pip install "openai-agents-python>=0.6.2" "litellm>=1.0.0" "tenacity>=8.0.0"

# Verify installation
uv pip list | grep -E "openai-agents|litellm|tenacity"
```

#### Create RAG Agent

Create `backend/app/rag_agent.py`:

```python
from agents import Agent
from agents.models import LitellmModel

# Initialize Gemini model via LiteLLM
model = LitellmModel(model="gemini/gemini-1.5-flash")

# Create RAG agent
docs_agent = Agent(
    name="DocsRAGAgent",
    model=model,
    instructions="""You are a helpful assistant for Physical AI and Humanoid Robotics documentation.

RULES:
1. Always cite sources using the provided documentation chunks
2. If the question is unrelated to documentation, politely decline and suggest relevant topics
3. Keep answers concise but comprehensive
4. Format code examples with proper syntax highlighting
5. Use bullet points for lists

When answering:
- Start with the most relevant information
- Include specific examples from the docs
- End with source links to the documentation pages
""",
)
```

#### Create Session Manager

Create `backend/app/session_manager.py`:

```python
from typing import Dict, List
from datetime import datetime, timedelta
from pydantic import BaseModel
import uuid

class ChatMessage(BaseModel):
    message_id: str
    session_id: str
    role: str  # "user" | "assistant"
    content: str
    timestamp: datetime
    source_refs: List[str] | None = None

class ChatSession(BaseModel):
    session_id: str
    created_at: datetime
    messages: List[ChatMessage] = []

# In-memory storage
sessions: Dict[str, ChatSession] = {}

def create_session() -> ChatSession:
    session = ChatSession(
        session_id=str(uuid.uuid4()),
        created_at=datetime.utcnow(),
    )
    sessions[session.session_id] = session
    return session

def get_session(session_id: str) -> ChatSession | None:
    return sessions.get(session_id)

def add_message(session_id: str, role: str, content: str, source_refs: List[str] | None = None) -> ChatMessage:
    session = sessions.get(session_id)
    if not session:
        raise ValueError(f"Session {session_id} not found")

    message = ChatMessage(
        message_id=str(uuid.uuid4()),
        session_id=session_id,
        role=role,
        content=content,
        timestamp=datetime.utcnow(),
        source_refs=source_refs,
    )
    session.messages.append(message)
    return message

def cleanup_expired_sessions():
    """Remove sessions inactive for > 30 minutes"""
    now = datetime.utcnow()
    expired = [
        sid for sid, session in sessions.items()
        if now - session.created_at > timedelta(minutes=30)
    ]
    for sid in expired:
        del sessions[sid]
```

#### Update `backend/main.py` with Chat Endpoints

Add to `backend/main.py`:

```python
from fastapi import FastAPI, HTTPException
from fastapi.responses import StreamingResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from agents import Runner
from agents.rag_agent import docs_agent
from agents.session_manager import create_session, get_session, add_message
import asyncio

app = FastAPI()

# Enable CORS for local development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    message: str
    session_id: str

@app.post("/api/chatkit/session")
async def create_chatkit_session():
    session = create_session()
    # In production, generate JWT client_secret here
    client_secret = f"mock_token_{session.session_id}"
    return {
        "client_secret": client_secret,
        "session_id": session.session_id,
    }

@app.post("/api/chat")
async def chat(request: ChatRequest):
    session = get_session(request.session_id)
    if not session:
        raise HTTPException(status_code=404, detail="Session not found")

    # Add user message
    add_message(request.session_id, "user", request.message)

    async def generate():
        # Stream response from agent
        result = Runner.run_streamed(docs_agent, input=request.message)

        assistant_content = ""
        async for event in result.stream_events():
            if event.type == "raw_response_event":
                delta = event.data.delta
                assistant_content += delta
                yield f"data: {{'type': 'token', 'delta': '{delta}'}}\n\n"

        # Add assistant message (simplified - in production, extract sources)
        add_message(request.session_id, "assistant", assistant_content, source_refs=[])
        yield f"data: {{'type': 'complete'}}\n\n"

    return StreamingResponse(generate(), media_type="text/event-stream")
```

### 6. Run Local Development Servers

#### Terminal 1: Backend

```bash
cd backend
uv run uvicorn main:app --reload --port 8000

# Expected output:
# INFO:     Uvicorn running on http://127.0.0.1:8000
# INFO:     Application startup complete
```

#### Terminal 2: Frontend

```bash
cd .. # Back to project root
npm run start

# Expected output:
# [INFO] Starting the development server...
# [SUCCESS] Docusaurus website is running at http://localhost:3000
```

### 7. Test Locally

1. **Open Browser**: Navigate to `http://localhost:3000/docs/intro`
2. **Verify Widget**: Chat widget should appear in bottom-right corner
3. **Test Highlight**:
   - Highlight text: "NVIDIA Isaac Sim 2025"
   - Widget should open with pre-filled text
4. **Ask Question**: "How does this work with ROS 2?"
5. **Verify Response**:
   - Answer streams token-by-token
   - Source links appear (once Qdrant is indexed)

---

## Production Deployment

### 1. Prepare Render Deployment

Create `render.yaml` in project root:

```yaml
services:
  - type: web
    name: rag-chatbot-api
    env: python
    region: oregon
    plan: free
    buildCommand: cd backend && pip install -r requirements.txt
    startCommand: cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: GEMINI_API_KEY
        sync: false
      - key: PYTHON_VERSION
        value: 3.11
```

### 2. Deploy Backend to Render

```bash
# Push to GitHub
git add .
git commit -m "Add RAG chatbot backend"
git push origin main

# In Render Dashboard:
# 1. Click "New +" → "Blueprint"
# 2. Connect GitHub repository
# 3. Select repository: physical-ai-and-humanoid-robotics
# 4. Render auto-detects render.yaml
# 5. Click "Apply"

# Add environment variables in Render dashboard:
# - GEMINI_API_KEY: <your-key-from-backend/.env>
# - COHERE_API_KEY: <your-key-from-backend/main.py>
# - QDRANT_URL: <your-url-from-backend/main.py>
# - QDRANT_API_KEY: <your-key-from-backend/main.py>
```

### 3. Update Frontend to Use Production Backend

Edit `src/components/ChatKitWidget.tsx`:

```tsx
const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://rag-chatbot-api.onrender.com'
  : 'http://localhost:8000';

// Update fetch URLs
const res = await fetch(`${API_URL}/api/chatkit/session`, { ... });
```

### 4. Deploy Frontend to GitHub Pages

```bash
# Build and deploy
npm run build
npm run deploy

# Verify deployment
# Open: https://<username>.github.io/physical-ai-and-humanoid-robotics/
```

---

## Auto-Reindex Setup (GitHub Actions)

### 1. Create Reindex Workflow

Create `.github/workflows/reindex.yml`:

```yaml
name: Reindex Documentation

on:
  push:
    branches: [main]
    paths:
      - 'docs/**/*.md'

jobs:
  reindex:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - uses: actions/setup-python@v5
        with:
          python-version: '3.11'

      - name: Install dependencies
        run: |
          cd backend
          pip install -r requirements.txt

      - name: Run reindex script
        run: python backend/scripts/reindex_docs.py
        env:
          COHERE_API_KEY: ${{ secrets.COHERE_API_KEY }}
          QDRANT_URL: ${{ secrets.QDRANT_URL }}
          QDRANT_API_KEY: ${{ secrets.QDRANT_API_KEY }}
```

### 2. Create Reindex Script

Create `backend/scripts/reindex_docs.py`:

```python
import os
import glob
from qdrant_client import QdrantClient
import cohere

# Initialize clients
cohere_client = cohere.Client(os.getenv('COHERE_API_KEY'))
qdrant = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY'),
)

def chunk_markdown(file_path: str, max_tokens: int = 384):
    """Split Markdown file into chunks at heading boundaries"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Simple chunking by ## headings
    sections = content.split('\n## ')
    chunks = []
    for i, section in enumerate(sections):
        if i > 0:
            section = '## ' + section
        chunks.append({
            'content': section[:2000],  # Rough token limit approximation
            'chunk_id': f"{file_path}:{i}",
            'source_file': file_path,
        })
    return chunks

def main():
    # Find all Markdown files in docs/
    md_files = glob.glob('docs/**/*.md', recursive=True)

    all_chunks = []
    for file_path in md_files:
        chunks = chunk_markdown(file_path)
        all_chunks.extend(chunks)

    # Batch embed chunks
    texts = [chunk['content'] for chunk in all_chunks]
    embeddings_response = cohere_client.embed(
        texts=texts,
        model='embed-english-v3.0',
        input_type='search_document',
    )

    # Upsert to Qdrant
    points = []
    for chunk, embedding in zip(all_chunks, embeddings_response.embeddings):
        points.append({
            'id': hash(chunk['chunk_id']),  # Convert to int ID
            'vector': embedding,
            'payload': chunk,
        })

    qdrant.upsert(
        collection_name='hackathon-api-cluster',
        points=points,
    )

    print(f"Indexed {len(all_chunks)} chunks from {len(md_files)} files")

if __name__ == '__main__':
    main()
```

### 3. Add Secrets to GitHub

In GitHub repository settings → Secrets and variables → Actions:

1. Click "New repository secret"
2. Add:
   - `COHERE_API_KEY`: <your-cohere-key>
   - `QDRANT_URL`: <your-qdrant-url>
   - `QDRANT_API_KEY`: <your-qdrant-key>

---

## Testing Checklist

### Local Testing

- [ ] Backend starts without errors (`uvicorn main:app`)
- [ ] Frontend starts and builds successfully (`npm run start`)
- [ ] Chat widget appears on all documentation pages
- [ ] Text highlighting pre-fills composer
- [ ] Question submission triggers streaming response
- [ ] Source links appear in responses (after initial reindex)
- [ ] Follow-up questions maintain conversation context
- [ ] Session expires after 30 minutes of inactivity

### Deployment Testing

- [ ] Render deployment completes successfully (green check)
- [ ] Backend health check returns 200 OK (`/health`)
- [ ] GitHub Pages deployment succeeds
- [ ] Chat widget loads on production site
- [ ] CORS is configured correctly (no console errors)
- [ ] API calls to Render backend succeed
- [ ] Streaming responses work over HTTPS

### Auto-Reindex Testing

- [ ] GitHub Actions workflow triggers on push to `main`
- [ ] Reindex script runs without errors
- [ ] Changed files are detected correctly
- [ ] Qdrant receives updated chunks
- [ ] Chat responses reflect new content within 5 minutes

---

## Common Issues & Solutions

### Issue: ChatKit widget not appearing

**Solution**:
- Check browser console for errors
- Verify `@openai/chatkit-react` is installed (`npm list`)
- Ensure `Layout` component is properly swizzled (`src/theme/Layout/index.js` exists)

### Issue: Backend CORS errors

**Solution**:
- Add `CORSMiddleware` to `main.py` with correct origins
- In production, set `allow_origins=["https://<your-github-pages-url>"]`

### Issue: Streaming not working

**Solution**:
- Verify `StreamingResponse` returns `text/event-stream` media type
- Check that events are formatted as `data: {...}\n\n`
- Ensure backend uses `async def` and `yield` for streaming

### Issue: Qdrant connection fails

**Solution**:
- Verify `QDRANT_URL` and `QDRANT_API_KEY` in environment variables
- Test connection: `qdrant.get_collections()` should not raise error
- Check Qdrant dashboard for collection `hackathon-api-cluster`

### Issue: Gemini rate limit errors

**Solution**:
- Verify exponential backoff retry is implemented
- Check free tier quota: 60 requests/minute
- Show "Processing..." message to user during retry

---

## Next Steps

1. ✅ Local development complete
2. ✅ Backend deployed to Render
3. ✅ Frontend deployed to GitHub Pages
4. ✅ Auto-reindex workflow configured
5. ⏭️ Run `/sp.tasks` to generate implementation tasks
6. ⏭️ Implement task-by-task following TDD approach

---

## Status

✅ **Quickstart guide complete** - Ready for agent context update (Phase 1 final step)
