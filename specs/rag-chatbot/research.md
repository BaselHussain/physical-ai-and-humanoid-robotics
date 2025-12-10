# Research: RAG Chatbot Technology Stack

**Feature**: RAG Chatbot for Physical AI & Humanoid Robotics Book
**Date**: 2025-12-07
**Status**: Completed

## Executive Summary

This document consolidates research findings for implementing a production-ready RAG chatbot using the locked technology stack: Chatkit-JS (frontend), FastAPI + Chatkit-Python + LiteLLM (backend), Cohere (embeddings), Qdrant Cloud (vector DB), and Gemini API (LLM).

---

## 1. Chatkit-Python (Python)

### Decision
Use **Chatkit-Python v1.4.0+** (`/openai/chatkit-python`) for backend agent orchestration with streaming responses and session management, using LiteLLM as a translation layer for Gemini API compatibility.

### Rationale
- **Native Streaming Support**: Built-in `stream_agent_response()` async iterator for token-by-token streaming
- **Session Management API**: Automatic conversation history with `thread_id` and context management
- **LiteLLM Integration**: Supports non-OpenAI models (Gemini) via `LitellmModel` wrapper
- **High Code Coverage**: 40 code snippets with 59.3 benchmark score (High reputation source)
- **Production Features**: Built-in tracing, error handling, and usage tracking
- **OpenAI-Compatible API**: Works seamlessly with Chatkit-JS frontend components

### Key Implementation Patterns

**Streaming Responses**:
```python
async def stream_agent_response(
    context: AgentContext,
    result: RunnerStreamed,
) -> AsyncIterator[ThreadStreamEvent]:
    # Yields chunks as they arrive
    async for event in result.stream_events():
        yield event
```

**Session Management**:
- `Runner.run_streamed(agent, input, context=agent_context)` maintains context
- Thread-based conversation history with `thread_id`
- Custom `AgentContext` for request-specific data passing

### Alternatives Considered
- **LangChain**: Rejected - heavier abstraction layer, more complex for simple RAG
- **LlamaIndex**: Rejected - not optimized for multi-model streaming (Gemini + Cohere)
- **Raw FastAPI + LiteLLM**: Rejected - no built-in session management or handoffs

---

## 2. Chatkit-JS (Frontend)

### Decision
Use **Chatkit-JS** (`/openai/chatkit-js`) as the official OpenAI chat widget library for React/Docusaurus integration.

### Rationale
- **Batteries-Included UI**: Pre-built chat interface with streaming, typing indicators, error states
- **Deep Customization**: Theme system (colors, fonts, radius, density) aligns with Docusaurus design
- **Text Highlighting Support**: `entities` API for tagging selected text in questions
- **Session Token Refresh**: `getClientSecret()` callback handles token expiration gracefully
- **Event System**: `onError`, `onResponseStart`, `onResponseEnd`, `onThreadChange` for UI state management
- **High Benchmark**: 56.8 score with High reputation source (official OpenAI library)

### Key Implementation Patterns

**React Integration**:
```tsx
import { ChatKit, useChatKit } from '@openai/chatkit-react';

function MyChat() {
  const { control, sendUserMessage } = useChatKit({
    api: {
      async getClientSecret(existing) {
        const res = await fetch('/api/chatkit/session', { method: 'POST' });
        return (await res.json()).client_secret;
      },
    },
    theme: {
      colorScheme: 'light',
      color: { accent: { primary: '#2D8CFF' } },
      radius: 'round',
      density: 'compact',
    },
    onError: ({ error }) => console.error(error),
    onResponseStart: () => setLoading(true),
    onResponseEnd: () => setLoading(false),
  });

  return <ChatKit control={control} className="h-[600px] w-[400px]" />;
}
```

**Text Highlighting (Entity Tagging)**:
```tsx
entities: {
  onTagSearch: async (query: string): Promise<Entity[]> => {
    // Pre-populate highlighted text as entity
    return [{ id: 'highlight-1', title: query, interactive: true }];
  },
}
```

### Alternatives Considered
- **Custom React Chat UI**: Rejected - months of development for feature parity (typing indicators, markdown rendering, code highlighting)
- **Vercel AI SDK UI**: Rejected - requires OpenAI models, not compatible with Gemini
- **Botpress Chat Widget**: Rejected - lacks streaming token-by-token display

---

## 3. Docusaurus Integration

### Decision
Use **Docusaurus v3.x** swizzling (`/websites/docusaurus_io`) to inject Chatkit-JS widget into all documentation pages via layout component override.

### Rationale
- **Swizzle System**: Copy and wrap core components without forking Docusaurus
- **Layout Wrapper Pattern**: `@theme-original/Layout` allows adding widgets to every page
- **React Compatibility**: Docusaurus uses React 18+, fully compatible with ChatKit React hooks
- **High Coverage**: 1,953 code snippets with 83.3 benchmark score

### Key Implementation Patterns

**Swizzle Layout Component**:
```bash
npm run swizzle @docusaurus/theme-classic Layout -- --wrap
```

**Wrap Layout with ChatKit**:
```jsx
// src/theme/Layout/index.js
import React from 'react';
import Layout from '@theme-original/Layout';
import { ChatKitWidget } from '@site/src/components/ChatKitWidget';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <ChatKitWidget /> {/* Fixed position: bottom-right */}
    </>
  );
}
```

**Highlight Text Listener**:
```jsx
useEffect(() => {
  const handleSelection = () => {
    const selectedText = window.getSelection()?.toString().trim();
    if (selectedText && selectedText.length > 3) {
      setComposerValue(`${selectedText}\n\n`); // Pre-fill composer
      focusComposer();
    }
  };
  document.addEventListener('mouseup', handleSelection);
  return () => document.removeEventListener('mouseup', handleSelection);
}, [setComposerValue, focusComposer]);
```

### Alternatives Considered
- **Docusaurus Plugin**: Rejected - overkill for widget injection, harder to customize
- **Script Tag Injection**: Rejected - loses React state management and theme integration
- **Iframe Embed**: Rejected - cross-origin issues, poor UX (no shared state)

---

## 4. Embedding & Vector Search

### Decision
Use **Cohere embed-english-v3.0** (already configured) with **Qdrant Cloud** for semantic search with 384-token chunks.

### Rationale
- **Already Configured**: API key in `backend/.env`, client in `backend/main.py`
- **Performance**: Cohere v3 optimized for retrieval (better than OpenAI text-embedding-3-small for RAG)
- **Token Limit**: Supports up to 512 tokens (384-token chunks fit comfortably)
- **Cost**: Free tier sufficient for <10,000 chunks (200 Markdown files × ~50 chunks/file)

### Chunking Strategy (384 tokens)
- **Markdown Heading Boundaries**: Split on `##` headers first
- **Paragraph Fallback**: If section > 384 tokens, split by paragraphs
- **Code Block Preservation**: Keep entire code blocks together (split at function boundaries if needed)
- **Metadata**: Store `source_file`, `heading`, `page_url`, `last_updated` with each chunk

### Qdrant Collection Schema
```python
from qdrant_client.models import Distance, VectorParams

client.create_collection(
    collection_name="hackathon-api-cluster",
    vectors_config=VectorParams(size=1024, distance=Distance.COSINE),  # Cohere embed-english-v3.0
)
```

### Alternatives Considered
- **OpenAI text-embedding-3-small**: Rejected - violates "no OpenAI API" constraint
- **Sentence Transformers**: Rejected - requires local GPU, slow on free tier deployment
- **512-token chunks**: Rejected - user preference for 384 (medium granularity)

---

## 5. LLM (Gemini API)

### Decision
Use **Gemini API** (key already in `backend/.env`) via LiteLLM extension in OpenAI Agents SDK.

### Rationale
- **Already Configured**: `GEMINI_API_KEY` exists in environment
- **Rate Limit Handling**: Agents SDK supports exponential backoff retry (FR-015a requirement)
- **Streaming**: Native streaming support via `stream=True` parameter
- **Free Tier**: 60 requests/minute sufficient for initial rollout (<1000 queries/day assumption)

### Integration Pattern
```python
from litellm import acompletion

model = LitellmModel(model="gemini/gemini-1.5-flash")  # Fast, cost-effective
agent = Agent(
    name="DocsRAGAgent",
    model=model,
    instructions="You are a helpful assistant for Physical AI documentation. Always cite sources.",
)
```

### Rate Limit Recovery (FR-015a)
```python
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=2, max=10),
    reraise=True,
)
async def query_with_retry(prompt: str, context: str):
    return await agent.stream_response(input=f"{context}\n\nQuestion: {prompt}")
```

### Alternatives Considered
- **GPT-4**: Rejected - violates "no OpenAI API" constraint
- **Claude**: Rejected - not in locked tech stack
- **Ollama (local)**: Rejected - Render/Railway free tier lacks GPU

---

## 6. Auto-Reindexing (GitHub Actions)

### Decision
Use **GitHub Actions workflow** triggered on `push` to `main` branch, targeting `docs/**/*.md` files.

### Rationale
- **Native Integration**: Already using GitHub for version control
- **Zero Infrastructure**: No cron jobs or external schedulers needed
- **Fast**: Typical reindex <5 minutes for 200 files (SC-004 requirement)
- **Secure**: Secrets (Qdrant API key, Cohere API key) stored in GitHub Secrets

### Workflow Pattern
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
      - run: pip install -r backend/requirements.txt
      - run: python backend/scripts/reindex_docs.py
        env:
          COHERE_API_KEY: ${{ secrets.COHERE_API_KEY }}
          QDRANT_URL: ${{ secrets.QDRANT_URL }}
          QDRANT_API_KEY: ${{ secrets.QDRANT_API_KEY }}
```

### Reindexing Script Strategy
1. **Fetch Changed Files**: Parse `git diff --name-only HEAD~1` for modified `.md` files
2. **Read & Chunk**: Parse Markdown, split on headings, limit to 384 tokens
3. **Embed**: Batch embed chunks via Cohere API (max 96 chunks/request)
4. **Upsert to Qdrant**: Use `upsert()` to replace old chunks (keyed by `file_path:chunk_index`)
5. **Delete Removed Files**: Query Qdrant for chunks from deleted files, call `delete()`

### Alternatives Considered
- **Manual Trigger**: Rejected - violates "auto-reindex" requirement (FR-006)
- **Webhook from Docusaurus**: Rejected - requires custom server, complex auth
- **Scheduled Cron**: Rejected - wasteful (reindexes even when no changes)

---

## 7. Deployment (Render vs Railway)

### Decision
Deploy to **Render** free tier (as user requested).

### Rationale
- **Faster Cold Starts**: Render keeps instances warm longer than Railway (15 min vs 5 min)
- **WebSocket Support**: Native support for persistent connections (ChatKit streaming)
- **Free Tier Specs**: 512 MB RAM, sufficient for FastAPI + Agents SDK + in-memory sessions
- **Environment Variables**: Easy secret management via dashboard
- **Health Checks**: Built-in `/health` endpoint monitoring

### Deployment Configuration

**render.yaml**:
```yaml
services:
  - type: web
    name: rag-chatbot-api
    env: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: GEMINI_API_KEY
        sync: false
      - key: COHERE_API_KEY
        sync: false
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
```

**Health Check**:
```python
@app.get("/health")
async def health_check():
    return {"status": "healthy", "timestamp": datetime.utcnow().isoformat()}
```

### Comparison Table

| Feature | Render (CHOSEN) | Railway |
|---------|-----------------|---------|
| Cold Start | 15-30s | 30-60s |
| RAM (Free Tier) | 512 MB | 512 MB |
| WebSocket Support | ✅ Native | ✅ Native |
| Monthly Hours | 750 hrs | 500 hrs |
| Custom Domains | ✅ Free | ✅ Free |
| Secrets Management | Dashboard | CLI + Dashboard |
| Auto-Deploy from GitHub | ✅ Yes | ✅ Yes |

### Alternatives Considered
- **Vercel**: Rejected - 10s serverless timeout, incompatible with long-running streaming
- **Heroku**: Rejected - no free tier
- **Fly.io**: Rejected - complex setup, less generous free tier

---

## 8. Risk Mitigation

### Qdrant Cloud Downtime
**Risk**: Vector DB unavailable, users cannot get answers.

**Mitigation**:
1. **Fallback Response**: Return polite error message with retry suggestion
2. **Circuit Breaker**: After 3 failed Qdrant requests, return cached error for 60s
3. **Health Check**: Monitor Qdrant availability every 5 minutes, log to Render dashboard

```python
from circuitbreaker import circuit

@circuit(failure_threshold=3, recovery_timeout=60)
async def search_qdrant(query_embedding: list[float]):
    return qdrant_client.search(
        collection_name="hackathon-api-cluster",
        query_vector=query_embedding,
        limit=5,
    )
```

### Gemini API Rate Limits
**Risk**: Free tier quota exhausted (60 req/min), users see errors.

**Mitigation**:
1. **Exponential Backoff Retry**: Max 3 attempts with 2s, 4s, 8s delays (FR-015a)
2. **Queue with User Notification**: Show "Processing..." message during retry
3. **Rate Limit Monitoring**: Log rate limit hits, alert if >50% quota used

```python
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=2, min=2, max=10),
    retry=retry_if_exception_type(RateLimitError),
)
async def query_gemini_with_retry(prompt: str):
    # Yields "Processing..." event before retry
    yield {"type": "status", "message": "Processing..."}
    result = await agent.stream_response(input=prompt)
    async for chunk in result:
        yield chunk
```

### Cold Starts (Render Free Tier)
**Risk**: First request after 15 min inactivity takes 30s, poor UX.

**Mitigation**:
1. **Warmup Ping**: GitHub Actions cron job pings `/health` every 10 minutes
2. **Loading State**: ChatKit shows "Connecting..." during cold start
3. **Lazy Loading**: Don't initialize Qdrant/Cohere clients until first request

```yaml
# .github/workflows/warmup.yml
on:
  schedule:
    - cron: '*/10 * * * *'  # Every 10 minutes
jobs:
  warmup:
    runs-on: ubuntu-latest
    steps:
      - run: curl https://rag-chatbot-api.onrender.com/health
```

---

## 9. Testing Strategy

### Local Testing

**Test 1: Highlight → Ask → Get Answer with Source Link**
```bash
# Start backend
cd backend
uv run uvicorn main:app --reload --port 8000

# Start frontend (Docusaurus)
cd ../
npm run start

# Manual test:
# 1. Open http://localhost:3000/docs/intro
# 2. Highlight text: "NVIDIA Isaac Sim 2025"
# 3. Click chat widget (opens with pre-filled text)
# 4. Ask: "How does this work with ROS 2?"
# 5. Verify: Streamed answer with clickable source link
```

**Expected Outcome**:
- Widget opens with highlighted text in composer
- Answer streams token-by-token (visible typing)
- At least 1 source link appears (e.g., `docs/004-isaac-platform/integration.md#ros2`)

---

**Test 2: Push Change → Wait 5 Min → Ask About New Content**
```bash
# Add new doc file
echo "# Test Topic\nThis is test content about cuVSLAM." > docs/test-topic.md

# Commit and push
git add docs/test-topic.md
git commit -m "Add test topic for reindex validation"
git push origin main

# Wait 5 minutes for GitHub Actions reindex

# Query chatbot
# Ask: "What is cuVSLAM?"
# Verify: Answer references "test-topic.md"
```

**Expected Outcome**:
- GitHub Actions workflow completes in <5 min
- Qdrant contains new chunks (check via Qdrant dashboard)
- Chatbot answer includes content from `test-topic.md`

---

**Test 3: Rate Limit Simulation → Verify "Processing..." + Retry**
```bash
# Modify backend to force rate limit error
# In main.py, add:
class MockRateLimitError(Exception):
    pass

@app.post("/api/chat")
async def chat(request: ChatRequest):
    if random.random() < 0.5:  # 50% chance
        raise MockRateLimitError("Rate limit exceeded")
    # ... normal flow

# Send 10 rapid requests
for i in {1..10}; do
  curl -X POST http://localhost:8000/api/chat \
    -H "Content-Type: application/json" \
    -d '{"message": "Test question '$i'", "session_id": "test-session"}' &
done

# Observe frontend:
# - Some requests show "Processing..." message
# - Requests retry with exponential backoff
# - All eventually succeed or fail gracefully
```

**Expected Outcome**:
- ChatKit displays "Processing..." during retry
- Backend logs show exponential backoff delays (2s, 4s, 8s)
- No silent failures (user always sees outcome)

---

### Deployment Testing

**Test 4: Deploy to Render → Verify Zero Failed Deployments (SC-008)**
```bash
# Push to main (triggers auto-deploy)
git push origin main

# Monitor Render dashboard
# Check build logs for:
# - "pip install" success
# - "uvicorn main:app" starts without errors
# - Health check `/health` returns 200

# Verify deployment
curl https://rag-chatbot-api.onrender.com/health
# Expected: {"status": "healthy", "timestamp": "2025-12-07T..."}
```

**Expected Outcome**:
- Build completes in <3 minutes
- No Python import errors
- Health check returns 200 OK
- Frontend can connect to deployed backend

---

## 10. Open Questions Resolved

| Question | Resolution |
|----------|------------|
| How to handle off-topic questions? | Polite refusal + suggested topics (Clarification Q1) |
| What chunk size for embeddings? | 384 tokens (Clarification Q2) |
| How to handle Gemini rate limits? | Queue + exponential backoff + "Processing..." (Clarification Q3) |
| Render vs Railway deployment? | Render (faster cold starts, user preference) |
| How to trigger auto-reindex? | GitHub Actions on `push` to `main` with `docs/**/*.md` filter |
| How to integrate ChatKit in Docusaurus? | Swizzle Layout component, wrap with ChatKitWidget |
| How to handle text highlighting? | `entities.onTagSearch` API + `mouseup` event listener |

---

## 11. Technology Stack Summary

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| **Frontend Widget** | ChatKit JS | Latest | Official OpenAI library, streaming, theme system |
| **Frontend Framework** | Docusaurus | 3.x | Existing, React-based, swizzle system |
| **Backend Framework** | FastAPI | Latest | Async, WebSocket support, existing setup |
| **Agent Orchestration** | OpenAI Agents SDK | 0.6.2+ | Streaming, session mgmt, LiteLLM integration |
| **LLM** | Gemini API | gemini-1.5-flash | Locked stack, free tier, fast |
| **Embeddings** | Cohere | embed-english-v3.0 | Locked stack, optimized for retrieval |
| **Vector DB** | Qdrant Cloud | Latest | Locked stack, <10k chunks capacity |
| **Deployment** | Render | Free Tier | Faster cold starts, WebSocket support |
| **CI/CD** | GitHub Actions | N/A | Native integration, secure secrets |

---

## Next Steps

1. ✅ Research complete
2. ⏭️ Phase 1: Create `data-model.md` (entities: DocumentChunk, ChatMessage, ChatSession, IndexingJob)
3. ⏭️ Phase 1: Generate API contracts (`contracts/chat-api.yaml`, `contracts/reindex-api.yaml`)
4. ⏭️ Phase 1: Create architecture diagram (Mermaid)
5. ⏭️ Phase 1: Generate `quickstart.md` (local setup, deployment)
6. ⏭️ Phase 1: Update agent context (add ChatKit JS, OpenAI Agents SDK, Docusaurus swizzling)
7. ⏭️ Phase 2: `/sp.tasks` (not part of `/sp.plan`)

---

**Status**: ✅ All research complete, ready for Phase 1 design.
