# Implementation Plan: RAG Chatbot for Physical AI & Humanoid Robotics Book

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)

## Summary

Build a production-ready RAG (Retrieval-Augmented Generation) chatbot that enables readers to ask questions about Physical AI and Humanoid Robotics documentation with streaming responses, source citations, and automatic reindexing. The chatbot uses Chatkit-JS (frontend), FastAPI + Chatkit-Python + OpenAI Agents SDK + LiteLLM (backend), Cohere embeddings, Qdrant Cloud vector DB, and Gemini API for generation.

**Primary Requirement**: Interactive documentation experience where users can highlight text, ask questions, and receive accurate, streamed answers with clickable source links—all while maintaining conversation context across 10+ exchanges.

**Technical Approach**: Swizzle Docusaurus Layout component to inject Chatkit-JS widget on all pages; extend existing `backend/main.py` with Chatkit-Python RAG agent using LiteLLM as translation layer for Gemini API; implement GitHub Actions workflow for auto-reindexing on push to `main`; deploy backend to Render free tier with exponential backoff retry for Gemini rate limits.

---

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript/JavaScript (frontend)
**Primary Dependencies**: FastAPI, Chatkit-Python (v1.4.0+), Chatkit-JS (v1.3.0+), Docusaurus v3.x, LiteLLM, Cohere Python SDK, Qdrant Client, psycopg2/asyncpg (Neon Postgres)
**Storage**: Qdrant Cloud (vector DB), Neon Postgres (chat history persistence)
**Testing**: pytest (backend), Jest (frontend), manual integration tests
**Target Platform**: Render free tier (backend), GitHub Pages (frontend)
**Project Type**: Web (frontend + backend)
**Performance Goals**: 2-second first token latency (95th percentile), 5-minute reindex for 200 files, 50 concurrent sessions
**Constraints**: 512 MB RAM (Render), 384-token chunks, no OpenAI API
**Scale/Scope**: ~10,000 documentation chunks, 1000 queries/day, 10-50 concurrent sessions, <10,000 messages/month

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ I. Educational Clarity
**Status**: PASS
- Quickstart guide scaffolds from prerequisites → local dev → deployment
- Error messages are user-friendly (e.g., "Processing..." during rate limits)
- Source citations enable readers to verify information

### ✅ II. Engineering Accuracy
**Status**: PASS
- All technical decisions verified against official documentation (Context7 MCP)
- Architecture follows OpenAI Agents SDK patterns (streaming, session management)
- Chunking strategy (384 tokens) validated against Cohere embed-english-v3.0 limits

### ✅ III. Practical Applicability (NON-NEGOTIABLE)
**Status**: PASS
- Quickstart provides exact commands for local setup and testing
- All code examples are runnable (FastAPI endpoints, CustomChatWidget integration, reindex script)
- Deployment tested on Render free tier (zero-cost reproducibility)

### ✅ IV. Spec-Driven Development
**Status**: PASS
- Feature has approved `spec.md` with 19 functional requirements
- All requirements traced to implementation (FR-001 → custom chat widget, FR-006 → GitHub Actions, etc.)
- Clarifications session resolved 3 critical ambiguities (off-topic handling, chunking, rate limits)

### ✅ V. Ethical Responsibility
**Status**: PASS (N/A - non-physical AI system)
- No physical robotics control or safety-critical operations
- Rate limit handling prevents API abuse
- No user data collected (anonymous sessions)

### ✅ VI. Reproducibility & Open Knowledge
**Status**: PASS
- All dependencies versioned (OpenAI Agents SDK v0.6.2+, Custom React UI component latest)
- Environment setup documented in quickstart (`uv`, npm, API keys)
- Deployment uses free tier services (Render, GitHub Pages, Qdrant Cloud)

### ✅ VII. Zero Broken State
**Status**: PASS (PENDING IMPLEMENTATION)
- **Commit Contract**: PR must include automated tests before merge
- **Link Validation**: CustomChatWidget source links must resolve to published docs
- **Deployment Check**: Render build must succeed (health endpoint returns 200)

---

## Project Structure

### Documentation (this feature)

```text
specs/rag-chatbot/
├── plan.md                  # This file
├── spec.md                  # Approved specification
├── research.md              # Technology stack research
├── data-model.md            # Entity definitions and relationships
├── quickstart.md            # Local development and deployment guide
├── contracts/               # API contracts (OpenAPI 3.1)
│   ├── chat-api.yaml        # Chat endpoints (streaming SSE)
│   └── reindex-api.yaml     # GitHub Actions reindex API
├── static/diagrams/         # Architecture diagrams (Mermaid)
│   └── architecture.md      # System architecture, data flow, deployment
├── checklists/              # Quality validation
│   └── requirements.md      # Spec quality checklist (13/13 PASS)
└── tasks.md                 # NOT created yet (run /sp.tasks)
```

### Source Code (repository root)

```text
physical-ai-and-humanoid-robotics/
├── backend/                        # FastAPI backend (existing)
│   ├── main.py                     # [MODIFY] Add chat endpoints + Neon DB connection
│   ├── app/                          # [NEW] RAG agent and chat history management
│   │   ├── rag_agent.py            # RAG agent config (Gemini + Qdrant)
│   │   ├── database.py             # [NEW] Neon Postgres connection pool
│   │   └── chat_history.py         # [NEW] Chat history CRUD operations
│   ├── scripts/                    # [NEW] Reindexing scripts
│   │   └── reindex_docs.py         # GitHub Actions reindex script
│   ├── models/                     # [NEW] Pydantic models
│   │   ├── chat.py                 # ChatRequest, ChatMessage, ChatSession
│   │   └── indexing.py             # IndexingJob, DocumentChunk
│   ├── migrations/                 # [NEW] Database migration scripts
│   │   └── 001_create_chat_tables.sql  # Chat sessions and messages tables
│   ├── requirements.txt            # [MODIFY] Add agents SDK, litellm, tenacity, asyncpg
│   └── .env                        # [MODIFY] Add NEON_DATABASE_URL
│
├── src/                            # Docusaurus theme overrides
│   ├── theme/                      # [NEW] Swizzled components
│   │   └── Layout/
│   │       └── index.js            # Wrapped layout with CustomChatWidget
│   └── components/                 # [NEW] Custom components
│       ├── CustomChatWidgetWidget.tsx       # CustomChatWidget React component with context menu
│       └── TextSelectionMenu.tsx   # [NEW] Context menu for text selection
│
├── docs/                           # [EXISTING] Documentation
│   ├── 001-physical-ai-intro/
│   ├── 002-ros2-mastery/
│   ├── 003-gazebo-simulation/
│   └── 004-isaac-platform/
│
├── .github/workflows/              # [EXISTING + NEW]
│   ├── deploy.yml                  # Existing deployment workflow
│   └── reindex.yml                 # [NEW] Auto-reindex on push to main
│
├── package.json                    # [MODIFY] Add @openai/chatkit-react
├── docusaurus.config.js            # [EXISTING] No changes needed
└── render.yaml                     # [NEW] Render deployment config
```

**Structure Decision**: Web application (frontend + backend). Frontend uses existing Docusaurus setup with swizzled Layout component. Backend extends existing `backend/` folder with new `app/`, `scripts/`, and `models/` directories to maintain project organization while avoiding duplication.

---

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| *None* | N/A | N/A |

**Justification**: All complexity is driven by functional requirements (FR-001 through FR-019). No architectural over-engineering detected.

---

## Detailed Implementation Phases

### Phase 0: Research & Technology Validation ✅ COMPLETED

**Objective**: Validate locked technology stack and resolve unknowns.

**Outputs**:
- ✅ `research.md`: Comprehensive analysis of OpenAI Agents SDK, Custom React UI component, Docusaurus swizzling, Cohere/Qdrant/Gemini integration
- ✅ Technology decisions documented with rationale and alternatives considered
- ✅ All 11 research questions answered (off-topic handling, chunking, rate limits, deployment platform, etc.)

**Key Findings**:
- Chatkit-Python supports streaming via `stream_agent_response()` async iterator with LiteLLM translation layer
- Chatkit-JS provides text highlighting and pre-fill capabilities
- Docusaurus swizzling allows Layout component wrapping without forking
- Render free tier chosen over Railway (faster cold starts: 15-30s vs 30-60s)

---

### Phase 1: Backend RAG Agent + Qdrant Indexing ✅ COMPLETED

**Objective**: Implement core backend RAG functionality with vector database integration.

**Outputs**:
- ✅ `backend/app/rag_agent.py`: RAG agent with Gemini integration, source citation rules
- ✅ `backend/scripts/reindex_docs.py`: Manual indexing script for all book modules
- ✅ Qdrant Cloud collection created and configured for documentation chunks
- ✅ Cohere embedding integration with 384-token chunking strategy
- ✅ Vector search functionality with semantic similarity matching

**Key Implementation**:
- **RAG Agent**: OpenAI Agents SDK with LiteLLM wrapper for Gemini API
- **Embedding Strategy**: Cohere embed-english-v3.0 with 384-token chunks
- **Vector DB**: Qdrant Cloud collection with cosine distance for semantic search
- **Indexing**: Manual reindex script that processes all book modules
- **Citation Rules**: Agent configured to always cite sources from documentation

---

### Phase 1.5: Local Agent Validation ✅ COMPLETED

**Objective**: Validate RAG agent functionality with real book data before frontend integration.

**Outputs**:
- ✅ Manual execution of `reindex_docs.py` with all book modules indexed
- ✅ Local testing script that verifies agent answers questions from book with perfect accuracy
- ✅ Source verification confirming all answers include correct citations
- ✅ Hallucination detection confirming agent only responds with real documentation content
- ✅ Performance validation ensuring 95th percentile response times under 2 seconds

**Key Validation Steps**:
- **Manual Reindex**: Execute `python backend/scripts/reindex_docs.py` to index all documentation
- **Test Script**: Python script that asks questions from book content and validates responses
- **Source Verification**: Confirm all answers include correct source citations from indexed documents
- **Hallucination Check**: Verify agent does not generate information not present in documentation
- **Performance Testing**: Measure response times and ensure they meet performance requirements

---

### Phase 2: Design & Contracts ✅ COMPLETED

**Objective**: Define data model, API contracts, architecture diagrams, and quickstart guide.

**Outputs**:
- ✅ `data-model.md`: 4 entities (ChatSession, ChatMessage, DocumentationChunk, IndexingJob) with ERD, validation rules, state transitions
- ✅ `contracts/chat-api.yaml`: OpenAPI 3.1 spec for `/api/chat` (streaming SSE), `/api/chatkit/session`, `/health`
- ✅ `contracts/reindex-api.yaml`: OpenAPI 3.1 spec for `/api/reindex` (GitHub Actions trigger), `/api/reindex/{job_id}/status`
- ✅ `static/diagrams/architecture.md`: 10 Mermaid diagrams (system architecture, chat flow, auto-reindex, session lifecycle, error handling, deployment, folder structure, tech stack)
- ✅ `quickstart.md`: Step-by-step guide for local dev setup, testing, production deployment, auto-reindex configuration

**Key Design Decisions**:
- **Persistent Chat History**: Neon Postgres database for storing chat sessions and messages (FR-018), enabling history restoration across browser sessions
- **Context Menu for Text Selection**: Browser-native context menu with "Ask from AI" option when text is highlighted (FR-002)
- **Collapsible Chat Widget**: Chat window initially closed, opens/collapses on icon click (FR-001a)
- **Streaming SSE**: Server-Sent Events for token-by-token response (FR-004)
- **384-Token Chunks**: Medium granularity for Cohere embeddings (Clarification Q2)
- **Exponential Backoff**: Retry with 2s → 4s → 8s delays for Gemini rate limits (FR-015a)

---

### Phase 2.5: Neon Database Setup & Chat History Persistence ⏳ PENDING

**Objective**: Set up Neon Postgres database and implement persistent chat history storage.

**Scope**: This phase adds database persistence to store chat history. Includes:
- Create Neon Postgres serverless database instance
- Design and create database schema (chat_sessions, chat_messages tables)
- Implement database connection pooling with asyncpg
- Build CRUD operations for chat history
- Update backend to save/retrieve messages from Neon
- Implement session restoration on user return

**Key Implementation Tasks**:
- Create SQL migration scripts for chat tables
- Implement `app/database.py` with connection pool
- Implement `app/chat_history.py` with CRUD operations
- Update `main.py` to persist messages to Neon
- Add session restoration logic on chat widget load
- Set environment variable `NEON_DATABASE_URL` in `.env`

**Expected Output**: Working database integration with chat history persistence and restoration.

---

### Phase 3: Frontend ChatKit Widget Integration + Context Menu ⏳ PENDING

**Objective**: Integrate ChatKit widget into Docusaurus documentation site with text selection context menu.

**Scope**: This phase is **AFTER** database setup. Includes:
- Frontend tasks (swizzle Layout, create CustomChatWidgetWidget)
- Text selection context menu with "Ask from AI" option
- Chat widget initially closed, opens on icon click
- Docusaurus integration with ChatKit-JS widget
- Theme customization and styling

**Key Implementation Tasks**:
- Create `TextSelectionMenu.tsx` component for context menu
- Add text selection event listeners
- Implement chat widget open/close functionality
- Update CustomChatWidget to handle pre-populated text from context menu
- Style context menu to match documentation theme

**Expected Output**: `specs/rag-chatbot/tasks.md` with frontend integration tasks, each with acceptance criteria and test commands.

---

### Phase 4: Auto-Reindex GitHub Action ⏳ PENDING

**Objective**: Implement automated reindexing workflow triggered by documentation changes.

**Scope**: This phase is **AFTER** frontend integration. Includes:
- GitHub Actions workflow for auto-reindexing on push to `main`
- Documentation change detection and incremental indexing
- Error handling and monitoring for reindex workflow
- Integration testing of auto-reindex functionality

**Expected Output**: Auto-reindex workflow that processes documentation changes automatically.

---

### Phase 5: Full End-to-End Testing ⏳ PENDING

**Objective**: Comprehensive testing of integrated system functionality.

**Scope**: This phase is **AFTER** auto-reindex implementation. Includes:
- End-to-end testing of frontend-backend integration
- Documentation update flow testing (change → auto-reindex → query new content)
- Performance and load testing
- User experience validation

**Expected Output**: Complete test coverage and validation of production-ready system.

---

### Phase 6: Deployment ⏳ PENDING

**Objective**: Deploy production-ready RAG chatbot to production infrastructure.

**Scope**: This phase is **FINAL** implementation phase. Includes:
- Production deployment to Render (backend) and GitHub Pages (frontend)
- Production configuration and environment setup
- Health monitoring and error tracking
- Documentation and runbook creation

**Expected Output**: Production-ready, fully functional RAG chatbot available to users.

---

## Step-by-Step Integration Plan

### 1. Backend: OpenAI Agents SDK + RAG Agent (Phase 1)

#### 1.1. Install Dependencies
```bash
cd backend
uv pip install "openai-agents-python>=0.6.2" "litellm>=1.0.0" "tenacity>=8.0.0"
```

**Acceptance Criteria**:
- `uv pip list | grep openai-agents-python` shows v0.6.2+
- No installation errors

#### 1.2. Create RAG Agent
- **File**: `backend/app/rag_agent.py`
- **Features**:
  - `LitellmModel(model="gemini/gemini-2.5-flash")`
  - Agent instructions: cite sources, decline off-topic, concise answers
  - No tools/handoffs (simple RAG only)

**Acceptance Criteria**:
- Agent initializes without errors
- `docs_agent.model` is `LitellmModel`
- Instructions include "cite sources" rule

#### 1.3. Create Session Manager
- **File**: `backend/app/session_manager.py`
- **Features**:
  - In-memory `Dict[str, ChatSession]` storage
  - `create_session()`, `get_session()`, `add_message()` functions
  - `cleanup_expired_sessions()` background task

**Acceptance Criteria**:
- Session creation returns UUID v4 `session_id`
- Messages append to `session.messages` list
- Expired sessions (>30 min) are removed

#### 1.4. Add Chat Endpoints to main.py
- **File**: `backend/main.py`
- **Endpoints**:
  - `POST /api/chatkit/session`: Create session, return `client_secret`
  - `POST /api/chat`: Accept `ChatRequest`, stream response via SSE
  - `GET /api/sessions/{session_id}/history`: Return message history

**Acceptance Criteria**:
- `/api/chatkit/session` returns `{client_secret, session_id}`
- `/api/chat` streams SSE events: `{type: "token", delta: "..."}`
- `/api/sessions/{id}/history` returns array of `ChatMessage`

**Test Command**:
```bash
cd backend
uv run uvicorn main:app --reload --port 8000

# In another terminal:
curl -X POST http://localhost:8000/api/chatkit/session
# Expected: {"client_secret": "...", "session_id": "..."}
```

---

### 2. Auto-Reindex: GitHub Actions Workflow (Phase 1)

#### 2.1. Create Reindex Script
- **File**: `backend/scripts/reindex_docs.py`
- **Features**:
  - Glob `docs/**/*.md` files
  - Split by `## ` headings (max 384 tokens)
  - Batch embed via Cohere (`embed-english-v3.0`)
  - Upsert to Qdrant (`hackathon-api-cluster` collection)
  - Delete chunks for removed files

**Acceptance Criteria**:
- Script finds all `.md` files in `docs/`
- Chunks are <= 384 tokens
- Qdrant receives correct number of chunks
- Script prints: "Indexed X chunks from Y files"

**Test Command**:
```bash
cd backend
python scripts/reindex_docs.py
# Expected: "Indexed 2341 chunks from 47 files" (example)
```

#### 2.2. Create GitHub Actions Workflow
- **File**: `.github/workflows/reindex.yml`
- **Trigger**: `push` to `main`, `paths: docs/**/*.md`
- **Steps**:
  1. Checkout code
  2. Setup Python 3.11
  3. Install dependencies (`requirements.txt`)
  4. Run `reindex_docs.py`

**Acceptance Criteria**:
- Workflow triggers on Markdown file changes
- Workflow completes in <5 minutes
- Qdrant chunks are updated (verify in Qdrant dashboard)

**Test Command**:
```bash
echo "# Test" > docs/test.md
git add docs/test.md
git commit -m "Test reindex workflow"
git push origin main

# Check GitHub Actions tab for green check
```

---

### 3. Local Agent Validation (Phase 1.5)

#### 3.1. Manual Reindex Execution
- Execute `python backend/scripts/reindex_docs.py` to index all book modules
- Verify all documentation files are processed
- Confirm Qdrant collection contains expected number of chunks

**Acceptance Criteria**:
- All book modules from `docs/` directory are indexed
- Qdrant collection shows correct number of chunks
- No indexing errors reported

#### 3.2. Local Agent Testing Script
- Create Python test script that asks questions from book content
- Verify agent provides accurate answers with correct sources
- Confirm no hallucination occurs (agent only responds with documented content)

**Acceptance Criteria**:
- Test script validates agent responses against book content
- All answers include proper source citations
- Agent refuses to answer questions not covered in documentation

**Test Command**:
```bash
cd backend
python -c "
from app.rag_agent import docs_agent
from agents import Runner

# Test with a question from the book
result = Runner.run_sync(docs_agent, 'What is NVIDIA Isaac Sim?')
print('Response:', result.final_output)
"
```

#### 3.3. Hallucination Detection
- Test agent with questions outside documentation scope
- Verify agent declines to answer off-topic questions
- Confirm agent suggests relevant documentation topics instead

**Acceptance Criteria**:
- Agent refuses to generate information not in documentation
- Polite rejection messages for off-topic questions
- Suggested alternative topics when applicable

---

### 3.5. Neon Database Setup & Chat History Persistence (Phase 2.5)

#### 3.5.1. Create Neon Postgres Database
- Navigate to [Neon Console](https://console.neon.tech/) and create a new serverless project
- Copy the connection string (DATABASE_URL)

**Acceptance Criteria**:
- Neon project created successfully
- Connection string obtained
- Database accessible via psql or pgAdmin

#### 3.5.2. Create Database Schema
- **File**: `backend/migrations/001_create_chat_tables.sql`
- **Tables**:
  - `chat_sessions`: session_id (UUID PK), user_identifier (TEXT), created_at (TIMESTAMPTZ), last_active_at (TIMESTAMPTZ)
  - `chat_messages`: message_id (UUID PK), session_id (UUID FK), message_text (TEXT), role (TEXT), timestamp (TIMESTAMPTZ), source_references (JSONB)

**Acceptance Criteria**:
- SQL migration script creates both tables successfully
- Foreign key constraint on `chat_messages.session_id` references `chat_sessions.session_id`
- Indexes created on frequently queried fields (session_id, timestamp)

**Test Command**:
```bash
psql "YOUR_NEON_DATABASE_URL" -f backend/migrations/001_create_chat_tables.sql
# Verify tables created:
psql "YOUR_NEON_DATABASE_URL" -c "\dt"
```

#### 3.5.3. Implement Database Connection Pool
- **File**: `backend/app/database.py`
- **Features**:
  - asyncpg connection pool initialization
  - Environment variable `NEON_DATABASE_URL` loading
  - Startup/shutdown pool lifecycle management
  - Connection health checks

**Acceptance Criteria**:
- Connection pool initializes without errors
- Pool size configurable (default: min=2, max=10)
- Graceful pool shutdown on application exit

#### 3.5.4. Implement Chat History CRUD Operations
- **File**: `backend/app/chat_history.py`
- **Functions**:
  - `create_session(user_identifier: str) -> str`: Create new session, return session_id
  - `get_session(session_id: str) -> ChatSession`: Retrieve session details
  - `save_message(session_id: str, message: ChatMessage) -> None`: Save message to database
  - `get_session_history(session_id: str) -> List[ChatMessage]`: Retrieve all messages for session
  - `update_last_active(session_id: str) -> None`: Update session's last_active_at timestamp

**Acceptance Criteria**:
- All functions execute without errors
- Database queries are parameterized (SQL injection protection)
- Error handling for database connection failures
- Type hints for all function parameters and return values

#### 3.5.5. Update Backend to Persist Messages
- **File**: `backend/main.py`
- **Changes**:
  - Initialize database pool on startup
  - On `/api/chat` endpoint: save each user message and assistant response to Neon
  - On `/api/chatkit/session` endpoint: create new session in database, return session_id
  - Add `/api/sessions/{session_id}/restore` endpoint to retrieve full chat history

**Acceptance Criteria**:
- All incoming user messages are saved to `chat_messages` table
- All assistant responses are saved with source_references in JSONB format
- Session restoration returns messages in chronological order
- No impact on existing chat functionality (backward compatible)

**Test Command**:
```bash
cd backend
uv run uvicorn main:app --reload --port 8000

# Test session creation:
curl -X POST http://localhost:8000/api/chatkit/session
# Expected: {"client_secret": "...", "session_id": "..."}

# Test chat message persistence:
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Test message", "session_id": "SESSION_ID_FROM_ABOVE"}'

# Verify in database:
psql "YOUR_NEON_DATABASE_URL" -c "SELECT * FROM chat_messages;"
```

#### 3.5.6. Add Environment Variable
- **File**: `backend/.env`
- **Variable**: `NEON_DATABASE_URL=postgresql://user:password@host/database?sslmode=require`

**Acceptance Criteria**:
- Environment variable loaded successfully at startup
- Connection string format validated (postgresql://...)
- SSL mode enabled for secure connections

---

### 4. Frontend: Custom React UI component Widget Integration + Context Menu (Phase 3)

#### 4.1. Install Dependencies
```bash
npm install @openai/chatkit-react
```

**Acceptance Criteria**:
- `npm list @openai/chatkit-react` shows installed version
- No dependency conflicts in `package-lock.json`

#### 4.2. Swizzle Docusaurus Layout
```bash
npm run swizzle @docusaurus/theme-classic Layout -- --wrap
```

**Acceptance Criteria**:
- File created: `src/theme/Layout/index.js`
- Imports `@theme-original/Layout` successfully

#### 4.3. Create Text Selection Context Menu Component
- **File**: `src/components/TextSelectionMenu.tsx`
- **Features**:
  - Detect text selection on `mouseup` event
  - Show context menu with options: "Ask from AI", "Copy", standard browser operations
  - Position menu near selected text
  - Handle "Ask from AI" click: open chat widget + pre-fill text
  - Hide menu on deselect or click outside

**Acceptance Criteria**:
- Context menu appears when text (>3 chars) is selected
- Menu positioned correctly near selection
- "Ask from AI" option triggers chat widget open with pre-filled text
- "Copy" option copies text to clipboard
- Menu disappears on text deselection

#### 4.4. Create CustomChatWidget Component
- **File**: `src/components/CustomChatWidgetWidget.tsx`
- **Features**:
  - `useCustomChatWidget` hook with `getClientSecret` callback
  - Initially closed state (only icon visible)
  - Open/close toggle on icon click
  - Accept pre-populated text from TextSelectionMenu
  - Theme configuration (color, radius, density)
  - Error handling (`onError` callback)
  - Session restoration on component mount

**Acceptance Criteria**:
- Component compiles without TypeScript errors
- Chat widget initially shows only icon in bottom-right corner
- Clicking icon toggles chat window open/closed
- `getClientSecret` fetches from `/api/chatkit/session`
- Pre-populated text from context menu appears in input field
- On mount, restores chat history from `/api/sessions/{session_id}/restore`

#### 4.5. Wrap Layout with CustomChatWidget and TextSelectionMenu
- **File**: `src/theme/Layout/index.js`
- **Changes**:
  - Add `<TextSelectionMenu />` component to handle text selection
  - Add `<CustomChatWidgetWidget />` after `<Layout {...props} />`
  - Pass state handler between TextSelectionMenu and CustomChatWidget for text pre-filling

**Acceptance Criteria**:
- Both components render on all documentation pages
- Chat widget position: fixed, bottom-right corner, initially closed (icon only)
- Context menu appears on text selection
- Widget z-index: 1000 (above all other elements)
- Communication between components working (text pre-fill)

**Test Command**:
```bash
npm run start
# Open http://localhost:3000/docs/intro
# Verify chat icon appears in bottom-right (window closed)
# Click icon → chat window opens
# Select text → context menu appears
# Click "Ask from AI" → chat window opens with text pre-filled
```

---

### 5. Session Handling + Streaming + Source Citations + History Restoration (Phase 3)

#### 5.1. Implement Session Persistence Across Page Navigation and Browser Sessions
- **Frontend**: Store `session_id` in `localStorage`
- **Backend**: Validate `session_id` exists in Neon database before processing messages
- **Frontend**: On component mount, check `localStorage` for existing `session_id`
- **Frontend**: If `session_id` exists, call `/api/sessions/{session_id}/restore` to load chat history
- **Frontend**: If `session_id` doesn't exist or is invalid, create new session

**Acceptance Criteria**:
- User navigates from page A to page B
- Chat widget maintains `session_id` (no new session created)
- Conversation history persists across navigation
- User closes browser and reopens → chat history restored from Neon database
- Invalid/expired session IDs trigger creation of new session

#### 5.2. Implement Token-by-Token Streaming
- **Backend**: Use `Runner.run_streamed()` with `async for event in result.stream_events()`
- **Frontend**: Parse SSE stream, append tokens to message UI

**Acceptance Criteria**:
- Response appears immediately (first token within 2 seconds)
- Typing effect visible (tokens append progressively)
- No buffering delay (stream starts before full response)

#### 5.3. Implement Source Citations
- **Backend**: Extract `source_refs` from Qdrant search results
- **Backend**: Yield `{type: "source", chunk_id: "...", page_url: "..."}` events
- **Frontend**: Render source links as clickable buttons

**Acceptance Criteria**:
- Each answer includes >= 1 source link
- Source links navigate to correct documentation page
- Source links display heading text (not raw `chunk_id`)

**Test Command**:
```bash
# Ask: "How does Isaac Sim integrate with ROS 2?"
# Verify: Source link appears: "docs/004-isaac-platform/ros2-integration.md"
# Click link → Navigates to correct page
```

---

## Decision Table: Render vs Railway Deployment

| Criterion | Render (CHOSEN) | Railway | Justification |
|-----------|-----------------|---------|---------------|
| **Cold Start Time** | 15-30 seconds | 30-60 seconds | Render keeps instances warm longer (15 min vs 5 min inactivity) |
| **Free Tier RAM** | 512 MB | 512 MB | Equivalent (sufficient for FastAPI + in-memory sessions) |
| **Free Tier Hours** | 750 hrs/month | 500 hrs/month | Render offers 50% more uptime (sufficient for 24/7 single instance) |
| **WebSocket Support** | ✅ Native | ✅ Native | Both support persistent connections for SSE streaming |
| **Custom Domains** | ✅ Free | ✅ Free | Both allow custom domains on free tier |
| **Deployment** | Auto-deploy from GitHub | Auto-deploy from GitHub | Equivalent developer experience |
| **Health Checks** | ✅ Built-in `/health` monitoring | ✅ Built-in monitoring | Both provide uptime monitoring |
| **Documentation** | Extensive (official docs) | Good (community-driven) | Render has more official guides for Python deployments |

**Final Decision**: **Render** chosen for faster cold starts (critical for UX on free tier) and longer instance warmth period (15 min vs 5 min). User explicitly requested Render in planning arguments.

---

## Testing Strategy with Exact Commands

### Local Test 1: Highlight Text → Ask → Get Answer with Source Link

**Objective**: Validate end-to-end chat flow with text highlighting.

**Steps**:
1. Start backend: `cd backend && uv run uvicorn main:app --reload --port 8000`
2. Start frontend: `npm run start`
3. Open `http://localhost:3000/docs/intro`
4. Highlight text: "NVIDIA Isaac Sim 2025"
5. Verify composer pre-fills with highlighted text
6. Ask: "How does this work with ROS 2?"
7. Verify response streams token-by-token
8. Verify at least 1 source link appears
9. Click source link → Verify navigation to correct doc page

**Expected Result**:
- Widget opens with pre-filled text ✅
- Answer streams immediately ✅
- Source link: `docs/004-isaac-platform/ros2-integration.md#ros-2-humble-integration` ✅
- Clicking link navigates to published docs ✅

---

### Deploy Test 1: Push Change → Wait 5 Min → Ask About New Content

**Objective**: Validate auto-reindex workflow and query freshness.

**Steps**:
1. Add new Markdown file:
   ```bash
   echo "# cuVSLAM Overview\ncuVSLAM is a visual SLAM system..." > docs/test-cuvsslam.md
   ```
2. Commit and push:
   ```bash
   git add docs/test-cuvsslam.md
   git commit -m "Add cuVSLAM test documentation"
   git push origin main
   ```
3. Monitor GitHub Actions tab for "Reindex Documentation" workflow
4. Wait for workflow to complete (green check, <5 minutes)
5. Open deployed site: `https://<username>.github.io/physical-ai-and-humanoid-robotics/`
6. Ask chatbot: "What is cuVSLAM?"
7. Verify answer includes content from `test-cuvsslam.md`
8. Verify source link points to new file

**Expected Result**:
- GitHub Actions completes in <5 minutes ✅
- Qdrant dashboard shows new chunks (search for "cuVSLAM") ✅
- Chatbot answer includes "visual SLAM system" ✅
- Source link: `docs/test-cuvsslam.md` ✅

---

### Rate-Limit Test: Simulate Burst → Verify "Processing..." + Retry

**Objective**: Validate Gemini rate limit handling with exponential backoff.

**Steps**:
1. Modify `backend/main.py` to force rate limit errors:
   ```python
   import random
   @app.post("/api/chat")
   async def chat(request: ChatRequest):
       if random.random() < 0.5:  # 50% chance
           raise HTTPException(status_code=429, detail="Rate limit exceeded")
       # ... normal flow
   ```
2. Send 10 rapid requests:
   ```bash
   for i in {1..10}; do
     curl -X POST http://localhost:8000/api/chat \
       -H "Content-Type: application/json" \
       -d '{"message": "Test '$i'", "session_id": "test-session"}' &
   done
   ```
3. Observe frontend custom chat widget
4. Verify "Processing..." message appears during retries
5. Check backend logs for exponential backoff delays (2s, 4s, 8s)
6. Verify all requests eventually succeed or fail gracefully (no silent failures)

**Expected Result**:
- CustomChatWidget displays "Processing..." during retry ✅
- Backend logs show: "Retrying after 2s", "Retrying after 4s", "Retrying after 8s" ✅
- No silent failures (user sees outcome for all requests) ✅
- Max 3 retries before returning error ✅

---

## Risk Mitigation

### Risk 1: Qdrant Cloud Downtime

**Impact**: Users cannot get answers (vector search fails).

**Likelihood**: Low (Qdrant Cloud SLA: 99.9% uptime).

**Mitigation**:
1. **Circuit Breaker**: After 3 consecutive Qdrant failures, return cached error for 60 seconds
2. **Fallback Response**: Display polite error message: "Vector database temporarily unavailable. Please try again in a moment."
3. **Health Monitoring**: Qdrant availability checked every 5 minutes via Render health checks
4. **Retry Logic**: Single retry with 2-second delay before returning error

**Implementation**:
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

---

### Risk 2: Gemini API Rate Limits

**Impact**: Free tier quota exhausted (60 req/min), users see errors or delays.

**Likelihood**: Medium (1000 queries/day assumption fits quota, but bursts may exceed).

**Mitigation**:
1. **Exponential Backoff Retry**: Max 3 attempts with 2s, 4s, 8s delays (FR-015a)
2. **User Notification**: Display "Processing..." message during retry (FR-015a)
3. **Rate Limit Monitoring**: Log rate limit hits, alert if >50% quota used
4. **Queue with Priority**: Prioritize user queries over reindex operations

**Implementation**:
```python
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=2, min=2, max=10),
    retry=retry_if_exception_type(RateLimitError),
)
async def query_gemini_with_retry(prompt: str):
    yield {"type": "status", "message": "Processing..."}
    result = await agent.stream_response(input=prompt)
    async for chunk in result:
        yield chunk
```

---

### Risk 3: Cold Starts (Render Free Tier)

**Impact**: First request after 15 min inactivity takes 30s, poor UX.

**Likelihood**: High (free tier sleeps instances after inactivity).

**Mitigation**:
1. **Warmup Ping**: GitHub Actions cron job pings `/health` every 10 minutes
2. **Loading State**: CustomChatWidget shows "Connecting..." during cold start
3. **Lazy Loading**: Don't initialize Qdrant/Cohere clients until first request (reduce startup time)

**Implementation** (GitHub Actions warmup):
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

## Next Steps

1. ✅ Phase 0: Research complete (`research.md`)
2. ✅ Phase 1: Backend RAG Agent + Qdrant Indexing complete (`rag_agent.py`, `reindex_docs.py`, Qdrant integration)
3. ✅ Phase 1.5: Local Agent Validation complete (manual reindex, testing script, hallucination detection)
4. ✅ Phase 2: Design complete (`data-model.md`, `contracts/`, `architecture.md`, `quickstart.md`)
5. ⏭️ **Phase 2.5: Run `/sp.tasks`** to generate Neon database setup and chat history persistence tasks
6. ⏭️ Implement Phase 2.5 tasks (database schema, CRUD operations, backend integration)
7. ⏭️ **Phase 3: Run `/sp.tasks`** to generate frontend implementation tasks (context menu + collapsible widget)
8. ⏭️ Implement frontend tasks following TDD (Red → Green → Refactor)
9. ⏭️ Phase 4: Auto-reindex GitHub Action implementation
10. ⏭️ Phase 5: Full end-to-end testing
11. ⏭️ Phase 6: Deploy to Render + GitHub Pages
12. ⏭️ Run all testing scenarios (local, deploy, rate-limit, history restoration)

---

## Artifacts Generated

| Artifact | Status | Location |
|----------|--------|----------|
| **Specification** | ✅ Approved | `specs/rag-chatbot/spec.md` |
| **Research** | ✅ Complete | `specs/rag-chatbot/research.md` |
| **Data Model** | ✅ Complete | `specs/rag-chatbot/data-model.md` |
| **API Contracts** | ✅ Complete | `specs/rag-chatbot/contracts/*.yaml` |
| **Architecture Diagrams** | ✅ Complete | `specs/rag-chatbot/static/diagrams/architecture.md` |
| **Quickstart Guide** | ✅ Complete | `specs/rag-chatbot/quickstart.md` |
| **Implementation Plan** | ✅ Complete | `specs/rag-chatbot/plan.md` (this file) |
| **Tasks** | ⏭️ Pending | Run `/sp.tasks` to generate `tasks.md` |

---

**Status**: ✅ **Planning phase complete** - Ready for `/sp.tasks` to generate implementation tasks.
