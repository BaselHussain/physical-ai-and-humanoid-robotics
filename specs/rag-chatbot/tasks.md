# Tasks: RAG Chatbot for Physical AI & Humanoid Robotics Book - FINAL CLEAN VERSION

**Input**: Design documents from `/specs/rag-chatbot/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅, quickstart.md ✅

**Tests**: INCLUDED (constitution requirement for automated tests)

**Organization**: Tasks organized by 3 phases following the final clean architecture (Pure FastAPI → Frontend → Deploy)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/` (existing FastAPI), `src/` (Docusaurus overrides)
- Frontend widget: `src/components/CustomChatWidgetWidget.tsx`, `src/theme/Layout/index.js`
- Backend agents: `backend/app/`, `backend/models/`, `backend/scripts/`

---

## Phase 3: Pure FastAPI Routes (5 tasks)

**Goal**: Create clean FastAPI routes with our working OpenAI Agents SDK RAG agent (User Story 1)

**Duration**: 75 minutes total (15 min per task)

- [ ] T100 [P] [US1] Use Context7 MCP server to research latest FastAPI documentation for streaming responses
  - **Action**: Query Context7 MCP for FastAPI SSE streaming best practices
  - **Output**: Extract correct StreamingResponse patterns and event streaming examples
  - **Acceptance**: Document proper FastAPI streaming implementation patterns for token-by-token responses
  - **Duration**: 15 minutes

- [ ] T101 [US1] Create clean FastAPI routes in backend/main.py: POST /api/session → returns session_id, POST /api/chat → accepts {session_id, message} → calls our working docs_agent → streams response with source links
  - **File**: backend/main.py (modify existing)
  - **Acceptance**: POST /api/session creates new session and returns session_id, POST /api/chat accepts {session_id, message} and uses our working docs_agent to return streaming response with source citations, CORS configured for book domain (https://baselhussain.github.io), no ChatKit-Python dependencies
  - **Duration**: 15 minutes

- [ ] T102 [US1] Test new routes locally with curl: curl POST /api/session → get session_id → curl POST /api/chat with {session_id, message}
  - **Command**: `cd backend && uv run uvicorn main:app --reload --port 8000`
  - **Test**: Terminal 2: `curl -X POST http://localhost:8000/api/session` then `curl -X POST http://localhost:8000/api/chat -d '{"session_id":"...", "message":"What is ROS 2?"}' -H "Content-Type: application/json"`
  - **Acceptance**: Session creation returns valid session_id, chat endpoint streams response with sources, no errors in console, streaming works perfectly with source citations
  - **Duration**: 15 minutes

- [ ] T103 [P] [US1] Update Pydantic models in backend/models/chat.py to support new API structure
  - **File**: backend/models/chat.py
  - **Acceptance**: Create SessionResponse (session_id), Update ChatRequest to accept {session_id, message}, ensure compatibility with streaming responses
  - **Duration**: 15 minutes

- [ ] T104 [US1] Add proper CORS configuration for book domain in backend/main.py
  - **File**: backend/main.py (modify existing)
  - **Acceptance**: CORS middleware allows https://baselhussain.github.io for production use, maintains local development access
  - **Duration**: 15 minutes

**Checkpoint**: New API routes working perfectly - POST /api/session and POST /api/chat with streaming responses and source citations

---

## Phase 4: Frontend - ChatKit-JS Widget (4 tasks)

**Goal**: Implement interactive chat widget using ChatKit-JS that connects to new API routes (User Story 1 + 3)

**Duration**: 60 minutes total (15 min per task)

- [ ] T001 [P] [US3] Install ChatKit-JS dependency and verify installation in package.json
  - **Command**: `npm install @openai/chatkit-client`
  - **Output**: package.json modified, node_modules/@openai/chatkit-client exists
  - **Acceptance**: `npm list @openai/chatkit-client` shows version number, no dependency conflicts, no ChatKit-Python in dependencies
  - **Duration**: 15 minutes

- [ ] T002 [US3] Swizzle Docusaurus Layout component to inject custom chat widget
  - **Command**: `npm run swizzle @docusaurus/theme-classic Layout -- --wrap`
  - **Output**: src/theme/Layout/index.js created
  - **Acceptance**: File exists and contains wrapped Layout export, imports original Layout successfully
  - **Duration**: 15 minutes

- [ ] T003 [P] [US1] Create CustomChatWidgetWidget component that connects to new /api/chat in src/components/CustomChatWidgetWidget.tsx
  - **File**: src/components/CustomChatWidgetWidget.tsx
  - **Acceptance**: Component makes API calls to new endpoints (POST /api/session, POST /api/chat), implements text highlighting listener (mouseup event), widget styled with position fixed/bottom-right/z-index 1000, handles streaming responses from new API
  - **Duration**: 15 minutes

- [ ] T004 [US3] Update swizzled Layout to render CustomChatWidgetWidget on all pages in src/theme/Layout/index.js
  - **File**: src/theme/Layout/index.js
  - **Acceptance**: Layout wraps original with CustomChatWidgetWidget, widget appears on localhost:3000/docs/intro after `npm run start`, connects to new API routes (not ChatKit-Python)
  - **Duration**: 15 minutes

**Checkpoint**: Widget visible on all pages, connects to new API, highlights text and streams responses

---

## Phase 5: Auto-Reindex + Full Testing + Deploy (6 tasks)

**Goal**: Complete auto-reindex, full testing and production deployment with pure FastAPI stack (User Story 2)

**Duration**: 90 minutes total (15 min per task)

- [ ] T010 [P] [US2] Create reindex script in backend/scripts/reindex_docs.py (if not already present)
  - **File**: backend/scripts/reindex_docs.py
  - **Acceptance**: Script finds all docs/**/*.md files, chunks by ## headings with 384-token max (FR-017), embeds via Cohere embed-english-v3.0, upserts to Qdrant collection "hackathon-api-cluster", deletes removed files, prints summary
  - **Duration**: 15 minutes

- [ ] T011 [US2] Create GitHub Actions reindex workflow in .github/workflows/reindex.yml
  - **File**: .github/workflows/reindex.yml
  - **Acceptance**: Triggers on push to main with paths: docs/**/*.md, runs Python 3.11, installs backend/requirements.txt, executes reindex_docs.py with secrets (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY), no ChatKit-Python dependencies
  - **Duration**: 15 minutes

- [ ] T012 [US2] Test reindex workflow with pure FastAPI backend locally
  - **Command**: `cd backend && uv run python scripts/reindex_docs.py`
  - **Acceptance**: Script runs correctly with pure FastAPI dependencies, found all documentation files, created chunks, started embedding (hit API rate limit as expected with trial tier)
  - **Duration**: 15 minutes

- [ ] T013 [US1] Test full flow: highlight text → ask → streamed answer with sources via pure FastAPI
  - **Command**: Terminal 1: `cd backend && uv run uvicorn main:app --reload --port 8000`, Terminal 2: `npm run start`
  - **Acceptance**: Open http://localhost:3000/docs/intro, highlight "NVIDIA Isaac Sim 2025", widget opens with pre-filled text, ask "How does this work with ROS 2?", receive streamed answer via pure FastAPI with source links in <3 seconds (SC-001)
  - **Duration**: 15 minutes

- [ ] T014 [US1] Test rate limit exponential backoff with pure FastAPI backend
  - **Test**: Verify current rate limit handling in backend/main.py with tenacity retry
  - **Acceptance**: UI shows "Processing..." message (FR-015a), retries with 2s/4s/8s backoff visible in backend logs, succeeds on retry or fails gracefully after 3 attempts
  - **Duration**: 15 minutes

- [ ] T015 [US1] Deploy pure FastAPI backend to Render and validate production endpoints
  - **Steps**: Create render.yaml with pure FastAPI dependencies (no ChatKit-Python), deploy to Render, update frontend to use production API
  - **Acceptance**: Render deployment shows green check, GET https://rag-chatbot-api.onrender.com/api/session returns 200 OK, update CustomChatWidgetWidget.tsx API_URL to use Render URL in production, deploy frontend to GitHub Pages with `npm run deploy`, test chatbot on live site
  - **Duration**: 15 minutes

**Checkpoint**: Production site live with pure FastAPI backend, full functionality working without ChatKit-Python

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Backend - Completed)**: OpenAI Agents SDK RAG agent working perfectly with Qdrant indexing (tasks T005-T009, T005.5 completed)
- **Phase 2 (Local Validation - Completed)**: Agent validation passed with no hallucination (task T005.5 completed)
- **Phase 3 (Pure FastAPI Routes)**: Depends on completed backend RAG functionality
- **Phase 4 (Frontend)**: Depends on Phase 3 (new API routes working)
- **Phase 5 (Auto-Reindex + Deploy)**: Depends on all previous phases passing

### User Story Dependencies

- **User Story 1 (P1) - Interactive Reading**: Tasks T100, T101, T102, T103, T104, T003, T004, T013, T014
  - Can start after Phase 3 (new backend routes)
- **User Story 2 (P2) - Fresh Content Sync**: Tasks T010, T011, T012
  - Can start after Phase 3 (new backend routes working)
- **User Story 3 (P3) - Zero-Config Widget**: Tasks T001, T002, T004
  - Can start immediately but requires Phase 3 backend routes to function

### Within Each Phase

- **Phase 3**: T100 → T103 → T101 → T104 → T102 (T102 sequential after T101)
- **Phase 4**: T001 → T002 → T003 and T004 in parallel
- **Phase 5**: T010, T011 in parallel → T012 → T013 → T014 → T015

### Parallel Opportunities

**Within Phase 4**:
- T003 and T004 can run in parallel after T001, T002 completed

---

## Implementation Strategy

### MVP First (User Story 1 - Single Sprint)

Given the tight scope (15 tasks, 15 min each = ~3.75 hours total):

1. **Phase 3**: New API routes (5 tasks) - 75 minutes
2. **Phase 4**: Frontend widget (4 tasks) - 60 minutes
3. **Phase 5**: Testing and deployment (6 tasks) - 90 minutes

**Total Duration**: 3.75 hours (225 minutes) for 15 tasks

### Parallel Team Strategy

With 2 developers:

1. **Developer A**: Phase 3 (backend routes) - 75 minutes
2. **Developer B**: Phase 4 (frontend) - starts after Phase 3 complete - 60 minutes
3. **Both**: Phase 5 (testing/deploy) - 90 minutes

---

## Notes

- **[P] tasks** = different files, no dependencies, can run in parallel
- **[Story] labels**: US1 (Interactive Reading), US2 (Fresh Content Sync), US3 (Zero-Config Widget)
- **NO ChatKit-Python**: Pure FastAPI + OpenAI Agents SDK stack only (the winning stack)
- **Existing backend**: Use existing OpenAI Agents SDK implementation, just add new routes
- **Context7 MCP**: Use for FastAPI, OpenAI Agents SDK, Qdrant, ChatKit-JS, Docusaurus documentation lookup
- **Commit strategy**: Commit after each phase completion
- **Checkpoints**: Each phase ends with a working, testable increment
- **Avoid**: ChatKit-Python dependencies, only use pure FastAPI + OpenAI Agents SDK

---

## Final Acceptance Criteria

✅ **Pure FastAPI API routes working**: POST /api/session and POST /api/chat with streaming responses (T101, T102)
✅ **Frontend connects to new endpoints**: No ChatKit-Python dependency (T003, T004)
✅ **Highlight text → ask → streamed answer with source link via pure FastAPI in <3 sec** (T013)
✅ **Push new Markdown → wait <5 min → ask about new content → correct answer via pure FastAPI** (T012)
✅ **Rate limit → shows "Processing…" via pure FastAPI + retries successfully** (T014)
✅ **Deployed live on Render with pure FastAPI backend** (T015)
✅ **NO ChatKit-Python dependencies anywhere in the stack** (verified throughout)

**Total Tasks**: 15 (within 15-18 target)
**Total Duration**: ~225 minutes (3.75 hours)
**Parallel Opportunities**: Within Phase 4 (T003, T004)