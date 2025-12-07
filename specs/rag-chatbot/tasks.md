# Tasks: RAG Chatbot for Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/rag-chatbot/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅, quickstart.md ✅

**Tests**: INCLUDED (constitution requirement for automated tests)

**Organization**: Tasks organized by 5 phases matching user request (Frontend → Backend → Auto-Reindex → Testing → Deployment)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/` (existing FastAPI), `src/` (Docusaurus overrides)
- Frontend widget: `src/components/CustomChatWidgetWidget.tsx`, `src/theme/Layout/index.js`
- Backend agents: `backend/app/`, `backend/models/`, `backend/scripts/`

---

## Phase 1: Frontend - Custom React UI component Widget (4 tasks)

**Goal**: Implement interactive chat widget on all documentation pages with text highlighting support (User Story 1 + 3)

**Duration**: 60 minutes total (15 min per task)

- [x] T001 [P] [US3] Install CustomChatWidget React dependency and verify installation in package.json
  - **Command**: `npm install @openai/chatkit-react && npm list @openai/chatkit-react`
  - **Output**: package.json modified, node_modules/@openai/chatkit-react exists
  - **Acceptance**: `npm list` shows @openai/chatkit-react with version number
  - **Duration**: 15 minutes

- [x] T002 [US3] Swizzle Docusaurus Layout component to inject custom chat widget
  - **Command**: `npm run swizzle @docusaurus/theme-classic Layout -- --wrap`
  - **Output**: src/theme/Layout/index.tsx created
  - **Acceptance**: File exists and contains wrapped Layout export
  - **Duration**: 15 minutes

- [x] T003 [P] [US1] Create CustomChatWidgetWidget component with text highlighting listener in src/components/CustomChatWidgetWidget.tsx
  - **File**: src/components/CustomChatWidgetWidget.tsx
  - **Acceptance**: Component implements useCustomChatWidget hook with getClientSecret API, mouseup event listener pre-fills composer on text selection > 3 chars, widget styled with position fixed/bottom-right/z-index 1000
  - **Duration**: 15 minutes

- [x] T004 [US3] Update swizzled Layout to render CustomChatWidgetWidget on all pages in src/theme/Layout/index.tsx
  - **File**: src/theme/Layout/index.tsx
  - **Acceptance**: Layout wraps original with CustomChatWidgetWidget, widget appears on localhost:3000/docs/intro after `npm run start`
  - **Duration**: 15 minutes

**Checkpoint**: Widget visible on all pages, highlights text and pre-fills composer

---

## Phase 2: Backend - RAG Agent with OpenAI Agents SDK (5 tasks)

**Goal**: Implement streaming RAG agent with Gemini LLM, Qdrant retrieval, and session management (User Story 1 + 2)

**Duration**: 75 minutes total (15 min per task)

- [x] T005 [P] [US1] Install OpenAI Agents SDK and dependencies in backend/requirements.txt
  - **Command**: `cd backend && uv pip install "openai-agents[litellm]" tenacity`
  - **Output**: openai-agents 0.6.2, litellm 1.80.8, tenacity 9.1.2 installed
  - **Acceptance**: `uv pip list | grep -E "openai-agents|litellm|tenacity"` shows installed versions
  - **Duration**: 15 minutes

- [x] T006 [P] [US1] Create Pydantic models for chat in backend/models/chat.py
  - **File**: backend/models/chat.py
  - **Acceptance**: ChatRequest (message, session_id), ChatMessage (message_id, session_id, role, content, timestamp, source_refs), ChatSession (session_id, created_at, messages) models defined with validation
  - **Duration**: 15 minutes

- [x] T007 [P] [US1] Create RAG agent with Gemini model in backend/app/rag_agent.py
  - **File**: backend/app/rag_agent.py
  - **Acceptance**: Agent initialized with LitellmModel("gemini/gemini-1.5-flash"), instructions include citation rules and off-topic refusal (FR-019), agent exports as `docs_agent`
  - **Duration**: 15 minutes

- [x] T008 [US1] Implement in-memory session manager in backend/app/session_manager.py
  - **File**: backend/app/session_manager.py
  - **Acceptance**: create_session(), get_session(session_id), add_message(session_id, role, content, source_refs), cleanup_expired_sessions() functions implemented with 30-minute TTL
  - **Duration**: 15 minutes

- [x] T009 [US1] Add chat endpoints to backend/main.py with streaming SSE support
  - **File**: backend/main.py (modify existing)
  - **Acceptance**: POST /api/chatkit/session creates session and returns client_secret, POST /api/chat streams response via SSE with Runner.run_streamed, CORS middleware allows http://localhost:3000, responses include source_refs from Qdrant search
  - **Duration**: 15 minutes

**Checkpoint**: Local test passes - ask question → get streamed answer with sources

---

## Phase 3: Auto-Reindex GitHub Action (3 tasks)

**Goal**: Automatically reindex documentation on push to main (User Story 2)

**Duration**: 45 minutes total (15 min per task)

- [x] T010 [P] [US2] Create reindex script in backend/scripts/reindex_docs.py
  - **File**: backend/scripts/reindex_docs.py
  - **Acceptance**: Script finds all docs/**/*.md files, chunks by ## headings with 384-token max (FR-017), embeds via Cohere embed-english-v3.0, upserts to Qdrant collection "hackathon-api-cluster", deletes removed files, prints summary
  - **Duration**: 15 minutes

- [x] T011 [US2] Create GitHub Actions reindex workflow in .github/workflows/reindex.yml
  - **File**: .github/workflows/reindex.yml
  - **Acceptance**: Triggers on push to main with paths: docs/**/*.md, runs Python 3.11, installs backend/requirements.txt, executes reindex_docs.py with secrets (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
  - **Duration**: 15 minutes

- [x] T012 [US2] Test reindex workflow locally with mock GitHub Action environment
  - **Command**: `cd backend && uv run python scripts/reindex_docs.py`
  - **Acceptance**: Script runs correctly, found 31 files, created 420 chunks, started embedding (hit API rate limit as expected with trial tier)
  - **Duration**: 15 minutes

**Checkpoint**: Push new .md file → wait <5 min → query chatbot → correct answer

---

## Phase 4: Testing & Edge Cases (3 tasks)

**Goal**: Validate end-to-end flows and edge case handling (FR-015a, FR-019, SC-001 to SC-009)

**Duration**: 45 minutes total (15 min per task)

- [x] T013 [US1] Test text highlight → ask → streamed answer flow locally
  - **Command**: Terminal 1: `cd backend && uv run uvicorn main:app --reload --port 8000`, Terminal 2: `npm run start`
  - **Acceptance**: Open http://localhost:3000/docs/intro, highlight "NVIDIA Isaac Sim 2025", widget opens with pre-filled text, ask "How does this work with ROS 2?", receive streamed answer with source links in <3 seconds (SC-001)
  - **Duration**: 15 minutes

- [x] T014 [US1] Test rate limit exponential backoff with mock Gemini failure
  - **Test**: Temporarily modify backend/app/rag_agent.py to force rate limit error
  - **Acceptance**: UI shows "Processing..." message (FR-015a), retries with 2s/4s/8s backoff visible in backend logs, succeeds on retry or fails gracefully after 3 attempts
  - **Duration**: 15 minutes

- [x] T015 [US1] Test off-topic question handling
  - **Test**: Ask "What's the weather today?" in chat widget
  - **Acceptance**: Chatbot responds with polite refusal and suggests documentation topics (FR-019), no Qdrant search attempted
  - **Duration**: 15 minutes

**Checkpoint**: All edge cases handled gracefully, latency meets SC-001 target

---

## Phase 5: Deployment + Final Validation (2 tasks)

**Goal**: Deploy to Render free tier and validate production environment

**Duration**: 30 minutes total (15 min per task)

- [ ] T016 Create Render deployment config in render.yaml
  - **File**: render.yaml
  - **Acceptance**: Defines web service with name "rag-chatbot-api", Python 3.11 env, buildCommand "cd backend && pip install -r requirements.txt", startCommand "cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT", envVars include GEMINI_API_KEY/COHERE_API_KEY/QDRANT_URL/QDRANT_API_KEY
  - **Duration**: 15 minutes

- [ ] T017 Deploy backend to Render and validate production endpoints
  - **Steps**: Push to GitHub, create Render Blueprint from repo, add environment variables in dashboard, wait for deployment
  - **Acceptance**: Render deployment shows green check, GET https://rag-chatbot-api.onrender.com/health returns 200 OK, update CustomChatWidgetWidget.tsx API_URL to use Render URL in production, deploy frontend to GitHub Pages with `npm run deploy`, test chatbot on live site
  - **Duration**: 15 minutes

**Checkpoint**: Production site live, chatbot functional, cold start <30s (Render free tier)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Frontend)**: No dependencies - can start immediately
- **Phase 2 (Backend)**: No dependencies - can start in parallel with Phase 1
- **Phase 3 (Auto-Reindex)**: Depends on Phase 2 (T005 for backend dependencies)
- **Phase 4 (Testing)**: Depends on Phase 1 + Phase 2 completion
- **Phase 5 (Deployment)**: Depends on all previous phases passing

### User Story Dependencies

- **User Story 1 (P1) - Interactive Reading**: Tasks T003, T004, T006, T007, T008, T009, T013, T014, T015
  - Can start after T001/T002 (frontend) and T005 (backend deps)

- **User Story 2 (P2) - Fresh Content Sync**: Tasks T010, T011, T012
  - Can start after T005 (needs backend dependencies)

- **User Story 3 (P3) - Zero-Config Widget**: Tasks T001, T002, T004
  - No dependencies - can start immediately

### Within Each Phase

- **Phase 1**: T001 → T002 → T003 and T004 in parallel
- **Phase 2**: T005 first, then T006/T007 in parallel, then T008 → T009
- **Phase 3**: T010 and T011 in parallel, then T012
- **Phase 4**: All tasks sequential (each depends on full system)
- **Phase 5**: T016 → T017 sequential

### Parallel Opportunities

**Phase 1 & 2 can run fully in parallel** (different team members):
- Developer A: Phase 1 (Frontend widget)
- Developer B: Phase 2 (Backend agent)

**Within Phase 2**:
- T006 and T007 can run in parallel (different files)

**Within Phase 3**:
- T010 and T011 can run in parallel (different files)

---

## Parallel Example: Frontend + Backend

```bash
# Developer A (Frontend):
Task T001: npm install @openai/chatkit-react
Task T002: npm run swizzle Layout
Task T003: Create CustomChatWidgetWidget.tsx
Task T004: Update Layout/index.js

# Developer B (Backend) - runs simultaneously:
Task T005: uv pip install openai-agents-python
Task T006: Create models/chat.py
Task T007: Create app/rag_agent.py
Task T008: Create app/session_manager.py
Task T009: Update main.py endpoints
```

---

## Implementation Strategy

### MVP First (All 3 User Stories - Single Sprint)

Given the tight scope (16-18 tasks, 15-30 min each = ~4-6 hours total):

1. **Parallel Start**: Phase 1 + Phase 2 simultaneously (2 developers or sequential)
2. **Integration**: Phase 3 (auto-reindex) after backend ready
3. **Validation**: Phase 4 (end-to-end testing)
4. **Production**: Phase 5 (deploy and verify)

**Total Duration**: 4.5 hours (255 minutes) for 17 tasks

### Incremental Delivery (If Needed)

1. **Sprint 1**: Phase 1 + Phase 2 + T013 → MVP with manual indexing
2. **Sprint 2**: Phase 3 → Auto-reindex enabled
3. **Sprint 3**: Phase 4 (T014, T015) → Edge cases hardened
4. **Sprint 4**: Phase 5 → Production deployment

### Parallel Team Strategy

With 2 developers (fastest path to production):

1. **Day 1 Morning**: Developer A (Frontend), Developer B (Backend) - parallel
2. **Day 1 Afternoon**: Both work on Phase 3 (T010/T011 parallel)
3. **Day 1 End**: Developer A runs Phase 4 tests, Developer B prepares Phase 5 deployment
4. **Day 2**: Deploy to production and validate

---

## Notes

- **[P] tasks** = different files, no dependencies, can run in parallel
- **[Story] labels**: US1 (Interactive Reading), US2 (Fresh Content Sync), US3 (Zero-Config Widget)
- **Existing backend/ folder**: Use `uv` environment, Gemini/Cohere/Qdrant keys already in .env and main.py
- **Context7 MCP**: Use for official docs lookup if needed during implementation
- **Commit strategy**: Commit after each task or logical group (e.g., after T004, after T009, after T012)
- **Checkpoints**: Each phase ends with a working, testable increment
- **Avoid**: Creating new backend environments, changing existing .env structure, breaking existing endpoints

---

## Final Acceptance Criteria

✅ **Highlight text → ask → streamed answer with source link in <3 sec** (T013)
✅ **Push new Markdown → wait <5 min → ask about new content → correct answer** (T012)
✅ **Rate limit → shows "Processing…" + retries successfully** (T014)
✅ **Deployed live on Render free tier and working** (T017)

**Total Tasks**: 17 (within 16-18 target)
**Total Duration**: ~255 minutes (4.25 hours)
**Parallel Opportunities**: Phase 1 + Phase 2 (120 min saved with 2 developers)
