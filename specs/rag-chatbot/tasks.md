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

## Phase 3: Verify Existing Chat API (OPTIONAL - SKIP IF WORKING) (1 task)

**Goal**: Verify existing chat API in backend/main.py works, skip if already confirmed working

**Duration**: 15 minutes (SKIP if chatbot already working)

- [ ] T100 [US1] Verify existing chat API endpoints work correctly
  - **Command**: `cd backend && uv run uvicorn main:app --reload --port 8000`
  - **Test**: Test existing chat endpoint with a question
  - **Acceptance**: Existing chat endpoint streams response with sources, no errors
  - **Duration**: 15 minutes
  - **NOTE**: ⚠️ SKIP THIS ENTIRE PHASE if chatbot is already working - go directly to Phase 3.5

**Checkpoint**: Existing chat API confirmed working - ready to ADD database persistence (not rebuild routes)

---

## Phase 3.5: Neon Database Setup & Chat History Persistence (6 tasks)

**Goal**: Set up Neon Postgres database and implement persistent chat history storage (FR-018)

**Duration**: 90 minutes total (15 min per task)

- [X] T105 [P] [US1] Create Neon Postgres serverless database and obtain connection string
  - **Action**: Create account at console.neon.tech, create new project, copy DATABASE_URL
  - **Acceptance**: Neon project created, connection string obtained, database accessible
  - **Duration**: 15 minutes

- [X] T106 [P] [US1] Create database migration script for chat tables in backend/migrations/001_create_chat_tables.sql
  - **File**: backend/migrations/001_create_chat_tables.sql (new)
  - **Acceptance**: SQL script creates chat_sessions table (session_id UUID PK, user_identifier TEXT, created_at TIMESTAMPTZ, last_active_at TIMESTAMPTZ) and chat_messages table (message_id UUID PK, session_id UUID FK, message_text TEXT, role TEXT, timestamp TIMESTAMPTZ, source_references JSONB), foreign key constraint created, indexes on session_id and timestamp
  - **Duration**: 15 minutes

- [X] T107 [US1] Run migration script to create database schema
  - **Command**: `psql "YOUR_NEON_DATABASE_URL" -f backend/migrations/001_create_chat_tables.sql`
  - **Acceptance**: Tables created successfully, verify with `psql "YOUR_NEON_DATABASE_URL" -c "\dt"`, both tables exist
  - **Duration**: 15 minutes

- [X] T108 [P] [US1] Implement database connection pool in backend/app/database.py
  - **File**: backend/app/database.py (new)
  - **Acceptance**: asyncpg connection pool initialization, NEON_DATABASE_URL environment variable loading, startup/shutdown lifecycle management, connection health checks, configurable pool size (min=2, max=10)
  - **Duration**: 15 minutes

- [X] T109 [P] [US1] Implement chat history CRUD operations in backend/app/chat_history.py
  - **File**: backend/app/chat_history.py (new)
  - **Acceptance**: create_session(user_identifier) returns session_id, get_session(session_id) retrieves session, save_message(session_id, message) saves to DB, get_session_history(session_id) retrieves all messages, update_last_active(session_id) updates timestamp, all queries parameterized (SQL injection protection), error handling for DB failures
  - **Duration**: 15 minutes

- [X] T110 [US1] ADD database persistence to existing chat routes in backend/main.py
  - **File**: backend/main.py (modify existing routes, don't rebuild them)
  - **Acceptance**: Initialize database pool on startup, MODIFY existing chat endpoint to ALSO save messages to Neon (keep all current functionality), ADD new GET /api/sessions/{session_id}/restore endpoint for history, add NEON_DATABASE_URL to backend/.env (user provides value), ⚠️ CRITICAL: existing chatbot functionality remains unchanged, only adds database saving
  - **Duration**: 15 minutes

**Checkpoint**: Chat history persists in Neon database, session restoration working, GET /api/sessions/{id}/restore returns messages

---

## Phase 4: Frontend - ChatKit-JS Widget + Context Menu + Collapsible UI (6 tasks)

**Goal**: Implement interactive chat widget with text selection context menu and collapsible UI (User Story 1 + 3, FR-001a, FR-002, FR-002a)

**Duration**: 90 minutes total (15 min per task)

- [X] T001 [P] [US3] Install ChatKit-JS dependency and verify installation in package.json
  - **Command**: `npm install @openai/chatkit-client`
  - **Output**: package.json modified, node_modules/@openai/chatkit-client exists
  - **Acceptance**: `npm list @openai/chatkit-client` shows version number, no dependency conflicts, no ChatKit-Python in dependencies
  - **Duration**: 15 minutes

- [X] T002 [US3] Swizzle Docusaurus Layout component to inject custom chat widget
  - **Command**: `npm run swizzle @docusaurus/theme-classic Layout -- --wrap`
  - **Output**: src/theme/Layout/index.js created
  - **Acceptance**: File exists and contains wrapped Layout export, imports original Layout successfully
  - **Duration**: 15 minutes

- [X] T003 [P] [US1] Create TextSelectionMenu component for context menu in src/components/TextSelectionMenu.tsx
  - **File**: src/components/TextSelectionMenu.tsx (new)
  - **Acceptance**: Detects text selection on mouseup event (>3 chars), shows context menu with options "Ask from AI" and "Copy", positions menu near selected text, handles "Ask from AI" click to trigger chat widget open with pre-filled text, handles "Copy" click to copy to clipboard, hides menu on deselect or click outside
  - **Duration**: 15 minutes

- [X] T004 [P] [US1] Create CustomChatWidget component with collapsible UI in src/components/CustomChatWidgetWidget.tsx
  - **File**: src/components/CustomChatWidgetWidget.tsx
  - **Acceptance**: Initially closed (only icon visible), opens/collapses on icon click (FR-001a), makes API calls to POST /api/session and POST /api/chat, accepts pre-populated text from TextSelectionMenu (FR-002a), widget styled position fixed/bottom-right/z-index 1000, handles streaming responses, on component mount checks localStorage for session_id and calls GET /api/sessions/{session_id}/restore if exists (FR-018c)
  - **Duration**: 15 minutes

- [ ] T005 [US3] Update swizzled Layout to render both TextSelectionMenu and CustomChatWidget in src/theme/Layout/index.js
  - **File**: src/theme/Layout/index.js
  - **Acceptance**: Layout includes TextSelectionMenu and CustomChatWidget components, state handler passes selected text from TextSelectionMenu to CustomChatWidget, both components appear on localhost:3000/docs/intro after `npm run start`
  - **Duration**: 15 minutes

- [ ] T006 [US1] Test complete frontend flow: text selection → context menu → chat widget open → history restoration
  - **Command**: `npm run start`, open http://localhost:3000/docs/intro
  - **Acceptance**: Chat icon appears (window closed), click icon → window opens, select text → context menu appears, click "Ask from AI" → chat opens with text pre-filled, close browser → reopen → chat history restored from Neon DB
  - **Duration**: 15 minutes

**Checkpoint**: Widget with collapsible UI working, context menu functional, text pre-fill working, history restoration from Neon DB successful

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
- **Phase 3.5 (Neon Database)**: Depends on Phase 3 (new API routes) to integrate with
- **Phase 4 (Frontend)**: Depends on Phase 3.5 (database persistence for history restoration)
- **Phase 5 (Auto-Reindex + Deploy)**: Depends on all previous phases passing

### User Story Dependencies

- **User Story 1 (P1) - Interactive Reading**: Tasks T100, T101, T102, T103, T104, T105-T110 (database), T003, T004, T005, T006, T013, T014
  - Requires Phase 3 (API routes) + Phase 3.5 (database persistence) + Phase 4 (frontend with context menu)
- **User Story 2 (P2) - Fresh Content Sync**: Tasks T010, T011, T012
  - Can start after Phase 3 (new backend routes working)
- **User Story 3 (P3) - Zero-Config Widget**: Tasks T001, T002, T005
  - Requires Phase 3 backend routes and Phase 3.5 database to function

### Within Each Phase

- **Phase 3**: T100 → T103 → T101 → T104 → T102 (T102 sequential after T101)
- **Phase 3.5**: T105, T106, T108, T109 in parallel → T107 → T110
- **Phase 4**: T001 → T002 → T003, T004 in parallel → T005 → T006
- **Phase 5**: T010, T011 in parallel → T012 → T013 → T014 → T015

### Parallel Opportunities

**Within Phase 3.5**:
- T105, T106, T108, T109 can run in parallel (different components, no dependencies)

**Within Phase 4**:
- T003 and T004 can run in parallel after T001, T002 completed

---

## Implementation Strategy

### MVP First (User Story 1 - Single Sprint)

Given the scope (19 tasks, 15 min each = ~4.75 hours total):

1. **Phase 3**: SKIP (chatbot already working) - 0 minutes
2. **Phase 3.5**: ADD Neon database to existing routes (6 tasks) - 90 minutes
3. **Phase 4**: ADD context menu + collapsible UI to frontend (6 tasks) - 90 minutes
4. **Phase 5**: Testing and deployment (6 tasks) - 90 minutes

**Total Duration**: 4.75 hours (285 minutes) for 18 tasks (assumes Phase 3 skipped)

### Parallel Team Strategy

With 2 developers (or solo developer approach):

1. **Developer A**: Phase 3.5 backend (T105, T108, T109) - 45 minutes in parallel
2. **Developer B**: Phase 3.5 database (T106, T107) - 30 minutes
3. **Both collaborate**: T110 (ADD database to existing routes) - 15 minutes
4. **Developer A**: Phase 4 backend support - minimal
5. **Developer B**: Phase 4 frontend (T001-T006) - 90 minutes
6. **Both**: Phase 5 (testing/deploy) - 90 minutes

**Optimized Duration with Parallelization**: ~3.5-4 hours

**Solo Developer**: Work sequentially through phases, ~4.75 hours total

---

## Notes

- **[P] tasks** = different files, no dependencies, can run in parallel
- **[Story] labels**: US1 (Interactive Reading), US2 (Fresh Content Sync), US3 (Zero-Config Widget)
- **NO new routes needed**: Existing chat API in main.py already works, we're only ADDING 3 features
- **New Features Added** (ADDITIONS only, no rebuilding):
  - **Neon Postgres**: ADD database saving to existing chat routes (FR-018) - survives browser restarts
  - **Context Menu**: NEW component for text selection with "Ask from AI" option (FR-002, FR-002a)
  - **Collapsible Widget**: UPDATE frontend to show icon initially, open on click (FR-001a)
- **Existing backend**: Keep ALL current functionality, only ADD database.save_message() calls to existing routes
- **Context7 MCP**: Use for FastAPI, OpenAI Agents SDK, Qdrant, ChatKit-JS, Docusaurus, asyncpg documentation lookup
- **Commit strategy**: Commit after each phase completion
- **Checkpoints**: Each phase ends with a working, testable increment
- **Avoid**: ChatKit-Python dependencies, only use pure FastAPI + OpenAI Agents SDK
- **IMPORTANT**: Do NOT modify existing working chatbot functionality, only ADD new features on top

---

## Final Acceptance Criteria

✅ **Existing chat API verified working**: Current chatbot functionality unchanged (Phase 3 skipped if already working)
✅ **Neon database integration**: Database persistence ADDED to existing chat routes (T105-T110)
✅ **Session restoration**: GET /api/sessions/{id}/restore returns full chat history (T110)
✅ **Context menu for text selection**: Appears when text selected, "Ask from AI" option pre-fills chat (T003, FR-002, FR-002a)
✅ **Collapsible chat widget**: Initially closed (icon only), opens on click (T004, FR-001a)
✅ **Chat history restoration**: Close browser → reopen → history loaded from Neon (T006, FR-018c)
✅ **Frontend connects to new endpoints**: No ChatKit-Python dependency (T004, T005)
✅ **Highlight text → context menu → ask → streamed answer with source link via pure FastAPI in <3 sec** (T013)
✅ **Push new Markdown → wait <5 min → ask about new content → correct answer via pure FastAPI** (T012)
✅ **Rate limit → shows "Processing…" via pure FastAPI + retries successfully** (T014)
✅ **Deployed live on Render with pure FastAPI backend + Neon database** (T015)
✅ **NO ChatKit-Python dependencies anywhere in the stack** (verified throughout)
✅ **Existing chatbot functionality UNCHANGED** (only new features added on top)

**Total Tasks**: 19 actual tasks (Phase 3 is optional/skippable, Phases 3.5, 4, 5 are required)
**Total Duration**: ~315 minutes (5.25 hours) if Phase 3 skipped, ~330 min if Phase 3 run
**Parallel Opportunities**: Within Phase 3.5 (T105, T106, T108, T109) and Phase 4 (T003, T004)