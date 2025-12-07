---
id: 0004
title: RAG Chatbot Task Generation
stage: tasks
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot
branch: 001-rag-chatbot
user: BASEL
command: /sp.tasks
labels: ["tasks", "rag-chatbot", "task-breakdown", "implementation-plan", "atomic-tasks"]
links:
  spec: specs/rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/rag-chatbot/tasks.md
tests:
  - None (task planning phase)
---

## Prompt

RAG Chatbot for Physical AI & Humanoid Robotics Book

Follow the just-approved spec and plan exactly.

Break into 5 phases with tiny atomic tasks (15–30 min each).
Total tasks: 16–18 max.

Phase 1 → Frontend: ChatKit JS Widget (4 tasks)
Phase 2 → Backend: RAG Agent with OpenAI Agents SDK (5 tasks)
Phase 3 → Auto-Reindex GitHub Action (3 tasks)
Phase 4 → Testing & Edge Cases (3 tasks)
Phase 5 → Deployment + Final Validation (2 tasks)

Every task must:
- Use existing backend/ folder + uv environment
- Use Gemini + Cohere + Qdrant keys already in .env / main.py
- Use Context7 MCP server for docs when needed
- End with a working, testable piece (widget appears, agent answers, reindex runs, etc.)

Final acceptance:
- Highlight text → ask → get streamed answer with source link in <3 sec
- Push new Markdown → wait <5 min → ask about new content → correct answer
- Rate limit → shows "Processing…" + retries successfully
- Deployed live on Render (free tier) and working

Deliver the full task list as TASKS.md in the specs folder with:
- Task name
- Duration
- Dependency
- Acceptance criteria
- Exact output file/command

Go.

## Response snapshot

Created comprehensive task breakdown with 17 atomic tasks organized across 5 phases:

**Phase 1 - Frontend (4 tasks, 60 min)**: Install ChatKit React, swizzle Docusaurus Layout, create ChatKitWidget.tsx with text highlighting, integrate widget into all pages. Deliverable: Widget visible on all pages with text pre-fill working.

**Phase 2 - Backend (5 tasks, 75 min)**: Install OpenAI Agents SDK + dependencies, create Pydantic models (chat.py), RAG agent (rag_agent.py with Gemini), session manager (session_manager.py with 30-min TTL), add streaming chat endpoints to main.py with CORS. Deliverable: Local test passes - ask question → streamed answer with sources.

**Phase 3 - Auto-Reindex (3 tasks, 45 min)**: Create reindex script (reindex_docs.py with 384-token chunking), GitHub Actions workflow (reindex.yml triggered on docs/**/*.md changes), test locally. Deliverable: Push new .md → wait <5 min → chatbot knows new content.

**Phase 4 - Testing (3 tasks, 45 min)**: Test highlight→ask→stream flow (<3s latency), rate limit exponential backoff (2s/4s/8s), off-topic refusal. Deliverable: All edge cases validated.

**Phase 5 - Deployment (2 tasks, 30 min)**: Create render.yaml config, deploy to Render free tier and GitHub Pages. Deliverable: Production chatbot live and functional.

**Key Decisions**: Each task 15-30 min (atomic), includes exact file paths + commands + acceptance criteria, parallel opportunities identified (Phase 1+2 simultaneous, T006/T007 parallel, T010/T011 parallel), total 255 min (~4.25 hours) for complete implementation.

## Outcome

- ✅ Impact: Complete executable task list ready for immediate implementation
- 🧪 Tests: 3 explicit test tasks (T013, T014, T015) covering end-to-end flows and edge cases
- 📁 Files: 1 task file (tasks.md with 17 tasks)
- 🔁 Next prompts: `/sp.implement` or begin task-by-task execution starting with T001
- 🧠 Reflection: Task organization by phase (not user story) matches user's explicit request while maintaining traceability via [Story] labels

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Format validation PASS (all tasks follow checkbox format with IDs, durations, file paths)
- Prompt variant (if applicable): Standard /sp.tasks with explicit 5-phase breakdown requirement
- Next experiment (smallest change to try): Add complexity scores per task (trivial/moderate/complex)
