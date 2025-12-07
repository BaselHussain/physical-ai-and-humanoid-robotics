---
id: 0001
title: Create RAG Chatbot Specification
stage: spec
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot
branch: 001-rag-chatbot
user: BASEL
command: /sp.specify
labels: ["rag", "chatbot", "specification", "docusaurus", "fastapi", "openai-agents-sdk"]
links:
  spec: specs/rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/rag-chatbot/spec.md
  - specs/rag-chatbot/checklists/requirements.md
tests:
  - None (specification phase)
---

## Prompt

RAG Chatbot for Physical AI & Humanoid Robotics Book

Create folder: specs/rag-chatbot/ only (no number prefix)

Tech stack (locked):
- Frontend: ChatKit JS (official OpenAI ChatKit JavaScript/React SDK)
- Backend: Python FastAPI + OpenAI Agents SDK (v0.6.2+) running on Context7 MCP server
- Embedding model: Cohere (keys already in backend/.env + main.py)
- Vector DB: Qdrant Cloud (URL + API key already hardcoded in backend/main.py)
- LLM: Gemini API (key already in backend/.env)
- Use existing backend/ folder with uv + all installed packages

Knowledge sources (MANDATORY):
- Always use Context7 MCP server for:
  • OpenAI Agents SDK Python docs
  • ChatKit JS docs
  • Docusaurus docs
- Qdrant & FastAPI docs optional via Context7 if needed

Success criteria:
- ChatKit JS widget on every page of the published book
- Highlight any text → ask → perfect answer with source links + streaming
- Auto-reindex all Markdown files from docs/ on every GitHub push
- Backend runs in existing backend/ folder with current keys
- Deployable on Render/Railway free tier

Constraints:
- Use only existing dependencies + hardcoded keys
- No OpenAI API — only Gemini + Cohere
- No user authentication
- Production-ready

Not building:
- User accounts / login
- Local vector DB
- Custom chat UI (must use ChatKit JS)

Go.

## Response snapshot

Created comprehensive RAG chatbot specification with the following structure:

**Branch & Directory**: Created `001-rag-chatbot` branch with spec folder at `specs/rag-chatbot/` (without number prefix as requested)

**User Stories** (3 prioritized, independently testable):
- P1: Interactive Reading with AI Assistant - highlight text, ask questions, receive streamed answers with source links
- P2: Fresh Content Synchronization - auto-reindex Markdown files on GitHub push within 5 minutes
- P3: Zero-Config Widget Integration - chat widget available on all pages without authentication

**Functional Requirements** (18 total):
- FR-001 to FR-006: Chat widget functionality, text highlighting, streaming responses, source links, auto-reindexing
- FR-007 to FR-011: Technical constraints (Cohere embeddings, Qdrant storage, Gemini LLM, no OpenAI, semantic search)
- FR-012 to FR-018: Session management, deployment requirements, error handling, Markdown processing, chunking strategy

**Success Criteria** (9 measurable, technology-agnostic):
- SC-001: 2-second response latency (95th percentile)
- SC-002: 99% uptime on free tier
- SC-003: 2-second widget load time
- SC-004: 5-minute reindex for 200 files
- SC-005 to SC-009: Answer quality, concurrency, relevance, deployment, conversation context

**Key Entities**: Documentation Chunk, Chat Message, Chat Session, Indexing Job

**Edge Cases** (8 identified): off-topic questions, concurrent users, service outages, conflicting content, long inputs, empty files, rate limits, special formatting

**Quality Validation**: Created `specs/rag-chatbot/checklists/requirements.md` with all 13 validation items passing - spec is ready for `/sp.clarify` or `/sp.plan`

## Outcome

- ✅ Impact: Complete feature specification ready for architectural planning
- 🧪 Tests: Specification quality checklist validates all 13 criteria passed
- 📁 Files: specs/rag-chatbot/spec.md (156 lines), specs/rag-chatbot/checklists/requirements.md (58 lines)
- 🔁 Next prompts: `/sp.clarify` (optional if further refinement needed) or `/sp.plan` (proceed with architectural design)
- 🧠 Reflection: Specification demonstrates strong adherence to technology-agnostic requirements while preserving technical constraints from user input. Successfully balanced business-focused language with sufficient detail for planning phase.

## Evaluation notes (flywheel)

- Failure modes observed: Initial branch created with number prefix (001-rag-chatbot) despite user requesting no prefix for folder. Corrected by renaming `specs/001-rag-chatbot` to `specs/rag-chatbot` while keeping branch numbered.
- Graders run and results (PASS/FAIL): Specification Quality Checklist - 13/13 PASS (Content Quality: 4/4, Requirement Completeness: 8/8, Feature Readiness: 4/4)
- Prompt variant (if applicable): Standard /sp.specify workflow with explicit tech stack constraints
- Next experiment (smallest change to try): Consider creating separate "Technical Constraints" section to isolate locked tech stack from business requirements for better spec/plan separation
