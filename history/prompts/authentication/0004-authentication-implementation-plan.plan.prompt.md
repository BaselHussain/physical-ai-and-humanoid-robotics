---
id: 0004
title: Authentication implementation plan
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5
feature: authentication
branch: authentication
user: user
command: /sp.plan
labels: ["planning", "architecture", "better-auth", "neon-postgresql", "personalization"]
links:
  spec: specs/authentication/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/authentication/plan.md
 - specs/authentication/research.md
 - specs/authentication/data-model.md
 - specs/authentication/quickstart.md
 - specs/authentication/contracts/auth-api.yaml
 - specs/authentication/contracts/personalization-api.yaml
 - CLAUDE.md (agent context updated)
tests:
 - None (planning phase - no code tests yet)
---

## Prompt

User invoked `/sp.plan` with request:
"RAG Chatbot Authentication with Better Auth - Follow the just-approved spec in specs/authentication/spec.md exactly. Create the full execution plan including: 1. Architecture diagram (Mermaid) at blocked 6. Risk mitigation: Neon downtime, session expiration, background fetch failures. Output everything in specs/authentication/ (PLAN.md + diagrams in static/diagrams/). Go."

Specification context:
- Feature: RAG chatbot authentication with Better Auth
- Custom background profiling (programming experience, ROS 2 familiarity, hardware access)
- Personalized RAG responses based on user profile
- Neon PostgreSQL database via DATABASE_URL
- Replace in-memory sessions with Better Auth sessions
- Render free tier deployment

## Response snapshot

Created comprehensive implementation plan with full Phase 0 (Research) and Phase 1 (Design & Contracts) outputs.

### Artifacts Created:

1. **plan.md** - Complete implementation plan including:
   - Technical context (Python 3.10+, FastAPI, Better Auth, Neon PostgreSQL, React/Docusaurus)
   - Constitution Check (all 7 principles PASS)
   - Project structure (backend + frontend web app architecture)
   - 4 architecture diagrams embedded as Mermaid (system architecture, auth flow, session management, personalization flow)
   - Phase 0 (Research) and Phase 1 (Design) specifications
   - Risk mitigation strategies for 5 identified risks
   - Success metrics & validation criteria
   - 4-phase implementation timeline (10-15 days estimated)

2. **research.md** (Phase 0) - Technology validation research:
   - Better Auth integration with FastAPI + Neon PostgreSQL
   - Custom user metadata schema (JSON field in Better Auth user model)
   - Better Auth React SDK integration patterns
   - Neon PostgreSQL connection pooling (asyncpg, min=2 max=10)
   - Session expiration handling (Axios interceptors + localStorage)
   - All decisions documented with rationales and rejected alternatives

3. **data-model.md** (Phase 1) - Database schema design:
   - User entity (id, email, password_hash, metadata JSONB, timestamps)
   - BackgroundProfile (embedded in User.metadata.background)
   - Session entity (id, user_id, token, expires_at, created_at)
   - Expertise level mapping rules (beginner/intermediate/advanced)
   - Performance considerations (connection pooling, caching, query optimization)
   - Security considerations (Argon2 hashing, SQL injection prevention)

4. **contracts/auth-api.yaml** (Phase 1) - Authentication API OpenAPI spec:
   - POST /auth/signup (create user + background profile)
   - POST /auth/signin (authenticate user)
   - POST /auth/signout (invalidate session)
   - GET /auth/session (validate current session)
   - GET /auth/background/{user_id} (fetch background profile)
   - Complete request/response schemas with examples (beginner vs advanced)

5. **contracts/personalization-api.yaml** (Phase 1) - Chat API OpenAPI spec:
   - POST /chat/message (personalized RAG responses)
   - Detailed personalization rules (beginner → simple language, advanced → technical depth)
   - Hardware context integration (simulation vs physical hardware)
   - Error handling (session expiration, background fetch failures)

6. **quickstart.md** (Phase 1) - Setup and deployment guide:
   - Prerequisites (Python 3.10+, Node 18+, Neon account, Render account)
   - Local development setup (backend + frontend step-by-step)
   - Testing procedures (signup, signin, personalization validation)
   - Production deployment to Render free tier
   - Troubleshooting guide (5 common issues + solutions)
   - Monitoring & maintenance guide

7. **Agent Context Update** - CLAUDE.md updated with:
   - Python 3.10+ (FastAPI backend), JavaScript/TypeScript (Docusaurus/React frontend)
   - Neon PostgreSQL for users, sessions, background profiles

### Architecture Decisions:

**Technology Stack**:
- Backend: Better Auth Python SDK, FastAPI, asyncpg (Neon driver), Pydantic
- Frontend: Better Auth React SDK, React 18+, Axios, Docusaurus 3.x
- Database: Neon PostgreSQL (free tier) with connection pooling
- Deployment: Render (backend), GitHub Pages (frontend)

**Key Design Patterns**:
- Better Auth metadata JSON field for custom background profiles (avoid separate table)
- Asyncpg connection pool (min=2, max=10) for Neon free tier compatibility
- React Context API + custom hooks for global auth state
- Axios interceptors for session expiration handling
- In-memory caching (5-min TTL) for background profiles (reduce DB queries)

**Risk Mitigations**:
1. Neon downtime → Connection retry + cache fallback
2. Session expiration → Preserve typed message + re-auth modal
3. Background fetch failure → Default intermediate personalization
4. Better Auth rate limits → Client-side throttling
5. Chat widget latency → Parallel background fetch + response generation

### Constitution Compliance:

All 7 constitution principles validated:
- ✅ Educational Clarity: Authentication prerequisite for personalized learning
- ✅ Engineering Accuracy: Industry-standard Better Auth + Neon PostgreSQL
- ✅ Practical Applicability: Fully executable, dependencies specified
- ✅ Spec-Driven Development: Follows approved spec.md (FR-001 through FR-019)
- ✅ Ethical Responsibility: Password security, SQL injection prevention
- ✅ Reproducibility: All dependencies versioned, setup documented
- ✅ Zero Broken State: Incremental deployment, CI/CD tests required

## Outcome

- ✅ Impact: Comprehensive implementation plan ready for Phase 2 (task generation via /sp.tasks)
- 🧪 Tests: No tests yet (planning phase)
- 📁 Files: Created 7 planning artifacts (plan.md, research.md, data-model.md, quickstart.md, 2 API contracts, updated CLAUDE.md)
- 🔁 Next prompts: /sp.tasks to generate detailed implementation task breakdown
- 🧠 Reflection: Complete Phase 0 research and Phase 1 design completed successfully. All unknowns resolved via Context7 MCP research. Architecture diagrams embedded in plan.md (Mermaid format). Ready for task decomposition.

## Evaluation notes (flywheel)

- Failure modes observed: None - all planning steps completed successfully
- Graders run and results (PASS/FAIL): Constitution Check: ALL PASS (7/7 principles)
- Prompt variant (if applicable): Standard /sp.plan workflow with comprehensive research + design phases
- Next experiment (smallest change to try): Consider extracting Mermaid diagrams to separate .mmd files in static/diagrams/ for better version control (currently embedded in plan.md)
