# Tasks: RAG Chatbot Authentication with FastAPI-Users

**Input**: Design documents from `/specs/authentication/`
**Prerequisites**: spec.md (complete), plan.md (complete), research.md (complete), data-model.md (complete), contracts/ (complete)

**Branch**: `authentication` → **Integration Branch**: TBD (will be created from main)
**Updated**: 2025-12-25 (Navbar Integration)
**Total Tasks**: 18 atomic tasks (15-30 min each)
**Estimated Duration**: 9-12 hours total

## Integration Context

**Main Branch**: Contains redesigned navbar with dummy Sign In/Sign Up buttons at `physical-robotics-ai-book/src/theme/Navbar/index.tsx:30-43`.

**Auth Branch**: Contains complete auth implementation (SignupForm, SigninForm, AuthContext, AuthPrompt, Navbar Content).

**Integration Goal**: Merge auth components into main and connect them to the navbar buttons.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[US1/US2/US3]**: Maps to User Story 1/2/3 from spec.md
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/src/auth/`, `backend/tests/auth/`
- **Frontend**: `frontend/src/components/`, `frontend/src/services/`
- **Database**: Neon PostgreSQL via DATABASE_URL (already configured in `.env`)

---

## Phase 1: Backend - Install & Configure Better Auth with Neon DB (4 tasks)

**Purpose**: Set up Better Auth library and integrate with Neon PostgreSQL database

**Duration**: ~2 hours

- [X] T001 Install FastAPI-Users and configure with Neon PostgreSQL in `backend/src/auth/config.py`
  - **Duration**: 30 min
  - **Dependencies**: None
  - **Acceptance**: FastAPI-Users initialized, DATABASE_URL loaded from .env, connection pooling configured
  - **Command**: `cd backend && uv add "fastapi-users[sqlalchemy]" "passlib[argon2]" asyncpg sqlalchemy python-jose[cryptography] python-multipart`
  - **Output**: `backend/src/auth/__init__.py`, `backend/src/auth/config.py`
  - **Note**: Better Auth is JavaScript-only, using FastAPI-Users (industry standard for Python/FastAPI)

- [X] T002 [P] Define User and Session models with BackgroundProfile schema in `backend/src/auth/models.py`
  - **Duration**: 20 min
  - **Dependencies**: T001
  - **Acceptance**: User model with metadata JSONB field, BackgroundProfile Pydantic model with 3 enum fields
  - **Output**: `backend/src/auth/models.py`
  - **Validation**: Enum values match spec exactly ("0-2 years", "3-5 years", "6-10 years", "10+ years", etc.) ✅

- [X] T003 [P] Create Pydantic schemas for API requests/responses in `backend/src/auth/schemas.py`
  - **Duration**: 15 min
  - **Dependencies**: T002
  - **Acceptance**: SignupRequest, SigninRequest, AuthSuccessResponse, SessionResponse, ErrorResponse schemas ✅
  - **Output**: `backend/src/auth/schemas.py`
  - **Validation**: All schemas match contracts/auth-api.yaml exactly ✅

- [X] T004 Run database migration to create users table in Neon PostgreSQL
  - **Duration**: 15 min
  - **Dependencies**: T001, T002
  - **Acceptance**: Table created (users), indexes created (idx_users_email) ✅
  - **Command**: `cd backend && uv run python -m src.auth.migrate` ✅
  - **Validation**: Verified columns: id, email, hashed_password, is_active, is_superuser, is_verified, metadata, created_at, updated_at ✅

---

## Phase 2: Backend - Custom Background Questions + Personalization (4 tasks)

**Purpose**: Implement signup/signin endpoints with background collection and personalization mapping

**Duration**: ~2.5 hours

- [X] T005 [US1] Implement POST /auth/signup endpoint with background question collection in `backend/src/auth/routes.py`
  - **Duration**: 30 min
  - **Dependencies**: T003, T004
  - **Acceptance**: Endpoint creates user with background in metadata, returns session token, auto-signs in ✅
  - **Output**: `backend/src/auth/routes.py` (signup function) ✅
  - **Created**: UserManager with custom create method, signup endpoint with BackgroundProfile storage
  - **Validation**: Returns token, user_id, background, expires_at ✅

- [X] T006 [US2] Implement POST /auth/signin endpoint in `backend/src/auth/routes.py`
  - **Duration**: 20 min
  - **Dependencies**: T005
  - **Acceptance**: Endpoint authenticates user, creates session, returns token (7-day expiration) ✅
  - **Output**: `backend/src/auth/routes.py` (signin function) ✅
  - **Created**: signin endpoint with JWT token generation, 7-day expiration
  - **Validation**: Returns token, user_id, background, expires_at ✅

- [X] T007 [P] [US1] Implement personalization mapping logic in `backend/src/auth/personalization.py`
  - **Duration**: 30 min
  - **Dependencies**: T002
  - **Acceptance**: Function maps BackgroundProfile → expertise level (beginner/intermediate/advanced), generates prompt adjustments ✅
  - **Output**: `backend/src/auth/personalization.py` ✅
  - **Created**: map_background_to_expertise(), get_prompt_adjustments(), generate_system_prompt_prefix()
  - **Validation**: Tested with all 3 expertise levels - outputs correct prompt prefixes ✅

- [X] T008 [P] [US1] [US2] Implement session validation middleware in `backend/src/auth/middleware.py`
  - **Duration**: 25 min
  - **Dependencies**: T004
  - **Acceptance**: Middleware validates session token, fetches user background, attaches to request context ✅
  - **Output**: `backend/src/auth/middleware.py` ✅
  - **Created**: SessionValidationMiddleware, get_current_user_with_background(), error helpers
  - **Validation**: Invalid token → 401 response, expired session → 401 with "Session expired" error ✅

---

## Phase 3: Frontend - Navbar Integration & Auth Modals (4 tasks)

**Purpose**: Integrate existing auth components with redesigned Navbar from main branch

**Duration**: ~2.5 hours

**Note**: Auth components (SignupForm, SigninForm, AuthPrompt, AuthContext) **ALREADY EXIST** in authentication branch and will be merged into the integration branch. This phase focuses on connecting them to the Navbar.

- [X] T009 [US1] SignupForm component with email, password, and 3 background dropdown fields
  - **Status**: ✅ COMPLETED in auth branch
  - **Location**: `physical-robotics-ai-book/src/components/Auth/SignupForm.tsx`
  - **Integration**: Copy to integration branch, no changes needed

- [X] T010 [US2] SigninForm component with email and password fields
  - **Status**: ✅ COMPLETED in auth branch
  - **Location**: `physical-robotics-ai-book/src/components/Auth/SigninForm.tsx`
  - **Integration**: Copy to integration branch, no changes needed

- [X] T011 [US3] AuthPrompt component for guest users
  - **Status**: ✅ COMPLETED in auth branch
  - **Location**: `physical-robotics-ai-book/src/components/Auth/AuthPrompt.tsx`
  - **Integration**: Copy to integration branch, use in ChatWidget for guest restriction

- [X] T012 AuthContext and useAuth hook for global state management
  - **Status**: ✅ COMPLETED in auth branch
  - **Location**: `physical-robotics-ai-book/src/components/Auth/AuthContext.tsx`
  - **Integration**: Copy to integration branch, already wrapped in Layout/index.tsx

---

## Phase 4: Integration - Navbar + Chat Widget Auth (3 tasks)

**Purpose**: Connect Navbar buttons to auth modals, protect chat widget, implement personalization

**Duration**: ~2 hours

- [ ] T013 [US1][US2] **[INTEGRATION TASK]** Modify Navbar to trigger auth modals on button click
  - **Duration**: 30 min
  - **Dependencies**: T009-T012 (auth components exist)
  - **File**: `physical-robotics-ai-book/src/theme/Navbar/index.tsx`
  - **Changes**:
    1. Import `useState` for modal state management
    2. Import `SignupForm`, `SigninForm` from `@site/src/components/Auth`
    3. Add state: `const [showSignup, setShowSignup] = useState(false)` and `const [showSignin, setShowSignin] = useState(false)`
    4. Replace dummy `onClick={() => console.log('Sign In clicked')}` with `onClick={() => setShowSignin(true)}`
    5. Replace dummy `onClick={() => console.log('Sign Up clicked')}` with `onClick={() => setShowSignup(true)}`
    6. Add modal overlays after navbar closing tag:
       ```tsx
       {showSignup && (
         <div className={styles.modalOverlay} onClick={() => setShowSignup(false)}>
           <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
             <SignupForm
               onSubmit={async (email, password, background) => {
                 await signup(email, password, background);
                 setShowSignup(false);
               }}
               onSwitchToSignin={() => {
                 setShowSignup(false);
                 setShowSignin(true);
               }}
             />
           </div>
         </div>
       )}
       ```
       (same pattern for signin modal)
  - **Acceptance**: Click "Sign Up" in navbar → modal appears with signup form, click "Sign In" → signin modal appears
  - **Validation**:
    - Modal closes when clicking outside
    - Successful signup → modal closes, navbar shows user email (from Navbar Content)
    - Switch between signup/signin works

- [ ] T013b [US3] ChatWidget guest restriction (already implemented in auth branch)
  - **Status**: ✅ COMPLETED in auth branch
  - **Location**: ChatWidget already checks `isAuthenticated` and shows AuthPrompt for guests
  - **Integration**: Copy ChatWidget from auth branch, ensure AuthPrompt imports work

- [ ] T014 [US1] [US2] Modify backend /chat/message endpoint to fetch user background and adjust RAG agent prompt in `backend/src/chat/routes.py`
  - **Duration**: 30 min
  - **Dependencies**: T007, T008
  - **Acceptance**: Endpoint requires auth, fetches background, calls personalization.py, adjusts docs_agent system prompt before generating response
  - **Output**: `backend/src/chat/routes.py` (modified to add session middleware and personalization)
  - **Test**: Beginner user asks "What is ROS 2?" → response is simple language, Advanced user asks same → response is technical
  - **Validation**: Response includes expertise_level field, personalized=true

- [ ] T015 Implement session expiration handling with message preservation in `frontend/src/services/api.ts` (axios interceptor)
  - **Duration**: 25 min
  - **Dependencies**: T012, T014
  - **Acceptance**: Session expired (401) → preserve typed message in localStorage, show re-auth modal, restore message after successful signin
  - **Output**: `frontend/src/services/api.ts` (modified), `frontend/src/components/ChatWidget/SessionExpiredModal.tsx` (new)
  - **Validation**: Type message, expire session (manually delete from DB), send → modal shown, re-auth → message preserved

---

## Phase 5: Testing & Deployment Validation (3 tasks)

**Purpose**: End-to-end testing, deployment, and acceptance validation

**Duration**: ~2 hours

- [ ] T016 [P] Write backend integration tests for signup, signin, and personalization in `backend/tests/auth/test_integration.py`
  - **Duration**: 30 min
  - **Dependencies**: T005, T006, T007
  - **Acceptance**: Tests cover: signup with background, signin, duplicate email error, personalization mapping (beginner vs advanced)
  - **Output**: `backend/tests/auth/test_integration.py`
  - **Command**: `cd backend && pytest tests/auth/test_integration.py -v`
  - **Validation**: All tests pass (4-5 test cases)

- [ ] T017 [P] Write frontend E2E tests for authentication flows in `frontend/src/components/Auth/__tests__/auth.test.tsx`
  - **Duration**: 30 min
  - **Dependencies**: T009, T010, T011, T013
  - **Acceptance**: Tests cover: guest sees AuthPrompt, signup flow with background, signin flow, authenticated chat access
  - **Output**: `frontend/src/components/Auth/__tests__/auth.test.tsx`
  - **Command**: `cd frontend && npm test -- auth.test.tsx`
  - **Validation**: All tests pass (3-4 test cases)

- [ ] T018 Deploy to Render and validate end-to-end acceptance criteria from spec.md
  - **Duration**: 30 min
  - **Dependencies**: T001-T017 (all prior tasks)
  - **Acceptance**:
    1. ✅ Guest → chat widget shows "Please sign in"
    2. ✅ New user → signup with background → logged in → chat response personalized (simple for beginner: "0-2 years, None", technical for advanced: "10+ years, Advanced")
    3. ✅ Existing user → signin → chat response matches saved background
    4. ✅ Deployed live on Render, DATABASE_URL connected to Neon PostgreSQL
  - **Commands**:
    ```bash
    # Deploy backend to Render
    git push origin authentication
    # Render auto-deploys from branch

    # Build and deploy frontend to GitHub Pages
    cd frontend && npm run build && npm run deploy
    ```
  - **Validation Steps**:
    1. Visit deployed site as guest → see "Please sign in" on chat widget ✅
    2. Signup as beginner (0-2 years, None, None) → ask "What is ROS 2?" → response is simple ✅
    3. Signup as advanced (10+ years, Advanced, Physical robots/sensors) in new browser → ask "What is ROS 2?" → response is technical ✅
    4. Signout and signin with beginner account → response still simple ✅
    5. Check Render logs: `render logs -s rag-chatbot-backend` → no errors ✅

---

## Dependencies & Execution Order

### Phase Dependencies

1. **Phase 1** (Backend Setup): No dependencies - start immediately
2. **Phase 2** (Backend Implementation): Depends on Phase 1 complete
3. **Phase 3** (Frontend UI): Independent from Phases 1-2, can start in parallel
4. **Phase 4** (Integration): Depends on Phases 2 AND 3 complete
5. **Phase 5** (Testing): Depends on Phases 1-4 complete

### User Story Mapping

- **User Story 1 (New User Registration)**: T005, T007, T009, T012, T014
- **User Story 2 (Returning User Authentication)**: T006, T008, T010, T012, T014
- **User Story 3 (Guest User Restriction)**: T011, T013

### Parallel Opportunities

**Within Phase 1**:
- T002 and T003 can run in parallel after T001 completes

**Within Phase 2**:
- T007 and T008 can run in parallel (different files)

**Within Phase 3**:
- T009, T010, T011 can all run in parallel (different components, no dependencies)

**Within Phase 5**:
- T016 and T017 can run in parallel (backend vs frontend tests)

**Across Phases**:
- Phase 3 (Frontend) can start immediately in parallel with Phases 1-2 (Backend)

---

## Parallel Example: Maximum Parallelization

```bash
# Start Phase 1 (Backend Setup):
Task T001: Install Better Auth (30 min)
  → Then parallel:
    Task T002: Define models (20 min)
    Task T003: Create schemas (15 min)

# Simultaneously start Phase 3 (Frontend):
Task T009: SignupForm (30 min)
Task T010: SigninForm (20 min)
Task T011: AuthPrompt (15 min)

# After T002/T003 complete:
Task T004: Run migrations (15 min)

# After T004 complete:
Task T005: Signup endpoint (30 min)
Task T006: Signin endpoint (20 min)
  → Then parallel:
    Task T007: Personalization logic (30 min)
    Task T008: Session middleware (25 min)

# After all frontend tasks (T009-T011) complete:
Task T012: AuthContext (25 min)

# After Phase 2 (T005-T008) AND T012 complete:
Task T013: Protect ChatWidget (20 min)
Task T014: Personalize chat endpoint (30 min)

# After T012 and T014 complete:
Task T015: Session expiration handling (25 min)

# After everything complete, parallel tests:
Task T016: Backend integration tests (30 min)
Task T017: Frontend E2E tests (30 min)

# Finally:
Task T018: Deploy and validate (30 min)
```

**Total Duration (with parallelization)**: ~5-6 hours
**Total Duration (sequential)**: ~9-12 hours

---

## Implementation Strategy

### MVP First (User Stories 1-3 Complete)

1. Complete Phase 1: Backend Setup (~2 hours)
2. Complete Phase 2: Backend Implementation (~2.5 hours)
3. Complete Phase 3: Frontend UI (~2.5 hours) - can run parallel with Phases 1-2
4. Complete Phase 4: Integration (~2 hours)
5. Complete Phase 5: Testing & Deployment (~2 hours)
6. **STOP and VALIDATE**: All 3 user stories functional

### Incremental Validation Checkpoints

**After Phase 1**:
- ✅ Better Auth configured, tables created in Neon PostgreSQL

**After Phase 2**:
- ✅ Signup endpoint works: `curl -X POST .../auth/signup` → user created with background
- ✅ Signin endpoint works: `curl -X POST .../auth/signin` → session token returned
- ✅ Personalization mapping works: beginner profile → "simple language" prompt

**After Phase 3**:
- ✅ Signup form renders with 5 fields (all required)
- ✅ Signin form renders with 2 fields
- ✅ AuthPrompt shows for guest users

**After Phase 4**:
- ✅ Guest user sees "Please sign in" on chat widget
- ✅ Authenticated user can send chat messages
- ✅ Beginner user gets simple responses, advanced user gets technical responses

**After Phase 5**:
- ✅ All tests pass (backend + frontend)
- ✅ Deployed to Render and working end-to-end

---

## Task Checklist Format Validation

All tasks follow the required format:

```
✅ Checkbox: - [ ]
✅ Task ID: T001, T002, ... T018
✅ [P] marker: Present for parallelizable tasks
✅ [Story] label: Present for user story tasks (US1, US2, US3)
✅ Description: Clear action with exact file path
✅ Duration: 15-30 min (atomic tasks)
✅ Dependencies: Explicitly listed
✅ Acceptance: Clear, testable criteria
✅ Output: Exact file paths
✅ Command/Test: Validation command provided
```

**Total Tasks**: 18 (matches requirement: 16-18 max)
**Task Duration**: 15-30 min each (atomic)
**Phases**: 5 (as requested)

---

## Notes

- Use Context7 MCP server when needed: `@context7 better-auth python sdk setup`
- DATABASE_URL already configured in `backend/.env` - use existing
- Use `uv` for Python package management (already set up)
- Better Auth auto-migration creates tables on first run
- Session tokens stored in httpOnly cookies (frontend automatic)
- Personalization mapping: beginner (0-2 years, ROS2=None) → simple, advanced (10+ years, ROS2=Advanced) → technical
- All background fields required (spec FR-002a)
- Session duration: 7 days (spec FR-015)
- Deploy backend to Render, frontend to GitHub Pages

---

## Acceptance Criteria Summary

**From spec.md - Final Validation**:

1. ✅ Guest → chat widget shows "Please sign in to use the personalized chat"
2. ✅ New user → signup with background questions (email, password, prog_exp, ros2_fam, hardware) → automatically logged in → chat responses personalized based on background
3. ✅ Beginner (0-2 years, ROS2=None) asks "What is ROS 2?" → simple language, step-by-step, no jargon
4. ✅ Advanced (10+ years, ROS2=Advanced) asks "What is ROS 2?" → technical terms, code examples, in-depth
5. ✅ Existing user → signin with email/password → chat responses match their saved background profile
6. ✅ Session persists across browser refresh (7-day duration)
7. ✅ Deployed live on Render (backend) and working with Neon PostgreSQL (DATABASE_URL)

**All 18 tasks map to these acceptance criteria and deliver complete feature functionality.**
