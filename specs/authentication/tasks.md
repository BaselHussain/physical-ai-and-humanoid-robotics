# Tasks: RAG Chatbot Authentication with Better Auth Microservice

**Input**: Design documents from `/specs/authentication/`
**Prerequisites**: spec.md (complete), plan.md (complete), research.md (to be created), data-model.md (to be created), contracts/ (to be created)

**Branch**: `authentication` → **Integration Branch**: TBD (will be created from main)
**Updated**: 2025-12-25 (Better Auth Microservice Architecture)
**Total Tasks**: 28 atomic tasks (15-45 min each)
**Estimated Duration**: 16-22 hours total (spread across 3 services)

## Integration Context

**Main Branch**: Contains redesigned navbar with dummy Sign In/Sign Up buttons at `physical-robotics-ai-book/src/theme/Navbar/index.tsx:30-43`.

**Auth Branch**: Contains auth UI components (SignupForm, SigninForm, AuthContext, AuthPrompt, Navbar Content) - needs update to Better Auth client SDK.

**Integration Goal**:
1. Set up Better Auth as separate Node.js microservice
2. Update FastAPI to validate JWT tokens via JWKS
3. Update frontend to use Better Auth client SDK
4. Connect auth modals to navbar buttons

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[US1/US2/US3]**: Maps to User Story 1/2/3 from spec.md
- Include exact file paths in descriptions

## Path Conventions

- **Better Auth Service**: `better-auth-service/src/`, `better-auth-service/tests/`
- **FastAPI Backend**: `backend/src/auth/`, `backend/tests/auth/`
- **Frontend**: `physical-robotics-ai-book/src/components/`, `physical-robotics-ai-book/src/theme/`
- **Database**: Neon PostgreSQL via DATABASE_URL (shared between Better Auth and FastAPI)

---

## Phase 1: Better Auth Service - Setup & Configuration (6 tasks)

**Purpose**: Create Better Auth Node.js microservice with Kysely + Neon PostgreSQL

**Duration**: ~3.5 hours

- [ ] T001 Initialize Better Auth Service project structure
  - **Duration**: 20 min
  - **Dependencies**: None
  - **Acceptance**: Directory created with package.json, tsconfig.json, .env template
  - **Command**: `mkdir better-auth-service && cd better-auth-service && npm init -y && npx tsc --init`
  - **Output**: `better-auth-service/package.json`, `better-auth-service/tsconfig.json`, `better-auth-service/.env.example`
  - **Validation**: package.json exists, TypeScript configured for Node.js 18+

- [ ] T002 Install Better Auth and dependencies
  - **Duration**: 15 min
  - **Dependencies**: T001
  - **Acceptance**: Better Auth, Kysely, Express, pg driver installed
  - **Command**: `cd better-auth-service && npm install better-auth kysely pg express dotenv cors`
  - **Output**: Updated `package.json`, `node_modules/`
  - **Validation**: Dependencies listed in package.json

- [ ] T003 [P] Configure Kysely PostgreSQL adapter in `better-auth-service/src/database.ts`
  - **Duration**: 30 min
  - **Dependencies**: T002
  - **Acceptance**: Kysely connected to Neon PostgreSQL via DATABASE_URL from .env
  - **Output**: `better-auth-service/src/database.ts`
  - **Validation**: Connection test succeeds, connection pool configured
  - **Test**: Run `npm run test:db` to verify connection

- [ ] T004 [P] Implement Better Auth configuration in `better-auth-service/src/auth.ts`
  - **Duration**: 45 min
  - **Dependencies**: T003
  - **Acceptance**: Better Auth initialized with credential provider, Kysely adapter, metadata schema extension for user background
  - **Output**: `better-auth-service/src/auth.ts`
  - **Validation**: Better Auth instance created, metadata fields defined (programming_experience, ros2_familiarity, hardware_access)
  - **Test**: Signup test creates user with metadata

- [ ] T005 Implement custom claims plugin in `better-auth-service/src/plugins/custom-claims.ts`
  - **Duration**: 45 min
  - **Dependencies**: T004
  - **Acceptance**: Plugin adds user background from metadata to JWT custom claims namespace `https://yourdomain.com/claims`
  - **Output**: `better-auth-service/src/plugins/custom-claims.ts`
  - **Validation**: JWT tokens include custom claims with background fields (programming_experience, ros2_familiarity, hardware_access)
  - **Test**: Decode JWT and verify custom claims present

- [ ] T006 Create Express server in `better-auth-service/src/server.ts`
  - **Duration**: 45 min
  - **Dependencies**: T004, T005
  - **Acceptance**: Express server exposes Better Auth routes at /api/auth/*, CORS configured for production domain + subdomains (*.yourdomain.com), localhost allowed in dev
  - **Command**: `npm run dev` starts server on port 3000
  - **Output**: `better-auth-service/src/server.ts`
  - **Validation**: JWKS endpoint accessible at `/.well-known/jwks.json`, signup endpoint at `/api/auth/sign-up/email`, signin at `/api/auth/sign-in/email`
  - **Test**: `curl http://localhost:3000/.well-known/jwks.json` returns public keys

---

## Phase 2: FastAPI Service - JWT Validation (6 tasks)

**Purpose**: Implement FastAPI JWT validation layer to consume Better Auth tokens

**Duration**: ~3 hours

- [ ] T007 Install PyJWT and httpx dependencies in FastAPI service
  - **Duration**: 10 min
  - **Dependencies**: None (FastAPI already exists)
  - **Acceptance**: PyJWT[crypto], httpx installed for RS256 validation and JWKS fetching
  - **Command**: `cd backend && uv add "PyJWT[crypto]" httpx`
  - **Output**: Updated `backend/pyproject.toml` or `requirements.txt`
  - **Validation**: Dependencies installed successfully

- [ ] T008 [P] Implement JWKS fetching and caching in `backend/src/auth/jwks_cache.py`
  - **Duration**: 35 min
  - **Dependencies**: T007
  - **Acceptance**: Fetch JWKS from Better Auth `/.well-known/jwks.json`, cache for 1 hour, refresh if key ID not found
  - **Output**: `backend/src/auth/jwks_cache.py`
  - **Validation**: JWKS fetched and cached, cache TTL working
  - **Test**: Mock JWKS endpoint, verify caching behavior

- [ ] T009 [P] Implement JWT validation middleware in `backend/src/auth/middleware.py`
  - **Duration**: 45 min
  - **Dependencies**: T008
  - **Acceptance**: Middleware validates JWT signature using JWKS public keys (RS256), checks expiration, rejects invalid tokens with 401
  - **Output**: `backend/src/auth/middleware.py`
  - **Validation**: Valid JWT passes, expired/invalid JWT returns 401
  - **Test**: Unit test with mock JWTs (valid, expired, wrong signature)

- [ ] T010 [P] Implement JWT claims extractor in `backend/src/auth/claims_extractor.py`
  - **Duration**: 30 min
  - **Dependencies**: T009
  - **Acceptance**: Extract custom claims from JWT (programming_experience, ros2_familiarity, hardware_access), return as BackgroundProfile object
  - **Output**: `backend/src/auth/claims_extractor.py`
  - **Validation**: Custom claims extracted correctly
  - **Test**: Decode JWT with custom claims, verify extraction

- [ ] T011 Configure CORS for FastAPI in `backend/src/main.py`
  - **Duration**: 20 min
  - **Dependencies**: None
  - **Acceptance**: CORS allows requests from production domain + subdomains (*.yourdomain.com), localhost in dev
  - **Output**: Modified `backend/src/main.py`
  - **Validation**: CORS headers present in responses
  - **Test**: Frontend can call FastAPI from different origin

- [ ] T012 Add JWT middleware to FastAPI app in `backend/src/main.py`
  - **Duration**: 20 min
  - **Dependencies**: T009
  - **Acceptance**: JWT middleware applied to protected routes (/chat/message), extracts claims and makes available via dependency injection
  - **Output**: Modified `backend/src/main.py`
  - **Validation**: Protected routes require valid JWT
  - **Test**: Call `/chat/message` without JWT → 401, with valid JWT → success

---

## Phase 3: Frontend - Better Auth Client SDK Integration (6 tasks)

**Purpose**: Update existing auth components to use Better Auth client SDK

**Duration**: ~3 hours

- [ ] T013 Install Better Auth React client SDK
  - **Duration**: 10 min
  - **Dependencies**: None
  - **Acceptance**: @better-auth/react installed in frontend
  - **Command**: `cd physical-robotics-ai-book && npm install @better-auth/react`
  - **Output**: Updated `package.json`
  - **Validation**: Package installed successfully

- [ ] T014 [US1][US2] Update AuthContext to use Better Auth client SDK in `physical-robotics-ai-book/src/components/Auth/AuthContext.tsx`
  - **Duration**: 45 min
  - **Dependencies**: T013
  - **Acceptance**: AuthContext uses Better Auth hooks, calls Better Auth endpoints (auth.yourdomain.com/api/auth/*), stores access + refresh tokens
  - **Output**: Modified `AuthContext.tsx`
  - **Validation**: signup(), signin(), signout() call Better Auth, tokens stored in localStorage/cookies
  - **Test**: Signup creates user in Better Auth database

- [ ] T015 [US1] Update SignupForm to call Better Auth with metadata in `physical-robotics-ai-book/src/components/Auth/SignupForm.tsx`
  - **Duration**: 30 min
  - **Dependencies**: T014
  - **Acceptance**: SignupForm sends user background (programming_experience, ros2_familiarity, hardware_access) in metadata field to Better Auth `/api/auth/sign-up/email`
  - **Output**: Modified `SignupForm.tsx`
  - **Validation**: User created with metadata in Neon database
  - **Test**: Submit signup form, verify metadata in database

- [ ] T016 [US2] Update SigninForm to call Better Auth in `physical-robotics-ai-book/src/components/Auth/SigninForm.tsx`
  - **Duration**: 20 min
  - **Dependencies**: T014
  - **Acceptance**: SigninForm calls Better Auth `/api/auth/sign-in/email`, receives access + refresh tokens
  - **Output**: Modified `SigninForm.tsx`
  - **Validation**: Signin succeeds, tokens stored
  - **Test**: Signin with valid credentials succeeds

- [ ] T017 Implement automatic token refresh interceptor in `physical-robotics-ai-book/src/utils/api.ts`
  - **Duration**: 40 min
  - **Dependencies**: T014
  - **Acceptance**: Axios/Fetch interceptor checks access token expiration (15 min), automatically refreshes using refresh token before expiration, retries failed request
  - **Output**: `physical-robotics-ai-book/src/utils/api.ts`
  - **Validation**: Access token renewed automatically
  - **Test**: Wait 15+ minutes, verify token refreshed before next request

- [ ] T018 Implement Better Auth service unavailability handling in `physical-robotics-ai-book/src/components/Auth/ServiceUnavailable.tsx`
  - **Duration**: 35 min
  - **Dependencies**: T014
  - **Acceptance**: Display error banner "Authentication service temporarily unavailable. Please try again." when Better Auth unreachable, auto-retry every 30 seconds
  - **Output**: `ServiceUnavailable.tsx` component, integrated into AuthContext
  - **Validation**: Stop Better Auth service, banner appears, auto-retries
  - **Test**: Mock network failure, verify banner + retry logic

---

## Phase 4: Integration - Navbar + Personalization (5 tasks)

**Purpose**: Connect navbar buttons to auth modals, implement chat personalization

**Duration**: ~2.5 hours

- [ ] T019 [US1][US2] Modify Navbar to trigger auth modals in `physical-robotics-ai-book/src/theme/Navbar/index.tsx`
  - **Duration**: 35 min
  - **Dependencies**: T013-T016
  - **Acceptance**: Click "Sign Up" in navbar → SignupForm modal appears, Click "Sign In" → SigninForm modal appears
  - **Output**: Modified `Navbar/index.tsx`
  - **Validation**: Modals open/close correctly, successful auth closes modal and shows user email in navbar
  - **Test**: Click buttons, verify modals, submit forms, verify UI updates

- [ ] T020 [US1][US2] Verify Navbar Content shows user email + Sign Out when authenticated (already in auth branch)
  - **Duration**: 10 min
  - **Dependencies**: T019
  - **Acceptance**: After successful signin, navbar shows user email and Sign Out button (from `Navbar/Content/index.tsx`)
  - **Output**: No changes needed (already implemented in auth branch)
  - **Validation**: User email displays, Sign Out button works
  - **Test**: Sign in, verify navbar updates, click Sign Out, verify signout

- [ ] T021 [P] [US1] Implement personalization mapping logic in `backend/src/chat/personalization.py`
  - **Duration**: 35 min
  - **Dependencies**: T010 (JWT claims extraction)
  - **Acceptance**: Function maps background (from JWT claims) → expertise level (beginner/intermediate/advanced), generates prompt adjustments
  - **Output**: `backend/src/chat/personalization.py`
  - **Validation**: Beginner → simple language prompts, Advanced → technical prompts
  - **Test**: Test with different backgrounds, verify prompt adjustments

- [ ] T022 [US1][US2] Modify /chat/message endpoint to extract background from JWT and personalize in `backend/src/chat/routes.py`
  - **Duration**: 40 min
  - **Dependencies**: T012, T021
  - **Acceptance**: Endpoint extracts user background from JWT custom claims (no database query), adjusts docs_agent system prompt, generates personalized response
  - **Output**: Modified `backend/src/chat/routes.py`
  - **Validation**: Beginner user gets simple response, Advanced user gets technical response for same question
  - **Test**: Ask "What is ROS 2?" with different user backgrounds, compare responses

- [ ] T023 [US3] Verify ChatWidget guest restriction (already implemented in auth branch)
  - **Duration**: 10 min
  - **Dependencies**: T014
  - **Acceptance**: Guest users see AuthPrompt "Please sign in to use the chat" instead of chat input
  - **Output**: No changes needed (ChatWidget already checks `isAuthenticated`)
  - **Validation**: Guest sees auth prompt, authenticated user sees chat
  - **Test**: Access chat while signed out, verify prompt appears

---

## Phase 5: Testing & Deployment (5 tasks)

**Purpose**: End-to-end testing, deployment validation, security verification

**Duration**: ~3.5 hours

- [X] T024 [P] Write Better Auth service tests in `better-auth-service/tests/`
  - **Duration**: 40 min
  - **Dependencies**: T001-T006
  - **Acceptance**: Tests cover signup, signin, token generation, custom claims, JWKS endpoint
  - **Output**: `better-auth-service/tests/auth.test.ts`
  - **Command**: `npm test`
  - **Validation**: All tests pass

- [X] T025 [P] Write FastAPI JWT validation tests in `backend/tests/auth/`
  - **Duration**: 40 min
  - **Dependencies**: T007-T012
  - **Acceptance**: Tests cover JWT validation, claims extraction, expired token handling, invalid signature rejection
  - **Output**: `backend/tests/auth/test_jwt_validation.py`, `test_claims_extraction.py`, `test_jwks_cache.py`
  - **Command**: `pytest backend/tests/auth/`
  - **Validation**: All tests pass

- [X] T026 [P] Write frontend auth flow tests in `physical-robotics-ai-book/src/components/Auth/__tests__/`
  - **Duration**: 40 min
  - **Dependencies**: T013-T018
  - **Acceptance**: Tests cover signup flow, signin flow, token refresh, service unavailability handling
  - **Output**: `Auth/__tests__/AuthContext.test.tsx`, `SignupForm.test.tsx`, `auth.integration.test.tsx`
  - **Command**: `npm test`
  - **Validation**: All tests pass

- [ ] T027 Deploy Better Auth to Render and FastAPI to Render (two separate services)
  - **Duration**: 60 min
  - **Dependencies**: All previous tasks
  - **Acceptance**: Better Auth deployed at auth.yourdomain.com, FastAPI at api.yourdomain.com, both use shared Neon PostgreSQL, CORS configured correctly
  - **Command**: Configure Render build/start commands, set environment variables
  - **Validation**: Services accessible, JWKS endpoint works, FastAPI can validate Better Auth tokens
  - **Test**: Signup from production frontend, verify JWT validation in FastAPI

- [ ] T028 End-to-end production validation
  - **Duration**: 40 min
  - **Dependencies**: T027
  - **Acceptance**: Complete flow works in production: signup → signin → chat with personalization → signout, token refresh works automatically, service unavailability handling tested
  - **Validation**: All user stories pass in production environment
  - **Test Cases**:
    - US1: New user signup with background → personalized first response
    - US2: Returning user signin → personalization persists
    - US3: Guest user sees auth prompt
    - Token refresh: Wait 15+ min, verify auto-renewal
    - Service downtime: Stop Better Auth, verify error banner + retry

---

## Task Summary

**Total Tasks**: 28
**Estimated Duration**: 16-22 hours

**Breakdown by Phase**:
- Phase 1: Better Auth Service - 6 tasks (3.5 hours)
- Phase 2: FastAPI JWT Validation - 6 tasks (3 hours)
- Phase 3: Frontend Better Auth Integration - 6 tasks (3 hours)
- Phase 4: Integration & Personalization - 5 tasks (2.5 hours)
- Phase 5: Testing & Deployment - 5 tasks (3.5 hours)

**Next Steps**:
1. Create integration branch from main
2. Begin with Phase 1: Better Auth Service setup
3. Test each phase before moving to next
4. Deploy to staging for validation
5. Deploy to production

**Dependencies**:
- Neon PostgreSQL database (shared)
- Render accounts for deployment
- Domain configuration for auth.yourdomain.com and api.yourdomain.com

---

## Acceptance Criteria Summary

All tasks must meet these criteria:
- [ ] Better Auth service running and accessible
- [ ] FastAPI validates JWT tokens via JWKS
- [ ] Frontend uses Better Auth client SDK
- [ ] Navbar buttons trigger auth modals
- [ ] Chat personalization based on JWT custom claims
- [ ] Access token (15 min) auto-renewed via refresh token (7 days)
- [ ] Better Auth service unavailability handled gracefully
- [ ] All three user stories pass (US1, US2, US3)
- [ ] CORS configured correctly for microservices
- [ ] All tests passing (unit, integration, E2E)
