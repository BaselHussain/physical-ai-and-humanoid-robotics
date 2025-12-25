# Feature Specification: RAG Chatbot Authentication with Better Auth

**Feature Branch**: `authentication`
**Created**: 2025-12-15
**Updated**: 2025-12-25 (Better Auth Microservice Architecture)
**Status**: In Progress - Architecture Updated
**Input**: User description: "RAG Chatbot Authentication with Better Auth - Implement Signup and Signin using Better Auth. During signup, ask custom questions about user's software and hardware background (years of programming experience, ROS 2/robotics familiarity, hardware access). Store this in Better Auth user profile/metadata. Use this background to personalize RAG chatbot responses by dynamically adjusting the docs_agent system prompt. Must use Neon PostgreSQL (DATABASE_URL from .env), replace in-memory sessions with JWT-based authentication, and be deployable on Render free tier."

## Integration Context (2025-12-25)

**Main Branch Changes**: The main branch now includes a redesigned site with a new header/navbar component that contains Sign In and Sign Up buttons. These buttons are currently dummy placeholders.

**Integration Scope**:
- **Auth Entry Point**: Navbar Sign In/Sign Up buttons (`physical-robotics-ai-book/src/theme/Navbar/index.tsx:30-43`)
- **Implementation**: Replace dummy onClick handlers with auth modal triggers
- **After Auth**: Navbar shows user email and Sign Out button (already implemented in authentication branch at `Navbar/Content/index.tsx`)
- **Auth Forms**: Display as modals/overlays when navbar buttons are clicked
- **Layout Integration**: AuthProvider already wraps app in `Layout/index.tsx` on authentication branch

## Architectural Decision: Better Auth Microservice Architecture

**Requirement**: Better Auth
**Implementation**: Better Auth as separate Node.js microservice + FastAPI for RAG chatbot

**Rationale**: After analyzing Better Auth documentation via Better Auth MCP server, confirmed that Better Auth can be deployed as a **standalone authentication microservice** that FastAPI consumes via JWT validation. This provides:

1. **Proper separation of concerns**: Auth service handles authentication, FastAPI handles RAG chatbot
2. **Meets original requirement**: Uses Better Auth as specified
3. **Modern microservices architecture**: Each service has single responsibility
4. **Better Auth advantages**:
   - Native Neon PostgreSQL support (Kysely adapter)
   - Built-in JWT with custom claims for user background
   - Exposes REST API endpoints (`/api/auth/*`)
   - Production-ready session management
   - Secure password hashing (Argon2)
5. **FastAPI integration**: Validates JWT tokens using Better Auth's JWKS endpoint, extracts user profile from JWT claims

**Architecture**: Better Auth (Node.js) ↔ Neon PostgreSQL | Frontend ↔ Better Auth (signup/signin) | Frontend ↔ FastAPI (chat, sends JWT) | FastAPI validates JWT → personalizes responses

## Clarifications

### Session 2025-12-15

- Q: Should all background questions (programming experience, ROS 2 familiarity, hardware access) be mandatory during signup, or can users skip some? → A: All fields required - users must answer all background questions to complete signup
- Q: How should the system respond when a user tries to sign up with an email that's already registered? → A: Show error "Email already registered. Try signing in instead." with a link to the sign-in form
- Q: How should the system handle session expiration when a user is actively using the chat widget? → A: Allow current message to be typed; show re-authentication prompt when user tries to send; preserve typed message after re-login
- Q: What should the default session duration be for authenticated users? → A: 7 days - users must re-authenticate weekly
- Q: What specific input format should be used for collecting programming experience? → A: Dropdown with predefined ranges - "0-2 years", "3-5 years", "6-10 years", "10+ years"

### Session 2025-12-17

- Q: What are the password strength requirements beyond minimum 8 characters? → A: Better Auth library handles all password validation using default secure settings with Argon2 hashing
- Q: What input format should be used for hardware access field - single-select dropdown or multi-select? → A: Single-select dropdown with mutually exclusive options: "None", "Simulation only", "Physical robots/sensors"
- Q: How should the system handle invalid email formats? → A: Client-side validation shows instant error "Invalid email format" before submission + Better Auth server-side validation rejects with same error message if client validation bypassed
- Q: How should the system handle network failures during authentication? → A: Show user-friendly error message "Connection failed. Please check your internet and try again." with retry button; no automatic retry
- Q: How should the system handle very long or malformed input in background questions? → A: Enforce dropdown selection only (no free text input) - users cannot enter malformed data since all values are pre-defined options from dropdowns

### Session 2025-12-25 (Architecture Update)

- Q: Should we use FastAPI-Users or Better Auth as originally required? → A: Use Better Auth as a separate Node.js microservice; FastAPI validates JWT tokens
- Q: How will FastAPI validate Better Auth tokens? → A: FastAPI uses PyJWT to validate JWT tokens against Better Auth's JWKS endpoint (RS256 asymmetric signing)
- Q: Where will custom user background fields be stored? → A: In Better Auth's Neon PostgreSQL database, either as custom columns or JSONB metadata field
- Q: How will FastAPI access user background for personalization? → A: Custom fields included as JWT custom claims, or FastAPI fetches from Better Auth API using user ID (sub)
- Q: How will Better Auth and FastAPI services be deployed? → A: Better Auth and FastAPI as separate Render services with cross-origin communication (auth.yourdomain.com and api.yourdomain.com)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Registration with Personalized Profile (Priority: P1)

A new user visits the documentation site, wants to use the RAG chatbot, creates an account by providing their background information, and immediately experiences personalized chatbot responses tailored to their expertise level.

**Why this priority**: This is the core value proposition - personalized learning experience from the first interaction. Without this, the feature provides no differentiation from anonymous chat.

**Independent Test**: Can be fully tested by registering a new account with different background profiles (beginner vs. expert) and observing that the chatbot's first response adjusts its language and depth accordingly. Delivers immediate personalization value.

**Acceptance Scenarios**:

1. **Given** a guest user on the documentation site, **When** they click "Sign Up" in the navbar header, **Then** they see a registration modal with email, password, and background questions
2. **Given** a user on the signup form, **When** they fill in all required fields (email, password, years of programming experience, ROS 2 familiarity level, hardware access), **Then** their account is created and they are automatically signed in
3. **Given** a newly registered beginner user (0-2 years experience, no ROS 2 knowledge), **When** they ask "What is ROS 2?", **Then** the chatbot response uses simple language, step-by-step explanations, and avoids jargon
4. **Given** a newly registered expert user (5+ years experience, advanced ROS 2 knowledge), **When** they ask "What is ROS 2?", **Then** the chatbot response uses technical terminology, focuses on advanced concepts, and provides in-depth details

---

### User Story 2 - Returning User Authentication (Priority: P2)

A registered user returns to the documentation site, signs in with their existing credentials, and the RAG chatbot immediately provides personalized responses based on their saved background profile without needing to re-enter any information.

**Why this priority**: Essential for user retention and consistent personalized experience. Users should not lose their personalization settings across sessions.

**Independent Test**: Can be fully tested by creating an account, logging out, logging back in, and verifying that chatbot personalization persists. Delivers continuity of personalized experience.

**Acceptance Scenarios**:

1. **Given** an existing registered user, **When** they click "Sign In" in the navbar header, **Then** they see a login modal with email and password fields
2. **Given** a user on the signin modal, **When** they enter correct credentials and submit, **Then** they are authenticated, the modal closes, and the navbar displays their email with a Sign Out button
3. **Given** an authenticated user with a saved background profile, **When** they ask any question to the RAG chatbot, **Then** the response is personalized based on their stored expertise level and hardware context
4. **Given** an authenticated user, **When** they close and reopen the browser (session persistence), **Then** they remain signed in (navbar shows email + Sign Out) and chatbot personalization continues

---

### User Story 3 - Guest User Restriction (Priority: P3)

A guest user (not authenticated) visits the documentation site and attempts to use the RAG chatbot. The system prompts them to sign in or sign up to access the personalized chat feature.

**Why this priority**: Necessary for access control and to encourage registration, but lower priority than the core personalization flows. Can function as a soft gate to drive user registration.

**Independent Test**: Can be fully tested by accessing the chat widget while logged out and verifying that it displays a sign-in prompt instead of allowing chat. Delivers clear call-to-action for registration.

**Acceptance Scenarios**:

1. **Given** a guest user (not signed in), **When** they try to open the chat widget, **Then** they see a message "Please sign in to use the personalized chat" with Sign In and Sign Up buttons (which trigger the navbar auth modals)
2. **Given** a guest user viewing the sign-in prompt in chat widget, **When** they click "Sign Up", **Then** the signup modal opens (User Story 1)
3. **Given** a guest user viewing the sign-in prompt in chat widget, **When** they click "Sign In", **Then** the signin modal opens (User Story 2)
4. **Given** a guest user on the site, **When** they click "Sign In" or "Sign Up" in the navbar, **Then** the corresponding auth modal opens

---

### Edge Cases

- When a user enters an already-registered email during signup, the system displays error message "Email already registered. Try signing in instead." with a clickable link to the sign-in form
- When a user enters an invalid email format, client-side validation shows instant error "Invalid email format"; server-side validation also rejects if client validation is bypassed
- Password reset/forgot password is out of scope for this phase
- All background questions are required; users cannot submit signup form without completing all fields (programming experience, ROS 2 familiarity, hardware access)
- When the Better Auth session expires during an active chat, the system allows the user to continue typing their current message; when the user attempts to send, a re-authentication prompt appears; after successful re-login, the typed message is preserved and can be sent
- When network failures occur during authentication (signup/signin), the system shows user-friendly error message "Connection failed. Please check your internet and try again." with a retry button (no automatic retry)
- User profile editing after registration is out of scope for this phase
- Background questions use dropdown selection only (no free text input), preventing malformed or overly long data entry since all values are pre-defined options

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide a signup form accessible from the chat widget that collects email, password, and custom background questions
- **FR-002**: System MUST collect the following background information during signup using dropdown fields only (no free text input to prevent malformed data):
  - Years of programming experience (dropdown with options: "0-2 years", "3-5 years", "6-10 years", "10+ years")
  - Familiarity with ROS 2 (dropdown with options: "None", "Beginner", "Intermediate", "Advanced")
  - Hardware access (single-select dropdown with options: "None", "Simulation only", "Physical robots/sensors")
- **FR-002a**: System MUST require all background questions to be answered; signup form cannot be submitted until all fields (email, password, programming experience, ROS 2 familiarity, hardware access) are completed
- **FR-003**: System MUST validate email addresses for correct format using both client-side validation (instant feedback with error message "Invalid email format") and server-side validation (reject if client validation bypassed) before account creation
- **FR-003a**: System MUST use FastAPI-Users' default password validation rules with Argon2 hashing (strength requirements, minimum length, complexity) for all password inputs during signup and signin
- **FR-004**: System MUST securely hash and store passwords using FastAPI-Users' Argon2 hashing mechanism
- **FR-005**: System MUST store user background information in user profile metadata (JSONB field) associated with the authenticated user account
- **FR-006**: System MUST provide a signin form accessible from the chat widget that accepts email and password
- **FR-007**: System MUST authenticate users using FastAPI-Users and establish a JWT-based session
- **FR-008**: System MUST restrict chat widget functionality to authenticated users only
- **FR-009**: System MUST display "Please sign in to use the chat" message to unauthenticated users attempting to access the chat widget
- **FR-010**: System MUST fetch the authenticated user's background information before each chat request
- **FR-011**: System MUST dynamically adjust the RAG agent's system prompt based on user background information retrieved from user metadata
- **FR-012**: RAG agent personalization MUST adapt response complexity based on programming experience level (e.g., beginner → simple explanations, expert → technical depth)
- **FR-013**: RAG agent personalization MUST adapt ROS 2 explanations based on familiarity level (e.g., none → foundational concepts, advanced → advanced features)
- **FR-014**: RAG agent personalization MUST reference user's hardware context when relevant (e.g., "Simulation only" users get simulation-focused guidance, "Physical robots/sensors" users get hardware-specific advice, "None" users get general conceptual explanations)
- **FR-015**: System MUST replace current in-memory session management with FastAPI-Users' JWT-based session management, storing user data in Neon PostgreSQL with a default JWT token expiration of 7 days
- **FR-016**: System MUST handle session expiration gracefully by allowing users to continue typing their current message, displaying a re-authentication prompt when attempting to send, and preserving the typed message after successful re-login
- **FR-017**: System MUST prevent duplicate email registrations by checking existing accounts before creation and display error message "Email already registered. Try signing in instead." with a clickable link redirecting to the sign-in form
- **FR-018**: System MUST provide clear error messages for authentication failures (e.g., incorrect password, email not found)
- **FR-018a**: System MUST handle network failures during authentication operations by displaying user-friendly error message "Connection failed. Please check your internet and try again." with a retry button (no automatic retry attempts)
- **FR-019**: System MUST use Neon PostgreSQL database configured via DATABASE_URL environment variable from .env file for all authentication data storage (users, sessions, background profiles)

### Key Entities *(include if feature involves data)*

- **User Account**: Represents an authenticated user with email, hashed password (Argon2), and unique identifier managed by FastAPI-Users
- **User Background Profile**: Contains expertise metadata including programming experience level, ROS 2 familiarity, and hardware access information; stored in user metadata JSONB field
- **Session**: Represents an authenticated user's active JWT token managed by FastAPI-Users, with 7-day expiration
- **Chat Message**: Represents a user's query to the RAG chatbot, associated with the authenticated user and their background context
- **Personalized Response**: Represents the RAG agent's response, generated with system prompt dynamically adjusted based on user background

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete account registration, including background questions, in under 3 minutes
- **SC-002**: 100% of chatbot responses for authenticated users reflect their background profile (verified by testing with different expertise levels)
- **SC-003**: Guest users attempting to use chat are blocked 100% of the time and shown the sign-in prompt
- **SC-004**: Authenticated users can ask a question and receive a personalized response in under 5 seconds (including background data retrieval)
- **SC-005**: User sessions persist across browser refreshes without requiring re-authentication for the session duration
- **SC-006**: Authentication errors (wrong password, email not found) provide clear user-friendly messages 100% of the time
- **SC-007**: Beginner users (0-2 years experience) receive responses with simpler language and step-by-step guidance compared to expert users (5+ years experience) for the same question
- **SC-008**: Users with "no ROS 2 experience" receive foundational concept explanations, while "advanced" users receive technical implementation details for the same ROS 2 question

## Scope *(mandatory)*

### In Scope

- User signup with email/password authentication via FastAPI-Users
- Collection of custom background questions during signup (programming experience, ROS 2 familiarity, hardware access)
- Storage of user background in user metadata JSONB field
- User signin with email/password authentication
- JWT-based session management using FastAPI-Users
- Chat widget access restriction to authenticated users only
- Dynamic RAG agent system prompt adjustment based on user background
- Personalized chatbot responses reflecting user expertise level
- Session persistence across page refreshes
- Basic error handling for authentication failures

### Out of Scope

- **Password reset/forgot password functionality** - explicitly not required for this phase
- **Social login** (OAuth providers like Google, GitHub) - explicitly not required
- **Email verification** during signup - explicitly not required
- **Multi-factor authentication (MFA)** - explicitly not required
- **Role-based access control (RBAC)** - explicitly not required
- **Chat history persistence** - handled separately in another feature
- User profile editing after registration (can be added in future iteration)
- Admin panel for user management (can be added in future iteration)
- Analytics tracking of personalization effectiveness (can be added in future iteration)
- A/B testing of different personalization strategies (can be added in future iteration)

## Assumptions *(mandatory)*

- FastAPI-Users library is compatible with the existing FastAPI backend
- The existing RAG agent (docs_agent) supports dynamic system prompt modification
- Neon PostgreSQL database is already configured and accessible via DATABASE_URL in .env file
- DATABASE_URL environment variable is properly configured with valid Neon PostgreSQL connection string
- FastAPI-Users supports Neon PostgreSQL via SQLAlchemy async adapter
- User background questions will be displayed as dropdown fields in the signup UI
- Session duration is set to 7 days; users must re-authenticate weekly
- The existing chat widget UI can be extended to include authentication UI elements
- Network connectivity is stable during authentication flows
- Users will provide truthful background information (no verification against actual skill level)
- The personalization logic will be implemented in the RAG agent layer, not in Better Auth itself
- Programming experience ranges ("0-2 years", "3-5 years", "6-10 years", "10+ years") map to expertise levels for personalization (0-2 = beginner, 3-5 = intermediate, 6-10 = advanced, 10+ = expert)
- Render free tier supports Neon PostgreSQL connections and Better Auth deployment

## Dependencies *(mandatory)*

- **Better Auth** (Node.js library, runs as separate microservice)
- **Node.js 18+** runtime for Better Auth service
- **Neon PostgreSQL** database (via DATABASE_URL, shared between Better Auth and FastAPI)
- **Existing FastAPI backend** with uv environment (validates JWT via PyJWT)
- **Existing Docusaurus frontend** with React components (integrates Better Auth client SDK)
- **Existing RAG agent** (docs_agent) (supports dynamic system prompt injection)
- **Better Auth MCP server** for documentation and integration examples
- **PyJWT[crypto]** library for FastAPI JWT validation (RS256 support)

## Constraints *(mandatory)*

- Must use Better Auth as the authentication library (original requirement)
- Better Auth runs as **separate Node.js microservice**, not embedded in FastAPI
- **Deployment**: Better Auth and FastAPI deployed as **separate Render services** with distinct URLs (e.g., auth.yourdomain.com and api.yourdomain.com)
- **CORS Configuration**: Both services must configure CORS to allow cross-origin requests from Docusaurus frontend domain
- **Shared Database**: Both services connect to same Neon PostgreSQL instance via DATABASE_URL
- FastAPI validates JWT tokens but does NOT handle authentication logic directly
- Must replace current in-memory session management with JWT-based authentication
- Must be deployable on Render free tier (no features requiring paid infrastructure)
- Must not expose sensitive user data (passwords, email) in chat logs or RAG agent context
- Must keep existing RAG agent (docs_agent) functionality unchanged - only add personalization layer
- Background questions answered during signup only (not iteratively updated this phase)
- Personalization must be transparent to users
- Authentication UI accessible from navbar Sign In/Sign Up buttons (triggers modals)
- Frontend calls Better Auth REST API at auth service URL for signup/signin
- Frontend sends JWT to FastAPI at API service URL in Authorization header
- FastAPI validates tokens via Better Auth JWKS endpoint (cross-service communication)

## Non-Functional Requirements *(optional - include if relevant)*

### Performance

- Authentication operations (signup, signin) must complete within 2 seconds under normal conditions
- User background data retrieval must add no more than 500ms latency to chat requests
- Session validation must add no more than 100ms overhead to each authenticated request

### Security

- Passwords must be validated using FastAPI-Users' default password strength requirements and hashed using Argon2 (secure, modern hashing algorithm)
- Session tokens must be cryptographically secure and unpredictable
- User background data must not be exposed in client-side logs or network responses outside authenticated contexts
- SQL injection and XSS vulnerabilities must be prevented through parameterized queries and input sanitization

### Usability

- Signup and signin forms must be intuitive and require no technical knowledge to complete
- Error messages must be user-friendly and actionable (e.g., "Email already registered. Try signing in instead.")
- Background questions must include helpful descriptions/tooltips explaining what each field means
- Chat widget must clearly indicate authentication status (signed in vs. guest)

### Reliability

- Authentication service must handle concurrent signup/signin requests without race conditions
- JWT tokens remain valid across server restarts (stateless authentication)
- Failed authentication attempts must not crash the application or expose stack traces to users
- System must gracefully handle Neon PostgreSQL database unavailability and display user-friendly error messages

## Risks & Mitigations *(optional - include if relevant)*

### Risk 1: FastAPI-Users Integration Complexity

**Description**: FastAPI-Users may require complex configuration with async SQLAlchemy and Neon PostgreSQL.

**Mitigation**: Follow official FastAPI-Users documentation for async database adapters. Use asyncpg driver for optimal Neon PostgreSQL compatibility. Start with minimal configuration and expand incrementally.

### Risk 2: Personalization Not Noticeable

**Description**: Users may not perceive the difference in chatbot responses between expertise levels, reducing perceived value.

**Mitigation**: Design clear, testable personalization rules with distinct language/depth differences. Include examples in acceptance testing that demonstrate obvious contrast between beginner and expert responses.

### Risk 3: User Background Inaccuracy

**Description**: Users may provide inaccurate background information (e.g., claiming expert status when they're beginners), leading to mismatched personalization.

**Mitigation**: Accept this as a known limitation for MVP. In future iterations, adaptive personalization could adjust based on user interaction patterns. For now, trust user self-reporting.

### Risk 4: Session Management Migration Conflicts

**Description**: Replacing existing in-memory session management with JWT-based sessions may cause conflicts or break existing functionality that depends on the current session structure.

**Mitigation**: Fully replace in-memory sessions with JWT-based authentication. Audit all endpoints that currently use session data and update them to use JWT validation middleware. Create comprehensive test cases for session creation, validation, expiration, and refresh flows. Test with both new and existing chat functionality to ensure no regressions.

### Risk 5: Neon PostgreSQL and Render Free Tier Limitations

**Description**: Neon PostgreSQL free tier may have connection limits, storage limits, or performance constraints that impact authentication performance. Render free tier may have deployment limitations that affect FastAPI-Users or database connectivity.

**Mitigation**: Review Neon PostgreSQL free tier limits (connections, storage, queries) before implementation. Implement connection pooling to minimize database connections. Monitor database usage during testing. Verify FastAPI-Users works on Render free tier with Neon PostgreSQL by testing deployment early in development cycle. Have fallback plan to upgrade to paid tier if free tier proves insufficient.

## Open Questions *(optional - include if relevant)*

- How should we handle users who want to update their background profile after registration?
- Should there be a visible indicator to users showing how their background affects chatbot responses?
- Should we log personalization decisions for analytics/debugging purposes?
