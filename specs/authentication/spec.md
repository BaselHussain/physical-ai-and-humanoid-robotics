# Feature Specification: RAG Chatbot Authentication with FastAPI-Users

**Feature Branch**: `authentication`
**Created**: 2025-12-15
**Updated**: 2025-12-17
**Status**: Draft
**Input**: User description: "RAG Chatbot Authentication - Implement Signup and Signin using FastAPI-Users (industry-standard Python authentication library). During signup, ask custom questions about user's software and hardware background (years of programming experience, ROS 2/robotics familiarity, hardware access). Store this in user profile metadata. Use this background to personalize RAG chatbot responses by dynamically adjusting the docs_agent system prompt. Must use Neon PostgreSQL (DATABASE_URL from .env), replace in-memory sessions with database-backed sessions, and be deployable on Render free tier."

## Architectural Decision Note

**Original Requirement**: Better Auth
**Implementation**: FastAPI-Users

**Rationale**: Better Auth is a JavaScript/TypeScript-only authentication library with no Python SDK. After researching via Better Auth MCP server, confirmed that Better Auth requires Node.js runtime and only supports JavaScript frameworks (Next.js, Express, Nuxt, etc.). For our Python/FastAPI backend, FastAPI-Users is the industry-standard equivalent, providing identical functionality: user registration with custom fields, email/password authentication, session management with PostgreSQL, JWT tokens, and secure password hashing (Argon2). All functional requirements (FR-001 through FR-019) are fully achievable with FastAPI-Users.

## Clarifications

### Session 2025-12-15

- Q: Should all background questions (programming experience, ROS 2 familiarity, hardware access) be mandatory during signup, or can users skip some? → A: All fields required - users must answer all background questions to complete signup
- Q: How should the system respond when a user tries to sign up with an email that's already registered? → A: Show error "Email already registered. Try signing in instead." with a link to the sign-in form
- Q: How should the system handle session expiration when a user is actively using the chat widget? → A: Allow current message to be typed; show re-authentication prompt when user tries to send; preserve typed message after re-login
- Q: What should the default session duration be for authenticated users? → A: 7 days - users must re-authenticate weekly
- Q: What specific input format should be used for collecting programming experience? → A: Dropdown with predefined ranges - "0-2 years", "3-5 years", "6-10 years", "10+ years"

### Session 2025-12-17

- Q: What are the password strength requirements beyond minimum 8 characters? → A: FastAPI-Users library handles all password validation (including strength requirements) using its default secure settings with Argon2 hashing
- Q: What input format should be used for hardware access field - single-select dropdown or multi-select? → A: Single-select dropdown with mutually exclusive options: "None", "Simulation only", "Physical robots/sensors"
- Q: How should the system handle invalid email formats? → A: Client-side validation shows instant error "Invalid email format" before submission + server-side validation rejects with same error message if client validation bypassed
- Q: How should the system handle network failures during authentication? → A: Show user-friendly error message "Connection failed. Please check your internet and try again." with retry button; no automatic retry
- Q: How should the system handle very long or malformed input in background questions? → A: Enforce dropdown selection only (no free text input) - users cannot enter malformed data since all values are pre-defined options from dropdowns

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Registration with Personalized Profile (Priority: P1)

A new user visits the documentation site, wants to use the RAG chatbot, creates an account by providing their background information, and immediately experiences personalized chatbot responses tailored to their expertise level.

**Why this priority**: This is the core value proposition - personalized learning experience from the first interaction. Without this, the feature provides no differentiation from anonymous chat.

**Independent Test**: Can be fully tested by registering a new account with different background profiles (beginner vs. expert) and observing that the chatbot's first response adjusts its language and depth accordingly. Delivers immediate personalization value.

**Acceptance Scenarios**:

1. **Given** a guest user on the documentation site, **When** they click "Sign Up" on the chat widget, **Then** they see a registration form with email, password, and background questions
2. **Given** a user on the signup form, **When** they fill in all required fields (email, password, years of programming experience, ROS 2 familiarity level, hardware access), **Then** their account is created and they are automatically signed in
3. **Given** a newly registered beginner user (0-2 years experience, no ROS 2 knowledge), **When** they ask "What is ROS 2?", **Then** the chatbot response uses simple language, step-by-step explanations, and avoids jargon
4. **Given** a newly registered expert user (5+ years experience, advanced ROS 2 knowledge), **When** they ask "What is ROS 2?", **Then** the chatbot response uses technical terminology, focuses on advanced concepts, and provides in-depth details

---

### User Story 2 - Returning User Authentication (Priority: P2)

A registered user returns to the documentation site, signs in with their existing credentials, and the RAG chatbot immediately provides personalized responses based on their saved background profile without needing to re-enter any information.

**Why this priority**: Essential for user retention and consistent personalized experience. Users should not lose their personalization settings across sessions.

**Independent Test**: Can be fully tested by creating an account, logging out, logging back in, and verifying that chatbot personalization persists. Delivers continuity of personalized experience.

**Acceptance Scenarios**:

1. **Given** an existing registered user, **When** they click "Sign In" on the chat widget, **Then** they see a login form with email and password fields
2. **Given** a user on the signin form, **When** they enter correct credentials and submit, **Then** they are authenticated and the chat widget opens
3. **Given** an authenticated user with a saved background profile, **When** they ask any question to the RAG chatbot, **Then** the response is personalized based on their stored expertise level and hardware context
4. **Given** an authenticated user, **When** they close and reopen the browser (session persistence), **Then** they remain signed in and chatbot personalization continues

---

### User Story 3 - Guest User Restriction (Priority: P3)

A guest user (not authenticated) visits the documentation site and attempts to use the RAG chatbot. The system prompts them to sign in or sign up to access the personalized chat feature.

**Why this priority**: Necessary for access control and to encourage registration, but lower priority than the core personalization flows. Can function as a soft gate to drive user registration.

**Independent Test**: Can be fully tested by accessing the chat widget while logged out and verifying that it displays a sign-in prompt instead of allowing chat. Delivers clear call-to-action for registration.

**Acceptance Scenarios**:

1. **Given** a guest user (not signed in), **When** they try to open the chat widget, **Then** they see a message "Please sign in to use the personalized chat" with Sign In and Sign Up buttons
2. **Given** a guest user viewing the sign-in prompt, **When** they click "Sign Up", **Then** they are taken to the registration flow (User Story 1)
3. **Given** a guest user viewing the sign-in prompt, **When** they click "Sign In", **Then** they are taken to the login flow (User Story 2)

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

- FastAPI-Users library (external dependency, must be installed and configured)
- Existing FastAPI backend with uv environment (native support for FastAPI-Users)
- Existing Docusaurus frontend with React components (must support authentication UI)
- Existing RAG agent (docs_agent) (must support dynamic system prompt injection)
- Neon PostgreSQL database (external service, accessed via DATABASE_URL from .env for user and session storage)
- DATABASE_URL environment variable in .env file (must contain valid Neon PostgreSQL connection string)
- SQLAlchemy async adapter with asyncpg driver for Neon PostgreSQL connection

## Constraints *(mandatory)*

- Must use FastAPI-Users as the authentication library (Python-native equivalent of Better Auth)
- Must use Neon PostgreSQL accessed via DATABASE_URL from .env (no SQLite or other databases)
- Must replace current in-memory session management completely with JWT-based sessions
- Must be deployable on Render free tier (no features requiring paid infrastructure)
- Must integrate with existing FastAPI backend without major architectural changes
- Must integrate with existing Docusaurus frontend without disrupting current documentation site functionality
- Must not expose sensitive user data (passwords, email) in chat logs or RAG agent context
- Must keep existing RAG agent (docs_agent) functionality unchanged - only add personalization layer
- Background questions must be answered during signup only (not iteratively updated during this phase)
- Personalization must be transparent to users (they should understand why responses differ)
- Authentication UI must be accessible from the existing chat widget interface

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
