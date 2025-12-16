# Feature Specification: RAG Chatbot for Physical AI & Humanoid Robotics Book

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-07
**Status**: Draft
**Input**: User description: "RAG Chatbot for Physical AI & Humanoid Robotics Book - Create folder: specs/rag-chatbot/ only (no number prefix)"

## Clarifications

### Session 2025-12-07

- Q: How should the chatbot respond to questions outside the documentation domain? → A: Polite refusal with suggested topics from the documentation
- Q: What should be the maximum chunk size limit for documentation segments? → A: 384 tokens per chunk (medium granularity)
- Q: How should the system respond when the Gemini API rate limit is exceeded? → A: Queue request with exponential backoff retry + show "Processing..." message to user
- Q: Which frontend chat UI library should we use? → A: Chatkit-JS for frontend chat UI, with Chatkit-Python backend connecting to OpenAI Agents SDK (using LiteLLM for Gemini compatibility)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Interactive Reading with AI Assistant (Priority: P1)

Readers browsing the Physical AI & Humanoid Robotics documentation need instant clarification on technical concepts without leaving the page or breaking their reading flow. They can highlight any text passage, ask a question, and receive accurate answers with source references streamed in real-time.

**Why this priority**: Core value proposition - transforms passive documentation into an interactive learning experience. Without this, the chatbot provides no differentiation from standard documentation.

**Independent Test**: Can be fully tested by navigating to any published documentation page, highlighting text such as "ROS 2 Humble integration", asking "How does this work with Isaac Sim?", and verifying the chatbot streams an accurate answer with clickable source links within 3 seconds.

**Acceptance Scenarios**:

1. **Given** a user is reading the "Isaac Platform Overview" page, **When** they highlight "NVIDIA Isaac Sim 2025", **Then** a context menu appears with options including "Ask from AI", "Copy", and other standard text operations
2. **Given** a user highlights text and the context menu appears, **When** they click "Ask from AI", **Then** the chat widget opens (if closed) with the highlighted text pre-populated in the input field
3. **Given** a user asks "What are the system requirements for Isaac Sim?", **When** the question is submitted, **Then** the chatbot streams the answer token-by-token and displays source links to relevant documentation sections
4. **Given** a user receives an answer with 3 source links, **When** they click a source link, **Then** they navigate to the exact documentation page mentioned in the answer
5. **Given** a user asks a follow-up question "Can it run on Windows?", **When** submitted, **Then** the chatbot maintains conversation context and references the previous question in its answer
6. **Given** a user closes and reopens the browser, **When** they return to the documentation site and open the chat widget, **Then** their previous conversation history is restored from the database

---

### User Story 2 - Fresh Content Synchronization (Priority: P2)

Documentation maintainers update Markdown files in the repository and push changes to GitHub. The knowledge base automatically re-indexes all content within 5 minutes, ensuring readers always query the latest documentation without manual intervention.

**Why this priority**: Critical for documentation accuracy and maintainer productivity. Without auto-reindexing, stale answers erode user trust and require manual processes.

**Independent Test**: Can be fully tested by committing a new Markdown file to `docs/new-section/test.md`, pushing to GitHub, waiting 5 minutes, then asking the chatbot "What is in the test documentation?", and verifying the answer reflects the new content.

**Acceptance Scenarios**:

1. **Given** a maintainer pushes a commit updating `docs/001-physical-ai-intro/overview.md`, **When** the GitHub Actions workflow completes, **Then** the vector database contains the updated content within 5 minutes
2. **Given** a new Markdown file `docs/new-topic/advanced.md` is added to the repository, **When** the auto-reindex process runs, **Then** users can ask questions about the new topic and receive accurate answers
3. **Given** a Markdown file is deleted from `docs/`, **When** the reindex completes, **Then** the chatbot no longer references the deleted content in answers
4. **Given** the reindex process encounters a malformed Markdown file, **When** processing completes, **Then** the system logs the error, skips the malformed file, and successfully indexes all valid files

---

### User Story 3 - Zero-Config Widget Integration (Priority: P3)

Readers accessing any page of the published documentation site automatically see the chat widget in the bottom-right corner without requiring login, configuration, or setup. The widget is immediately usable upon page load.

**Why this priority**: Essential for user accessibility and adoption. Reduces friction to zero for first-time users.

**Independent Test**: Can be fully tested by opening any documentation page in an incognito browser window and verifying the chat widget appears within 2 seconds of page load and accepts questions without authentication.

**Acceptance Scenarios**:

1. **Given** a first-time visitor opens `https://[book-domain]/docs/intro`, **When** the page loads, **Then** the chat icon appears in the bottom-right corner within 2 seconds with the chat window initially closed
2. **Given** the chat icon is visible, **When** a user clicks the chat icon, **Then** the chat widget window opens/expands without authentication prompts
3. **Given** the chat widget is open, **When** a user clicks the close button or chat icon again, **Then** the chat window collapses back to just the icon
4. **Given** a user navigates from page A to page B, **When** the new page loads, **Then** the chat widget persists and maintains conversation history
5. **Given** a user on a mobile device accesses the documentation, **When** the page loads, **Then** the chat widget is responsive and usable on small screens

---

### Edge Cases

- When a user asks a question completely unrelated to the documentation (e.g., "What's the weather today?"), the system responds with a polite refusal message and suggests relevant documentation topics
- How does the system handle simultaneous questions from 100+ concurrent users?
- What happens if the Qdrant Cloud service is temporarily unavailable?
- How does the chatbot respond when asked about content that exists in multiple documentation pages with conflicting information?
- What happens if a user submits an extremely long question (5000+ characters)?
- How does the system handle Markdown files with no readable text content (e.g., only images or diagrams)?
- When the Gemini API rate limit is exceeded, the system queues the request with exponential backoff retry and displays a "Processing..." message to the user
- How does the chatbot handle code blocks, Mermaid diagrams, and special formatting in answers?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a chat icon on every page of the published documentation site, with the chat window initially closed
- **FR-001a**: When user clicks the chat icon, the chat window MUST open/expand; clicking again or the close button MUST collapse it back to just the icon
- **FR-002**: When user highlights any text on the page, system MUST display a context menu with options including "Ask from AI", "Copy", and other standard text operations
- **FR-002a**: When user selects "Ask from AI" from the context menu, the chat widget MUST open (if closed) and pre-populate the chat input with the highlighted text
- **FR-003**: System MUST accept natural language questions without requiring user authentication or login
- **FR-004**: System MUST stream chatbot responses token-by-token as they are generated (no waiting for complete response)
- **FR-005**: System MUST include clickable source links in every answer that reference the specific documentation pages used to generate the response
- **FR-006**: System MUST automatically re-index all Markdown files from the `docs/` directory whenever changes are pushed to the GitHub repository
- **FR-007**: System MUST use Cohere embedding model to generate vector embeddings for all documentation content
- **FR-008**: System MUST store vector embeddings in the existing Qdrant Cloud instance (URL and API key already configured in `backend/main.py`)
- **FR-009**: System MUST use Gemini API as the language model for generating responses (API key already configured in `backend/.env`)
- **FR-010**: System MUST NOT use OpenAI API for any functionality
- **FR-011**: System MUST retrieve relevant documentation chunks from Qdrant based on semantic similarity to the user's question
- **FR-012**: System MUST maintain conversation context for follow-up questions within the same session
- **FR-013**: Backend MUST run in the existing `backend/` folder using the current Python environment managed by `uv`
- **FR-014**: System MUST be deployable to Render or Railway free tier without modification
- **FR-015**: System MUST handle errors gracefully (network failures, empty results) with user-friendly messages
- **FR-015a**: When Gemini API rate limits are exceeded, system MUST queue the request with exponential backoff retry and display a "Processing..." message to the user until the request completes or fails after maximum retry attempts
- **FR-016**: System MUST process Markdown files including frontmatter, code blocks, tables, and Mermaid diagrams during indexing
- **FR-017**: System MUST chunk large documentation pages into semantically meaningful segments (e.g., by heading, paragraph, or code block) with a maximum size of 384 tokens per chunk
- **FR-018**: System MUST persist conversation history in Neon Postgres database, allowing users to restore their chat history across browser sessions and devices
- **FR-018a**: Chat widget MUST maintain conversation history as users navigate between documentation pages within the same session
- **FR-018b**: System MUST create a unique session identifier for each user and store it along with all chat messages (user and assistant) in the Neon database
- **FR-018c**: When a user returns to the documentation site, system MUST retrieve and display their previous conversation history from the Neon database
- **FR-019**: System MUST respond to off-topic questions (those unrelated to the documentation domain) with a polite refusal message and suggest relevant documentation topics for the user to explore

### Key Entities

- **Documentation Chunk**: Represents a semantically meaningful segment of a Markdown file (maximum 384 tokens), including the chunk text, source file path, heading/section title, vector embedding, and metadata (page URL, last updated timestamp)
- **Chat Message**: Represents a single message in a conversation, stored in Neon Postgres database with fields: message_id (UUID), session_id (UUID foreign key), message_text (text), role (enum: user/assistant), timestamp (timestamptz), source_references (JSONB array, if assistant message)
- **Chat Session**: Represents a persistent conversation context, stored in Neon Postgres database with fields: session_id (UUID primary key), user_identifier (text, derived from browser fingerprint or local storage), created_at (timestamptz), last_active_at (timestamptz)
- **Indexing Job**: Represents a batch re-indexing operation, including list of Markdown files processed, success/failure status, error logs, and completion timestamp

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can ask a question and receive the first token of the streamed response within 2 seconds (95th percentile latency)
- **SC-002**: System maintains 99% uptime when deployed to Render or Railway free tier over a 30-day period
- **SC-003**: Chat widget loads and becomes interactive within 2 seconds of page load on standard broadband connections
- **SC-004**: Auto-reindex process completes within 5 minutes for documentation sites with up to 200 Markdown files
- **SC-005**: 90% of user questions receive answers with at least one source reference link
- **SC-006**: System handles 50 concurrent chat sessions without response degradation (latency increase < 20%)
- **SC-007**: Chatbot provides relevant answers (subjectively rated "helpful" by test users) for 85% of in-domain questions
- **SC-008**: Zero failed deployments during initial rollout to production environment
- **SC-009**: Conversation context is maintained across at least 10 back-and-forth exchanges within a single session

## Assumptions *(auto-generated)*

1. **Documentation Format**: All content in `docs/` directory is in Markdown format compatible with Docusaurus v3.x
2. **Deployment Environment**: Target deployment platforms (Render/Railway) support Python 3.10+ FastAPI applications with persistent WebSocket connections
3. **GitHub Integration**: Repository has GitHub Actions enabled and permissions to trigger workflows on push events
4. **Vector Database Capacity**: Existing Qdrant Cloud instance has sufficient storage and query capacity for the documentation corpus (estimated < 10,000 chunks)
5. **API Rate Limits**: Gemini API free tier provides sufficient quota for expected query volume (estimated < 1000 queries/day during initial rollout)
6. **Browser Compatibility**: Target users access documentation via modern browsers supporting ES6+ JavaScript (Chrome 90+, Firefox 88+, Safari 14+) with localStorage support
7. **Database Persistence**: Neon Postgres serverless database provides sufficient free tier capacity for storing conversation history (estimated < 10,000 messages/month during initial rollout)
8. **Session Identification**: Browser localStorage and/or fingerprinting provides reliable user session identification without requiring authentication
9. **Content Ownership**: All documentation content is suitable for public indexing and does not contain sensitive/proprietary information requiring access control

## Dependencies *(auto-generated)*

### External Dependencies

- **Qdrant Cloud**: Vector database service must be operational and accessible at the configured URL
- **Gemini API**: Google Gemini API service must be available and the configured API key must have valid quota
- **Cohere API**: Cohere embedding API must be available and the configured API key must have valid quota
- **Neon Postgres**: Neon serverless Postgres database must be operational and accessible with valid connection string for storing chat history
- **GitHub Actions**: Repository must have Actions enabled for auto-reindexing workflow execution
- **Deployment Platform**: Render or Railway platform must be accessible and configured with necessary environment variables

### Internal Dependencies

- **Documentation Site**: Published Docusaurus site must be accessible and serving Markdown content
- **Existing Backend**: Current `backend/` folder with `uv` environment and installed packages must remain functional
- **Environment Configuration**: `backend/.env` must contain valid `GEMINI_API_KEY` value
- **Main Application**: `backend/main.py` must retain existing Cohere client and Qdrant client configurations

### Out of Scope

- User authentication or authorization (session identification via browser fingerprinting/localStorage is sufficient)
- User account management or profile storage (beyond session tracking)
- Local vector database deployment (must use Qdrant Cloud)
- Multi-language support (English only)
- Analytics or usage tracking dashboards
- Admin interface for managing indexed content
- Content moderation or filtering
- Export conversation history functionality
- Sharing conversations between users
