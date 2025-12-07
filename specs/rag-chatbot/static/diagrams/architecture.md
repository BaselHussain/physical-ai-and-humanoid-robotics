# RAG Chatbot Architecture Diagrams

**Feature**: RAG Chatbot for Physical AI & Humanoid Robotics Book
**Date**: 2025-12-07

---

## 1. System Architecture Overview

```mermaid
graph TB
    subgraph "User Browser"
        A[Docusaurus Pages]
        B[ChatKit JS Widget]
    end

    subgraph "Frontend Layer"
        A -->|Swizzled Layout| B
        B -->|getClientSecret| C[FastAPI Backend]
    end

    subgraph "Backend Layer (Render)"
        C -->|Session Mgmt| D[In-Memory Sessions]
        C -->|RAG Agent| E[OpenAI Agents SDK]
        E -->|LLM Calls| F[Gemini API]
        E -->|Vector Search| G[Qdrant Cloud]
        E -->|Embeddings| H[Cohere API]
    end

    subgraph "Data Layer"
        G[(Qdrant<br/>Vector DB)]
        D[(In-Memory<br/>Dict)]
    end

    subgraph "CI/CD (GitHub Actions)"
        I[Push to main]
        I -->|Trigger| J[Reindex Workflow]
        J -->|Parse MD| K[Chunking Script]
        K -->|Embed| H
        K -->|Upsert| G
    end

    style B fill:#2D8CFF,color:#fff
    style E fill:#10a37f,color:#fff
    style F fill:#1a73e8,color:#fff
    style G fill:#db4437,color:#fff
    style H fill:#ea4c89,color:#fff
```

---

## 2. Chat Flow (User Query → Streaming Response)

```mermaid
sequenceDiagram
    actor User
    participant Widget as ChatKit Widget
    participant API as FastAPI Backend
    participant Agent as OpenAI Agents SDK
    participant Cohere as Cohere API
    participant Qdrant as Qdrant Cloud
    participant Gemini as Gemini API

    User->>Widget: Highlight text & ask question
    Widget->>Widget: Pre-fill composer with highlighted text
    User->>Widget: Submit question
    Widget->>API: POST /api/chat (message + session_id)

    API->>Agent: run_streamed(input=message)
    Agent->>Cohere: Generate embedding for query
    Cohere-->>Agent: Query vector [1024 dims]

    Agent->>Qdrant: Search similar chunks (top 5)
    Qdrant-->>Agent: [chunk1, chunk2, ..., chunk5]

    Agent->>Gemini: Generate response (streaming=true)
    Gemini-->>Agent: Token stream starts

    loop Stream tokens
        Agent-->>API: Yield token delta
        API-->>Widget: SSE event: {type: "token", delta: "..."}
        Widget-->>User: Display token (typing effect)
    end

    Agent-->>API: Yield source references
    API-->>Widget: SSE event: {type: "source", chunk_id: "..."}
    Widget-->>User: Display source link

    Agent-->>API: Yield complete event
    API-->>Widget: SSE event: {type: "complete"}
    Widget-->>User: Enable input for next question
```

---

## 3. Auto-Reindex Flow (GitHub Push → Vector DB Update)

```mermaid
sequenceDiagram
    actor Dev as Developer
    participant GH as GitHub
    participant Actions as GitHub Actions
    participant Script as Reindex Script
    participant Cohere as Cohere API
    participant Qdrant as Qdrant Cloud

    Dev->>GH: Push commit to main
    GH->>GH: Detect changes in docs/**/*.md
    GH->>Actions: Trigger "Reindex Documentation" workflow

    Actions->>Script: Run reindex_docs.py
    Script->>Script: Parse changed files

    loop For each Markdown file
        Script->>Script: Split into chunks (max 384 tokens)
        Script->>Cohere: Batch embed chunks (96 chunks/request)
        Cohere-->>Script: Return embeddings [1024 dims each]
        Script->>Qdrant: Upsert chunks (key: file_path:chunk_index)
        Qdrant-->>Script: Acknowledge upsert
    end

    Script->>Qdrant: Delete chunks for removed files
    Qdrant-->>Script: Acknowledge deletion

    Script->>Actions: Report success (files processed, chunks created)
    Actions->>GH: Mark workflow as complete (green check)

    Note over Script,Qdrant: Total time: <5 minutes for 200 files
```

---

## 4. Session Management & Context Persistence

```mermaid
stateDiagram-v2
    [*] --> SessionCreated: POST /api/chatkit/session
    SessionCreated --> Active: First message sent

    Active --> Active: User sends message\n(conversation context maintained)
    Active --> Expired: 30 min inactivity
    Active --> Deleted: Server restart

    Expired --> [*]: Garbage collected
    Deleted --> [*]: Memory cleared

    note right of Active
        In-Memory Storage:
        - session_id → ChatSession
        - Messages list
        - Max 10 exchanges
    end note

    note right of Expired
        Background task runs
        every 5 minutes to
        remove expired sessions
    end note
```

---

## 5. Error Handling & Rate Limit Recovery

```mermaid
flowchart TD
    A[User sends message] --> B{Qdrant available?}
    B -->|No| C[Return error:<br/>'Vector DB unavailable']
    B -->|Yes| D[Search for relevant chunks]

    D --> E{Gemini API call}
    E -->|Success| F[Stream response tokens]
    E -->|Rate Limit Error| G{Retry attempt < 3?}

    G -->|Yes| H[Show 'Processing...' message]
    H --> I[Wait with exponential backoff<br/>2s → 4s → 8s]
    I --> E

    G -->|No| J[Return error:<br/>'Rate limit exceeded, try again']

    F --> K[Send source references]
    K --> L[Mark response complete]

    C --> M[User sees error message]
    J --> M
    L --> N[User can ask follow-up]

    style H fill:#FFA500,color:#fff
    style I fill:#FF6347,color:#fff
    style M fill:#DC143C,color:#fff
```

---

## 6. Deployment Architecture (Render Free Tier)

```mermaid
graph TB
    subgraph "GitHub Repository"
        A[main branch]
    end

    subgraph "Render Cloud"
        B[Web Service<br/>FastAPI Backend]
        C[Environment Variables<br/>GEMINI_API_KEY<br/>COHERE_API_KEY<br/>QDRANT_URL<br/>QDRANT_API_KEY]
        B -.->|Reads| C
    end

    subgraph "External Services"
        D[Qdrant Cloud<br/>Vector Database]
        E[Gemini API<br/>LLM]
        F[Cohere API<br/>Embeddings]
    end

    subgraph "GitHub Pages"
        G[Docusaurus Site<br/>Static HTML + ChatKit]
    end

    A -->|Auto-deploy| B
    A -->|Build & Deploy| G

    B -->|HTTPS| D
    B -->|HTTPS| E
    B -->|HTTPS| F

    G -->|fetch() calls| B

    H[User Browser] -->|HTTPS| G
    H -->|WebSocket| B

    style B fill:#0db7ed,color:#fff
    style G fill:#25c2a0,color:#fff
    style D fill:#db4437,color:#fff
    style E fill:#1a73e8,color:#fff
    style F fill:#ea4c89,color:#fff
```

---

## 7. Component Interaction Matrix

| Component | Depends On | Provides To | Communication Protocol |
|-----------|-----------|-------------|----------------------|
| **ChatKit JS Widget** | FastAPI Backend | User Browser | HTTPS (REST + SSE) |
| **FastAPI Backend** | Gemini, Cohere, Qdrant | ChatKit Widget, GitHub Actions | HTTPS (REST) |
| **OpenAI Agents SDK** | FastAPI, Gemini, Cohere | FastAPI Backend | Python async/await |
| **Qdrant Cloud** | - | FastAPI Backend | HTTPS (gRPC) |
| **Gemini API** | - | OpenAI Agents SDK | HTTPS (REST) |
| **Cohere API** | - | OpenAI Agents SDK, Reindex Script | HTTPS (REST) |
| **GitHub Actions** | Cohere, Qdrant | - | HTTPS (REST) |
| **Docusaurus** | ChatKit Widget | User Browser | Static HTML |

---

## 8. Data Flow: Text Highlighting → Contextualized Answer

```mermaid
flowchart LR
    A[User highlights text] --> B[mouseup event listener]
    B --> C{Text length > 3 chars?}
    C -->|Yes| D[Pre-fill ChatKit composer]
    C -->|No| E[Ignore selection]

    D --> F[User clicks send]
    F --> G[POST /api/chat<br/>message: 'What is NVIDIA Isaac Sim?'<br/>context.highlighted_text: 'NVIDIA Isaac Sim 2025']

    G --> H[Backend embeds query]
    H --> I[Qdrant returns relevant chunks<br/>including chunks mentioning 'Isaac Sim 2025']

    I --> J[Gemini generates answer<br/>with context from chunks]
    J --> K[Streamed answer:<br/>'Isaac Sim is a robotics simulation platform...']

    K --> L[Source links displayed:<br/>docs/004-isaac-platform/overview.md]

    style D fill:#2D8CFF,color:#fff
    style I fill:#db4437,color:#fff
    style J fill:#1a73e8,color:#fff
```

---

## 9. Folder Structure (Implementation)

```
physical-ai-and-humanoid-robotics/
├── backend/                        # FastAPI backend (existing)
│   ├── main.py                     # Updated with chat endpoints
│   ├── app/                        # NEW: RAG agent and session management
│   │   ├── rag_agent.py            # RAG agent configuration
│   │   └── session_manager.py      # In-memory session storage
│   ├── scripts/                    # NEW: Reindexing scripts
│   │   └── reindex_docs.py         # GitHub Actions reindex script
│   ├── models/                     # Pydantic models
│   │   ├── chat.py                 # ChatRequest, ChatMessage
│   │   └── indexing.py             # IndexingJob, DocumentChunk
│   ├── requirements.txt            # Updated with agents SDK
│   └── .env                        # Existing (GEMINI_API_KEY, etc.)
│
├── src/                            # Docusaurus theme overrides
│   ├── theme/                      # NEW: Swizzled components
│   │   └── Layout/
│   │       └── index.js            # Wrapped layout with ChatKit
│   └── components/                 # NEW: Custom components
│       └── ChatKitWidget.tsx       # ChatKit React component
│
├── docs/                           # Documentation (existing)
│   ├── 001-physical-ai-intro/
│   ├── 002-ros2-mastery/
│   ├── 003-gazebo-simulation/
│   └── 004-isaac-platform/
│
├── .github/
│   └── workflows/
│       ├── deploy.yml              # Existing
│       └── reindex.yml             # NEW: Auto-reindex workflow
│
├── specs/
│   └── rag-chatbot/
│       ├── spec.md
│       ├── plan.md
│       ├── research.md
│       ├── data-model.md
│       ├── contracts/
│       │   ├── chat-api.yaml
│       │   └── reindex-api.yaml
│       └── static/diagrams/
│           └── architecture.md     # This file
│
├── package.json                    # Updated with ChatKit dependency
├── docusaurus.config.js            # Existing
└── render.yaml                     # NEW: Render deployment config
```

---

## 10. Technology Stack Diagram

```mermaid
graph LR
    subgraph "Frontend"
        A[ChatKit JS<br/>@openai/chatkit-react]
        B[Docusaurus v3<br/>Static Site Generator]
        C[React 18]
    end

    subgraph "Backend"
        D[FastAPI<br/>Python 3.11]
        E[OpenAI Agents SDK<br/>v0.6.2+]
        F[Pydantic<br/>Data Validation]
    end

    subgraph "AI/ML Services"
        G[Gemini API<br/>gemini-1.5-flash]
        H[Cohere API<br/>embed-english-v3.0]
    end

    subgraph "Data Storage"
        I[Qdrant Cloud<br/>Vector Database]
        J[In-Memory Dict<br/>Session Storage]
    end

    subgraph "Deployment"
        K[Render<br/>Free Tier]
        L[GitHub Pages<br/>Static Hosting]
        M[GitHub Actions<br/>CI/CD]
    end

    C --> A
    B --> C
    A --> D
    D --> E
    E --> G
    E --> H
    E --> I
    D --> F
    D --> J

    B --> L
    D --> K
    M --> K
    M --> I

    style A fill:#2D8CFF,color:#fff
    style E fill:#10a37f,color:#fff
    style G fill:#1a73e8,color:#fff
    style H fill:#ea4c89,color:#fff
    style I fill:#db4437,color:#fff
    style K fill:#0db7ed,color:#fff
```

---

## Status

✅ **All architecture diagrams complete** - Ready for quickstart guide (Phase 1 next step)
