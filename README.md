# Physical AI & Robotics RAG Chatbot

A production-ready Retrieval-Augmented Generation (RAG) chatbot embedded directly into the Physical AI & Robotics documentation site. Uses OpenAI ChatKit widget + Cohere embeddings + Qdrant Cloud + Neon Serverless Postgres + FastAPI backend. Supports selected-text questioning and full conversation history.

## Features

- **RAG-powered**: Answers based only on your site's content (no hallucinations)
- **Selected Text Support**: Highlight any text and ask questions about it
- **Full Conversation History**: Maintains context across messages
- **Beautiful UI**: Professional, mobile-responsive chat widget
- **GDPR-Safe**: Session-based conversation handling

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      FRONTEND                                │
│  ┌─────────────────────────────────────────────────────┐    │
│  │              ChatKit Widget                         │    │
│  │  ┌──────────────┐  ┌──────────────┐                 │    │
│  │  │ Text Select  │  │   Chat UI    │                 │    │
│  │  │   Handler    │──│   (styled)   │                 │    │
│  │  └──────────────┘  └──────────────┘                 │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                              │
                              │ HTTP POST /chatkit
                              │ SSE Streaming
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                      BACKEND                                 │
│  ┌─────────────────────────────────────────────────────┐    │
│  │              FastAPI Server                         │    │
│  │  ┌──────────────┐  ┌──────────────┐                 │    │
│  │  │   RAG Agent  │  │   History    │                 │    │
│  │  │   (Cohere+   │──│  (Neon PG)   │                 │    │
│  │  │   Qdrant)    │  │              │                 │    │
│  │  └──────────────┘  └──────────────┘                 │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

## Setup Instructions

### 1. Environment Setup

```bash
# Install dependencies
cd backend
uv pip install -e .

# Copy environment file
cp backend/.env backend/.env.example  # if you have .env
# Fill in your API keys in .env
```

### 2. Vector Database Setup

```bash
# Index your content into Qdrant
cd backend
python ingest.py
```

### 3. Run Backend

```bash
# Start the backend server
cd backend
uv run uvicorn main:app --reload --port 8000
```

## API Endpoints

- `GET /health` - Health check
- `POST /chatkit` - ChatKit protocol endpoint
- `POST /api/chatkit/session` - Create ChatKit session

## Frontend Integration

The chat widget is automatically injected into all pages via the Docusaurus Layout wrapper. The widget features:

- Beautiful, customizable UI
- Selected text support
- Conversation history
- Mobile-responsive design

## Deployment

### Backend (Render)
1. Create a new Web Service on Render
2. Connect to your GitHub repo
3. Set the build command: `pip install -e .`
4. Set the start command: `uvicorn main:app:app --host 0.0.0.0 --port $PORT`
5. Add your environment variables

### Frontend (GitHub Pages)
1. The widget will automatically connect to your deployed backend
2. Make sure the `apiBase` in the frontend points to your backend URL

## Environment Variables

- `COHERE_API_KEY` - Cohere API key for embeddings
- `QDRANT_URL` - Qdrant Cloud URL
- `QDRANT_API_KEY` - Qdrant Cloud API key
- `GEMINI_API_KEY` - Gemini API key (for LLM responses)

## Ingestion Script

The `ingest.py` script indexes your site content:

```bash
# Index all markdown files from docs/
cd backend
python ingest.py

# The script will:
# 1. Find all .md/.mdx files in your docs
# 2. Chunk content appropriately
# 3. Embed using Cohere
# 4. Store in Qdrant vector DB
```

## Local Development

```bash
# Terminal 1: Start backend
cd backend
uv run uvicorn main:app --reload --port 8000

# Terminal 2: Start frontend (in physical-robotics-ai-book/)
cd physical-robotics-ai-book
npm run start
```

Visit http://localhost:3000 to see the site with the chat widget.

## Production Features

- ✅ Answers only from your site content (no hallucinations)
- ✅ Highlight any text → "Ask about this" functionality
- ✅ Full conversation history per user (Neon Postgres)
- ✅ Mobile-responsive, animated, professional design
- ✅ GDPR-safe session handling
- ✅ Rate limiting and error handling
- ✅ Health monitoring and logging

Your RAG chatbot is 100% ready!
Deploy backend → Run ingestion → Paste widget → Done!
Users can now talk to your website like it's alive.