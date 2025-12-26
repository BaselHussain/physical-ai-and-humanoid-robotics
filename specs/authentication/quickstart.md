# Quickstart: RAG Chatbot Authentication

**Feature**: Authentication with Better Auth
**Last Updated**: 2025-12-17

## Overview

This guide walks through setting up the authentication feature locally and deploying to Render free tier. Includes backend (FastAPI + Better Auth + Neon PostgreSQL) and frontend (Docusaurus + React auth UI) setup.

## Prerequisites

### Required Tools
- **Python**: 3.10 or higher ([Download](https://www.python.org/downloads/))
- **Node.js**: 18 LTS or higher ([Download](https://nodejs.org/))
- **uv**: Python package manager ([Install](https://github.com/astral-sh/uv))
- **Git**: Version control ([Download](https://git-scm.com/))

### Required Accounts
- **Neon PostgreSQL**: Free tier account ([Sign up](https://neon.tech/))
- **Render**: Free tier account for deployment ([Sign up](https://render.com/))

### Knowledge Prerequisites
- Basic Python and JavaScript/TypeScript
- Familiarity with REST APIs
- Understanding of environment variables

## Local Development Setup

### Step 1: Clone Repository

```bash
git clone https://github.com/your-username/your-repo.git
cd your-repo
git checkout authentication  # Feature branch
```

### Step 2: Backend Setup

#### 2.1 Create Neon PostgreSQL Database

1. Log in to [Neon Console](https://console.neon.tech/)
2. Create new project: "rag-chatbot-auth"
3. Copy connection string (starts with `postgres://`)
4. Store securely - you'll need it for `.env` file

#### 2.2 Configure Environment Variables

Create `backend/.env` file:

```bash
# Database
DATABASE_URL=postgresql://user:password@ep-xxx.neon.tech/neondb?sslmode=require

# Better Auth
BETTER_AUTH_SECRET=your-random-secret-key-min-32-chars  # Generate with: openssl rand -base64 32
BETTER_AUTH_SESSION_DURATION_DAYS=7

# FastAPI
FASTAPI_ENV=development
FASTAPI_PORT=8000
```

**Security Note**: Never commit `.env` to version control! Add to `.gitignore`.

#### 2.3 Install Dependencies

```bash
cd backend
uv venv  # Create virtual environment
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install -e .  # Install from pyproject.toml/requirements.txt
```

Expected dependencies (auto-installed):
- `better-auth>=2.0.0`
- `fastapi>=0.100.0`
- `asyncpg>=0.28.0`
- `pydantic>=2.0.0`
- `uvicorn>=0.20.0`

#### 2.4 Run Database Migrations

Better Auth auto-migration creates tables on first startup:

```bash
uv run python -m src.auth.migrate  # Or run main.py, auto-migration triggers
```

Expected output:
```
INFO: Better Auth initializing database...
INFO: Created tables: users, sessions
INFO: Indexes created: idx_users_email, idx_sessions_token
INFO: Migration complete ✅
```

#### 2.5 Start Backend Server

```bash
uv run uvicorn src.main:app --reload --port 8000
```

Expected output:
```
INFO: Uvicorn running on http://127.0.0.1:8000
INFO: Application startup complete
INFO: Connected to Neon PostgreSQL
```

**Verify Backend**:
- Open [http://localhost:8000/docs](http://localhost:8000/docs) (FastAPI Swagger UI)
- Should see endpoints: `/auth/signup`, `/auth/signin`, `/auth/signout`, `/chat/message`

### Step 3: Frontend Setup

#### 3.1 Install Dependencies

```bash
cd frontend  # From repo root
npm install  # or: yarn install
```

Expected dependencies (auto-installed from `package.json`):
- `@better-auth/react@^2.0.0`
- `react@^18.0.0`
- `@docusaurus/core@^3.0.0`
- `axios@^1.5.0`

#### 3.2 Configure Environment Variables

Create `frontend/.env.local`:

```bash
REACT_APP_API_URL=http://localhost:8000
```

#### 3.3 Start Frontend Dev Server

```bash
npm start  # Docusaurus dev server
```

Expected output:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

**Verify Frontend**:
- Open [http://localhost:3000](http://localhost:3000)
- Chat widget should show "Please sign in to use the chat" (guest state)

### Step 4: Test Authentication Flow

#### 4.1 Signup Test

1. Navigate to [http://localhost:3000](http://localhost:3000)
2. Click chat widget → "Sign Up" button
3. Fill signup form:
   - Email: `test@example.com`
   - Password: `Test1234!`
   - Programming Experience: `6-10 years`
   - ROS 2 Familiarity: `Intermediate`
   - Hardware Access: `Simulation only`
4. Submit → Should auto-signin, chat widget becomes active

**Backend Verification**:
```bash
# Check user created in database (from backend terminal)
psql $DATABASE_URL -c "SELECT email, metadata FROM users;"
```

Expected output:
```
       email        |                    metadata
--------------------+------------------------------------------------
 test@example.com   | {"background": {"programming_experience": ...}}
```

#### 4.2 Signin Test

1. Sign out (if signed in)
2. Click chat widget → "Sign In" button
3. Enter credentials: `test@example.com` / `Test1234!`
4. Submit → Chat widget becomes active

#### 4.3 Personalization Test

**Beginner User Test**:
1. Sign up as beginner (`0-2 years`, `ROS2=None`)
2. Ask: "What is ROS 2?"
3. Expected response: Simple language, step-by-step, no jargon

**Advanced User Test**:
1. Sign up as advanced (`10+ years`, `ROS2=Advanced`)
2. Ask: "What is ROS 2?"
3. Expected response: Technical terms, code examples, in-depth

**Comparison**: Responses should be noticeably different (SC-007, SC-008)

### Step 5: Run Tests

#### Backend Tests

```bash
cd backend
pytest tests/  # Run all tests
pytest tests/auth/  # Run auth tests only
pytest -v --cov=src.auth  # With coverage report
```

Expected output:
```
tests/auth/test_signup.py::test_signup_success PASSED
tests/auth/test_signin.py::test_signin_success PASSED
tests/auth/test_session.py::test_session_validation PASSED
tests/auth/test_personalization.py::test_beginner_vs_advanced PASSED
...
========== 25 passed in 5.23s ==========
```

#### Frontend Tests

```bash
cd frontend
npm test  # Jest tests
```

Expected output:
```
PASS  src/components/Auth/SignupForm.test.tsx
PASS  src/components/Auth/SigninForm.test.tsx
PASS  src/components/ChatWidget/ChatWidget.test.tsx
...
Tests:       15 passed, 15 total
```

## Production Deployment (Render Free Tier)

### Step 1: Prepare Neon PostgreSQL for Production

1. In Neon Console, create **production database** (separate from dev)
2. Copy production `DATABASE_URL`
3. Enable connection pooling (Settings → Connection Pooling → Enable)

### Step 2: Deploy Backend to Render

#### 2.1 Create Render Web Service

1. Log in to [Render Dashboard](https://dashboard.render.com/)
2. New → Web Service
3. Connect GitHub repository
4. Configure:
   - **Name**: `rag-chatbot-backend`
   - **Environment**: `Python 3`
   - **Build Command**: `uv pip install -e .`
   - **Start Command**: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
   - **Plan**: `Free`

#### 2.2 Add Environment Variables

In Render service settings → Environment:

```
DATABASE_URL=postgresql://user:password@ep-xxx.neon.tech/neondb?sslmode=require
BETTER_AUTH_SECRET=<generated-secret-32-chars>
BETTER_AUTH_SESSION_DURATION_DAYS=7
FASTAPI_ENV=production
```

#### 2.3 Deploy

1. Click "Create Web Service"
2. Wait for deployment (~3-5 minutes)
3. Note deployed URL: `https://rag-chatbot-backend.onrender.com`

**Verify Backend**:
- Open `https://rag-chatbot-backend.onrender.com/docs`
- Should see Swagger UI with auth endpoints

### Step 3: Deploy Frontend to GitHub Pages

#### 3.1 Update Frontend Config

Edit `frontend/docusaurus.config.js`:

```js
module.exports = {
  // ... existing config
  url: 'https://your-username.github.io',
  baseUrl: '/your-repo/',
  // ... rest of config
};
```

Edit `frontend/.env.production`:

```
REACT_APP_API_URL=https://rag-chatbot-backend.onrender.com
```

#### 3.2 Build and Deploy

```bash
cd frontend
npm run build  # Create production build
npm run deploy  # Deploy to GitHub Pages (if configured)
```

Expected output:
```
> Build success!
> Published to gh-pages branch
> Site live at: https://your-username.github.io/your-repo/
```

**Verify Frontend**:
- Open `https://your-username.github.io/your-repo/`
- Test signup/signin with production backend
- Verify chat personalization works

## Troubleshooting

### Common Issues

#### 1. Backend: "Cannot connect to database"
**Cause**: Invalid DATABASE_URL or Neon database unavailable
**Solution**:
```bash
# Test connection manually
psql $DATABASE_URL -c "SELECT 1;"
# If fails, verify DATABASE_URL in .env matches Neon console
```

#### 2. Frontend: "CORS error when calling backend"
**Cause**: Backend not configured to allow frontend origin
**Solution** (in `backend/src/main.py`):
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "https://your-username.github.io"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

#### 3. Auth: "Session expired" immediately after signin
**Cause**: Frontend not storing session token in cookies
**Solution**: Verify `withCredentials: true` in axios config (`frontend/src/services/api.ts`)

#### 4. Personalization: All responses identical regardless of background
**Cause**: Personalization layer not fetching background or not adjusting prompt
**Solution**:
```bash
# Check logs for background fetch
tail -f backend/logs/app.log | grep "background"
# Should see: "Fetched background for user_id=..."
```

#### 5. Render Deployment: 503 Service Unavailable
**Cause**: Build failed or env variables missing
**Solution**:
1. Check Render logs (Service → Logs tab)
2. Verify all env variables set
3. Re-deploy: Manual Deploy → Deploy latest commit

### Performance Issues

#### Slow Response Times (>5 seconds)
**Diagnosis**:
```bash
# Check database query performance
psql $DATABASE_URL -c "EXPLAIN ANALYZE SELECT * FROM users WHERE email='test@example.com';"
```

**Solutions**:
- Enable connection pooling in Neon (Settings → Connection Pooling)
- Increase backend connection pool: `max_size=20` (in `src/database.py`)
- Cache background profiles (5-minute TTL)

#### Neon Free Tier Connection Limit Exceeded
**Symptoms**: "Too many connections" error
**Solution**:
- Reduce backend connection pool: `max_size=5`
- Enable Neon connection pooling (automatic)
- Upgrade to Neon Pro tier (if traffic warrants)

## Monitoring & Maintenance

### Health Checks

**Backend Health**:
```bash
curl https://rag-chatbot-backend.onrender.com/health
# Expected: {"status": "healthy", "database": "connected"}
```

**Database Health**:
```bash
psql $DATABASE_URL -c "SELECT COUNT(*) FROM users;"
# Expected: (number of registered users)
```

### Logs

**Render Backend Logs**:
- Dashboard → Service → Logs tab
- Real-time streaming, 7-day retention

**Neon Database Logs**:
- Console → Project → Monitoring → Query Performance

### Metrics to Monitor

- **Signup Rate**: Track `COUNT(*) FROM users GROUP BY DATE(created_at)`
- **Session Count**: `SELECT COUNT(*) FROM sessions WHERE expires_at > NOW()`
- **Database Connections**: Neon Console → Monitoring
- **API Latency**: Render → Metrics → Response Time

## Next Steps

1. ✅ Local development working
2. ✅ Production deployed
3. ⏭️ Run `/sp.tasks` to generate implementation task breakdown
4. ⏭️ Implement authentication endpoints (backend)
5. ⏭️ Implement auth UI components (frontend)
6. ⏭️ Implement personalization layer
7. ⏭️ Write comprehensive tests
8. ⏭️ Load testing & performance optimization

## Support & Resources

- **Better Auth Docs**: https://www.better-auth.com/docs
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Neon Docs**: https://neon.tech/docs
- **Render Docs**: https://render.com/docs
- **Feature Spec**: [spec.md](./spec.md)
- **Implementation Plan**: [plan.md](./plan.md)
