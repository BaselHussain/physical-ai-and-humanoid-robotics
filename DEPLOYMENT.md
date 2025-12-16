# Deployment Guide - RAG Chatbot to Render

Complete guide to deploy your FastAPI backend to Render and configure the frontend.

## Prerequisites

Make sure you have:
- ✅ Neon Postgres database set up
- ✅ Qdrant Cloud account with collection created
- ✅ Cohere API key
- ✅ Gemini API key
- ✅ GitHub account
- ✅ Render account (sign up at render.com)

---

## Step 1: Push Code to GitHub

```bash
git add .
git commit -m "feat: Add Render deployment configuration"
git push origin main
```

---

## Step 2: Deploy Backend to Render

### Option A: Deploy via render.yaml (Recommended)

1. Go to https://dashboard.render.com/
2. Click "New +" → "Blueprint"
3. Connect your GitHub repository
4. Render will automatically detect `render.yaml`
5. Click "Apply"

### Option B: Manual Deployment

1. Go to https://dashboard.render.com/
2. Click "New +" → "Web Service"
3. Connect your GitHub repository
4. Configure:
   - **Name**: `rag-chatbot-backend`
   - **Runtime**: Python 3
   - **Root Directory**: `backend`
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn main:app --host 0.0.0.0 --port $PORT`

---

## Step 3: Configure Environment Variables on Render

Go to your service → "Environment" tab → Add these variables:

```bash
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
COHERE_API_KEY=your_cohere_api_key
GEMINI_API_KEY=your_gemini_api_key
NEON_DATABASE_URL=postgresql://user:password@host/database?sslmode=require
COLLECTION_NAME=physical-robotics-ai-book
```

**Important**: Get these values from:
- Qdrant: https://cloud.qdrant.io/
- Cohere: https://dashboard.cohere.com/api-keys
- Gemini: https://ai.google.dev/
- Neon: https://console.neon.tech/

---

## Step 4: Wait for Deployment

Render will:
1. Build your service (install dependencies)
2. Start the service with uvicorn
3. Assign a URL like: `https://rag-chatbot-backend.onrender.com`

**First deploy takes ~5-10 minutes**

Check deployment logs for any errors.

---

## Step 5: Test Backend API

Once deployed, test these endpoints:

### Health Check
```bash
curl https://rag-chatbot-backend.onrender.com/
```
Expected: `{"message": "RAG Chatbot API is running"}`

### Chat Session
```bash
curl https://rag-chatbot-backend.onrender.com/api/session
```
Expected: `{"session_id": "uuid-here"}`

### Debug Sessions
```bash
curl https://rag-chatbot-backend.onrender.com/api/debug/sessions
```
Expected: JSON with sessions array

---

## Step 6: Update Frontend API URL

### For Local Development (keep localhost)
File: `physical-robotics-ai-book/src/components/ChatKitWidget.tsx`

```typescript
const { control, setThreadId, setComposerValue } = useChatKit({
  api: {
    url: 'http://localhost:8000/chatkit',  // Keep for local dev
    domainKey: 'localhost',
  },
  // ... rest of config
});
```

### For Production (use environment variable)
Create: `physical-robotics-ai-book/.env.production`

```bash
REACT_APP_API_URL=https://rag-chatbot-backend.onrender.com
```

Update `ChatKitWidget.tsx`:
```typescript
const API_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const { control, setThreadId, setComposerValue } = useChatKit({
  api: {
    url: `${API_URL}/chatkit`,
    domainKey: API_URL.includes('localhost') ? 'localhost' : 'production',
  },
  // ... rest of config
});
```

---

## Step 7: Configure CORS on Backend

Make sure `backend/main.py` has proper CORS setup:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://your-github-username.github.io",
        "https://rag-chatbot-backend.onrender.com"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

Commit and push changes - Render will auto-redeploy.

---

## Step 8: Deploy Frontend to GitHub Pages (Optional)

If you want to deploy the Docusaurus site:

1. Update `docusaurus.config.ts`:
```typescript
const config: Config = {
  url: 'https://your-username.github.io',
  baseUrl: '/your-repo-name/',
  organizationName: 'your-username',
  projectName: 'your-repo-name',
  // ... rest
};
```

2. Deploy:
```bash
cd physical-robotics-ai-book
npm run deploy
```

---

## Step 9: Configure GitHub Actions Secrets

For auto-reindex on docs changes:

1. Go to GitHub repo → Settings → Secrets and variables → Actions
2. Add these secrets:
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`

Now when you push new docs to `main`, GitHub Actions will automatically reindex.

---

## Troubleshooting

### Backend not starting
- Check Render logs: Dashboard → your service → Logs
- Verify all environment variables are set
- Check that `requirements.txt` has all dependencies

### CORS errors in frontend
- Add your frontend URL to `allow_origins` in `backend/main.py`
- Redeploy backend after CORS changes

### Database connection errors
- Verify `NEON_DATABASE_URL` format: `postgresql://user:password@host/database?sslmode=require`
- Check Neon database is active (not paused)

### Rate limit errors
- Cohere free tier has limits
- Check Cohere dashboard for usage
- Exponential backoff will retry automatically

### Cold starts (Free tier)
- Render free tier spins down after 15 minutes of inactivity
- First request after sleep takes ~30-60 seconds
- Upgrade to paid tier for always-on service

---

## Production Checklist

- [ ] Backend deployed to Render
- [ ] All environment variables configured
- [ ] Backend health check returns 200 OK
- [ ] CORS configured for your frontend domain
- [ ] Frontend updated with production API URL
- [ ] GitHub Actions secrets configured
- [ ] Test chat widget on live site
- [ ] Test text selection → Ask from AI
- [ ] Verify chat history persists in Neon database
- [ ] Monitor Render logs for errors

---

## Monitoring

### Check Backend Status
```bash
curl https://rag-chatbot-backend.onrender.com/
```

### Check Database Sessions
```bash
curl https://rag-chatbot-backend.onrender.com/api/debug/sessions
```

### Check Render Logs
Dashboard → your service → Logs (real-time)

---

## Costs (Free Tier)

- **Render**: Free (spins down after 15 min inactivity)
- **Neon**: Free (0.5GB storage, 3GB data transfer/month)
- **Qdrant Cloud**: Free (1GB storage, 100k vectors)
- **Cohere**: Free trial credits
- **Gemini**: Free tier available

**Upgrade if you need:**
- Always-on backend: Render $7/month
- More storage: Neon $19/month
- More vectors: Qdrant paid plans
- Production Cohere usage: Pay-as-you-go

---

## Next Steps

1. Deploy backend to Render (follow steps above)
2. Test all endpoints
3. Update frontend with production URL
4. Deploy frontend to GitHub Pages (optional)
5. Monitor logs and usage
6. Add new chapters → auto-reindex via GitHub Actions

Your RAG chatbot is now production-ready!