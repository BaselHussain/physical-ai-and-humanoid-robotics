# Deployment Guide

This guide explains how to deploy the Physical AI & Humanoid Robotics course on both GitHub Pages and Vercel.

## 📁 Project Structure

The Docusaurus site is located in a subdirectory:
```
Project 1/
└── physical-robotics-ai-book/    ← Docusaurus site
    ├── docs/
    ├── src/
    ├── package.json
    ├── docusaurus.config.ts
    └── vercel.json
```

## 🚀 Deployment Platforms

### GitHub Pages

**Automatic deployment via GitHub Actions**

1. **Enable GitHub Pages**
   - Go to: https://github.com/BaselHussain/physical-ai-and-humanoid-robotics/settings/pages
   - Under **"Build and deployment"**:
     - Source: Select **"GitHub Actions"**
   - Click **Save**

2. **Workflow runs automatically**
   - Triggers on every push to `main`
   - Uses Node.js 20
   - Sets `baseUrl: /physical-ai-and-humanoid-robotics/`

3. **Live URL**
   ```
   https://BaselHussain.github.io/physical-ai-and-humanoid-robotics/
   ```

### Vercel

**Manual configuration required (one-time setup)**

#### Step 1: Import Project

1. Go to https://vercel.com/new
2. Import repository: `BaselHussain/physical-ai-and-humanoid-robotics`
3. Click **Continue**

#### Step 2: Configure Build Settings

⚠️ **CRITICAL**: Set the Root Directory because the Docusaurus site is in a subdirectory!

```
Framework Preset: Docusaurus 2
Root Directory: physical-robotics-ai-book    ← IMPORTANT!
Build Command: npm run build
Output Directory: build
Install Command: npm install
Node.js Version: 20.x
```

#### Step 3: Environment Variables

Vercel should auto-detect from `vercel.json`, but verify these are set:

```
DOCUSAURUS_URL = https://physical-ai-and-humanoid-robotics-ten.vercel.app
DOCUSAURUS_BASE_URL = /
NODE_VERSION = 20
```

#### Step 4: Deploy

- Click **Deploy**
- Wait for build completion

#### Step 5: Verify

Your site will be at:
```
https://physical-ai-and-humanoid-robotics-ten.vercel.app/
```

## 🔧 Configuration Details

### Environment Variables

The project uses environment-based configuration to support both platforms:

- **GitHub Pages**: Uses `/physical-ai-and-humanoid-robotics/` base path
- **Vercel**: Uses `/` base path

This is handled automatically via:
- `.github/workflows/deploy.yml` (GitHub Pages)
- `vercel.json` (Vercel)
- `docusaurus.config.ts` (reads environment variables)

### Node.js Version

Both platforms require **Node.js 20+**:
- GitHub Actions: Specified in workflow file
- Vercel: Specified in `vercel.json` and project settings

## 🐛 Troubleshooting

### 404 Error on GitHub Pages

**Cause**: Base URL mismatch

**Fix**: Ensure GitHub Actions workflow sets:
```yaml
env:
  DOCUSAURUS_URL: https://BaselHussain.github.io
  DOCUSAURUS_BASE_URL: /physical-ai-and-humanoid-robotics/
```

### 404 Error on Vercel

**Cause**: Root directory not set or base URL incorrect

**Fix**:
1. Check Vercel project settings
2. Ensure **Root Directory** = `physical-robotics-ai-book`
3. Verify `vercel.json` has `DOCUSAURUS_BASE_URL: "/"`

### Build Fails with Node.js Error

**Error**: `Minimum Node.js version not met`

**Fix**:
- GitHub Actions: Update `node-version: 20` in workflow
- Vercel: Set Node.js version to 20.x in project settings

### Assets Not Loading

**Cause**: Incorrect base URL

**Fix**: Clear browser cache and verify environment variables

## 📝 Local Development

For local development (optional):

```bash
cd physical-robotics-ai-book
npm install
npm start
```

Local dev uses:
```
URL: http://localhost:3000
Base URL: /
```

## 🔄 Auto-Deployment

Both platforms auto-deploy on push to `main`:
- **GitHub Pages**: Via GitHub Actions workflow
- **Vercel**: Via Git integration

## 📚 Documentation

- Docusaurus: https://docusaurus.io/docs/deployment
- GitHub Pages: https://docs.github.com/pages
- Vercel: https://vercel.com/docs
