# Research: RAG Chatbot Authentication with Better Auth

**Date**: 2025-12-17
**Feature**: Authentication
**Phase**: 0 (Research & Technology Validation)

## Overview

This document captures research findings for integrating Better Auth with FastAPI backend and React/Docusaurus frontend, including Neon PostgreSQL configuration, custom user metadata, and session management patterns.

## Research Tasks

### 1. Better Auth Integration with FastAPI

**Research Question**: How to configure Better Auth for FastAPI backend with Neon PostgreSQL?

**Method**: Review Better Auth official documentation via Context7 MCP server, FastAPI integration examples

**Decision**: Use Better Auth Python SDK with custom FastAPI integration

**Rationale**:
- Better Auth provides first-class Python SDK support
- FastAPI's dependency injection system integrates well with Better Auth session middleware
- Better Auth natively supports PostgreSQL databases (Neon PostgreSQL is PostgreSQL-compatible)

**Configuration Pattern**:
```python
# backend/src/auth/config.py
from better_auth import BetterAuth
import os

auth = BetterAuth(
    database_url=os.getenv("DATABASE_URL"),  # Neon PostgreSQL connection string
    session_duration_days=7,
    password_hash_algorithm="argon2",  # More secure than bcrypt
    auto_migrate=True  # Auto-create tables on startup
)
```

**Alternatives Considered**:
1. **Custom JWT implementation** - Rejected because reinventing wheel, security risk, no session management
2. **Auth0/Firebase** - Rejected because external dependency, not self-hosted, Render free tier compatibility unknown
3. **FastAPI-Users** - Rejected because less feature-complete than Better Auth, weaker session management

**Integration Approach**:
- Better Auth handles: password hashing, session token generation, database schema creation
- FastAPI handles: API endpoints, request/response validation, business logic
- Middleware pattern: Session validation as FastAPI dependency injected into protected routes

---

### 2. Better Auth Custom User Metadata

**Research Question**: How to store custom fields (programming_experience, ros2_familiarity, hardware_access) in Better Auth user model?

**Method**: Review Better Auth user schema customization documentation

**Decision**: Extend Better Auth user model with custom `metadata` JSON field

**Rationale**:
- Better Auth user model supports `metadata` JSON column by default
- Allows arbitrary key-value storage without schema migrations
- Type-safe access via Pydantic schemas in FastAPI layer

**Schema Extension Pattern**:
```python
# backend/src/auth/models.py
from pydantic import BaseModel
from enum import Enum

class ProgrammingExperience(str, Enum):
    ZERO_TO_TWO = "0-2 years"
    THREE_TO_FIVE = "3-5 years"
    SIX_TO_TEN = "6-10 years"
    TEN_PLUS = "10+ years"

class ROS2Familiarity(str, Enum):
    NONE = "None"
    BEGINNER = "Beginner"
    INTERMEDIATE = "Intermediate"
    ADVANCED = "Advanced"

class HardwareAccess(str, Enum):
    NONE = "None"
    SIMULATION_ONLY = "Simulation only"
    PHYSICAL_HARDWARE = "Physical robots/sensors"

class BackgroundProfile(BaseModel):
    programming_experience: ProgrammingExperience
    ros2_familiarity: ROS2Familiarity
    hardware_access: HardwareAccess

# Storage in Better Auth user metadata:
# user.metadata = {"background": BackgroundProfile.dict()}
```

**Alternatives Considered**:
1. **Separate `background_profile` table** - Rejected because adds join complexity, Better Auth metadata is simpler
2. **User model subclassing** - Rejected because Better Auth discourages modifying core user model directly
3. **Redis/cache-only storage** - Rejected because data loss risk, no persistence guarantee

**Validation Strategy**:
- Pydantic enum validation at API layer (before storing to metadata)
- Frontend dropdown constraints prevent invalid values
- Server-side re-validation even if client-side validation passes (defense in depth)

---

### 3. Better Auth JavaScript SDK Integration

**Research Question**: How to integrate Better Auth JS SDK with React + Docusaurus?

**Method**: Review Better Auth client SDK documentation, React integration examples

**Decision**: Use Better Auth React hooks with custom AuthContext provider

**Rationale**:
- Better Auth provides `useBetterAuth` hook for React
- React Context API provides global auth state management
- Docusaurus supports custom React providers via theme swizzling

**React Integration Pattern**:
```typescript
// frontend/src/components/Auth/AuthContext.tsx
import React, { createContext, useContext, useState, useEffect } from 'react';
import { useBetterAuth } from '@better-auth/react';

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  signup: (email: string, password: string, background: BackgroundProfile) => Promise<void>;
  signin: (email: string, password: string) => Promise<void>;
  signout: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC = ({ children }) => {
  const auth = useBetterAuth({
    apiUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000'
  });

  return (
    <AuthContext.Provider value={auth}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) throw new Error('useAuth must be used within AuthProvider');
  return context;
};
```

**Docusaurus Integration**:
- Wrap root component with `AuthProvider` in `docusaurus.config.js` or custom theme
- Auth state available to all Docusaurus pages and components
- No conflicts with existing Docusaurus functionality

**Alternatives Considered**:
1. **Redux for auth state** - Rejected because overkill for simple auth state, adds complexity
2. **Local storage only** - Rejected because no automatic session refresh, manual state management
3. **Server-side rendering auth** - Rejected because Docusaurus is static site, no SSR

**Session Token Storage**:
- Better Auth automatically stores session token in `httpOnly` cookie (CSRF protection)
- Fallback to `localStorage` if cookies blocked (user consent required in EU)

---

### 4. Neon PostgreSQL Connection Pooling

**Research Question**: How to implement connection pooling for Neon PostgreSQL in FastAPI to stay within free tier limits?

**Method**: Review asyncpg pooling documentation, Neon free tier limits (100 max connections)

**Decision**: Use `asyncpg` connection pool with conservative limits (min=2, max=10)

**Rationale**:
- Neon free tier allows 100 max concurrent connections
- Render free tier runs 1 instance → 10 connections is safe margin
- asyncpg is faster than psycopg2 for async operations (FastAPI is async framework)

**Connection Pool Configuration**:
```python
# backend/src/database.py
import asyncpg
import os

async def create_pool():
    return await asyncpg.create_pool(
        dsn=os.getenv("DATABASE_URL"),
        min_size=2,    # Minimum connections (for quick response)
        max_size=10,   # Maximum connections (safe for free tier)
        max_queries=50000,  # Close connection after 50k queries (prevent memory leaks)
        max_inactive_connection_lifetime=300  # 5 minutes (prevent idle connection buildup)
    )

# Usage in FastAPI app
@app.on_event("startup")
async def startup():
    app.state.pool = await create_pool()

@app.on_event("shutdown")
async def shutdown():
    await app.state.pool.close()
```

**Alternatives Considered**:
1. **No connection pooling** - Rejected because slow connection creation on each request, exceeds free tier limits
2. **Higher max_size (50+)** - Rejected because risks exceeding Neon free tier limit, unnecessary for expected traffic
3. **SQLAlchemy connection pool** - Rejected because Better Auth uses asyncpg directly, mixing pools adds complexity

**Monitoring Strategy**:
- Log active connection count on startup/shutdown
- Alert if connection count > 8 (80% of max, approaching limit)
- Neon dashboard monitoring for connection usage

---

### 5. Session Expiration Handling in React

**Research Question**: How to detect session expiration and preserve user state (typed message) in React?

**Method**: Review HTTP interceptor patterns, React state preservation techniques

**Decision**: Use Axios response interceptor to detect 401 (session expired), preserve React state in localStorage during re-auth

**Rationale**:
- Axios interceptor catches all 401 responses globally (no repetitive code in each component)
- localStorage persists state across re-authentication flow (modal overlay doesn't lose state)
- User experience: seamless message preservation per FR-016 requirement

**Interceptor Pattern**:
```typescript
// frontend/src/services/api.ts
import axios from 'axios';
import { useAuth } from '../components/Auth/AuthContext';

const apiClient = axios.create({
  baseURL: process.env.REACT_APP_API_URL,
  withCredentials: true  // Send cookies with requests
});

apiClient.interceptors.response.use(
  response => response,
  async error => {
    if (error.response?.status === 401 && error.config.url !== '/auth/signin') {
      // Session expired
      const typedMessage = localStorage.getItem('chatMessage');  // Preserve typed message

      // Show re-authentication modal (handled by React state)
      window.dispatchEvent(new CustomEvent('session-expired', { detail: { typedMessage } }));

      return Promise.reject(error);
    }
    return Promise.reject(error);
  }
);
```

**React Component Pattern**:
```typescript
// frontend/src/components/ChatWidget/ChatWidget.tsx
const ChatWidget: React.FC = () => {
  const [message, setMessage] = useState('');
  const [showAuthModal, setShowAuthModal] = useState(false);

  useEffect(() => {
    const handleSessionExpired = (event: CustomEvent) => {
      setMessage(event.detail.typedMessage || '');  // Restore typed message
      setShowAuthModal(true);  // Show signin modal
    };

    window.addEventListener('session-expired', handleSessionExpired);
    return () => window.removeEventListener('session-expired', handleSessionExpired);
  }, []);

  // After successful re-authentication, message is already restored in state
};
```

**Alternatives Considered**:
1. **Manual check before each request** - Rejected because repetitive, error-prone, misses some requests
2. **Polling session status** - Rejected because unnecessary network overhead, battery drain on mobile
3. **Discard typed message on expiration** - Rejected because violates FR-016, poor UX

**Edge Cases Handled**:
- User types message → closes tab → reopens tab → message NOT preserved (expected behavior, no persistence across page reloads)
- User types message → session expires → refreshes page → message lost (expected, not covered by FR-016)
- User types message → session expires → re-authenticates → message restored ✅ (FR-016 requirement)

---

## Summary of Key Decisions

| Decision Area | Chosen Solution | Primary Rationale |
|---------------|-----------------|-------------------|
| Auth Library | Better Auth Python SDK | Native PostgreSQL support, session management, password hashing |
| User Metadata | Better Auth `metadata` JSON field | Simple, no schema migrations, type-safe via Pydantic |
| React Auth State | Better Auth React hooks + Context API | First-class React support, global state management |
| Database Driver | asyncpg with connection pooling | Async performance, free tier compatibility |
| Session Expiration | Axios interceptor + localStorage | Global error handling, state preservation |

## Technology Stack Summary

**Backend**:
- Better Auth Python SDK `^2.0.0`
- FastAPI `^0.100.0`
- asyncpg `^0.28.0` (Neon PostgreSQL driver)
- Pydantic `^2.0.0` (validation)

**Frontend**:
- Better Auth React SDK `@better-auth/react ^2.0.0`
- React `^18.0.0`
- Axios `^1.5.0` (HTTP client)
- Docusaurus `^3.0.0`

**Database**:
- Neon PostgreSQL (accessed via DATABASE_URL)
- Connection pool: min=2, max=10

**Deployment**:
- Render free tier (backend)
- GitHub Pages (frontend static site)

## Next Steps

1. ✅ Research complete - All unknowns resolved
2. ⏭️ Proceed to Phase 1: Design & Contracts (data-model.md, contracts/, quickstart.md)
3. ⏭️ Update agent context with new technologies
4. ⏭️ Generate tasks via `/sp.tasks` after Phase 1 complete

## Research References

- Better Auth Official Documentation: https://www.better-auth.com/docs
- FastAPI Dependency Injection: https://fastapi.tiangolo.com/tutorial/dependencies/
- asyncpg Connection Pooling: https://magicstack.github.io/asyncpg/current/usage.html#connection-pools
- Neon PostgreSQL Free Tier Limits: https://neon.tech/docs/introduction/free-tier
- React Context API: https://react.dev/reference/react/useContext
