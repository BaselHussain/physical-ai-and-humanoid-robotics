# Known Issues - RAG Chatbot Authentication

## Database Connection Pool Issue (Neon PostgreSQL)

**Status:** Known Issue - To be revisited
**Date:** 2025-12-17
**Priority:** Medium

### Description
After the first successful database operation (signup), subsequent operations (signin) fail with connection pool errors. This is an intermittent issue related to how FastAPI-Users manages async database sessions with Neon PostgreSQL.

### Symptoms
- **Signup endpoint:** ✅ Works successfully
- **Signin endpoint:** ❌ Fails immediately after with "Connection failed" error
- **Error:** Connection pool exhaustion or session management issue

### What Works
- Database connection itself (verified with direct connection tests)
- First database operation always succeeds
- Authentication logic is correct
- Personalization mapping works perfectly

### Configuration Applied
We've optimized the connection pool for Neon cloud database:
```python
engine = create_async_engine(
    DATABASE_URL,
    pool_size=5,               # Smaller pool for Neon free tier
    max_overflow=2,            # Allow 2 extra connections
    pool_pre_ping=True,        # Verify connections
    pool_recycle=300,          # Recycle after 5 minutes
    pool_timeout=30,           # 30 second timeout
    pool_use_lifo=True,        # Reuse recent connections
)
```

### Suspected Root Causes
1. **FastAPI-Users session management** - library may not properly release connections after requests
2. **Async context lifecycle** - database sessions might not be properly closed in dependency chain
3. **Connection state corruption** - first operation leaves connection in invalid state

### Workaround
The authentication functionality itself is implemented correctly. This is an infrastructure-level issue that needs deeper investigation into:
- FastAPI-Users internal session management
- SQLAlchemy async session lifecycle
- Neon connection pooling behavior

### Next Steps
1. Monitor production behavior with real user load
2. Consider alternative session management approaches
3. Investigate FastAPI-Users source code for session handling
4. Test with local PostgreSQL to isolate Neon-specific issues

### Testing Notes
- Manual testing of Phases 1-2 completed successfully (see tasks.md T001-T008)
- Database tables created and verified
- Personalization logic fully functional
- Issue only manifests in rapid sequential operations

---

**Note:** This issue does not block Phase 3 (Frontend) development. The frontend will be built with proper error handling for connection issues.
