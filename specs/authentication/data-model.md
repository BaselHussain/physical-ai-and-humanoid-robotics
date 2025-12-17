# Data Model: RAG Chatbot Authentication

**Date**: 2025-12-17
**Feature**: Authentication
**Phase**: 1 (Design & Contracts)
**Database**: Neon PostgreSQL

## Overview

This document defines the data model for Better Auth authentication with custom user background profiles. All entities are stored in Neon PostgreSQL database accessed via DATABASE_URL environment variable.

## Entity-Relationship Diagram

```
┌─────────────────────────┐
│        User             │
├─────────────────────────┤
│ id (UUID, PK)           │
│ email (STRING, UNIQUE)  │
│ password_hash (STRING)  │
│ metadata (JSONB)        │  ← Contains BackgroundProfile
│ created_at (TIMESTAMP)  │
│ updated_at (TIMESTAMP)  │
└─────────────────────────┘
            │
            │ 1:N
            ▼
┌─────────────────────────┐
│       Session           │
├─────────────────────────┤
│ id (UUID, PK)           │
│ user_id (UUID, FK)      │
│ token (STRING, UNIQUE)  │
│ expires_at (TIMESTAMP)  │
│ created_at (TIMESTAMP)  │
└─────────────────────────┘
```

## Entities

### 1. User

**Description**: Represents an authenticated user account managed by Better Auth.

**Table**: `users` (created by Better Auth auto-migration)

**Schema**:

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, NOT NULL, DEFAULT uuid_generate_v4() | Unique user identifier |
| `email` | VARCHAR(255) | UNIQUE, NOT NULL, INDEX | User email address (used for login) |
| `password_hash` | VARCHAR(255) | NOT NULL | Argon2 hashed password (Better Auth managed) |
| `metadata` | JSONB | DEFAULT '{}' | Custom user data (contains BackgroundProfile) |
| `created_at` | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW() | Account creation timestamp |
| `updated_at` | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW() | Last update timestamp |

**Indexes**:
- `idx_users_email` (B-tree on `email`) - For fast login lookups
- `idx_users_metadata` (GIN on `metadata`) - For JSON field queries (if needed)

**Constraints**:
- `email` must match RFC 5322 email format (validated at application layer)
- `password_hash` must be Argon2 hash (enforced by Better Auth)
- `metadata.background` must conform to BackgroundProfile schema (validated at application layer)

**Example Row**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "user@example.com",
  "password_hash": "$argon2id$v=19$m=65536,t=3,p=4$...",
  "metadata": {
    "background": {
      "programming_experience": "6-10 years",
      "ros2_familiarity": "Advanced",
      "hardware_access": "Physical robots/sensors"
    }
  },
  "created_at": "2025-12-17T10:00:00Z",
  "updated_at": "2025-12-17T10:00:00Z"
}
```

---

### 2. BackgroundProfile (Embedded in User.metadata)

**Description**: User's technical background for RAG chatbot personalization. Stored as JSON within `users.metadata.background` field.

**Storage**: JSONB field in `users` table (not separate table)

**Schema** (Pydantic model for validation):

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `programming_experience` | Enum(str) | NOT NULL, ONE OF: "0-2 years", "3-5 years", "6-10 years", "10+ years" | Years of programming experience |
| `ros2_familiarity` | Enum(str) | NOT NULL, ONE OF: "None", "Beginner", "Intermediate", "Advanced" | ROS 2 / robotics familiarity level |
| `hardware_access` | Enum(str) | NOT NULL, ONE OF: "None", "Simulation only", "Physical robots/sensors" | Access to physical hardware |

**Validation Rules**:
- All fields REQUIRED (per FR-002a)
- Values MUST be from predefined enum options (enforced by frontend dropdowns + backend Pydantic validation)
- No free text input allowed (prevents malformed data per clarifications)

**Mapping to Expertise Level** (for personalization):

| Programming Experience | ROS 2 Familiarity | Derived Expertise Level |
|------------------------|-------------------|-------------------------|
| "0-2 years" | "None" or "Beginner" | **Beginner** |
| "3-5 years" | "None" or "Beginner" | **Beginner** |
| "3-5 years" | "Intermediate" | **Intermediate** |
| "6-10 years" | Any | **Intermediate** |
| "10+ years" | "Intermediate" or "Advanced" | **Advanced** |
| "10+ years" | "None" or "Beginner" | **Intermediate** (edge case: experienced programmer but new to ROS 2) |

**Example JSON**:
```json
{
  "programming_experience": "6-10 years",
  "ros2_familiarity": "Advanced",
  "hardware_access": "Physical robots/sensors"
}
```

---

### 3. Session

**Description**: Active user session with expiration. Managed by Better Auth.

**Table**: `sessions` (created by Better Auth auto-migration)

**Schema**:

| Column | Type | Constraints | Description |
|--------|------|-------------|-------------|
| `id` | UUID | PRIMARY KEY, NOT NULL, DEFAULT uuid_generate_v4() | Unique session identifier |
| `user_id` | UUID | FOREIGN KEY (users.id) ON DELETE CASCADE, NOT NULL, INDEX | Reference to user |
| `token` | VARCHAR(255) | UNIQUE, NOT NULL, INDEX | Session token (sent in cookies/Authorization header) |
| `expires_at` | TIMESTAMP WITH TIME ZONE | NOT NULL | Session expiration timestamp (created_at + 7 days) |
| `created_at` | TIMESTAMP WITH TIME ZONE | NOT NULL, DEFAULT NOW() | Session creation timestamp |

**Indexes**:
- `idx_sessions_token` (B-tree on `token`) - For fast token validation
- `idx_sessions_user_id` (B-tree on `user_id`) - For user session lookups
- `idx_sessions_expires_at` (B-tree on `expires_at`) - For expired session cleanup

**Constraints**:
- `user_id` MUST reference existing user (Foreign Key)
- `token` MUST be cryptographically secure random string (Better Auth managed)
- `expires_at` = `created_at` + 7 days (per FR-015)

**Lifecycle**:
1. **Creation**: POST /auth/signup or POST /auth/signin → INSERT session with 7-day expiration
2. **Validation**: Every authenticated request → SELECT session WHERE token = ? AND expires_at > NOW()
3. **Expiration**: Automatic cleanup via cron job or lazy deletion on validation failure
4. **Deletion**: POST /auth/signout → DELETE session WHERE token = ?

**Example Row**:
```json
{
  "id": "660e9500-f39c-52e5-b827-557766551111",
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "token": "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
  "expires_at": "2025-12-24T10:00:00Z",  // created_at + 7 days
  "created_at": "2025-12-17T10:00:00Z"
}
```

---

## Database Migrations

**Migration Strategy**: Better Auth auto-migration enabled (configured in research.md)

**Initial Migration** (auto-generated by Better Auth on first startup):
```sql
-- Better Auth auto-migration creates:
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL
);

CREATE TABLE sessions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    token VARCHAR(255) UNIQUE NOT NULL,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW() NOT NULL
);

CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_sessions_token ON sessions(token);
CREATE INDEX idx_sessions_user_id ON sessions(user_id);
CREATE INDEX idx_sessions_expires_at ON sessions(expires_at);
```

**Future Migrations** (if schema changes needed):
- Better Auth supports versioned migrations
- Run `better-auth migrate` CLI command to generate migration files
- Commit migration files to version control
- Apply migrations during deployment

---

## Data Access Patterns

### High-Frequency Queries

1. **Session Validation** (every authenticated request):
   ```sql
   SELECT s.id, s.user_id, u.metadata->>'background' AS background
   FROM sessions s
   JOIN users u ON s.user_id = u.id
   WHERE s.token = $1 AND s.expires_at > NOW()
   LIMIT 1;
   ```
   - **Frequency**: 100+ requests/second (for active chat sessions)
   - **Optimization**: Index on `sessions.token`, consider caching (5-minute TTL)

2. **User Signup** (new user creation):
   ```sql
   INSERT INTO users (email, password_hash, metadata)
   VALUES ($1, $2, $3)
   RETURNING id, email, metadata;
   ```
   - **Frequency**: 10-50 signups/day (expected)
   - **Optimization**: None needed (infrequent operation)

3. **User Signin** (existing user authentication):
   ```sql
   SELECT id, password_hash, metadata
   FROM users
   WHERE email = $1
   LIMIT 1;
   ```
   - **Frequency**: 50-200 signins/day
   - **Optimization**: Index on `users.email` (already exists)

### Background Profile Fetch

**For Personalization** (on every chat request):
```sql
SELECT metadata->>'background' AS background
FROM users
WHERE id = $1;
```
- **Frequency**: Same as session validation (100+ requests/second)
- **Optimization**: Cache in-memory (5-minute TTL, invalidate on profile update)

---

## Performance Considerations

### Connection Pooling
- **Configuration**: min=2, max=10 connections (per research.md)
- **Rationale**: Neon free tier allows 100 max connections, Render free tier runs 1 instance
- **Monitoring**: Alert if active connections > 8 (80% capacity)

### Query Optimization
1. **Session validation**: Use prepared statements to avoid query parsing overhead
2. **Background fetch**: Combine with session validation query (single JOIN instead of 2 queries)
3. **Expired session cleanup**: Run cron job daily at low-traffic hours (3 AM UTC)

### Caching Strategy
- **Session tokens**: Cache valid sessions in-memory (TTL: 5 minutes, max 1000 entries)
- **Background profiles**: Cache per user_id (TTL: 5 minutes, invalidate on profile update)
- **Cache invalidation**: On user profile update or session deletion

---

## Security Considerations

### Password Storage
- **Algorithm**: Argon2id (Better Auth default per research.md)
- **Parameters**: m=65536, t=3, p=4 (Better Auth managed, industry standard)
- **Salt**: Unique random salt per password (Better Auth managed)

### Session Tokens
- **Format**: 64-character hex string (256 bits entropy)
- **Generation**: Cryptographically secure random (Better Auth managed)
- **Storage**: Database only (not in application logs or client-side storage except httpOnly cookies)

### SQL Injection Prevention
- **Method**: Parameterized queries via asyncpg (all values bound as parameters, never concatenated)
- **Validation**: Pydantic models validate all inputs before database operations

### Data Exposure
- **Password hashes**: Never returned in API responses
- **Session tokens**: Only sent in httpOnly cookies (JavaScript cannot access)
- **Background profiles**: Only returned for authenticated user's own profile

---

## Data Retention & Cleanup

### Expired Sessions
- **Cleanup Schedule**: Daily cron job at 3 AM UTC
- **Query**: `DELETE FROM sessions WHERE expires_at < NOW()`
- **Impact**: Minimal (auto-indexed on expires_at for fast deletes)

### User Deletion
- **Cascade**: DELETE user → CASCADE DELETE all sessions (enforced by FK constraint)
- **Future**: Soft delete (add `deleted_at` column) if user data retention required

### Backup & Recovery
- **Neon PostgreSQL**: Automatic point-in-time recovery (PITR) included in free tier
- **Retention**: 7 days of transaction history
- **Manual Backups**: Use `pg_dump` for critical snapshots before major migrations

---

## Testing Data

### Test Users (for E2E testing)

```json
// Beginner user
{
  "email": "beginner@test.com",
  "password": "Test1234!",
  "background": {
    "programming_experience": "0-2 years",
    "ros2_familiarity": "None",
    "hardware_access": "None"
  }
}

// Intermediate user
{
  "email": "intermediate@test.com",
  "password": "Test1234!",
  "background": {
    "programming_experience": "6-10 years",
    "ros2_familiarity": "Intermediate",
    "hardware_access": "Simulation only"
  }
}

// Advanced user
{
  "email": "advanced@test.com",
  "password": "Test1234!",
  "background": {
    "programming_experience": "10+ years",
    "ros2_familiarity": "Advanced",
    "hardware_access": "Physical robots/sensors"
  }
}
```

---

## Next Steps

1. ✅ Data model defined
2. ⏭️ Generate API contracts (contracts/)
3. ⏭️ Generate quickstart.md
4. ⏭️ Implement database models in `backend/src/auth/models.py`
5. ⏭️ Run Better Auth auto-migration to create tables
