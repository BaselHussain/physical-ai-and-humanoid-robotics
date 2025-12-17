"""
Authentication Configuration for RAG Chatbot

Uses FastAPI-Users with Neon PostgreSQL (asyncpg driver).
Session duration: 7 days
Password hashing: Argon2id
"""

import os
from typing import AsyncGenerator
from fastapi import Depends
from fastapi_users.db import SQLAlchemyUserDatabase
from sqlalchemy.ext.asyncio import AsyncSession, async_sessionmaker, create_async_engine

# Load DATABASE_URL from environment (check both DATABASE_URL and NEON_DATABASE_URL)
DATABASE_URL = os.getenv("DATABASE_URL") or os.getenv("NEON_DATABASE_URL")
if not DATABASE_URL:
    raise ValueError("DATABASE_URL or NEON_DATABASE_URL environment variable not set. Please configure Neon PostgreSQL connection string in .env file.")

# Convert postgres:// to postgresql+asyncpg:// for asyncpg driver
if DATABASE_URL.startswith("postgres://"):
    DATABASE_URL = DATABASE_URL.replace("postgres://", "postgresql+asyncpg://", 1)
elif DATABASE_URL.startswith("postgresql://"):
    DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)

# Remove sslmode query parameter (asyncpg doesn't support it, SSL is default for neon.tech)
if "?sslmode=" in DATABASE_URL or "&sslmode=" in DATABASE_URL:
    import re
    DATABASE_URL = re.sub(r'[?&]sslmode=\w+', '', DATABASE_URL)

# Create async engine with connection pooling (Neon cloud database optimized)
engine = create_async_engine(
    DATABASE_URL,
    pool_size=5,               # Smaller pool for Neon free tier (avoid exhaustion)
    max_overflow=2,            # Allow 2 extra connections during spikes
    pool_pre_ping=True,        # Verify connection before using (detect stale connections)
    pool_recycle=300,          # Recycle connections after 5 minutes (Neon timeout prevention)
    pool_timeout=30,           # Wait up to 30 seconds for a connection from pool
    connect_args={
        "timeout": 10,         # Connection timeout: 10 seconds
        "command_timeout": 10,  # Command timeout: 10 seconds
        "server_settings": {
            "application_name": "rag_chatbot_auth"
        }
    },
    echo=False,                # Disable SQL logging in production
    pool_use_lifo=True,        # Use LIFO to reuse recent connections (better for cloud DBs)
)

# Create async session maker
async_session_maker = async_sessionmaker(engine, expire_on_commit=False)


async def get_async_session() -> AsyncGenerator[AsyncSession, None]:
    """
    Dependency to get async database session.
    Used by FastAPI-Users for database operations.
    """
    async with async_session_maker() as session:
        yield session


async def get_user_db(session: AsyncSession = Depends(get_async_session)):
    """
    Dependency to get FastAPI-Users database adapter.
    Provides user CRUD operations with custom User model.
    """
    from .models import User  # Import here to avoid circular dependency
    yield SQLAlchemyUserDatabase(session, User)


# Authentication configuration constants
SESSION_DURATION_DAYS = 7  # FR-015: 7-day session duration
PASSWORD_HASH_ALGORITHM = "argon2"  # Secure password hashing (better than bcrypt)
