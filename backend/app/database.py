"""
Neon Postgres Database Connection Pool
Manages asyncpg connection pooling for chat history persistence
"""

import os
import asyncpg
from typing import Optional
import logging

logger = logging.getLogger(__name__)

# Global connection pool
_pool: Optional[asyncpg.Pool] = None


async def get_pool() -> asyncpg.Pool:
    """
    Get or create the database connection pool.

    Returns:
        asyncpg.Pool: Database connection pool

    Raises:
        ValueError: If NEON_DATABASE_URL environment variable is not set
        asyncpg.PostgresError: If connection fails
    """
    global _pool

    if _pool is None:
        database_url = os.getenv("NEON_DATABASE_URL")

        if not database_url:
            raise ValueError(
                "NEON_DATABASE_URL environment variable not set. "
                "Please add it to backend/.env file"
            )

        # Validate connection string format
        if not database_url.startswith("postgresql://") and not database_url.startswith("postgres://"):
            raise ValueError(
                "NEON_DATABASE_URL must start with 'postgresql://' or 'postgres://'. "
                f"Got: {database_url[:20]}..."
            )

        logger.info("Initializing Neon Postgres connection pool...")

        try:
            _pool = await asyncpg.create_pool(
                dsn=database_url,
                min_size=2,  # Minimum number of connections in pool
                max_size=10,  # Maximum number of connections in pool
                command_timeout=60,  # 60 second timeout for commands
                server_settings={
                    'application_name': 'rag-chatbot-backend'
                }
            )

            # Test connection
            async with _pool.acquire() as conn:
                await conn.fetchval("SELECT 1")

            logger.info("✅ Neon Postgres connection pool initialized successfully")

        except asyncpg.PostgresError as e:
            logger.error(f"❌ Failed to connect to Neon Postgres: {e}")
            raise
        except Exception as e:
            logger.error(f"❌ Unexpected error initializing database pool: {e}")
            raise

    return _pool


async def close_pool():
    """
    Close the database connection pool gracefully.
    Should be called on application shutdown.
    """
    global _pool

    if _pool is not None:
        logger.info("Closing Neon Postgres connection pool...")
        await _pool.close()
        _pool = None
        logger.info("✅ Connection pool closed")


async def health_check() -> bool:
    """
    Perform a health check on the database connection.

    Returns:
        bool: True if database is accessible, False otherwise
    """
    try:
        pool = await get_pool()
        async with pool.acquire() as conn:
            await conn.fetchval("SELECT 1")
        return True
    except Exception as e:
        logger.error(f"Database health check failed: {e}")
        return False


async def get_connection():
    """
    Get a database connection from the pool (for use in 'async with' blocks).

    Example:
        async with get_connection() as conn:
            result = await conn.fetch("SELECT * FROM chat_sessions")

    Returns:
        asyncpg.connection.Connection: Database connection
    """
    pool = await get_pool()
    return pool.acquire()
