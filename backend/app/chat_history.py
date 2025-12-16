"""
Chat History CRUD Operations
Provides database operations for chat sessions and messages
"""

import uuid
from datetime import datetime
from typing import List, Optional, Dict, Any
import logging
import json

from .database import get_pool

logger = logging.getLogger(__name__)


class ChatMessage:
    """Represents a chat message"""
    def __init__(
        self,
        message_id: uuid.UUID,
        session_id: uuid.UUID,
        message_text: str,
        role: str,
        timestamp: datetime,
        source_references: List[Dict[str, Any]] = None
    ):
        self.message_id = message_id
        self.session_id = session_id
        self.message_text = message_text
        self.role = role
        self.timestamp = timestamp
        self.source_references = source_references or []

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization"""
        return {
            "message_id": str(self.message_id),
            "session_id": str(self.session_id),
            "message_text": self.message_text,
            "role": self.role,
            "timestamp": self.timestamp.isoformat(),
            "source_references": self.source_references
        }


class ChatSession:
    """Represents a chat session"""
    def __init__(
        self,
        session_id: uuid.UUID,
        user_identifier: str,
        created_at: datetime,
        last_active_at: datetime
    ):
        self.session_id = session_id
        self.user_identifier = user_identifier
        self.created_at = created_at
        self.last_active_at = last_active_at

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for JSON serialization"""
        return {
            "session_id": str(self.session_id),
            "user_identifier": self.user_identifier,
            "created_at": self.created_at.isoformat(),
            "last_active_at": self.last_active_at.isoformat()
        }


async def create_session(user_identifier: str) -> str:
    """
    Create a new chat session in the database.

    Args:
        user_identifier: Unique identifier for the user (e.g., browser fingerprint, UUID from localStorage)

    Returns:
        str: The UUID of the created session

    Raises:
        asyncpg.PostgresError: If database operation fails
    """
    try:
        pool = await get_pool()

        async with pool.acquire() as conn:
            session_id = await conn.fetchval(
                """
                INSERT INTO chat_sessions (user_identifier, created_at, last_active_at)
                VALUES ($1, NOW(), NOW())
                RETURNING session_id
                """,
                user_identifier
            )

        logger.info(f"✅ Created new chat session: {session_id} for user: {user_identifier}")
        return str(session_id)

    except Exception as e:
        logger.error(f"❌ Failed to create session: {e}")
        raise


async def get_session(session_id: str) -> Optional[ChatSession]:
    """
    Retrieve a chat session by ID.

    Args:
        session_id: UUID of the session

    Returns:
        ChatSession: The session object, or None if not found

    Raises:
        asyncpg.PostgresError: If database operation fails
    """
    try:
        pool = await get_pool()

        async with pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT session_id, user_identifier, created_at, last_active_at
                FROM chat_sessions
                WHERE session_id = $1
                """,
                uuid.UUID(session_id)
            )

        if row is None:
            logger.warning(f"⚠️ Session not found: {session_id}")
            return None

        return ChatSession(
            session_id=row['session_id'],
            user_identifier=row['user_identifier'],
            created_at=row['created_at'],
            last_active_at=row['last_active_at']
        )

    except Exception as e:
        logger.error(f"❌ Failed to get session {session_id}: {e}")
        raise


async def save_message(
    session_id: str,
    message_text: str,
    role: str,
    source_references: List[Dict[str, Any]] = None
) -> str:
    """
    Save a chat message to the database.

    Args:
        session_id: UUID of the session
        message_text: The message content
        role: 'user' or 'assistant'
        source_references: List of source references (for assistant messages)

    Returns:
        str: The UUID of the created message

    Raises:
        asyncpg.PostgresError: If database operation fails
        ValueError: If role is invalid
    """
    if role not in ['user', 'assistant']:
        raise ValueError(f"Invalid role: {role}. Must be 'user' or 'assistant'")

    try:
        pool = await get_pool()

        # Convert source_references to JSON
        source_refs_json = json.dumps(source_references or [])

        async with pool.acquire() as conn:
            # Use a transaction to ensure both operations succeed or fail together
            async with conn.transaction():
                # Insert message
                message_id = await conn.fetchval(
                    """
                    INSERT INTO chat_messages (session_id, message_text, role, timestamp, source_references)
                    VALUES ($1, $2, $3, NOW(), $4::jsonb)
                    RETURNING message_id
                    """,
                    uuid.UUID(session_id),
                    message_text,
                    role,
                    source_refs_json
                )

                # Update session last_active_at
                await conn.execute(
                    """
                    UPDATE chat_sessions
                    SET last_active_at = NOW()
                    WHERE session_id = $1
                    """,
                    uuid.UUID(session_id)
                )

        logger.info(f"✅ Saved {role} message {message_id} to session {session_id}")
        return str(message_id)

    except Exception as e:
        logger.error(f"❌ Failed to save message to session {session_id}: {e}")
        raise


async def get_session_history(session_id: str) -> List[ChatMessage]:
    """
    Retrieve all messages for a session in chronological order.

    Args:
        session_id: UUID of the session

    Returns:
        List[ChatMessage]: List of messages ordered by timestamp

    Raises:
        asyncpg.PostgresError: If database operation fails
    """
    try:
        pool = await get_pool()

        async with pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT message_id, session_id, message_text, role, timestamp, source_references
                FROM chat_messages
                WHERE session_id = $1
                ORDER BY timestamp ASC
                """,
                uuid.UUID(session_id)
            )

        messages = []
        for row in rows:
            # Parse JSONB source_references
            source_refs = row['source_references'] if row['source_references'] else []

            messages.append(ChatMessage(
                message_id=row['message_id'],
                session_id=row['session_id'],
                message_text=row['message_text'],
                role=row['role'],
                timestamp=row['timestamp'],
                source_references=source_refs
            ))

        logger.info(f"✅ Retrieved {len(messages)} messages for session {session_id}")
        return messages

    except Exception as e:
        logger.error(f"❌ Failed to get session history for {session_id}: {e}")
        raise


async def update_last_active(session_id: str) -> None:
    """
    Update the last_active_at timestamp for a session.

    Args:
        session_id: UUID of the session

    Raises:
        asyncpg.PostgresError: If database operation fails
    """
    try:
        pool = await get_pool()

        async with pool.acquire() as conn:
            result = await conn.execute(
                """
                UPDATE chat_sessions
                SET last_active_at = NOW()
                WHERE session_id = $1
                """,
                uuid.UUID(session_id)
            )

        # Check if session was found and updated
        if result == "UPDATE 0":
            logger.warning(f"⚠️ Session not found for update: {session_id}")
        else:
            logger.debug(f"Updated last_active for session {session_id}")

    except Exception as e:
        logger.error(f"❌ Failed to update last_active for session {session_id}: {e}")
        raise


async def get_or_create_session_by_thread(thread_id: str) -> str:
    """
    Get existing session for a thread_id or create a new one.
    Uses thread_id as user_identifier to map ChatKit threads to database sessions.

    Args:
        thread_id: ChatKit thread ID (string)

    Returns:
        str: The UUID of the session

    Raises:
        asyncpg.PostgresError: If database operation fails
    """
    try:
        pool = await get_pool()

        async with pool.acquire() as conn:
            # Try to find existing session for this thread_id
            session_id = await conn.fetchval(
                """
                SELECT session_id
                FROM chat_sessions
                WHERE user_identifier = $1
                ORDER BY created_at DESC
                LIMIT 1
                """,
                thread_id
            )

            if session_id:
                logger.info(f"✅ Found existing session {session_id} for thread {thread_id}")
                return str(session_id)

            # No existing session, create a new one
            session_id = await conn.fetchval(
                """
                INSERT INTO chat_sessions (user_identifier, created_at, last_active_at)
                VALUES ($1, NOW(), NOW())
                RETURNING session_id
                """,
                thread_id
            )

            logger.info(f"✅ Created new session {session_id} for thread {thread_id}")
            return str(session_id)

    except Exception as e:
        logger.error(f"❌ Failed to get or create session for thread {thread_id}: {e}")
        raise
