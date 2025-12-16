from typing import Dict, List, Optional
from datetime import datetime, timedelta
import uuid
import sys

# Conditional imports to support both running from root and from backend directory
# Check parent module's __package__ to determine import style
_parent_package = sys.modules['__main__'].__package__ if '__main__' in sys.modules else None

if _parent_package:
    # Running as package from root: uvicorn backend.main:app
    from ..models.chat import ChatMessage, ChatSession
else:
    # Running from backend directory: uvicorn main:app
    from models.chat import ChatMessage, ChatSession

# In-memory storage for sessions
sessions: Dict[str, ChatSession] = {}


def create_session() -> ChatSession:
    """
    Create a new chat session with a unique ID.

    Returns:
        ChatSession: Newly created session
    """
    session = ChatSession(
        session_id=str(uuid.uuid4()),
        created_at=datetime.utcnow(),
        messages=[]
    )
    sessions[session.session_id] = session
    return session


def get_session(session_id: str) -> Optional[ChatSession]:
    """
    Retrieve a session by its ID.

    Args:
        session_id: Session identifier

    Returns:
        ChatSession if found, None otherwise
    """
    return sessions.get(session_id)


def add_message(
    session_id: str,
    role: str,
    content: str,
    source_refs: Optional[List[str]] = None
) -> ChatMessage:
    """
    Add a message to an existing session.

    Args:
        session_id: Session identifier
        role: Message role ('user' or 'assistant')
        content: Message content
        source_refs: Optional list of source references for assistant messages

    Returns:
        ChatMessage: The created message

    Raises:
        ValueError: If session not found
    """
    session = sessions.get(session_id)
    if not session:
        raise ValueError(f"Session {session_id} not found")

    message = ChatMessage(
        message_id=str(uuid.uuid4()),
        session_id=session_id,
        role=role,
        content=content,
        timestamp=datetime.utcnow(),
        source_refs=source_refs
    )
    session.messages.append(message)
    return message


def cleanup_expired_sessions(ttl_minutes: int = 30) -> int:
    """
    Remove sessions inactive for more than ttl_minutes.

    Args:
        ttl_minutes: Time-to-live in minutes (default: 30)

    Returns:
        int: Number of sessions removed
    """
    now = datetime.utcnow()
    expired_ids = [
        sid for sid, session in sessions.items()
        if now - session.created_at > timedelta(minutes=ttl_minutes)
    ]

    for sid in expired_ids:
        del sessions[sid]

    return len(expired_ids)
