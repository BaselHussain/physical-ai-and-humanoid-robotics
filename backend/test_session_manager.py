import pytest
from app.rag_agent import docs_agent
from models.chat import ChatMessage, ChatSession
from app.session_manager import create_session, get_session, add_message, cleanup_expired_sessions
from datetime import datetime, timedelta
import uuid


def test_rag_agent_exists():
    """Test that the RAG agent is properly initialized"""
    assert docs_agent is not None
    assert hasattr(docs_agent, 'name')
    assert docs_agent.name == "DocsRAGAgent"


def test_create_session():
    """Test session creation"""
    session = create_session()
    assert session is not None
    assert isinstance(session.session_id, str)
    assert len(session.session_id) > 0
    assert session.created_at is not None
    assert isinstance(session.messages, list)
    assert len(session.messages) == 0


def test_get_session():
    """Test session retrieval"""
    # Create a session
    session = create_session()
    session_id = session.session_id

    # Retrieve the session
    retrieved_session = get_session(session_id)
    assert retrieved_session is not None
    assert retrieved_session.session_id == session_id
    assert retrieved_session.created_at == session.created_at
    assert retrieved_session.messages == session.messages


def test_add_message():
    """Test adding messages to a session"""
    session = create_session()
    session_id = session.session_id

    # Add a user message
    message = add_message(session_id, "user", "Hello, world!")
    assert message.role == "user"
    assert message.content == "Hello, world!"
    assert message.session_id == session_id
    assert message.message_id is not None

    # Verify the message was added to the session
    updated_session = get_session(session_id)
    assert len(updated_session.messages) == 1
    assert updated_session.messages[0].content == "Hello, world!"


def test_add_message_invalid_session():
    """Test adding a message to a non-existent session"""
    with pytest.raises(ValueError, match="Session non-existent-session not found"):
        add_message("non-existent-session", "user", "Hello, world!")


def test_cleanup_expired_sessions():
    """Test cleanup of expired sessions"""
    # Create a session
    session = create_session()
    session_id = session.session_id

    # Verify session exists
    assert get_session(session_id) is not None

    # Try to clean up with a short TTL (this won't remove the session as it's not old enough)
    removed_count = cleanup_expired_sessions(ttl_minutes=30)
    assert removed_count == 0
    assert get_session(session_id) is not None

    # Manually modify the session's created_at to be in the past
    from app.session_manager import sessions
    sessions[session_id].created_at = datetime.utcnow() - timedelta(minutes=31)

    # Now cleanup should remove it
    removed_count = cleanup_expired_sessions(ttl_minutes=30)
    assert removed_count == 1
    assert get_session(session_id) is None


def test_chat_message_model():
    """Test ChatMessage model validation"""
    message = ChatMessage(
        message_id=str(uuid.uuid4()),
        session_id=str(uuid.uuid4()),
        role="user",
        content="Test message"
    )
    assert message.role == "user"
    assert message.content == "Test message"


def test_chat_session_model():
    """Test ChatSession model validation"""
    session = ChatSession(
        session_id=str(uuid.uuid4()),
        created_at=datetime.utcnow(),
        messages=[]
    )
    assert session.session_id is not None
    assert session.created_at is not None
    assert session.messages == []


if __name__ == "__main__":
    pytest.main([__file__])