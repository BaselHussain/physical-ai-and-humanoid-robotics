import pytest
import asyncio
from fastapi.testclient import TestClient
from main import app
from app.session_manager import create_session
from models.chat import ChatRequest


@pytest.fixture
def client():
    """Create a test client for the FastAPI app"""
    return TestClient(app)


def test_health_endpoint(client):
    """Test the health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json() == {"status": "healthy", "service": "rag-chatbot-api"}


def test_create_session_endpoint(client):
    """Test the session creation endpoint"""
    response = client.post("/api/chatkit/session")
    assert response.status_code == 200
    data = response.json()
    assert "client_secret" in data
    assert "session_id" in data
    assert data["client_secret"].startswith("mock_token_")
    assert len(data["session_id"]) > 0


def test_chat_endpoint_missing_session(client):
    """Test chat endpoint with non-existent session"""
    chat_request = ChatRequest(session_id="non-existent-session", message="test message")
    response = client.post("/api/chat", json=chat_request.dict())
    assert response.status_code == 404


def test_chat_endpoint_valid_session(client):
    """Test chat endpoint with valid session"""
    # Create a session first
    session_response = client.post("/api/chatkit/session")
    assert session_response.status_code == 200
    session_data = session_response.json()
    session_id = session_data["session_id"]

    # Send a message to the chat endpoint
    chat_request = ChatRequest(session_id=session_id, message="Hello, how are you?")
    response = client.post("/api/chat", json=chat_request.dict())

    # The response should be successful (though content depends on the agent response)
    assert response.status_code == 200
    # The response is a streaming response, so we check the content type
    assert response.headers["content-type"].startswith("text/event-stream")


def test_refresh_session_endpoint(client):
    """Test the session refresh endpoint"""
    # Create a session first
    session_response = client.post("/api/chatkit/session")
    assert session_response.status_code == 200
    session_data = session_response.json()
    token = session_data["client_secret"]

    # Refresh the session
    refresh_request = {"token": token}
    response = client.post("/api/chatkit/refresh", json=refresh_request)
    assert response.status_code == 200
    data = response.json()
    assert data["client_secret"] == token


def test_refresh_session_endpoint_invalid_token(client):
    """Test the session refresh endpoint with invalid token"""
    refresh_request = {"token": "invalid-token"}
    response = client.post("/api/chatkit/refresh", json=refresh_request)
    assert response.status_code == 401


if __name__ == "__main__":
    pytest.main([__file__])