from pydantic import BaseModel, Field
from datetime import datetime
from typing import List, Optional


class SessionResponse(BaseModel):
    """Response model for session creation endpoint"""
    session_id: str = Field(..., description="Unique session identifier")


class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    message: str = Field(..., min_length=1, max_length=5000, description="User message")
    session_id: str = Field(..., description="Session identifier")


class ChatMessage(BaseModel):
    """Individual message in a chat session"""
    message_id: str = Field(..., description="Unique message identifier")
    session_id: str = Field(..., description="Session this message belongs to")
    role: str = Field(..., pattern="^(user|assistant)$", description="Message role: user or assistant")
    content: str = Field(..., description="Message content")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Message timestamp")
    source_refs: Optional[List[str]] = Field(default=None, description="Source references for assistant messages")


class ChatSession(BaseModel):
    """Chat session containing message history"""
    session_id: str = Field(..., description="Unique session identifier")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Session creation timestamp")
    messages: List[ChatMessage] = Field(default_factory=list, description="List of messages in session")
