-- RAG Chatbot Chat History Tables
-- Created: 2025-12-16
-- Purpose: Store chat sessions and messages for persistent history across browser sessions

-- Create chat_sessions table
CREATE TABLE IF NOT EXISTS chat_sessions (
    session_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_identifier TEXT NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    last_active_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

-- Create index on user_identifier for faster lookups
CREATE INDEX IF NOT EXISTS idx_chat_sessions_user_identifier
ON chat_sessions(user_identifier);

-- Create index on last_active_at for cleanup queries
CREATE INDEX IF NOT EXISTS idx_chat_sessions_last_active
ON chat_sessions(last_active_at);

-- Create chat_messages table
CREATE TABLE IF NOT EXISTS chat_messages (
    message_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(session_id) ON DELETE CASCADE,
    message_text TEXT NOT NULL,
    role TEXT NOT NULL CHECK (role IN ('user', 'assistant')),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    source_references JSONB DEFAULT '[]'::jsonb
);

-- Create index on session_id for faster message retrieval
CREATE INDEX IF NOT EXISTS idx_chat_messages_session_id
ON chat_messages(session_id);

-- Create index on timestamp for chronological ordering
CREATE INDEX IF NOT EXISTS idx_chat_messages_timestamp
ON chat_messages(timestamp);

-- Create composite index for session_id + timestamp (most common query pattern)
CREATE INDEX IF NOT EXISTS idx_chat_messages_session_timestamp
ON chat_messages(session_id, timestamp);

-- Add comment to tables for documentation
COMMENT ON TABLE chat_sessions IS 'Stores chat session metadata for RAG chatbot';
COMMENT ON TABLE chat_messages IS 'Stores individual chat messages with user/assistant role and optional source references';

-- Verify tables were created
SELECT 'Tables created successfully' AS status;
