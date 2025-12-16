"""
ChatKit Server Implementation for RAG Chatbot
Integrates with our existing RAG agent and implements the ID collision fix for Gemini
"""
import os
import asyncio
from typing import Any, AsyncIterator, Dict
from datetime import datetime
import logging

from chatkit.server import ChatKitServer
from chatkit.types import ThreadMetadata, UserMessageItem, AssistantMessageItem
from chatkit.agents import stream_agent_response, AgentContext
from agents import Runner

# Conditional imports to support both running from root and from backend directory
try:
    from .rag_agent import docs_agent
    from . import database, chat_history
except ImportError:
    from app.rag_agent import docs_agent
    from app import database, chat_history

logger = logging.getLogger(__name__)


class RAGChatKitServer(ChatKitServer):
    """
    ChatKit server implementation that integrates with our RAG agent
    Implements the ID collision fix for non-OpenAI providers like Gemini
    """

    def __init__(self, store):
        super().__init__(store)

    async def respond(
        self,
        thread: ThreadMetadata,
        input: UserMessageItem | None,
        context: Any,
    ) -> AsyncIterator:
        """Respond to a chat request using the RAG agent with ID collision fix"""

        # Prepare input for the agent
        if input:
            # Extract text from the input message
            user_input_text = self._extract_text(input)
        else:
            user_input_text = ""

        # Save to Neon database - get or create session for this thread
        thread_id = thread.id if thread and hasattr(thread, 'id') else 'default_thread'
        session_id = None

        try:
            # Get or create session for this thread
            session_id = await chat_history.get_or_create_session_by_thread(thread_id)
            logger.info(f"Using session {session_id} for thread: {thread_id}")
        except Exception as e:
            logger.warning(f"Failed to get/create session in database: {e}")

        # Save user message to database
        if user_input_text and session_id:
            try:
                await chat_history.save_message(
                    session_id=session_id,
                    message_text=user_input_text,
                    role="user",
                    source_references=[]
                )
                logger.info(f"User message saved to database")
            except Exception as e:
                logger.warning(f"Failed to save user message to database: {e}")

        # Track ID mappings to ensure unique IDs (fix for Gemini ID collision)
        id_mapping: Dict[str, str] = {}
        assistant_message_text = ""

        try:
            # Run the RAG agent with the user input
            result = Runner.run_streamed(
                docs_agent,
                input=user_input_text
            )

            # Create AgentContext with required parameters
            agent_context = AgentContext(
                thread=thread,
                store=self.store,
                request_context=context
            )

            # Stream the agent response with ID mapping fix
            async for event in stream_agent_response(agent_context, result):
                if hasattr(event, 'type') and event.type == "thread.item.added":
                    if hasattr(event, 'item') and isinstance(event.item, AssistantMessageItem):
                        old_id = event.item.id
                        if old_id not in id_mapping:
                            # Generate a new unique ID using the store
                            new_id = self.store.generate_item_id("message", thread, context)
                            id_mapping[old_id] = new_id
                        event.item.id = id_mapping[old_id]
                elif hasattr(event, 'type') and event.type == "thread.item.done":
                    if hasattr(event, 'item') and isinstance(event.item, AssistantMessageItem):
                        if event.item.id in id_mapping:
                            event.item.id = id_mapping[event.item.id]
                        # Collect assistant message text for database storage
                        assistant_message_text = self._extract_text(event.item)
                elif hasattr(event, 'type') and event.type == "thread.item.updated":
                    if hasattr(event, 'item_id') and event.item_id in id_mapping:
                        event.item_id = id_mapping[event.item_id]

                yield event

            # Save assistant message to database after streaming completes
            if assistant_message_text and session_id:
                try:
                    await chat_history.save_message(
                        session_id=session_id,
                        message_text=assistant_message_text,
                        role="assistant",
                        source_references=[]
                    )
                    logger.info(f"Assistant message saved to database")
                except Exception as e:
                    logger.warning(f"Failed to save assistant message to database: {e}")

        except Exception as e:
            # Handle any errors gracefully - just log and re-raise
            print(f"Error in ChatKit respond: {e}")
            import traceback
            traceback.print_exc()
            raise

    def _extract_text(self, item) -> str:
        """Extract text from message content parts"""
        text = ""
        if hasattr(item, 'content') and item.content:
            for part in item.content:
                if hasattr(part, 'text'):
                    text += part.text
        return text