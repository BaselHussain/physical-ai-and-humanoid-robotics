"""
ChatKit Store Implementation for RAG Chatbot
Following the ChatKit knowledge base specifications with all 14 required methods
"""
import uuid
from typing import Any, Dict, List, Optional
from datetime import datetime
import asyncio

from chatkit.store import Store
from chatkit.types import ThreadMetadata, ThreadItem, Page
from chatkit.types import UserMessageItem, AssistantMessageItem


class MemoryStore(Store[dict]):
    """
    In-memory store implementation for ChatKit following the knowledge base specifications.
    Stores thread metadata and items in memory with proper ID generation.
    """

    def __init__(self):
        # Thread storage: thread_id -> ThreadMetadata
        self._threads: Dict[str, ThreadMetadata] = {}
        # Thread items storage: thread_id -> list of ThreadItem
        self._thread_items: Dict[str, List[ThreadItem]] = {}
        # Attachment storage: attachment_id -> attachment data
        self._attachments: Dict[str, Any] = {}

    def generate_thread_id(self, context: dict) -> str:
        """Generate a unique thread ID"""
        return f"thread_{uuid.uuid4().hex[:12]}"

    def generate_item_id(self, item_type: str, thread: ThreadMetadata, context: dict) -> str:
        """Generate a unique item ID"""
        return f"{item_type}_{uuid.uuid4().hex[:12]}"

    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        """Load a thread by ID, creating a new one if it doesn't exist"""
        if thread_id in self._threads:
            return self._threads[thread_id]
        else:
            # Create a new thread if it doesn't exist
            new_thread = ThreadMetadata(
                id=thread_id,
                created_at=datetime.now(),
                metadata={}
            )
            self._threads[thread_id] = new_thread
            self._thread_items[thread_id] = []
            return new_thread

    async def save_thread(self, thread: ThreadMetadata, context: dict) -> None:
        """Save or update a thread"""
        self._threads[thread.id] = thread

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: dict
    ) -> Page[ThreadItem]:
        """Load thread items with pagination"""
        if thread_id not in self._thread_items:
            self._thread_items[thread_id] = []

        items = self._thread_items[thread_id]

        # Apply pagination
        if after:
            # Find the index after the specified item
            after_idx = -1
            for i, item in enumerate(items):
                if item.id == after:
                    after_idx = i
                    break
            if after_idx != -1:
                items = items[after_idx + 1:]

        # Apply limit
        if limit > 0:
            items = items[:limit]

        # Apply ordering
        if order == "desc":
            items = items[::-1]

        return Page(
            data=items,
            has_more=len(items) >= limit if limit > 0 else False,
            next_cursor=None  # Simplified for this implementation
        )

    async def add_thread_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        """Add an item to a thread"""
        if thread_id not in self._thread_items:
            self._thread_items[thread_id] = []
        self._thread_items[thread_id].append(item)

    async def save_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        """Save an item (alias for add_thread_item)"""
        await self.add_thread_item(thread_id, item, context)

    async def load_item(self, thread_id: str, item_id: str, context: dict) -> ThreadItem:
        """Load a specific item from a thread"""
        if thread_id not in self._thread_items:
            raise ValueError(f"Thread {thread_id} not found")

        for item in self._thread_items[thread_id]:
            if item.id == item_id:
                return item

        raise ValueError(f"Item {item_id} not found in thread {thread_id}")

    async def delete_thread_item(self, thread_id: str, item_id: str, context: dict) -> None:
        """Delete a specific item from a thread"""
        if thread_id not in self._thread_items:
            return

        self._thread_items[thread_id] = [
            item for item in self._thread_items[thread_id]
            if item.id != item_id
        ]

    async def load_threads(
        self,
        limit: int,
        after: str | None,
        order: str,
        context: dict
    ) -> Page[ThreadMetadata]:
        """Load all threads with pagination"""
        threads = list(self._threads.values())

        # Apply pagination
        if after:
            after_idx = -1
            for i, thread in enumerate(threads):
                if thread.id == after:
                    after_idx = i
                    break
            if after_idx != -1:
                threads = threads[after_idx + 1:]

        # Apply limit
        if limit > 0:
            threads = threads[:limit]

        # Apply ordering
        if order == "desc":
            threads = threads[::-1]

        return Page(
            data=threads,
            has_more=len(threads) >= limit if limit > 0 else False,
            next_cursor=None  # Simplified for this implementation
        )

    async def delete_thread(self, thread_id: str, context: dict) -> None:
        """Delete a thread and all its items"""
        if thread_id in self._threads:
            del self._threads[thread_id]
        if thread_id in self._thread_items:
            del self._thread_items[thread_id]

    # OFTEN FORGOTTEN - but required:
    async def save_attachment(self, attachment: Any, context: dict) -> None:
        """Save an attachment"""
        # For this implementation, we'll just store in memory
        attachment_id = f"attachment_{uuid.uuid4().hex[:12]}"
        self._attachments[attachment_id] = attachment
        return attachment_id

    async def load_attachment(self, attachment_id: str, context: dict) -> Any:
        """Load an attachment"""
        if attachment_id in self._attachments:
            return self._attachments[attachment_id]
        raise ValueError(f"Attachment {attachment_id} not found")

    async def delete_attachment(self, attachment_id: str, context: dict) -> None:
        """Delete an attachment"""
        if attachment_id in self._attachments:
            del self._attachments[attachment_id]