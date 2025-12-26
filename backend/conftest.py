"""
Pytest configuration for authentication tests.

Provides fixtures for async test setup with proper database connection management.
"""

import asyncio
import sys
from pathlib import Path

# Add backend directory to Python path
backend_dir = Path(__file__).resolve().parent
sys.path.insert(0, str(backend_dir))

import pytest
import pytest_asyncio
from dotenv import load_dotenv
from httpx import AsyncClient, ASGITransport
from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker, AsyncSession

# Load .env before any imports that might use environment variables
env_path = backend_dir / ".env"
load_dotenv(dotenv_path=env_path)

from main import app
from src.auth.config import engine as default_engine, async_session_maker as default_session_maker


@pytest.fixture(scope="session")
def event_loop():
    """
    Create an event loop for the entire test session.
    This prevents event loop closure issues with async database operations.
    """
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
    yield loop
    # Don't close the loop - let pytest-asyncio handle it
    # loop.close()


@pytest_asyncio.fixture(scope="function")
async def async_client():
    """
    Provide an async HTTP client for testing FastAPI endpoints.
    Uses httpx.AsyncClient for proper async support.
    """
    transport = ASGITransport(app=app)
    async with AsyncClient(transport=transport, base_url="http://test") as client:
        yield client


@pytest_asyncio.fixture(scope="function")
async def db_session():
    """
    Provide a database session for tests.
    Properly handles connection lifecycle and cleanup.
    """
    async with default_session_maker() as session:
        yield session
        await session.rollback()  # Rollback any uncommitted changes
        await session.close()


@pytest_asyncio.fixture(scope="session", autouse=True)
async def setup_teardown_database():
    """
    Setup and teardown for database connections.
    Ensures proper cleanup of connection pool after all tests.
    """
    # Setup: database is already initialized
    yield

    # Teardown: properly dispose of connection pool
    await default_engine.dispose()


@pytest.fixture(scope="session")
def test_user_data():
    """
    Provide test user data for authentication tests.
    """
    return {
        "beginner": {
            "email": "test_beginner@test.com",
            "password": "TestPass123!",
            "background": {
                "programming_experience": "0-2 years",
                "ros2_familiarity": "None",
                "hardware_access": "None"
            }
        },
        "advanced": {
            "email": "test_advanced@test.com",
            "password": "TestPass123!",
            "background": {
                "programming_experience": "10+ years",
                "ros2_familiarity": "Advanced",
                "hardware_access": "Physical robots/sensors"
            }
        }
    }
