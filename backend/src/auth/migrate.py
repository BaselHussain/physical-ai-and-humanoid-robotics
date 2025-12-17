"""
Database Migration Script for Authentication

Creates users table in Neon PostgreSQL with metadata JSONB field.
Uses SQLAlchemy for schema creation.
"""

import asyncio
import os
from pathlib import Path
from dotenv import load_dotenv
from sqlalchemy import inspect

# Load .env file from backend directory
env_path = Path(__file__).resolve().parent.parent.parent / ".env"
load_dotenv(dotenv_path=env_path)

from src.auth.config import engine
from src.auth.models import Base


async def create_tables():
    """
    Create all database tables defined in models.
    Uses SQLAlchemy metadata to create tables.
    """
    # Import all models to register them with Base
    from src.auth.models import User

    print("=" * 60)
    print("STARTING DATABASE MIGRATION")
    print("=" * 60)

    # Check DATABASE_URL
    database_url = os.getenv("DATABASE_URL") or os.getenv("NEON_DATABASE_URL")
    if not database_url:
        print("[ERROR] DATABASE_URL or NEON_DATABASE_URL not set in environment")
        print("Please configure Neon PostgreSQL connection string in .env file")
        return False

    print(f"[OK] DATABASE_URL configured: {database_url[:50]}...")

    try:
        # Create tables
        async with engine.begin() as conn:
            # Check existing tables
            def get_existing_tables(connection):
                inspector = inspect(connection)
                return inspector.get_table_names()

            existing_tables = await conn.run_sync(get_existing_tables)

            if "users" in existing_tables:
                print("\n[WARNING] Table 'users' already exists - skipping creation")
                print("   Run DROP TABLE users CASCADE; to recreate (WARNING: deletes all data)")
            else:
                # Create all tables
                await conn.run_sync(Base.metadata.create_all)
                print("\n[OK] Created table: users")
                print("  - Columns: id, email, hashed_password, is_active, is_superuser, is_verified, metadata, created_at, updated_at")
                print("  - Indexes: email (unique)")

        # Verify tables created
        async with engine.connect() as conn:
            tables_created = await conn.run_sync(get_existing_tables)
            print(f"\n[OK] Total tables in database: {len(tables_created)}")
            print(f"  Tables: {', '.join(tables_created)}")

        print("\n" + "=" * 60)
        print("MIGRATION COMPLETE [SUCCESS]")
        print("=" * 60)
        return True

    except Exception as e:
        print(f"\n[ERROR] during migration: {e}")
        print(f"   Error type: {type(e).__name__}")
        import traceback
        traceback.print_exc()
        return False


async def drop_tables():
    """
    Drop all tables (use with caution - deletes all data).
    Only for development/testing.
    """
    print("\n[WARNING] DROP TABLES requested")
    print("   This will delete ALL authentication data")

    from src.auth.models import User

    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)
        print("[OK] Dropped all tables")


if __name__ == "__main__":
    # Run migration
    success = asyncio.run(create_tables())
    exit(0 if success else 1)
