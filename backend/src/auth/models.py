"""
Database Models for Authentication

User model with custom BackgroundProfile metadata (JSONB storage).
FastAPI-Users integration with SQLAlchemy ORM.
"""

from enum import Enum
from typing import Optional
from datetime import datetime
from pydantic import BaseModel
from sqlalchemy import Column, String, Boolean, DateTime, JSON, Integer
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
from fastapi_users.db import SQLAlchemyBaseUserTable

Base = declarative_base()


# ============================================================================
# Pydantic Enums for BackgroundProfile validation
# ============================================================================

class ProgrammingExperience(str, Enum):
    """Years of programming experience (dropdown options from spec.md FR-002a)"""
    ZERO_TO_TWO = "0-2 years"
    THREE_TO_FIVE = "3-5 years"
    SIX_TO_TEN = "6-10 years"
    TEN_PLUS = "10+ years"


class ROS2Familiarity(str, Enum):
    """ROS 2 / robotics familiarity level (dropdown options from spec.md FR-002a)"""
    NONE = "None"
    BEGINNER = "Beginner"
    INTERMEDIATE = "Intermediate"
    ADVANCED = "Advanced"


class HardwareAccess(str, Enum):
    """Access to physical hardware (dropdown options from spec.md FR-002a)"""
    NONE = "None"
    SIMULATION_ONLY = "Simulation only"
    PHYSICAL_HARDWARE = "Physical robots/sensors"


# ============================================================================
# Pydantic Model for BackgroundProfile (stored in User.metadata JSONB)
# ============================================================================

class BackgroundProfile(BaseModel):
    """
    User's technical background for RAG chatbot personalization.
    Stored as JSON in User.metadata.background field.

    All fields REQUIRED per spec.md FR-002a.
    """
    programming_experience: ProgrammingExperience
    ros2_familiarity: ROS2Familiarity
    hardware_access: HardwareAccess

    class Config:
        use_enum_values = True  # Serialize enums as string values


# ============================================================================
# SQLAlchemy User Model (FastAPI-Users integration)
# ============================================================================

class User(SQLAlchemyBaseUserTable[int], Base):
    """
    User model with custom metadata for background profiling.
    Extends FastAPI-Users base table with custom fields.

    Table: users
    Primary Key: id (auto-increment integer)
    Unique: email (indexed)
    Custom: metadata (JSONB field for BackgroundProfile)
    """
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, autoincrement=True)
    email = Column(String(length=320), unique=True, index=True, nullable=False)
    hashed_password = Column(String(length=1024), nullable=False)
    is_active = Column(Boolean, default=True, nullable=False)
    is_superuser = Column(Boolean, default=False, nullable=False)
    is_verified = Column(Boolean, default=False, nullable=False)

    # Custom field: user_metadata JSONB (stores BackgroundProfile)
    # Note: 'metadata' is reserved by SQLAlchemy, so we use 'user_metadata'
    user_metadata = Column("metadata", JSON, default={}, nullable=False)

    # Timestamps
    created_at = Column(DateTime(timezone=True), server_default=func.now(), nullable=False)
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now(), nullable=False)

    def __repr__(self):
        return f"<User(id={self.id}, email={self.email})>"


# ============================================================================
# Helper Functions for BackgroundProfile
# ============================================================================

def set_background_profile(user: User, profile: BackgroundProfile) -> None:
    """
    Store BackgroundProfile in user metadata.

    Args:
        user: User model instance
        profile: Validated BackgroundProfile
    """
    if user.user_metadata is None:
        user.user_metadata = {}
    user.user_metadata["background"] = profile.model_dump()


def get_background_profile(user: User) -> Optional[BackgroundProfile]:
    """
    Retrieve BackgroundProfile from user metadata.

    Args:
        user: User model instance

    Returns:
        BackgroundProfile if exists, None otherwise
    """
    # Handle case where user object doesn't have user_metadata attribute loaded (connection pool issue)
    if not hasattr(user, 'user_metadata') or not user.user_metadata or "background" not in user.user_metadata:
        return None
    try:
        return BackgroundProfile(**user.user_metadata["background"])
    except (KeyError, TypeError, ValueError, AttributeError):
        return None
