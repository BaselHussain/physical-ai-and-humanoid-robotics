"""
Pydantic Schemas for Authentication API

Request/Response models matching contracts/auth-api.yaml specification.
All schemas follow OpenAPI 3.0 contract definitions.
"""

from typing import Optional
from datetime import datetime
from pydantic import BaseModel, EmailStr, Field
from fastapi_users import schemas

from .models import BackgroundProfile


# ============================================================================
# FastAPI-Users Base Schemas (User Read/Create/Update)
# ============================================================================

class UserRead(schemas.BaseUser[int]):
    """
    User response schema (returned by API endpoints).
    Includes user ID, email, and custom user_metadata field.
    """
    id: int
    email: EmailStr
    is_active: bool = True
    is_superuser: bool = False
    is_verified: bool = False
    user_metadata: dict = Field(default_factory=dict, alias="metadata")
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
        populate_by_name = True


class UserCreate(schemas.BaseUserCreate):
    """
    User creation schema (POST /auth/signup request body).
    Includes email, password, and required background profile.
    """
    email: EmailStr
    password: str = Field(..., min_length=8, description="Password (minimum 8 characters)")
    background: BackgroundProfile = Field(..., description="User background profile (required)")
    is_active: Optional[bool] = True
    is_superuser: Optional[bool] = False
    is_verified: Optional[bool] = False


class UserUpdate(schemas.BaseUserUpdate):
    """
    User update schema (PATCH /users/{id} - future use).
    Allows updating password, email, and metadata.
    """
    password: Optional[str] = Field(None, min_length=8)
    email: Optional[EmailStr] = None
    is_active: Optional[bool] = None
    is_superuser: Optional[bool] = None
    is_verified: Optional[bool] = None


# ============================================================================
# Custom Authentication Schemas (matching contracts/auth-api.yaml)
# ============================================================================

class SignupRequest(BaseModel):
    """
    Signup request schema (POST /auth/signup).
    Maps to contracts/auth-api.yaml SignupRequest.
    """
    email: EmailStr = Field(..., description="User email address (unique, used for login)")
    password: str = Field(..., min_length=8, description="Password (minimum 8 characters)")
    background: BackgroundProfile = Field(..., description="User background profile (all fields required)")

    class Config:
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "SecurePass123!",
                "background": {
                    "programming_experience": "6-10 years",
                    "ros2_familiarity": "Intermediate",
                    "hardware_access": "Simulation only"
                }
            }
        }


class SigninRequest(BaseModel):
    """
    Signin request schema (POST /auth/signin).
    Maps to contracts/auth-api.yaml SigninRequest.
    """
    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., description="Password")

    class Config:
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "SecurePass123!"
            }
        }


class AuthSuccessResponse(BaseModel):
    """
    Authentication success response (POST /auth/signup, POST /auth/signin).
    Maps to contracts/auth-api.yaml AuthSuccessResponse.
    """
    token: str = Field(..., description="Session token (JWT)")
    user_id: int = Field(..., description="Unique user identifier")
    background: Optional[BackgroundProfile] = Field(None, description="User background profile")
    expires_at: datetime = Field(..., description="Session expiration timestamp (7 days from creation)")

    class Config:
        json_schema_extra = {
            "example": {
                "token": "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
                "user_id": 123,
                "background": {
                    "programming_experience": "6-10 years",
                    "ros2_familiarity": "Intermediate",
                    "hardware_access": "Simulation only"
                },
                "expires_at": "2025-12-24T10:00:00Z"
            }
        }


class SessionResponse(BaseModel):
    """
    Session validation response (GET /auth/session).
    Maps to contracts/auth-api.yaml SessionResponse.
    """
    user_id: int = Field(..., description="Unique user identifier")
    email: EmailStr = Field(..., description="User email address")
    expires_at: datetime = Field(..., description="Session expiration timestamp")
    background: Optional[BackgroundProfile] = Field(None, description="User background profile")

    class Config:
        json_schema_extra = {
            "example": {
                "user_id": 123,
                "email": "user@example.com",
                "expires_at": "2025-12-24T10:00:00Z",
                "background": {
                    "programming_experience": "6-10 years",
                    "ros2_familiarity": "Intermediate",
                    "hardware_access": "Simulation only"
                }
            }
        }


class ErrorResponse(BaseModel):
    """
    Error response schema (400, 401, 403, 500 responses).
    Maps to contracts/auth-api.yaml ErrorResponse.
    """
    error: str = Field(..., description="Human-readable error message")
    code: str = Field(..., description="Machine-readable error code")
    preserve_message: Optional[bool] = Field(None, description="Client should preserve typed message (session expiration only)")

    class Config:
        json_schema_extra = {
            "example": {
                "error": "Session expired",
                "code": "SESSION_EXPIRED",
                "preserve_message": True
            }
        }


class BackgroundProfileResponse(BaseModel):
    """
    Background profile response (GET /auth/background/{user_id}).
    Maps to contracts/personalization-api.yaml.
    """
    programming_experience: str = Field(..., description="Years of programming experience")
    ros2_familiarity: str = Field(..., description="ROS 2 / robotics familiarity level")
    hardware_access: str = Field(..., description="Access to physical hardware")

    class Config:
        json_schema_extra = {
            "example": {
                "programming_experience": "6-10 years",
                "ros2_familiarity": "Intermediate",
                "hardware_access": "Simulation only"
            }
        }
