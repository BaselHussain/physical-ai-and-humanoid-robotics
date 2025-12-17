"""
FastAPI-Users Manager Configuration

Handles user authentication logic, password hashing, and user management.
"""

import os
from typing import Optional
from fastapi import Depends, Request
from fastapi_users import BaseUserManager, IntegerIDMixin, exceptions, models, schemas
from fastapi_users.password import PasswordHelper
from passlib.context import CryptContext

from .models import User, set_background_profile
from .config import get_user_db


# Secret key for JWT token generation (from environment)
SECRET = os.getenv("SECRET_KEY", "CHANGE_THIS_SECRET_KEY_IN_PRODUCTION_12345678901234567890")

# Password hashing context (Argon2id)
pwd_context = CryptContext(schemes=["argon2"], deprecated="auto")
password_helper = PasswordHelper(pwd_context)


class UserManager(IntegerIDMixin, BaseUserManager[User, int]):
    """
    Custom user manager for FastAPI-Users.
    Handles user creation with background profile storage.
    """
    reset_password_token_secret = SECRET
    verification_token_secret = SECRET

    async def on_after_register(self, user: User, request: Optional[Request] = None):
        """Called after successful user registration."""
        print(f"User {user.id} ({user.email}) has registered.")

    async def on_after_forgot_password(
        self, user: User, token: str, request: Optional[Request] = None
    ):
        """Called after password reset request."""
        print(f"User {user.id} has forgot their password. Reset token: {token}")

    async def on_after_request_verify(
        self, user: User, token: str, request: Optional[Request] = None
    ):
        """Called after email verification request."""
        print(f"Verification requested for user {user.id}. Verification token: {token}")

    async def create(
        self,
        user_create: schemas.UC,
        safe: bool = False,
        request: Optional[Request] = None,
    ) -> models.UP:
        """
        Override create method to handle background profile storage.

        Args:
            user_create: User creation schema (includes background field)
            safe: If True, validate email is not already registered
            request: Optional request context

        Returns:
            Created User model
        """
        # Check if email already exists
        await self.validate_password(user_create.password, user_create)
        existing_user = await self.user_db.get_by_email(user_create.email)
        if existing_user is not None:
            raise exceptions.UserAlreadyExists()

        # Create user dict without background field
        user_dict = (
            user_create.create_update_dict()
            if safe
            else user_create.create_update_dict_superuser()
        )

        # Extract background profile if present
        background_profile = user_dict.pop("background", None)

        # Add background profile to user_metadata before creation
        if background_profile:
            from .models import BackgroundProfile
            profile = BackgroundProfile(**background_profile)
            user_dict["user_metadata"] = {"background": profile.model_dump()}

        # Hash password
        password = user_dict.pop("password")
        user_dict["hashed_password"] = password_helper.hash(password)

        # Create user with background profile included
        created_user = await self.user_db.create(user_dict)

        await self.on_after_register(created_user, request)

        return created_user


async def get_user_manager(user_db=Depends(get_user_db)):
    """
    Dependency to get UserManager instance.
    Used by FastAPI-Users for authentication operations.
    """
    yield UserManager(user_db)
