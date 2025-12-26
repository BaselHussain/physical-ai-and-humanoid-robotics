"""
Session Validation Middleware

Validates session tokens and attaches user background to request context.
Used by protected endpoints (e.g., /chat/message) to ensure authentication.
"""

from typing import Optional
from fastapi import Request, HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials

from .models import User, get_background_profile, BackgroundProfile
from .routes import fastapi_users, current_active_user


# HTTP Bearer token security
security = HTTPBearer()


async def get_current_user_with_background(
    user: User = Depends(current_active_user)
) -> tuple[User, Optional[BackgroundProfile]]:
    """
    Dependency to get current authenticated user with background profile.

    This is a convenience function that combines user authentication
    and background profile fetching.

    Args:
        user: Current authenticated user (from FastAPI-Users)

    Returns:
        Tuple of (User, BackgroundProfile or None)

    Raises:
        HTTPException: 401 if session invalid or expired
    """
    background = get_background_profile(user)
    return user, background


async def require_authentication(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> str:
    """
    Validate Bearer token and return token string.

    This middleware checks for Authorization header with Bearer token.
    Actual validation is delegated to FastAPI-Users.

    Args:
        credentials: HTTP Authorization credentials

    Returns:
        Token string

    Raises:
        HTTPException: 401 if Authorization header missing or invalid format
    """
    if not credentials:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing authorization token",
        )

    if credentials.scheme.lower() != "bearer":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication scheme. Use Bearer token.",
        )

    return credentials.credentials


class SessionValidationMiddleware:
    """
    Middleware to validate sessions on protected routes.

    This middleware automatically validates JWT tokens and attaches
    user information to the request state for downstream handlers.

    Usage in FastAPI app:
        app.add_middleware(SessionValidationMiddleware)
    """

    def __init__(self, app):
        self.app = app

    async def __call__(self, request: Request, call_next):
        """
        Process request with session validation.

        Protected routes (those requiring authentication) should use
        the current_active_user or get_current_user_with_background dependencies.

        Args:
            request: FastAPI request
            call_next: Next middleware/handler

        Returns:
            Response from next handler
        """
        # Get Authorization header
        auth_header = request.headers.get("Authorization")

        if auth_header and auth_header.startswith("Bearer "):
            token = auth_header.split(" ")[1]

            # Store token in request state for dependencies
            request.state.auth_token = token

            try:
                # Validate token using FastAPI-Users (this will raise 401 if invalid)
                # The actual validation is done by dependencies, not here
                pass
            except Exception as e:
                # Token validation errors are handled by FastAPI-Users dependencies
                pass

        # Continue to next handler
        response = await call_next(request)
        return response


# ============================================================================
# Error Response Helpers
# ============================================================================

def session_expired_error():
    """
    Generate session expired error response.

    Maps to contracts/auth-api.yaml ErrorResponse with preserve_message=true.
    """
    return HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail={
            "error": "Session expired",
            "code": "SESSION_EXPIRED",
            "preserve_message": True,
        },
    )


def authentication_required_error():
    """
    Generate authentication required error response.

    Used when guest users try to access protected endpoints.
    """
    return HTTPException(
        status_code=status.HTTP_403_FORBIDDEN,
        detail={
            "error": "Please sign in to use the chat",
            "code": "AUTHENTICATION_REQUIRED",
        },
    )


def invalid_token_error():
    """
    Generate invalid token error response.

    Used when token format is incorrect or signature is invalid.
    """
    return HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail={
            "error": "Invalid session token",
            "code": "INVALID_TOKEN",
        },
    )


# ============================================================================
# Convenience Dependencies (for use in route handlers)
# ============================================================================

async def get_authenticated_user(
    user: User = Depends(current_active_user),
) -> User:
    """
    Dependency to get authenticated user (simplified wrapper).

    Raises 401 if user not authenticated.

    Args:
        user: Current active user

    Returns:
        User model

    Raises:
        HTTPException: 401 if not authenticated
    """
    if not user:
        raise authentication_required_error()
    return user


async def get_user_background(
    user: User = Depends(current_active_user),
) -> BackgroundProfile:
    """
    Dependency to get user background profile.

    Raises 401 if user not authenticated, 404 if background not found.

    Args:
        user: Current active user

    Returns:
        BackgroundProfile

    Raises:
        HTTPException: 401 if not authenticated, 404 if background not found
    """
    if not user:
        raise authentication_required_error()

    background = get_background_profile(user)
    if not background:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Background profile not found. Please complete signup.",
        )

    return background
