"""
Authentication API Routes

Implements signup, signin, signout endpoints with background profile collection.
Maps to contracts/auth-api.yaml specification.
"""

from datetime import datetime, timedelta
from typing import Optional
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi_users import FastAPIUsers
from fastapi_users.authentication import AuthenticationBackend, BearerTransport, JWTStrategy
from sqlalchemy.ext.asyncio import AsyncSession

from .models import User, get_background_profile
from .schemas import UserRead, UserCreate, SignupRequest, SigninRequest, AuthSuccessResponse, SessionResponse
from .manager import get_user_manager
from .config import get_async_session, SESSION_DURATION_DAYS


# JWT Strategy Configuration
SECRET = "CHANGE_THIS_SECRET_KEY_IN_PRODUCTION_12345678901234567890"  # TODO: Move to environment


def get_jwt_strategy() -> JWTStrategy:
    """
    Create JWT strategy for session management.
    Tokens expire after SESSION_DURATION_DAYS (7 days per spec).
    """
    return JWTStrategy(
        secret=SECRET,
        lifetime_seconds=SESSION_DURATION_DAYS * 24 * 60 * 60,  # 7 days in seconds
    )


# Bearer Transport (JWT token in Authorization header)
bearer_transport = BearerTransport(tokenUrl="auth/signin")

# Authentication Backend
auth_backend = AuthenticationBackend(
    name="jwt",
    transport=bearer_transport,
    get_strategy=get_jwt_strategy,
)

# FastAPI-Users Instance
fastapi_users = FastAPIUsers[User, int](
    get_user_manager,
    [auth_backend],
)

# Current active user dependency
current_active_user = fastapi_users.current_user(active=True)

# Router
router = APIRouter(prefix="/auth", tags=["Authentication"])


# ============================================================================
# Authentication Endpoints
# ============================================================================

@router.post("/signup", response_model=AuthSuccessResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    signup_data: SignupRequest,
    session: AsyncSession = Depends(get_async_session),
    user_manager=Depends(get_user_manager),
):
    """
    Create new user account with background profile.

    Maps to: POST /auth/signup from contracts/auth-api.yaml

    Request Body:
        - email: User email (unique)
        - password: Password (minimum 8 characters)
        - background: BackgroundProfile (programming_experience, ros2_familiarity, hardware_access)

    Returns:
        - token: JWT session token
        - user_id: Created user ID
        - background: Echoed background profile
        - expires_at: Token expiration timestamp (7 days from now)

    Errors:
        - 400: Email already registered, invalid email, weak password
        - 500: Database error
    """
    try:
        # Create UserCreate schema from signup data
        user_create = UserCreate(
            email=signup_data.email,
            password=signup_data.password,
            background=signup_data.background,
        )

        # Create user (manager handles background profile storage)
        user = await user_manager.create(user_create)

        # Generate JWT token
        strategy = get_jwt_strategy()
        token = await strategy.write_token(user)

        # Calculate expiration
        expires_at = datetime.utcnow() + timedelta(days=SESSION_DURATION_DAYS)

        # Get background profile
        background = get_background_profile(user)

        return AuthSuccessResponse(
            token=token,
            user_id=user.id,
            background=background,
            expires_at=expires_at,
        )

    except Exception as e:
        # Log the actual error for debugging
        import traceback
        print(f"[SIGNUP ERROR] {type(e).__name__}: {str(e)}")
        print(traceback.format_exc())

        # Handle specific exceptions
        if "already exists" in str(e).lower() or "duplicate" in str(e).lower():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered. Try signing in instead.",
            )
        elif "password" in str(e).lower():
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Password does not meet security requirements.",
            )
        else:
            # Generic server error - include error type for debugging
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail=f"Server error: {type(e).__name__}. Please try again.",
            )


@router.post("/signin", response_model=AuthSuccessResponse)
async def signin(
    signin_data: SigninRequest,
    user_manager=Depends(get_user_manager),
):
    """
    Authenticate existing user.

    Maps to: POST /auth/signin from contracts/auth-api.yaml

    Request Body:
        - email: User email
        - password: Password

    Returns:
        - token: JWT session token
        - user_id: User ID
        - background: User background profile
        - expires_at: Token expiration timestamp (7 days from now)

    Errors:
        - 401: Invalid credentials
        - 500: Database error
    """
    try:
        # Authenticate user
        user = await user_manager.authenticate(
            credentials={"email": signin_data.email, "password": signin_data.password}
        )

        if user is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password",
            )

        # Generate JWT token
        strategy = get_jwt_strategy()
        token = await strategy.write_token(user)

        # Calculate expiration
        expires_at = datetime.utcnow() + timedelta(days=SESSION_DURATION_DAYS)

        # Get background profile
        background = get_background_profile(user)

        return AuthSuccessResponse(
            token=token,
            user_id=user.id,
            background=background,
            expires_at=expires_at,
        )

    except HTTPException:
        raise
    except Exception as e:
        # Log the actual error for debugging
        import traceback
        import sys
        print(f"[SIGNIN ERROR] {type(e).__name__}: {str(e)}", file=sys.stderr, flush=True)
        traceback.print_exc(file=sys.stderr)
        sys.stderr.flush()

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Server error: {type(e).__name__}. Please try again.",
        )


@router.post("/signout")
async def signout(user: User = Depends(current_active_user)):
    """
    End user session.

    Maps to: POST /auth/signout from contracts/auth-api.yaml

    Note: JWT tokens are stateless, so signout is client-side only.
    Client should delete the token from storage.

    Returns:
        - message: Success message
    """
    return {"message": "Signed out successfully"}


@router.get("/session", response_model=SessionResponse)
async def validate_session(user: User = Depends(current_active_user)):
    """
    Validate current session.

    Maps to: GET /auth/session from contracts/auth-api.yaml

    Returns:
        - user_id: User ID
        - email: User email
        - expires_at: Token expiration (calculated from JWT)
        - background: User background profile

    Errors:
        - 401: Invalid or expired session token
    """
    # Get background profile
    background = get_background_profile(user)

    # Calculate expiration (JWT tokens don't store expiration in user model)
    # For now, return current time + 7 days as approximation
    expires_at = datetime.utcnow() + timedelta(days=SESSION_DURATION_DAYS)

    return SessionResponse(
        user_id=user.id,
        email=user.email,
        expires_at=expires_at,
        background=background,
    )


@router.get("/background/{user_id}")
async def get_background(
    user_id: int,
    current_user: User = Depends(current_active_user),
):
    """
    Fetch user background profile.

    Maps to: GET /auth/background/{user_id} from contracts/auth-api.yaml

    Security: User can only access their own profile.

    Returns:
        BackgroundProfile (programming_experience, ros2_familiarity, hardware_access)

    Errors:
        - 401: Unauthorized
        - 403: Forbidden (accessing another user's profile)
    """
    # Verify user can only access their own profile
    if current_user.id != user_id:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Forbidden: Cannot access another user's profile",
        )

    # Get background profile
    background = get_background_profile(current_user)

    if background is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Background profile not found",
        )

    return background
