"""
JWT Validation Middleware for Better Auth tokens.

Validates JWT tokens issued by Better Auth service using RS256 asymmetric signatures.
Fetches public keys from Better Auth JWKS endpoint for verification.
"""

import jwt
from jwt import PyJWKClient
from typing import Optional, Dict, Any
from fastapi import Request, HTTPException, status, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import os

from .jwks_cache import jwks_cache


# HTTP Bearer token security
security = HTTPBearer()


class JWTValidator:
    """
    Validates JWT tokens from Better Auth service.

    Uses RS256 asymmetric signatures with JWKS public key verification.
    """

    def __init__(self):
        """Initialize JWT validator with Better Auth configuration."""
        self.better_auth_url = os.getenv(
            'BETTER_AUTH_URL',
            'http://localhost:3000'
        )
        self.issuer = self.better_auth_url
        self.audience = self.better_auth_url
        self.algorithms = ['RS256']

    async def validate_token(self, token: str) -> Dict[str, Any]:
        """
        Validate JWT token and return decoded payload.

        Args:
            token: JWT token string

        Returns:
            Decoded JWT payload with claims

        Raises:
            HTTPException: 401 if token is invalid, expired, or signature fails
        """
        try:
            # Decode header without validation to get kid (key ID)
            unverified_header = jwt.get_unverified_header(token)
            kid = unverified_header.get('kid')

            if not kid:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail="JWT token missing key ID (kid) in header"
                )

            # Get the public key from JWKS
            key = await jwks_cache.get_key_by_id(kid)

            if not key:
                raise HTTPException(
                    status_code=status.HTTP_401_UNAUTHORIZED,
                    detail=f"Public key not found for kid: {kid}"
                )

            # Convert JWK to PEM format for PyJWT
            from jwt.algorithms import RSAAlgorithm
            public_key = RSAAlgorithm.from_jwk(key)

            # Validate and decode the JWT
            payload = jwt.decode(
                token,
                public_key,
                algorithms=self.algorithms,
                issuer=self.issuer,
                audience=self.audience,
                options={
                    'verify_signature': True,
                    'verify_exp': True,
                    'verify_iat': True,
                    'verify_iss': True,
                    'verify_aud': True,
                }
            )

            return payload

        except jwt.ExpiredSignatureError:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="JWT token has expired"
            )
        except jwt.InvalidTokenError as e:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail=f"Invalid JWT token: {str(e)}"
            )
        except Exception as e:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail=f"JWT validation failed: {str(e)}"
            )


# Global JWT validator instance
jwt_validator = JWTValidator()


async def get_jwt_payload(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> Dict[str, Any]:
    """
    FastAPI dependency to extract and validate JWT token.

    Args:
        credentials: HTTP Bearer token credentials

    Returns:
        Decoded JWT payload

    Raises:
        HTTPException: 401 if token missing or invalid
    """
    if not credentials:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Missing authorization token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    if credentials.scheme.lower() != "bearer":
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication scheme. Use Bearer token.",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Validate the token
    payload = await jwt_validator.validate_token(credentials.credentials)
    return payload


async def require_jwt_auth(
    payload: Dict[str, Any] = Depends(get_jwt_payload)
) -> Dict[str, Any]:
    """
    Dependency that requires valid JWT authentication.

    Use this in route handlers that need authentication.

    Args:
        payload: JWT payload (injected by get_jwt_payload)

    Returns:
        JWT payload with user information

    Example:
        @app.get("/protected")
        async def protected_route(payload: dict = Depends(require_jwt_auth)):
            user_id = payload["id"]
            user_email = payload["email"]
            return {"message": f"Hello {user_email}"}
    """
    return payload


async def get_optional_jwt(
    request: Request
) -> Optional[Dict[str, Any]]:
    """
    Extract and validate JWT token if present, return None if absent.

    Useful for routes that work for both authenticated and guest users.

    Args:
        request: FastAPI request

    Returns:
        JWT payload if token present and valid, None otherwise
    """
    auth_header = request.headers.get("Authorization")

    if not auth_header or not auth_header.startswith("Bearer "):
        return None

    try:
        token = auth_header.split(" ", 1)[1]
        payload = await jwt_validator.validate_token(token)
        return payload
    except HTTPException:
        # Invalid token - treat as unauthenticated
        return None
    except Exception:
        # Any other error - treat as unauthenticated
        return None
