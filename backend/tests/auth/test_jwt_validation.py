"""
Tests for JWT validation middleware and token processing.

Covers:
- JWT validation with valid/invalid tokens
- Expired token handling
- Invalid signature rejection
- JWKS key fetching and caching
"""

import pytest
import jwt
from datetime import datetime, timedelta
from typing import Dict, Any
from unittest.mock import AsyncMock, patch, MagicMock
from fastapi import HTTPException, status
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.backends import default_backend
import json

from backend.src.auth.jwt_middleware import (
    JWTValidator,
    get_jwt_payload,
    require_jwt_auth,
    get_optional_jwt
)
from backend.src.auth.jwks_cache import JWKSCache


# Test fixtures
@pytest.fixture
def rsa_key_pair():
    """Generate RSA key pair for testing."""
    private_key = rsa.generate_private_key(
        public_exponent=65537,
        key_size=2048,
        backend=default_backend()
    )
    public_key = private_key.public_key()
    return private_key, public_key


@pytest.fixture
def test_jwks(rsa_key_pair):
    """Create test JWKS with public key."""
    private_key, public_key = rsa_key_pair

    # Get public key in JWK format
    public_pem = public_key.public_bytes(
        encoding=serialization.Encoding.PEM,
        format=serialization.PublicFormat.SubjectPublicKeyInfo
    )

    # Convert to JWK (simplified for testing)
    from jwt.algorithms import RSAAlgorithm
    jwk = json.loads(RSAAlgorithm.to_jwk(public_key))
    jwk['kid'] = 'test-key-id'
    jwk['use'] = 'sig'
    jwk['alg'] = 'RS256'

    return {
        'keys': [jwk]
    }


@pytest.fixture
def valid_jwt_payload():
    """Create valid JWT payload."""
    return {
        'id': 'user123',
        'email': 'test@example.com',
        'name': 'Test User',
        'iss': 'http://localhost:3000',
        'aud': 'http://localhost:3000',
        'iat': int(datetime.utcnow().timestamp()),
        'exp': int((datetime.utcnow() + timedelta(minutes=15)).timestamp()),
        'https://yourdomain.com/claims': {
            'programming_experience': 'intermediate',
            'ros2_familiarity': 'basic',
            'hardware_access': 'simulation'
        }
    }


@pytest.fixture
def create_test_token(rsa_key_pair):
    """Factory fixture to create JWT tokens."""
    private_key, _ = rsa_key_pair

    def _create_token(payload: Dict[str, Any]) -> str:
        """Create a signed JWT token."""
        headers = {'kid': 'test-key-id'}
        return jwt.encode(
            payload,
            private_key,
            algorithm='RS256',
            headers=headers
        )

    return _create_token


# JWT Validator Tests
class TestJWTValidator:
    """Tests for JWTValidator class."""

    @pytest.mark.asyncio
    async def test_validate_valid_token(
        self,
        valid_jwt_payload,
        create_test_token,
        test_jwks
    ):
        """Test validation of a valid JWT token."""
        token = create_test_token(valid_jwt_payload)

        # Mock JWKS cache
        with patch('backend.src.auth.jwt_middleware.jwks_cache') as mock_cache:
            mock_cache.get_key_by_id = AsyncMock(return_value=test_jwks['keys'][0])

            validator = JWTValidator()
            payload = await validator.validate_token(token)

            assert payload['id'] == 'user123'
            assert payload['email'] == 'test@example.com'
            assert 'https://yourdomain.com/claims' in payload

    @pytest.mark.asyncio
    async def test_validate_expired_token(
        self,
        valid_jwt_payload,
        create_test_token,
        test_jwks
    ):
        """Test rejection of expired token."""
        # Create expired token
        expired_payload = valid_jwt_payload.copy()
        expired_payload['exp'] = int((datetime.utcnow() - timedelta(hours=1)).timestamp())

        token = create_test_token(expired_payload)

        # Mock JWKS cache
        with patch('backend.src.auth.jwt_middleware.jwks_cache') as mock_cache:
            mock_cache.get_key_by_id = AsyncMock(return_value=test_jwks['keys'][0])

            validator = JWTValidator()

            with pytest.raises(HTTPException) as exc_info:
                await validator.validate_token(token)

            assert exc_info.value.status_code == status.HTTP_401_UNAUTHORIZED
            assert "expired" in str(exc_info.value.detail).lower()

    @pytest.mark.asyncio
    async def test_validate_token_missing_kid(
        self,
        valid_jwt_payload,
        rsa_key_pair
    ):
        """Test rejection of token with missing kid header."""
        private_key, _ = rsa_key_pair

        # Create token without kid in header
        token = jwt.encode(
            valid_jwt_payload,
            private_key,
            algorithm='RS256'
            # No headers specified - no kid
        )

        validator = JWTValidator()

        with pytest.raises(HTTPException) as exc_info:
            await validator.validate_token(token)

        assert exc_info.value.status_code == status.HTTP_401_UNAUTHORIZED
        assert "missing key ID" in str(exc_info.value.detail).lower()

    @pytest.mark.asyncio
    async def test_validate_token_invalid_signature(
        self,
        valid_jwt_payload,
        create_test_token,
        test_jwks
    ):
        """Test rejection of token with invalid signature."""
        # Create a different key pair for signing
        wrong_private_key = rsa.generate_private_key(
            public_exponent=65537,
            key_size=2048,
            backend=default_backend()
        )

        # Sign with wrong key
        headers = {'kid': 'test-key-id'}
        token = jwt.encode(
            valid_jwt_payload,
            wrong_private_key,
            algorithm='RS256',
            headers=headers
        )

        # Mock JWKS cache to return the original public key
        with patch('backend.src.auth.jwt_middleware.jwks_cache') as mock_cache:
            mock_cache.get_key_by_id = AsyncMock(return_value=test_jwks['keys'][0])

            validator = JWTValidator()

            with pytest.raises(HTTPException) as exc_info:
                await validator.validate_token(token)

            assert exc_info.value.status_code == status.HTTP_401_UNAUTHORIZED

    @pytest.mark.asyncio
    async def test_validate_token_key_not_found(
        self,
        valid_jwt_payload,
        create_test_token
    ):
        """Test rejection when public key not found in JWKS."""
        token = create_test_token(valid_jwt_payload)

        # Mock JWKS cache to return None (key not found)
        with patch('backend.src.auth.jwt_middleware.jwks_cache') as mock_cache:
            mock_cache.get_key_by_id = AsyncMock(return_value=None)

            validator = JWTValidator()

            with pytest.raises(HTTPException) as exc_info:
                await validator.validate_token(token)

            assert exc_info.value.status_code == status.HTTP_401_UNAUTHORIZED
            assert "not found" in str(exc_info.value.detail).lower()


# Dependency Injection Tests
class TestJWTDependencies:
    """Tests for FastAPI JWT dependency injection functions."""

    @pytest.mark.asyncio
    async def test_get_jwt_payload_valid_token(
        self,
        valid_jwt_payload,
        create_test_token,
        test_jwks
    ):
        """Test get_jwt_payload with valid Bearer token."""
        token = create_test_token(valid_jwt_payload)

        # Mock credentials
        from fastapi.security import HTTPAuthorizationCredentials
        credentials = HTTPAuthorizationCredentials(
            scheme="Bearer",
            credentials=token
        )

        # Mock JWKS cache
        with patch('backend.src.auth.jwt_middleware.jwks_cache') as mock_cache:
            mock_cache.get_key_by_id = AsyncMock(return_value=test_jwks['keys'][0])

            payload = await get_jwt_payload(credentials)

            assert payload['id'] == 'user123'
            assert payload['email'] == 'test@example.com'

    @pytest.mark.asyncio
    async def test_get_jwt_payload_missing_credentials(self):
        """Test get_jwt_payload with missing credentials."""
        with pytest.raises(HTTPException) as exc_info:
            await get_jwt_payload(None)

        assert exc_info.value.status_code == status.HTTP_401_UNAUTHORIZED
        assert "missing" in str(exc_info.value.detail).lower()

    @pytest.mark.asyncio
    async def test_get_jwt_payload_wrong_scheme(self):
        """Test get_jwt_payload with non-Bearer scheme."""
        from fastapi.security import HTTPAuthorizationCredentials
        credentials = HTTPAuthorizationCredentials(
            scheme="Basic",
            credentials="some-token"
        )

        with pytest.raises(HTTPException) as exc_info:
            await get_jwt_payload(credentials)

        assert exc_info.value.status_code == status.HTTP_401_UNAUTHORIZED
        assert "scheme" in str(exc_info.value.detail).lower()

    @pytest.mark.asyncio
    async def test_require_jwt_auth(self, valid_jwt_payload):
        """Test require_jwt_auth dependency."""
        # This just passes through the payload
        result = await require_jwt_auth(valid_jwt_payload)
        assert result == valid_jwt_payload

    @pytest.mark.asyncio
    async def test_get_optional_jwt_with_valid_token(
        self,
        valid_jwt_payload,
        create_test_token,
        test_jwks
    ):
        """Test get_optional_jwt with valid token."""
        token = create_test_token(valid_jwt_payload)

        # Mock request
        from fastapi import Request
        mock_request = MagicMock(spec=Request)
        mock_request.headers.get.return_value = f"Bearer {token}"

        # Mock JWKS cache
        with patch('backend.src.auth.jwt_middleware.jwks_cache') as mock_cache:
            mock_cache.get_key_by_id = AsyncMock(return_value=test_jwks['keys'][0])

            payload = await get_optional_jwt(mock_request)

            assert payload is not None
            assert payload['id'] == 'user123'

    @pytest.mark.asyncio
    async def test_get_optional_jwt_without_token(self):
        """Test get_optional_jwt without token (returns None)."""
        from fastapi import Request
        mock_request = MagicMock(spec=Request)
        mock_request.headers.get.return_value = None

        payload = await get_optional_jwt(mock_request)
        assert payload is None

    @pytest.mark.asyncio
    async def test_get_optional_jwt_with_invalid_token(
        self,
        test_jwks
    ):
        """Test get_optional_jwt with invalid token (returns None)."""
        from fastapi import Request
        mock_request = MagicMock(spec=Request)
        mock_request.headers.get.return_value = "Bearer invalid-token"

        # Mock JWKS cache
        with patch('backend.src.auth.jwt_middleware.jwks_cache') as mock_cache:
            mock_cache.get_key_by_id = AsyncMock(return_value=test_jwks['keys'][0])

            payload = await get_optional_jwt(mock_request)
            assert payload is None


# Integration Tests
class TestJWTValidationIntegration:
    """Integration tests for full JWT validation flow."""

    @pytest.mark.asyncio
    async def test_full_validation_flow(
        self,
        valid_jwt_payload,
        create_test_token,
        test_jwks
    ):
        """Test complete validation flow from token to payload extraction."""
        token = create_test_token(valid_jwt_payload)

        from fastapi.security import HTTPAuthorizationCredentials
        credentials = HTTPAuthorizationCredentials(
            scheme="Bearer",
            credentials=token
        )

        # Mock JWKS cache
        with patch('backend.src.auth.jwt_middleware.jwks_cache') as mock_cache:
            mock_cache.get_key_by_id = AsyncMock(return_value=test_jwks['keys'][0])

            # Full flow: get_jwt_payload -> require_jwt_auth
            payload = await get_jwt_payload(credentials)
            final_payload = await require_jwt_auth(payload)

            assert final_payload['id'] == 'user123'
            assert final_payload['email'] == 'test@example.com'
            assert final_payload['https://yourdomain.com/claims']['programming_experience'] == 'intermediate'
