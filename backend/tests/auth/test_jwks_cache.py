"""
Tests for JWKS caching and fetching functionality.

Covers:
- JWKS fetching from Better Auth endpoint
- Cache TTL and expiration
- Key lookup by ID
- Cache refresh on key not found
- Error handling
"""

import pytest
import httpx
from datetime import datetime, timedelta
from unittest.mock import AsyncMock, patch, MagicMock
from backend.src.auth.jwks_cache import JWKSCache


# Test fixtures
@pytest.fixture
def sample_jwks():
    """Create sample JWKS response."""
    return {
        'keys': [
            {
                'kid': 'key-1',
                'kty': 'RSA',
                'use': 'sig',
                'alg': 'RS256',
                'n': 'sample-modulus-1',
                'e': 'AQAB'
            },
            {
                'kid': 'key-2',
                'kty': 'RSA',
                'use': 'sig',
                'alg': 'RS256',
                'n': 'sample-modulus-2',
                'e': 'AQAB'
            }
        ]
    }


@pytest.fixture
def fresh_cache():
    """Create a fresh JWKS cache instance."""
    cache = JWKSCache()
    cache.clear_cache()
    return cache


# JWKS Fetching Tests
class TestJWKSFetching:
    """Tests for JWKS fetching from Better Auth."""

    @pytest.mark.asyncio
    async def test_fetch_jwks_success(self, fresh_cache, sample_jwks):
        """Test successful JWKS fetch."""
        with patch('httpx.AsyncClient') as mock_client_class:
            # Mock the async context manager
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.json.return_value = sample_jwks
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            jwks = await fresh_cache.get_jwks()

            assert jwks == sample_jwks
            assert len(jwks['keys']) == 2
            assert jwks['keys'][0]['kid'] == 'key-1'

    @pytest.mark.asyncio
    async def test_fetch_jwks_network_error(self, fresh_cache):
        """Test JWKS fetch with network error."""
        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_client.get.side_effect = httpx.RequestError("Connection failed")
            mock_client_class.return_value.__aenter__.return_value = mock_client

            with pytest.raises(httpx.RequestError):
                await fresh_cache.get_jwks()

    @pytest.mark.asyncio
    async def test_fetch_jwks_http_error(self, fresh_cache):
        """Test JWKS fetch with HTTP error (404, 500, etc.)."""
        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.raise_for_status.side_effect = httpx.HTTPStatusError(
                "Not Found",
                request=MagicMock(),
                response=MagicMock(status_code=404)
            )
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            with pytest.raises(httpx.HTTPStatusError):
                await fresh_cache.get_jwks()


# JWKS Caching Tests
class TestJWKSCaching:
    """Tests for JWKS caching behavior."""

    @pytest.mark.asyncio
    async def test_cache_reuses_cached_jwks(self, fresh_cache, sample_jwks):
        """Test that cache returns cached JWKS without refetching."""
        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.json.return_value = sample_jwks
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            # First fetch
            jwks1 = await fresh_cache.get_jwks()

            # Second fetch (should use cache)
            jwks2 = await fresh_cache.get_jwks()

            # Should only call get once
            mock_client.get.assert_called_once()
            assert jwks1 == jwks2

    @pytest.mark.asyncio
    async def test_cache_expires_after_ttl(self, fresh_cache, sample_jwks):
        """Test that cache expires after TTL."""
        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.json.return_value = sample_jwks
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            # First fetch
            await fresh_cache.get_jwks()

            # Simulate time passing (set last_fetch to past)
            fresh_cache._last_fetch = datetime.now() - timedelta(hours=2)

            # Second fetch (should refetch due to expiration)
            await fresh_cache.get_jwks()

            # Should call get twice
            assert mock_client.get.call_count == 2

    @pytest.mark.asyncio
    async def test_force_refresh_bypasses_cache(self, fresh_cache, sample_jwks):
        """Test that force_refresh parameter bypasses cache."""
        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.json.return_value = sample_jwks
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            # First fetch
            await fresh_cache.get_jwks()

            # Force refresh
            await fresh_cache.get_jwks(force_refresh=True)

            # Should call get twice
            assert mock_client.get.call_count == 2

    @pytest.mark.asyncio
    async def test_clear_cache(self, fresh_cache, sample_jwks):
        """Test cache clearing."""
        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.json.return_value = sample_jwks
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            # Fetch and cache
            await fresh_cache.get_jwks()

            # Clear cache
            fresh_cache.clear_cache()

            # Next fetch should call get again
            await fresh_cache.get_jwks()

            assert mock_client.get.call_count == 2


# Key Lookup Tests
class TestKeyLookup:
    """Tests for key lookup by ID."""

    @pytest.mark.asyncio
    async def test_get_key_by_id_found(self, fresh_cache, sample_jwks):
        """Test successful key lookup."""
        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.json.return_value = sample_jwks
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            key = await fresh_cache.get_key_by_id('key-1')

            assert key is not None
            assert key['kid'] == 'key-1'
            assert key['n'] == 'sample-modulus-1'

    @pytest.mark.asyncio
    async def test_get_key_by_id_not_found_refreshes(self, fresh_cache):
        """Test that missing key triggers cache refresh."""
        sample_jwks_initial = {
            'keys': [
                {'kid': 'key-1', 'kty': 'RSA', 'n': 'modulus-1', 'e': 'AQAB'}
            ]
        }

        sample_jwks_updated = {
            'keys': [
                {'kid': 'key-1', 'kty': 'RSA', 'n': 'modulus-1', 'e': 'AQAB'},
                {'kid': 'key-2', 'kty': 'RSA', 'n': 'modulus-2', 'e': 'AQAB'}
            ]
        }

        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()

            # First call returns initial JWKS
            # Second call (after refresh) returns updated JWKS
            mock_response.json.side_effect = [sample_jwks_initial, sample_jwks_updated]
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            # Try to get key-2 (not in initial cache)
            key = await fresh_cache.get_key_by_id('key-2')

            # Should find it after refresh
            assert key is not None
            assert key['kid'] == 'key-2'

            # Should have called get twice (initial + refresh)
            assert mock_client.get.call_count == 2

    @pytest.mark.asyncio
    async def test_get_key_by_id_never_exists(self, fresh_cache, sample_jwks):
        """Test lookup for key that never exists."""
        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.json.return_value = sample_jwks
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            key = await fresh_cache.get_key_by_id('nonexistent-key')

            # Should return None
            assert key is None

            # Should have called get twice (initial + refresh attempt)
            assert mock_client.get.call_count == 2

    @pytest.mark.asyncio
    async def test_get_all_keys(self, fresh_cache, sample_jwks):
        """Test getting all keys from JWKS."""
        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.json.return_value = sample_jwks
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            keys = await fresh_cache.get_all_keys()

            assert len(keys) == 2
            assert keys[0]['kid'] == 'key-1'
            assert keys[1]['kid'] == 'key-2'

    @pytest.mark.asyncio
    async def test_get_all_keys_empty_jwks(self, fresh_cache):
        """Test getting all keys when JWKS is empty."""
        empty_jwks = {'keys': []}

        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.json.return_value = empty_jwks
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            keys = await fresh_cache.get_all_keys()

            assert len(keys) == 0


# Singleton Pattern Tests
class TestJWKSCacheSingleton:
    """Tests for singleton pattern of JWKS cache."""

    def test_singleton_returns_same_instance(self):
        """Test that multiple instantiations return the same instance."""
        cache1 = JWKSCache()
        cache2 = JWKSCache()

        assert cache1 is cache2

    @pytest.mark.asyncio
    async def test_singleton_shares_cache(self, sample_jwks):
        """Test that singleton instances share the same cache."""
        cache1 = JWKSCache()
        cache2 = JWKSCache()

        cache1.clear_cache()

        with patch('httpx.AsyncClient') as mock_client_class:
            mock_client = AsyncMock()
            mock_response = AsyncMock()
            mock_response.json.return_value = sample_jwks
            mock_response.raise_for_status = MagicMock()
            mock_client.get.return_value = mock_response
            mock_client_class.return_value.__aenter__.return_value = mock_client

            # Fetch with cache1
            await cache1.get_jwks()

            # cache2 should have the same cached data
            jwks = await cache2.get_jwks()

            # Should only call get once (cached)
            mock_client.get.assert_called_once()
            assert jwks == sample_jwks


# Configuration Tests
class TestJWKSCacheConfiguration:
    """Tests for JWKS cache configuration."""

    def test_default_better_auth_url(self, fresh_cache):
        """Test default Better Auth URL configuration."""
        assert fresh_cache._better_auth_url == 'http://localhost:3000'
        assert fresh_cache._jwks_endpoint == 'http://localhost:3000/.well-known/jwks.json'

    def test_custom_better_auth_url(self, monkeypatch):
        """Test custom Better Auth URL from environment."""
        monkeypatch.setenv('BETTER_AUTH_URL', 'https://auth.example.com')

        cache = JWKSCache()
        cache.clear_cache()
        cache._initialized = False
        cache.__init__()

        assert cache._better_auth_url == 'https://auth.example.com'
        assert cache._jwks_endpoint == 'https://auth.example.com/.well-known/jwks.json'

    def test_cache_duration_default(self, fresh_cache):
        """Test default cache duration is 1 hour."""
        assert fresh_cache._cache_duration == timedelta(hours=1)
