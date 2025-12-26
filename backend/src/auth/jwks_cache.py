"""
JWKS (JSON Web Key Set) fetching and caching module.

Fetches public keys from Better Auth service's JWKS endpoint and caches them
for efficient JWT signature validation.
"""

import httpx
import time
from typing import Dict, List, Optional
from datetime import datetime, timedelta
import os


class JWKSCache:
    """
    Caches JWKS (JSON Web Key Set) from Better Auth service.

    Features:
    - Fetches JWKS from /.well-known/jwks.json endpoint
    - Caches keys for 1 hour (configurable)
    - Auto-refreshes if requested key ID not found
    - Thread-safe singleton pattern
    """

    _instance: Optional['JWKSCache'] = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        """Initialize JWKS cache."""
        if self._initialized:
            return

        self._jwks: Optional[Dict] = None
        self._last_fetch: Optional[datetime] = None
        self._cache_duration = timedelta(hours=1)  # 1 hour cache
        self._better_auth_url = os.getenv(
            'BETTER_AUTH_URL',
            'http://localhost:3000'
        )
        self._jwks_endpoint = f'{self._better_auth_url}/.well-known/jwks.json'
        self._initialized = True

    async def get_jwks(self, force_refresh: bool = False) -> Dict:
        """
        Get JWKS, fetching from Better Auth if cache is stale or force refresh.

        Args:
            force_refresh: Force refetch even if cache is valid

        Returns:
            Dict containing JWKS with 'keys' array

        Raises:
            httpx.HTTPError: If JWKS fetch fails
        """
        now = datetime.now()

        # Check if cache is valid
        if not force_refresh and self._jwks and self._last_fetch:
            if now - self._last_fetch < self._cache_duration:
                return self._jwks

        # Fetch fresh JWKS
        async with httpx.AsyncClient() as client:
            response = await client.get(self._jwks_endpoint, timeout=10.0)
            response.raise_for_status()

            self._jwks = response.json()
            self._last_fetch = now

        return self._jwks

    async def get_key_by_id(self, kid: str) -> Optional[Dict]:
        """
        Get a specific key by its key ID (kid).

        If key not found in cache, refreshes JWKS once and tries again.

        Args:
            kid: Key ID to search for

        Returns:
            Key dict if found, None otherwise
        """
        # Try with current cache
        jwks = await self.get_jwks()
        key = self._find_key_by_id(jwks, kid)

        if key:
            return key

        # Key not found - refresh cache and try again
        jwks = await self.get_jwks(force_refresh=True)
        return self._find_key_by_id(jwks, kid)

    def _find_key_by_id(self, jwks: Dict, kid: str) -> Optional[Dict]:
        """
        Find a key in JWKS by its kid.

        Args:
            jwks: JWKS dict
            kid: Key ID to search for

        Returns:
            Key dict if found, None otherwise
        """
        keys = jwks.get('keys', [])
        for key in keys:
            if key.get('kid') == kid:
                return key
        return None

    async def get_all_keys(self) -> List[Dict]:
        """
        Get all keys from JWKS.

        Returns:
            List of key dicts
        """
        jwks = await self.get_jwks()
        return jwks.get('keys', [])

    def clear_cache(self):
        """Clear the JWKS cache, forcing a fresh fetch on next request."""
        self._jwks = None
        self._last_fetch = None


# Global singleton instance
jwks_cache = JWKSCache()
