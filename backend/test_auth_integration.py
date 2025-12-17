"""
Integration Tests for Authentication Endpoints

Tests signup, signin, and session validation with proper async support.
Run with: pytest test_auth_integration.py -v
"""

import pytest
from httpx import AsyncClient


@pytest.mark.asyncio
async def test_health_check(async_client: AsyncClient):
    """Test health endpoint returns 200"""
    response = await async_client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"


@pytest.mark.asyncio
async def test_signup_beginner(async_client: AsyncClient, test_user_data):
    """Test signup with beginner profile"""
    signup_data = test_user_data["beginner"]

    response = await async_client.post("/auth/signup", json=signup_data)

    # Accept both 201 (created) and 400 (already exists)
    assert response.status_code in [201, 400]

    if response.status_code == 201:
        data = response.json()
        assert "token" in data
        assert "user_id" in data
        assert "background" in data
        assert "expires_at" in data
        assert data["background"]["programming_experience"] == "0-2 years"
    else:
        # User already exists from previous test run
        data = response.json()
        assert "already registered" in data["detail"].lower()


@pytest.mark.asyncio
async def test_signup_advanced(async_client: AsyncClient, test_user_data):
    """Test signup with advanced profile"""
    signup_data = test_user_data["advanced"]

    response = await async_client.post("/auth/signup", json=signup_data)

    # Accept both 201 (created) and 400 (already exists)
    assert response.status_code in [201, 400]

    if response.status_code == 201:
        data = response.json()
        assert "token" in data
        assert data["background"]["programming_experience"] == "10+ years"


@pytest.mark.asyncio
async def test_signin_success(async_client: AsyncClient, test_user_data):
    """Test signin with valid credentials"""
    # First ensure user exists (signup or skip if exists)
    signup_data = test_user_data["beginner"]
    await async_client.post("/auth/signup", json=signup_data)

    # Now signin
    signin_data = {
        "email": signup_data["email"],
        "password": signup_data["password"]
    }

    response = await async_client.post("/auth/signin", json=signin_data)
    assert response.status_code == 200

    data = response.json()
    assert "token" in data
    assert "user_id" in data
    assert "background" in data
    assert "expires_at" in data


@pytest.mark.asyncio
async def test_signin_invalid_credentials(async_client: AsyncClient):
    """Test signin with invalid credentials returns 401"""
    signin_data = {
        "email": "nonexistent@test.com",
        "password": "WrongPassword123!"
    }

    response = await async_client.post("/auth/signin", json=signin_data)
    assert response.status_code == 401

    data = response.json()
    assert "invalid" in data["detail"].lower()


@pytest.mark.asyncio
async def test_session_validation_valid_token(async_client: AsyncClient, test_user_data):
    """Test session validation with valid token"""
    # Signup/signin to get token
    signup_data = test_user_data["beginner"]
    response = await async_client.post("/auth/signup", json=signup_data)

    if response.status_code == 400:
        # User exists, signin instead
        signin_data = {
            "email": signup_data["email"],
            "password": signup_data["password"]
        }
        response = await async_client.post("/auth/signin", json=signin_data)

    assert response.status_code in [200, 201]
    token = response.json()["token"]

    # Validate session
    headers = {"Authorization": f"Bearer {token}"}
    response = await async_client.get("/auth/session", headers=headers)

    assert response.status_code == 200
    data = response.json()
    assert "user_id" in data
    assert "email" in data
    assert "background" in data


@pytest.mark.asyncio
async def test_session_validation_invalid_token(async_client: AsyncClient):
    """Test session validation with invalid token returns 401"""
    headers = {"Authorization": "Bearer invalid_token_12345"}
    response = await async_client.get("/auth/session", headers=headers)

    assert response.status_code == 401


@pytest.mark.asyncio
async def test_background_profile_fetch(async_client: AsyncClient, test_user_data):
    """Test background profile fetch endpoint"""
    # Signup/signin to get token and user_id
    signup_data = test_user_data["beginner"]
    response = await async_client.post("/auth/signup", json=signup_data)

    if response.status_code == 400:
        # User exists, signin instead
        signin_data = {
            "email": signup_data["email"],
            "password": signup_data["password"]
        }
        response = await async_client.post("/auth/signin", json=signin_data)

    data = response.json()
    token = data["token"]
    user_id = data["user_id"]

    # Fetch background profile
    headers = {"Authorization": f"Bearer {token}"}
    response = await async_client.get(f"/auth/background/{user_id}", headers=headers)

    assert response.status_code == 200
    background = response.json()
    assert "programming_experience" in background
    assert "ros2_familiarity" in background
    assert "hardware_access" in background


@pytest.mark.asyncio
async def test_background_profile_forbidden(async_client: AsyncClient, test_user_data):
    """Test background profile fetch for another user returns 403"""
    # Signin as beginner
    signup_data = test_user_data["beginner"]
    signin_data = {
        "email": signup_data["email"],
        "password": signup_data["password"]
    }
    response = await async_client.post("/auth/signin", json=signin_data)
    token = response.json()["token"]

    # Try to fetch another user's profile (user_id = 99999)
    headers = {"Authorization": f"Bearer {token}"}
    response = await async_client.get("/auth/background/99999", headers=headers)

    assert response.status_code == 403
    assert "forbidden" in response.json()["detail"].lower()


@pytest.mark.asyncio
async def test_signout(async_client: AsyncClient, test_user_data):
    """Test signout endpoint"""
    # Signin first
    signup_data = test_user_data["beginner"]
    signin_data = {
        "email": signup_data["email"],
        "password": signup_data["password"]
    }
    response = await async_client.post("/auth/signin", json=signin_data)
    token = response.json()["token"]

    # Signout
    headers = {"Authorization": f"Bearer {token}"}
    response = await async_client.post("/auth/signout", headers=headers)

    assert response.status_code == 200
    assert "message" in response.json()
