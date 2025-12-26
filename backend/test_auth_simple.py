"""
Simple Authentication Endpoint Tests

Uses the actual FastAPI server to avoid async test issues.
Run with: pytest test_auth_simple.py -v
"""

import pytest
import requests
import time
import subprocess
import sys
from pathlib import Path
import uuid

# Test configuration
BASE_URL = "http://localhost:8000"
TIMEOUT = 5


@pytest.fixture(scope="module")
def server():
    """Start FastAPI server for testing"""
    # Start server in background
    print("\n[INFO] Starting FastAPI server...")
    proc = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"],
        cwd=Path(__file__).parent,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Wait for server to start
    for i in range(30):
        try:
            response = requests.get(f"{BASE_URL}/health", timeout=2)
            if response.status_code == 200:
                print("[INFO] Server is ready!")
                break
        except requests.exceptions.RequestException:
            time.sleep(1)
    else:
        proc.kill()
        raise RuntimeError("Server failed to start")

    yield proc

    # Cleanup
    print("\n[INFO] Stopping server...")
    proc.terminate()
    proc.wait(timeout=5)


def test_health(server):
    """Test health endpoint"""
    response = requests.get(f"{BASE_URL}/health", timeout=TIMEOUT)
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"


def test_signup_and_signin(server):
    """Test signup and signin flow with unique user"""
    # Use unique email to avoid conflicts
    unique_id = str(uuid.uuid4())[:8]
    test_email = f"test_{unique_id}@test.com"

    # Test signup
    signup_data = {
        "email": test_email,
        "password": "TestPass123!",
        "background": {
            "programming_experience": "3-5 years",
            "ros2_familiarity": "Intermediate",
            "hardware_access": "Simulation only (Gazebo/Isaac Sim)"
        }
    }

    response = requests.post(f"{BASE_URL}/auth/signup", json=signup_data, timeout=TIMEOUT)
    assert response.status_code == 201

    data = response.json()
    assert "token" in data
    assert "user_id" in data
    assert "background" in data

    token = data["token"]
    user_id = data["user_id"]

    # Test signin
    signin_data = {
        "email": test_email,
        "password": "TestPass123!"
    }

    response = requests.post(f"{BASE_URL}/auth/signin", json=signin_data, timeout=TIMEOUT)
    assert response.status_code == 200

    data = response.json()
    assert "token" in data
    assert data["user_id"] == user_id


def test_invalid_credentials(server):
    """Test signin with invalid credentials"""
    signin_data = {
        "email": "nonexistent@test.com",
        "password": "WrongPass123!"
    }

    response = requests.post(f"{BASE_URL}/auth/signin", json=signin_data, timeout=TIMEOUT)
    assert response.status_code == 401


def test_session_validation(server):
    """Test session validation with valid and invalid tokens"""
    # Test invalid token
    headers = {"Authorization": "Bearer invalid_token"}
    response = requests.get(f"{BASE_URL}/auth/session", headers=headers, timeout=TIMEOUT)
    assert response.status_code == 401

    # Create a user and get valid token
    unique_id = str(uuid.uuid4())[:8]
    signup_data = {
        "email": f"session_test_{unique_id}@test.com",
        "password": "TestPass123!",
        "background": {
            "programming_experience": "0-2 years",
            "ros2_familiarity": "None",
            "hardware_access": "None"
        }
    }

    response = requests.post(f"{BASE_URL}/auth/signup", json=signup_data, timeout=TIMEOUT)
    assert response.status_code == 201
    token = response.json()["token"]

    # Test valid token
    headers = {"Authorization": f"Bearer {token}"}
    response = requests.get(f"{BASE_URL}/auth/session", headers=headers, timeout=TIMEOUT)
    assert response.status_code == 200

    data = response.json()
    assert "user_id" in data
    assert "email" in data
    assert "background" in data


def test_background_profile(server):
    """Test background profile fetch"""
    # Create user
    unique_id = str(uuid.uuid4())[:8]
    signup_data = {
        "email": f"profile_test_{unique_id}@test.com",
        "password": "TestPass123!",
        "background": {
            "programming_experience": "10+ years",
            "ros2_familiarity": "Advanced",
            "hardware_access": "Physical robots/sensors"
        }
    }

    response = requests.post(f"{BASE_URL}/auth/signup", json=signup_data, timeout=TIMEOUT)
    data = response.json()
    token = data["token"]
    user_id = data["user_id"]

    # Fetch background profile
    headers = {"Authorization": f"Bearer {token}"}
    response = requests.get(f"{BASE_URL}/auth/background/{user_id}", headers=headers, timeout=TIMEOUT)
    assert response.status_code == 200

    background = response.json()
    assert background["programming_experience"] == "10+ years"
    assert background["ros2_familiarity"] == "Advanced"
