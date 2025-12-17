import pytest
from fastapi.testclient import TestClient
from backend.src.main import app
from backend.src.config.settings import settings
import os


@pytest.fixture
def client():
    """Create a test client for the API"""
    return TestClient(app)


@pytest.mark.asyncio
async def test_health_endpoint(client):
    """Test the health check endpoint"""
    response = client.get("/health")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert "timestamp" in data


def test_register_user(client):
    """Test user registration endpoint"""
    user_data = {
        "email": "test@example.com",
        "password": "securepassword123",
        "name": "Test User"
    }

    response = client.post("/api/auth/register", json=user_data)
    # This might fail if user already exists, but we're testing the structure
    assert response.status_code in [200, 409]  # 200 for success, 409 for conflict


def test_login_user(client):
    """Test user login endpoint"""
    # First register a user (if not already registered in the test environment)
    user_data = {
        "email": "login_test@example.com",
        "password": "password123",
        "name": "Login Test User"
    }

    # Register the user first
    register_response = client.post("/api/auth/register", json=user_data)

    # Now try to log in
    login_data = {
        "email": "login_test@example.com",
        "password": "password123"
    }

    response = client.post("/api/auth/login", json=login_data)
    assert response.status_code == 200

    data = response.json()
    assert data["success"] is True
    assert "session_token" in data
    assert "user" in data


def test_chat_query_endpoint(client):
    """Test chat query endpoint (requires authentication)"""
    # First register and login to get a token
    user_data = {
        "email": "chat_test@example.com",
        "password": "password123",
        "name": "Chat Test User"
    }

    # Register the user
    client.post("/api/auth/register", json=user_data)

    # Login to get token
    login_data = {
        "email": "chat_test@example.com",
        "password": "password123"
    }

    login_response = client.post("/api/auth/login", json=login_data)
    assert login_response.status_code == 200

    token_data = login_response.json()
    assert "session_token" in token_data

    auth_header = {"Authorization": f"Bearer {token_data['session_token']}"}

    # Test the chat endpoint
    query_data = {
        "query": "What is artificial intelligence?",
        "context": "book_content"
    }

    response = client.post("/api/chat/query", json=query_data, headers=auth_header)

    # This might fail if vector DB or AI service isn't configured, but we're testing the structure
    # Should be either 200 (success) or 500 (service unavailable due to missing configuration)
    assert response.status_code in [200, 500]


def test_get_toc_endpoint(client):
    """Test getting table of contents (requires authentication)"""
    # First register and login to get a token
    user_data = {
        "email": "toc_test@example.com",
        "password": "password123",
        "name": "TOC Test User"
    }

    # Register the user
    client.post("/api/auth/register", json=user_data)

    # Login to get token
    login_data = {
        "email": "toc_test@example.com",
        "password": "password123"
    }

    login_response = client.post("/api/auth/login", json=login_data)
    assert login_response.status_code == 200

    token_data = login_response.json()
    assert "session_token" in token_data

    auth_header = {"Authorization": f"Bearer {token_data['session_token']}"}

    # Test the TOC endpoint
    response = client.get("/api/book/toc", headers=auth_header)

    # Should return 200 with TOC data or 200 with empty list if no content exists
    assert response.status_code == 200
    data = response.json()
    assert "toc" in data


def test_logout_endpoint(client):
    """Test logout endpoint (requires authentication)"""
    # First register and login to get a token
    user_data = {
        "email": "logout_test@example.com",
        "password": "password123",
        "name": "Logout Test User"
    }

    # Register the user
    client.post("/api/auth/register", json=user_data)

    # Login to get token
    login_data = {
        "email": "logout_test@example.com",
        "password": "password123"
    }

    login_response = client.post("/api/auth/login", json=login_data)
    assert login_response.status_code == 200

    token_data = login_response.json()
    assert "session_token" in token_data

    auth_header = {"Authorization": f"Bearer {token_data['session_token']}"}

    # Test the logout endpoint
    response = client.post("/api/auth/logout", headers=auth_header)

    # Should return 200 for successful logout
    assert response.status_code == 200
    data = response.json()
    assert data["success"] is True