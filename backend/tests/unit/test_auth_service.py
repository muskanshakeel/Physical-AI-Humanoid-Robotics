import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from sqlalchemy.ext.asyncio import AsyncSession
from datetime import datetime
from backend.src.services.auth_service import AuthService
from backend.src.models.user import User
from backend.src.models.session import Session
from backend.src.models.request import UserRegisterRequest, UserLoginRequest
from fastapi import HTTPException


@pytest.fixture
def mock_db_session():
    """Create a mock database session for testing"""
    mock_db = AsyncMock(spec=AsyncSession)
    return mock_db


@pytest.mark.asyncio
async def test_register_user_success(mock_db_session):
    """Test successful user registration"""
    # Mock input data
    user_data = UserRegisterRequest(
        email='test@example.com',
        password='securepassword123',
        name='Test User'
    )

    # Create a mock result object for the database query
    mock_result = MagicMock()
    mock_result.scalars.return_value.first.return_value = None  # No existing user

    # Mock the database execute method to return the mock result
    mock_db_session.execute.return_value = mock_result

    # Create a mock user with id to be returned after commit/refresh
    mock_user = User(
        id='test-user-id-123',
        email=user_data.email,
        name=user_data.name,
        hashed_password='hashed_password',
        created_at=datetime.utcnow()
    )

    # Mock the refresh to update the user object with id
    with patch('backend.src.services.auth_service.AuthService.get_password_hash') as mock_hash, \
         patch('backend.src.middleware.auth.AuthMiddleware.create_access_token') as mock_token, \
         patch('uuid.uuid4', return_value='test-user-id-123'):

        mock_hash.return_value = 'hashed_password'
        mock_token.return_value = 'mock_jwt_token'

        # Mock the session addition and commit behavior
        def add_side_effect(obj):
            if hasattr(obj, '__tablename__') and obj.__tablename__ == 'users':
                # When a user is added, set its id
                obj.id = 'test-user-id-123'

        mock_db_session.add.side_effect = add_side_effect
        mock_db_session.refresh = AsyncMock()

        result = await AuthService.register_user(mock_db_session, user_data)

        # Verify the result
        assert result.success is True
        assert result.user.email == user_data.email
        assert result.user.name == user_data.name
        assert result.session_token == 'mock_jwt_token'
        assert mock_db_session.add.called
        assert mock_db_session.commit.called


@pytest.mark.asyncio
async def test_register_user_duplicate_email(mock_db_session):
    """Test registration with duplicate email"""
    user_data = UserRegisterRequest(
        email='existing@example.com',
        password='password123',
        name='Test User'
    )

    # Mock existing user
    existing_user = User(
        id=1,
        email=user_data.email,
        name='Existing User',
        hashed_password='hashed',
        created_at=None
    )

    # Create a mock result object for the database query
    mock_result = MagicMock()
    mock_result.scalars.return_value.first.return_value = existing_user
    mock_db_session.execute.return_value = mock_result

    with pytest.raises(HTTPException) as exc_info:
        await AuthService.register_user(mock_db_session, user_data)

    # Verify the exception
    assert exc_info.value.status_code == 409
    assert "User already exists" in exc_info.value.detail


@pytest.mark.asyncio
async def test_login_success(mock_db_session):
    """Test successful user login"""
    login_data = UserLoginRequest(
        email='test@example.com',
        password='securepassword123'
    )

    # Mock user data with hashed password
    user = User(
        id='test-user-id-456',
        email=login_data.email,
        name='Test User',
        hashed_password='$2b$12$34567890123456789012345678901234567890123456789012345',  # bcrypt hash
        created_at=datetime.utcnow(),
        is_active=True  # User must be active
    )

    # Create a mock result object for the database query
    mock_result = MagicMock()
    mock_result.scalars.return_value.first.return_value = user
    mock_db_session.execute.return_value = mock_result

    with patch('backend.src.services.auth_service.AuthService.verify_password', return_value=True), \
         patch('backend.src.middleware.auth.AuthMiddleware.create_access_token') as mock_token:

        mock_token.return_value = 'mock_jwt_token'

        result = await AuthService.login_user(mock_db_session, login_data)

        # Verify the result
        assert result.success is True
        assert result.user.email == login_data.email
        assert result.session_token == 'mock_jwt_token'


@pytest.mark.asyncio
async def test_login_invalid_credentials(mock_db_session):
    """Test login with invalid credentials"""
    login_data = UserLoginRequest(
        email='test@example.com',
        password='wrongpassword'
    )

    # Mock user with correct email but different password
    user = User(
        id=1,
        email=login_data.email,
        name='Test User',
        hashed_password='$2b$12$34567890123456789012345678901234567890123456789012345',  # bcrypt hash
        created_at=None
    )

    # Create a mock result object for the database query
    mock_result = MagicMock()
    mock_result.scalars.return_value.first.return_value = user
    mock_db_session.execute.return_value = mock_result

    with patch('backend.src.services.auth_service.AuthService.verify_password', return_value=False):
        with pytest.raises(HTTPException) as exc_info:
            await AuthService.login_user(mock_db_session, login_data)

        # Verify the exception
        assert exc_info.value.status_code == 401
        assert "Invalid credentials" in exc_info.value.detail


@pytest.mark.asyncio
async def test_login_user_not_found(mock_db_session):
    """Test login with non-existent user"""
    login_data = UserLoginRequest(
        email='nonexistent@example.com',
        password='password123'
    )

    # Create a mock result object for the database query
    mock_result = MagicMock()
    mock_result.scalars.return_value.first.return_value = None
    mock_db_session.execute.return_value = mock_result

    with pytest.raises(HTTPException) as exc_info:
        await AuthService.login_user(mock_db_session, login_data)

    # Verify the exception
    assert exc_info.value.status_code == 401
    assert "Invalid credentials" in exc_info.value.detail


@patch('backend.src.services.auth_service.pwd_context')
def test_verify_password_correct(mock_pwd_context):
    """Test password verification with correct password"""
    # Mock the pwd_context
    mock_pwd_context.verify.return_value = True

    password = 'shortpass123'
    hashed = 'mock_hashed_password'

    result = AuthService.verify_password(password, hashed)

    assert result is True
    mock_pwd_context.verify.assert_called_once_with(password, hashed)


@patch('backend.src.services.auth_service.pwd_context')
def test_verify_password_incorrect(mock_pwd_context):
    """Test password verification with incorrect password"""
    # Mock the pwd_context
    mock_pwd_context.verify.return_value = False

    password = 'shortpass123'
    wrong_password = 'wrongpass456'
    hashed = 'mock_hashed_password'

    result = AuthService.verify_password(wrong_password, hashed)

    assert result is False
    mock_pwd_context.verify.assert_called_once_with(wrong_password, hashed)


@patch('backend.src.services.auth_service.pwd_context')
def test_get_password_hash(mock_pwd_context):
    """Test password hashing function"""
    # Mock the pwd_context
    mock_pwd_context.hash.return_value = 'mock_hashed_password'

    password = 'shortpass123'

    hashed = AuthService.get_password_hash(password)

    assert hashed == 'mock_hashed_password'
    mock_pwd_context.hash.assert_called_once_with(password)