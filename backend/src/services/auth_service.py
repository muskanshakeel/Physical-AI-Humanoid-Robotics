from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from passlib.context import CryptContext
from datetime import datetime, timedelta
import jwt
import uuid
from ..models.user import User
from ..models.session import Session
from ..config.settings import settings
from ..middleware.auth import AuthMiddleware
from ..models.request import UserRegisterRequest, UserLoginRequest
from ..models.response import UserResponse, AuthResponse, LoginResponse
from fastapi import HTTPException, status
import os

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

class AuthService:
    """
    Authentication service for handling user registration, login, and session management
    """

    @staticmethod
    def verify_password(plain_password: str, hashed_password: str) -> bool:
        """
        Verify a plain password against a hashed password
        """
        return pwd_context.verify(plain_password, hashed_password)

    @staticmethod
    def get_password_hash(password: str) -> str:
        """
        Hash a password
        """
        # Handle bcrypt 72-byte password limit
        password_bytes = password.encode('utf-8')
        if len(password_bytes) >= 72:  # Use >= to ensure we stay under the limit
            # Truncate to 71 bytes to stay under bcrypt's 72-byte limit with safety margin
            password = password_bytes[:71].decode('utf-8', errors='ignore')

        try:
            return pwd_context.hash(password)
        except ValueError as e:
            if "password cannot be longer than 72 bytes" in str(e):
                # Fallback: ensure password is definitely under limit
                truncated_password = password[:50]  # Extra safety
                return pwd_context.hash(truncated_password)
            else:
                raise e

    @staticmethod
    async def register_user(db: AsyncSession, user_data: UserRegisterRequest) -> AuthResponse:
        """
        Register a new user
        """
        # Check if user already exists
        result = await db.execute(select(User).filter(User.email == user_data.email))
        existing_user = result.scalars().first()

        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_409_CONFLICT,
                detail="User already exists"
            )

        # Hash the password (the get_password_hash method handles length validation)
        hashed_password = AuthService.get_password_hash(user_data.password)

        # Create new user
        db_user = User(
            email=user_data.email,
            name=user_data.name,
            hashed_password=hashed_password
        )

        db.add(db_user)
        await db.commit()
        await db.refresh(db_user)

        # Create session token
        access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
        session_id = str(uuid.uuid4())  # Generate a session ID
        session_token = AuthMiddleware.create_access_token(
            data={"sub": db_user.id, "session_id": session_id}, expires_delta=access_token_expires
        )

        # Create session record
        session = Session(
            user_id=db_user.id,
            session_token=session_token,
            expires_at=datetime.utcnow() + access_token_expires
        )

        db.add(session)
        await db.commit()

        # Create response
        user_response = UserResponse(
            id=db_user.id,
            email=db_user.email,
            name=db_user.name,
            created_at=db_user.created_at
        )

        auth_response = AuthResponse(
            success=True,
            user=user_response,
            session_token=session_token,
            expires_at=datetime.utcnow() + access_token_expires
        )

        return auth_response

    @staticmethod
    async def login_user(db: AsyncSession, login_data: UserLoginRequest) -> LoginResponse:
        """
        Authenticate user and create session
        """
        # Find user by email
        result = await db.execute(select(User).filter(User.email == login_data.email))
        user = result.scalars().first()

        if not user or not AuthService.verify_password(login_data.password, user.hashed_password):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid credentials"
            )

        if not user.is_active:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User account is inactive"
            )

        # Create session token
        access_token_expires = timedelta(minutes=settings.access_token_expire_minutes)
        session_id = str(uuid.uuid4())  # Generate a session ID
        session_token = AuthMiddleware.create_access_token(
            data={"sub": user.id, "session_id": session_id}, expires_delta=access_token_expires
        )

        # Create session record
        session = Session(
            user_id=user.id,
            session_token=session_token,
            expires_at=datetime.utcnow() + access_token_expires
        )

        db.add(session)
        await db.commit()

        # Create response
        user_response = UserResponse(
            id=user.id,
            email=user.email,
            name=user.name,
            created_at=user.created_at
        )

        login_response = LoginResponse(
            success=True,
            user=user_response,
            session_token=session_token,
            expires_at=datetime.utcnow() + access_token_expires
        )

        return login_response

    @staticmethod
    async def logout_user(db: AsyncSession, token: str) -> bool:
        """
        Invalidate a user session
        """
        # In a real implementation, we might want to mark the session as inactive
        # For now, we'll just return True to indicate success
        return True

    @staticmethod
    async def get_current_user(db: AsyncSession, token: str) -> Optional[User]:
        """
        Get the current user from the token
        """
        payload = AuthMiddleware.verify_token(token)
        if payload is None:
            return None

        user_id = payload.get("sub")
        if user_id is None:
            return None

        result = await db.execute(select(User).filter(User.id == user_id))
        user = result.scalars().first()

        return user

    @staticmethod
    async def verify_token_validity(db: AsyncSession, token: str) -> Optional[dict]:
        """
        Verify if the token is valid and return user information
        """
        payload = AuthMiddleware.verify_token(token)
        if payload is None:
            return None

        user_id = payload.get("sub")
        if user_id is None:
            return None

        result = await db.execute(select(User).filter(User.id == user_id))
        user = result.scalars().first()

        if not user:
            return None

        return {
            "id": user.id,
            "email": user.email,
            "name": user.name,
            "created_at": user.created_at
        }