from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from ..db.connection import get_db_session
from ..services.auth_service import AuthService
from ..models.request import UserRegisterRequest, UserLoginRequest
from ..middleware.auth import get_current_user_dependency
from ..models.response import AuthResponse, LoginResponse, LogoutResponse, ErrorResponse
import uuid

router = APIRouter(tags=["Authentication"])

@router.post("/register", response_model=AuthResponse)
async def register(user_data: UserRegisterRequest, db: AsyncSession = Depends(get_db_session)):
    """
    Register a new user
    """
    try:
        result = await AuthService.register_user(db, user_data)
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Registration failed: {str(e)}"
        )

@router.post("/login", response_model=LoginResponse)
async def login(login_data: UserLoginRequest, db: AsyncSession = Depends(get_db_session)):
    """
    Authenticate user and create session
    """
    try:
        result = await AuthService.login_user(db, login_data)
        return result
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Login failed: {str(e)}"
        )

@router.post("/logout", response_model=LogoutResponse)
async def logout(token: str = Depends(get_current_user_dependency), db: AsyncSession = Depends(get_db_session)):
    """
    End user session
    """
    try:
        success = await AuthService.logout_user(db, token)
        if success:
            return LogoutResponse(
                success=True,
                message="Successfully logged out"
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="Logout failed"
            )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Logout failed: {str(e)}"
        )

@router.post("/verify-token", response_model=AuthResponse)
async def verify_token(token: str = Depends(get_current_user_dependency), db: AsyncSession = Depends(get_db_session)):
    """
    Verify if the provided token is valid
    """
    try:
        user_data = await AuthService.verify_token_validity(db, token)
        if user_data:
            return AuthResponse(
                success=True,
                user=UserResponse(**user_data),
                session_token=token,
                expires_at=None  # Not returning expiration time as it's already encoded in the token
            )
        else:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token"
            )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Token verification failed: {str(e)}"
        )