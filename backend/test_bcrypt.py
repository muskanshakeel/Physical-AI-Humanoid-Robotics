#!/usr/bin/env python3
"""Test bcrypt functionality"""

from passlib.context import CryptContext

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

def test_password_hashing():
    """Test password hashing with different lengths"""

    # Test short password
    try:
        short_pass = "short"
        hashed = pwd_context.hash(short_pass)
        print(f"Short password hashing: SUCCESS - {len(hashed)} chars")
    except Exception as e:
        print(f"Short password hashing: FAILED - {e}")

    # Test long password (over 72 bytes)
    try:
        long_pass = "a" * 80  # 80 characters, definitely over 72 bytes
        print(f"Long password length: {len(long_pass.encode('utf-8'))} bytes")
        hashed = pwd_context.hash(long_pass)
        print(f"Long password hashing: SUCCESS - {len(hashed)} chars")
    except Exception as e:
        print(f"Long password hashing: FAILED - {e}")

    # Test 72-byte password
    try:
        # Create a 72-byte password
        byte_72_pass = "a" * 72
        print(f"72-byte password length: {len(byte_72_pass.encode('utf-8'))} bytes")
        hashed = pwd_context.hash(byte_72_pass)
        print(f"72-byte password hashing: SUCCESS - {len(hashed)} chars")
    except Exception as e:
        print(f"72-byte password hashing: FAILED - {e}")

    # Test 71-byte password
    try:
        # Create a 71-byte password
        byte_71_pass = "a" * 71
        print(f"71-byte password length: {len(byte_71_pass.encode('utf-8'))} bytes")
        hashed = pwd_context.hash(byte_71_pass)
        print(f"71-byte password hashing: SUCCESS - {len(hashed)} chars")
    except Exception as e:
        print(f"71-byte password hashing: FAILED - {e}")

if __name__ == "__main__":
    test_password_hashing()