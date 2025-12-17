import React from 'react';
import { useAuth } from '../context/AuthContext';
import './AuthButton.css';

const AuthButton = ({ variant = 'primary', size = 'medium' }) => {
  const { user, logout, isAuthenticated } = useAuth();

  const handleLogout = async (e) => {
    e.preventDefault();
    await logout();
  };

  if (isAuthenticated && user) {
    return (
      <div className={`auth-button-container ${variant}`}>
        <div className="user-greeting">
          <span className="user-name">Hi, {user.name}</span>
        </div>
        <button 
          className={`auth-button ${size} auth-button-logout`} 
          onClick={handleLogout}
        >
          Logout
        </button>
      </div>
    );
  }

  return (
    <div className={`auth-button-container ${variant}`}>
      <a href="/auth" className={`auth-button ${size} auth-button-login`}>
        Sign In
      </a>
      <a href="/auth" className={`auth-button ${size} auth-button-register`}>
        Sign Up
      </a>
    </div>
  );
};

export default AuthButton;