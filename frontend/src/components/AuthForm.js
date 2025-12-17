import React, { useState } from 'react';
import './AuthForm.css';

const AuthForm = ({ type = 'login', onSubmit, errorMessage }) => {
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: ''
  });
  
  const [showPassword, setShowPassword] = useState(false);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    onSubmit(formData);
  };

  return (
    <div className="auth-container">
      <div className="auth-form-wrapper">
        <div className="auth-header">
          <h2>{type === 'login' ? 'Welcome Back' : 'Create Account'}</h2>
          <p>{type === 'login' 
            ? 'Sign in to continue to your account' 
            : 'Create an account to get started'}
          </p>
        </div>
        
        {errorMessage && (
          <div className="auth-error-message">
            {errorMessage}
          </div>
        )}
        
        <form onSubmit={handleSubmit} className="auth-form">
          {type === 'register' && (
            <div className="form-group">
              <label htmlFor="name">Full Name</label>
              <input
                type="text"
                id="name"
                name="name"
                value={formData.name}
                onChange={handleChange}
                required
                placeholder="Enter your full name"
                className="auth-input"
              />
            </div>
          )}
          
          <div className="form-group">
            <label htmlFor="email">Email Address</label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              required
              placeholder="Enter your email"
              className="auth-input"
            />
          </div>
          
          <div className="form-group">
            <label htmlFor="password">Password</label>
            <div className="password-input-wrapper">
              <input
                type={showPassword ? "text" : "password"}
                id="password"
                name="password"
                value={formData.password}
                onChange={handleChange}
                required
                placeholder="Enter your password"
                className="auth-input password-input"
              />
              <button
                type="button"
                className="toggle-password-btn"
                onClick={() => setShowPassword(!showPassword)}
              >
                {showPassword ? 'Hide' : 'Show'}
              </button>
            </div>
          </div>
          
          {type === 'login' && (
            <div className="form-options">
              <label className="remember-me">
                <input type="checkbox" /> Remember me
              </label>
              <a href="#forgot-password" className="forgot-password-link">Forgot Password?</a>
            </div>
          )}
          
          <button type="submit" className="auth-button">
            {type === 'login' ? 'Sign In' : 'Sign Up'}
          </button>
        </form>
        
        <div className="auth-footer">
          <p>
            {type === 'login' 
              ? "Don't have an account? " 
              : "Already have an account? "}
            <a 
              href="#" 
              className="switch-auth-type"
              onClick={(e) => {
                e.preventDefault();
                // Switch between login and register - handled by parent component
              }}
            >
              {type === 'login' ? 'Sign Up' : 'Sign In'}
            </a>
          </p>
        </div>
      </div>
    </div>
  );
};

export default AuthForm;