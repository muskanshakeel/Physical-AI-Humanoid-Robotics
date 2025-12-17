import React, { useState } from 'react';
import { useAuth } from '../context/AuthContext';
import AuthForm from '../components/AuthForm';
import './AuthPage.css';

const AuthPage = () => {
  const { login, register } = useAuth();
  const [authType, setAuthType] = useState('login'); // 'login' or 'register'
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState('');

  const handleSubmit = async (formData) => {
    setLoading(true);
    setMessage('');
    
    let result;
    if (authType === 'login') {
      result = await login(formData.email, formData.password);
    } else {
      result = await register(formData.name, formData.email, formData.password);
    }
    
    setLoading(false);
    
    if (result.success) {
      setMessage(result.message || `Successfully ${authType === 'login' ? 'logged in' : 'registered'}!`);
      // Redirect to home or previous page after successful login/register
      setTimeout(() => {
        window.location.href = '/';
      }, 1500);
    } else {
      setMessage(result.message || 'Operation failed');
    }
  };

  const switchAuthType = () => {
    setAuthType(authType === 'login' ? 'register' : 'login');
    setMessage('');
  };

  return (
    <div className="auth-page">
      <div className="auth-page-container">
        <div className="auth-page-content">
          {loading && (
            <div className="loading-overlay">
              <div className="spinner"></div>
              <p>Processing...</p>
            </div>
          )}
          
          <AuthForm 
            type={authType}
            onSubmit={handleSubmit}
            errorMessage={message}
          />
          
          <div className="auth-switch-section">
            <p>
              {authType === 'login' 
                ? "Don't have an account? " 
                : "Already have an account? "}
              <button 
                type="button" 
                className="auth-switch-btn"
                onClick={switchAuthType}
              >
                {authType === 'login' ? 'Sign Up' : 'Sign In'}
              </button>
            </p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default AuthPage;