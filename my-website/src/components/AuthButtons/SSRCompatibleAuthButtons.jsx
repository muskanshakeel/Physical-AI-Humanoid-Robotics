import React, { useState, useEffect } from 'react';
import { useAuth } from '@site/src/context/AuthContext';

const SSRCompatibleAuthButtons = () => {
  const { user, logout } = useAuth();
  const [isClient, setIsClient] = useState(false);
  const [showModal, setShowModal] = useState(null); // 'login', 'signup', or null

  useEffect(() => {
    // This ensures the component only renders client-side content after hydration
    setIsClient(true);
  }, []);

  // Handle modal events
  useEffect(() => {
    const handleOpenLoginModal = () => setShowModal('login');
    const handleOpenSignupModal = () => setShowModal('signup');
    const handleCloseModal = () => setShowModal(null);

    window.addEventListener('open-login-modal', handleOpenLoginModal);
    window.addEventListener('open-signup-modal', handleOpenSignupModal);
    window.addEventListener('close-auth-modal', handleCloseModal);

    return () => {
      window.removeEventListener('open-login-modal', handleOpenLoginModal);
      window.removeEventListener('open-signup-modal', handleOpenSignupModal);
      window.removeEventListener('close-auth-modal', handleCloseModal);
    };
  }, []);

  if (!isClient) {
    // Render a placeholder during SSR
    return <div className="auth-buttons-placeholder">Loading...</div>;
  }

  const handleLogout = () => {
    logout();
    setShowModal(null);
  };

  return (
    <div className="auth-buttons">
      {user ? (
        <div className="user-menu">
          <span className="user-name">Hello, {user.name}</span>
          <button
            className="logout-btn"
            onClick={handleLogout}
            style={{
              marginLeft: '10px',
              padding: '5px 10px',
              backgroundColor: '#007cba',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            Logout
          </button>
        </div>
      ) : (
        <div className="auth-options">
          <button
            className="login-btn"
            onClick={() => window.dispatchEvent(new CustomEvent('open-login-modal'))}
            style={{
              padding: '5px 10px',
              backgroundColor: '#007cba',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer',
              marginRight: '5px'
            }}
          >
            Login
          </button>
          <button
            className="signup-btn"
            onClick={() => window.dispatchEvent(new CustomEvent('open-signup-modal'))}
            style={{
              padding: '5px 10px',
              backgroundColor: '#28a745',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              cursor: 'pointer'
            }}
          >
            Sign Up
          </button>
        </div>
      )}

      {/* Modal placeholders - actual modals would be implemented separately */}
      {showModal && (
        <div className="auth-modal-overlay" style={{
          position: 'fixed',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          backgroundColor: 'rgba(0,0,0,0.5)',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          zIndex: 1000
        }}>
          <div className="auth-modal" style={{
            backgroundColor: 'white',
            padding: '20px',
            borderRadius: '8px',
            width: '400px',
            maxWidth: '90vw'
          }}>
            <h3>{showModal === 'login' ? 'Login' : 'Sign Up'}</h3>
            <p>Modal content for {showModal} would go here</p>
            <button
              onClick={() => window.dispatchEvent(new CustomEvent('close-auth-modal'))}
              style={{
                marginTop: '10px',
                padding: '5px 10px',
                backgroundColor: '#dc3545',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: 'pointer'
              }}
            >
              Close
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default SSRCompatibleAuthButtons;