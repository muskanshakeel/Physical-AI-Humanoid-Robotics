import React, { createContext, useContext, useState, useEffect } from 'react';
import api from '../services/api';

const AuthContext = createContext();

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [token, setToken] = useState(localStorage.getItem('token'));

  const login = async (email, password) => {
    try {
      const response = await api.post('/auth/login', { email, password });
      const { session_token, user: userData } = response.data;
      
      localStorage.setItem('token', session_token);
      setToken(session_token);
      setUser(userData);
      
      api.defaults.headers.common['Authorization'] = `Bearer ${session_token}`;
      
      return { success: true, user: userData };
    } catch (error) {
      console.error('Login error:', error);
      return { success: false, message: error.response?.data?.detail || 'Login failed' };
    }
  };

  const register = async (name, email, password) => {
    try {
      const response = await api.post('/auth/register', { name, email, password });
      const { session_token, user: userData } = response.data;
      
      localStorage.setItem('token', session_token);
      setToken(session_token);
      setUser(userData);
      
      api.defaults.headers.common['Authorization'] = `Bearer ${session_token}`;
      
      return { success: true, user: userData };
    } catch (error) {
      console.error('Registration error:', error);
      return { success: false, message: error.response?.data?.detail || 'Registration failed' };
    }
  };

  const logout = async () => {
    try {
      await api.post('/auth/logout');
    } catch (error) {
      console.error('Logout error:', error);
    } finally {
      localStorage.removeItem('token');
      setToken(null);
      setUser(null);
      delete api.defaults.headers.common['Authorization'];
    }
  };

  const checkAuthStatus = async () => {
    const storedToken = localStorage.getItem('token');

    if (!storedToken) {
      setLoading(false);
      return;
    }

    try {
      // Verify token by making an authenticated request to verify-token endpoint
      api.defaults.headers.common['Authorization'] = `Bearer ${storedToken}`;
      const response = await api.post('/auth/verify-token');

      if (response.data.success && response.data.user) {
        setToken(storedToken);
        setUser(response.data.user);
      } else {
        // Token is invalid, clear it
        localStorage.removeItem('token');
        setToken(null);
        setUser(null);
      }
    } catch (error) {
      console.error('Auth status check error:', error);
      // Clear token if verification fails
      localStorage.removeItem('token');
      setToken(null);
      setUser(null);
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    checkAuthStatus();
  }, []);

  const value = {
    user,
    login,
    register,
    logout,
    isAuthenticated: !!user,
    loading
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};