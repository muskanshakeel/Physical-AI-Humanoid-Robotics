import React, { createContext, useState, useContext, useEffect } from 'react';

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
  const [loading, setLoading] = useState(typeof window !== 'undefined'); // Only load on client side

  useEffect(() => {
    // Check if user is logged in on initial load
    if (typeof window !== 'undefined') {
      const storedUser = localStorage.getItem('user');
      if (storedUser) {
        try {
          const parsedUser = JSON.parse(storedUser);
          setUser(parsedUser);
        } catch (error) {
          console.error('Error parsing user data:', error);
        }
      }
      setLoading(false);
    }
  }, []);

  const login = (email, password) => {
    // In a real app, you would verify credentials with a backend
    // For this frontend-only implementation, we'll simulate successful login
    // if the email exists in localStorage (user is registered)
    if (typeof window !== 'undefined') {
      const users = JSON.parse(localStorage.getItem('users') || '[]');
      const foundUser = users.find(user => user.email === email && user.password === password);

      if (foundUser) {
        const userData = {
          id: foundUser.id,
          name: foundUser.name,
          email: foundUser.email,
          isLoggedIn: true
        };
        localStorage.setItem('user', JSON.stringify(userData));
        setUser(userData);
        return { success: true, user: userData };
      } else {
        return { success: false, message: 'Invalid email or password' };
      }
    }
    return { success: false, message: 'Window not available' };
  };

  const signup = (name, email, password) => {
    if (typeof window !== 'undefined') {
      // Check if user already exists
      const users = JSON.parse(localStorage.getItem('users') || '[]');
      const existingUser = users.find(user => user.email === email);

      if (existingUser) {
        return { success: false, message: 'User already exists with this email' };
      }

      // Create new user
      const newUser = {
        id: Date.now().toString(),
        name,
        email,
        password, // In a real app, this should be hashed
        createdAt: new Date().toISOString()
      };

      // Save user to localStorage
      const updatedUsers = [...users, newUser];
      localStorage.setItem('users', JSON.stringify(updatedUsers));

      // Automatically log in the new user
      const userData = {
        id: newUser.id,
        name: newUser.name,
        email: newUser.email,
        isLoggedIn: true
      };
      localStorage.setItem('user', JSON.stringify(userData));
      setUser(userData);

      return { success: true, user: userData };
    }
    return { success: false, message: 'Window not available' };
  };

  const logout = () => {
    if (typeof window !== 'undefined') {
      localStorage.removeItem('user');
      setUser(null);
    }
  };

  const value = {
    user,
    login,
    signup,
    logout,
    loading
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};