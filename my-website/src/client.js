import React from 'react';
import { AuthProvider } from './context/AuthContext';

// This is the client entry point for Docusaurus
export default function ClientApp({ children }) {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  );
}