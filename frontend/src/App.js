import React from 'react';
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom';
import { AuthProvider } from './context/AuthContext';
import './App.css';
import ChatInterface from './components/ChatInterface';
import AuthPage from './components/AuthPage';
import AuthButton from './components/AuthButton';

function App() {
  return (
    <AuthProvider>
      <Router>
        <div className="App">
          <header className="App-header">
            <h1>RAG Chatbot</h1>
            <div className="app-info">
              <span>Book Content Q&A System</span>
            </div>
            <div className="auth-button-position">
              <AuthButton />
            </div>
          </header>
          <main>
            <Routes>
              <Route path="/" element={
                <div className="chat-interface-wrapper">
                  <ChatInterface />
                </div>
              } />
              <Route path="/auth" element={<AuthPage />} />
              <Route path="*" element={<Navigate to="/" replace />} />
            </Routes>
          </main>
        </div>
      </Router>
    </AuthProvider>
  );
}

export default App;