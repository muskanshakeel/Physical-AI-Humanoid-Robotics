// import React, { useState, useEffect, useRef } from 'react';
// import './ChatInterface.css';
// import api from '../services/api';

// function ChatInterface() {
//   const [messages, setMessages] = useState([]);
//   const [inputMessage, setInputMessage] = useState('');
//   const [isLoading, setIsLoading] = useState(false);
//   const [error, setError] = useState('');
//   const messagesEndRef = useRef(null);

//   // Initialize with welcome message
//   useEffect(() => {
//     setMessages([
//       { id: 'welcome', role: 'assistant', content: 'Hello! I\'m your RAG Chatbot. Ask me anything about the book content!' }
//     ]);
//   }, []);

//   // Scroll to bottom when messages change
//   useEffect(() => {
//     scrollToBottom();
//   }, [messages]);

//   const scrollToBottom = () => {
//     messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
//   };

//   const handleSendMessage = async (e) => {
//     e.preventDefault();
//     if (!inputMessage.trim() || isLoading) return;

//     const userMessage = {
//       id: Date.now().toString(),
//       role: 'user',
//       content: inputMessage.trim()
//     };

//     // Add user message to chat
//     setMessages(prev => [...prev, userMessage]);
//     setInputMessage('');
//     setIsLoading(true);
//     setError('');

//     try {
//       const response = await api.post('/chat/query', {
//         query: inputMessage.trim(),
//         book_selection: null
//       });

//       if (response.data.success) {
//         const botMessage = {
//           id: response.data.query_id,
//           role: 'assistant',
//           content: response.data.response
//         };
//         setMessages(prev => [...prev, botMessage]);
//       } else {
//         throw new Error(response.data.error || 'Failed to get response');
//       }
//     } catch (err) {
//       console.error('Chat error:', err);
//       setError(err.message || 'Failed to send message');

//       // Add error message to chat
//       const errorMessage = {
//         id: `error-${Date.now()}`,
//         role: 'assistant',
//         content: `Error: ${err.message || 'Failed to process your request'}`
//       };
//       setMessages(prev => [...prev, errorMessage]);
//     } finally {
//       setIsLoading(false);
//     }
//   };

//   return (
//     <div className="chat-interface">
//       <header className="chat-header">
//         <div className="chat-title">
//           <h2>Book Content Assistant</h2>
//         </div>
//       </header>

//       <div className="chat-container">
//         <div className="messages">
//           {messages.map((message) => (
//             <div
//               key={message.id}
//               className={`message ${message.role === 'user' ? 'user-message' : 'bot-message'}`}
//             >
//               <div className="message-content">
//                 {message.content}
//               </div>
//             </div>
//           ))}
//           {isLoading && (
//             <div className="message bot-message">
//               <div className="message-content">
//                 <div className="typing-indicator">
//                   <span></span>
//                   <span></span>
//                   <span></span>
//                 </div>
//               </div>
//             </div>
//           )}
//           <div ref={messagesEndRef} />
//         </div>

//         {error && <div className="error-message">{error}</div>}

//         <form onSubmit={handleSendMessage} className="input-form">
//           <input
//             type="text"
//             value={inputMessage}
//             onChange={(e) => setInputMessage(e.target.value)}
//             placeholder="Ask about the book content..."
//             disabled={isLoading}
//             className="message-input"
//           />
//           <button type="submit" disabled={isLoading} className="send-btn">
//             Send
//           </button>
//         </form>
//       </div>
//     </div>
//   );
// }

// export default ChatInterface;




import React, { useState, useEffect, useRef } from 'react';
import './ChatInterface.css';
import axios from 'axios';

// API configuration
const API_BASE_URL = 'http://127.0.0.1:8000';

function ChatInterface() {
  const [messages, setMessages] = useState([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const [sessionId, setSessionId] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [contextOnly, setContextOnly] = useState(false);
  const messagesEndRef = useRef(null);

  // Initialize session and handle text selection
  useEffect(() => {
    // Generate unique session ID
    const id = 'session_' + Math.random().toString(36).substr(2, 9);
    setSessionId(id);
    
    // Load chat history
    loadChatHistory(id);
    
    // Handle text selection from book
    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();
      
      if (text.length > 0 && text.length < 1000) {
        setSelectedText(text);
        // Show notification about selected text
        if (messages.length === 1) { // Only welcome message
          setMessages(prev => [...prev, {
            id: 'selection-notice',
            role: 'system',
            content: `📝 Text selected: "${text.substring(0, 50)}${text.length > 50 ? '...' : ''}"`
          }]);
        }
      }
    };
    
    document.addEventListener('selectionchange', handleTextSelection);
    
    return () => {
      document.removeEventListener('selectionchange', handleTextSelection);
    };
  }, []);

  // Load initial welcome message
  useEffect(() => {
    if (messages.length === 0) {
      setMessages([
        { 
          id: 'welcome', 
          role: 'assistant', 
          content: 'Hello! I\'m your Book Content Assistant. Ask me anything about the book!\n\nYou can select text from the book and ask questions about it.'
        }
      ]);
    }
  }, []);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  const loadChatHistory = async (sessionId) => {
    try {
      const response = await axios.get(`${API_BASE_URL}/history/${sessionId}`);
      
      if (response.data.history && response.data.history.length > 0) {
        const historyMessages = response.data.history.flatMap(item => [
          { id: `user-${Date.now()}`, role: 'user', content: item.user_message },
          { id: `bot-${Date.now()}`, role: 'assistant', content: item.bot_response }
        ]);
        setMessages(prev => [...prev, ...historyMessages]);
      }
    } catch (error) {
      console.error('Failed to load chat history:', error);
    }
  };

  const handleSendMessage = async (e) => {
    e.preventDefault();
    if (!inputMessage.trim() || isLoading) return;

    const userMessage = {
      id: Date.now().toString(),
      role: 'user',
      content: inputMessage.trim()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputMessage('');
    setIsLoading(true);
    setError('');

    try {
      const response = await axios.post(`${API_BASE_URL}/chat`, {
        message: inputMessage.trim(),
        session_id: sessionId,
        selected_text: selectedText || null,
        context_only: contextOnly
      });

      const botMessage = {
        id: response.data.session_id,
        role: 'assistant',
        content: response.data.response,
        sources: response.data.sources
      };
      
      setMessages(prev => [...prev, botMessage]);

      // Clear selected text if contextOnly was used
      if (contextOnly && selectedText) {
        setSelectedText('');
        setContextOnly(false);
      }

    } catch (err) {
      console.error('Chat error:', err);
      
      let errorMsg = 'Failed to get response from server';
      if (err.response) {
        errorMsg = err.response.data.detail || err.response.statusText;
      } else if (err.message) {
        errorMsg = err.message;
      }
      
      setError(errorMsg);

      // Add error message to chat
      const errorMessage = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: `Sorry, I encountered an error: ${errorMsg}`
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage(e);
    }
  };

  const clearChat = () => {
    setMessages([{
      id: 'welcome',
      role: 'assistant',
      content: 'Hello! I\'m your Book Content Assistant. Ask me anything about the book!\n\nYou can select text from the book and ask questions about it.'
    }]);
    setSelectedText('');
    setContextOnly(false);
    setError('');
  };

  const handleContextOnly = () => {
    if (selectedText) {
      setContextOnly(true);
      setMessages(prev => [...prev, {
        id: `context-${Date.now()}`,
        role: 'system',
        content: `🔍 Now focusing ONLY on selected text: "${selectedText.substring(0, 50)}${selectedText.length > 50 ? '...' : ''}"`
      }]);
    }
  };

  return (
    <div className="chat-interface">
      <header className="chat-header">
        <div className="chat-title">
          <h2>Book Content Assistant</h2>
          {selectedText && (
            <div className="selected-text-indicator">
              <span className="indicator-dot"></span>
              <span className="indicator-text">
                {selectedText.length > 40 
                  ? `${selectedText.substring(0, 40)}...` 
                  : selectedText}
              </span>
              {!contextOnly && (
                <button 
                  onClick={handleContextOnly}
                  className="context-only-btn"
                  title="Focus only on selected text"
                >
                  🔍 Focus
                </button>
              )}
              {contextOnly && (
                <span className="focus-active">🔍 Focusing</span>
              )}
            </div>
          )}
        </div>
        <button onClick={clearChat} className="clear-chat-btn">
          Clear Chat
        </button>
      </header>

      <div className="chat-container">
        <div className="messages">
          {messages.map((message) => (
            <div
              key={message.id}
              className={`message ${message.role === 'user' ? 'user-message' : 
                message.role === 'system' ? 'system-message' : 'bot-message'}`}
            >
              <div className="message-role">
                {message.role === 'user' ? '👤 You' : 
                 message.role === 'system' ? 'ℹ️' : '🤖 Assistant'}
              </div>
              <div className="message-content">
                {message.content}
                {message.sources && message.sources.length > 0 && (
                  <div className="message-sources">
                    <small>Sources: {message.sources.join(', ')}</small>
                  </div>
                )}
              </div>
            </div>
          ))}
          {isLoading && (
            <div className="message bot-message">
              <div className="message-role">🤖 Assistant</div>
              <div className="message-content">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        {error && <div className="error-message">{error}</div>}

        <form onSubmit={handleSendMessage} className="input-form">
          <div className="input-container">
            <input
              type="text"
              value={inputMessage}
              onChange={(e) => setInputMessage(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask about the book content..."
              disabled={isLoading}
              className="message-input"
            />
            <button 
              type="submit" 
              disabled={isLoading || !inputMessage.trim()} 
              className="send-btn"
            >
              {isLoading ? 'Sending...' : 'Send'}
            </button>
          </div>
          <div className="input-help">
            {selectedText ? (
              <span className="help-text">
                📝 Text selected. Questions will consider this text.
                {!contextOnly && ' Click "Focus" to use only selected text.'}
              </span>
            ) : (
              <span className="help-text">
                Select text from the book to ask specific questions about it
              </span>
            )}
          </div>
        </form>
      </div>
    </div>
  );
}

export default ChatInterface;