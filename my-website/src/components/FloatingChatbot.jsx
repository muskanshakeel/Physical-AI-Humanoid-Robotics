import React, { useState } from 'react';

const FloatingChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [isLoading, setIsLoading] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async (message) => {
    if (message.trim() === '' || isLoading) return;

    const newMessage = {
      id: Date.now(),
      text: message,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, newMessage]);
    setIsLoading(true);

    try {
      // Call the backend API
      const response = await fetch('http://localhost:8000/api/chat/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: message,
          book_selection: 'Physical-AI-Humanoid-Robotics' // Specify the book to search in
        })
      });

      if (!response.ok) {
        throw new Error(`API request failed with status ${response.status}`);
      }

      const data = await response.json();

      if (data.success && data.response) {
        const botMessage = {
          id: Date.now() + 1,
          text: data.response.content,
          sender: 'bot',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, botMessage]);
      } else {
        const botMessage = {
          id: Date.now() + 1,
          text: "I'm sorry, I couldn't process your request. Please try again.",
          sender: 'bot',
          timestamp: new Date()
        };
        setMessages(prev => [...prev, botMessage]);
      }
    } catch (error) {
      console.error('Error sending message:', error);
      const botMessage = {
        id: Date.now() + 1,
        text: "I'm sorry, I'm having trouble connecting to the server. Please try again later.",
        sender: 'bot',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, botMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="floating-chatbot">
      {isOpen ? (
        <div className="chat-container">
          <div className="chat-header">
            <div className="chat-title">
              <h3>AI Assistant</h3>
            </div>
            <button
              onClick={toggleChat}
              className="close-btn"
              disabled={isLoading}
            >
              ×
            </button>
          </div>

          <div className="messages">
            {messages.length === 0 ? (
              <div style={{ fontStyle: 'italic', color: '#666' }}>
                Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book. Ask me anything about the content!
              </div>
            ) : (
              messages.map((msg) => (
                <div
                  key={msg.id}
                  className={`message ${msg.sender === 'user' ? 'user-message' : 'bot-message'}`}
                >
                  <div className="message-content">
                    {msg.text}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="message bot-message">
                <div className="message-content">
                  <em>Thinking...</em>
                </div>
              </div>
            )}
          </div>

          <div className="input-form">
            <form onSubmit={(e) => {
              e.preventDefault();
              const input = e.target.elements.chatInput;
              sendMessage(input.value);
              input.value = '';
            }}>
              <input
                type="text"
                name="chatInput"
                placeholder="Ask about the book content..."
                className="message-input"
                disabled={isLoading}
              />
              <button
                type="submit"
                className="send-btn"
                disabled={isLoading}
              >
                {isLoading ? 'Sending...' : 'Send'}
              </button>
            </form>
          </div>
        </div>
      ) : (
        <button
          onClick={toggleChat}
          className="chatbot-icon"
        >
          💬
        </button>
      )}
    </div>
  );
};

export default FloatingChatbot;