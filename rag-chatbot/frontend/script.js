document.addEventListener('DOMContentLoaded', () => {
    const userInput = document.getElementById('user-input');
    const sendButton = document.getElementById('send-button');
    const chatWindow = document.getElementById('chat-window');

    sendButton.addEventListener('click', sendMessage);
    userInput.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') {
            sendMessage();
        }
    });

    async function sendMessage() {
        const message = userInput.value.trim();
        if (message) {
            appendMessage('user', message);
            userInput.value = '';

            try {
                // Send message to FastAPI backend
                const response = await fetch('http://localhost:8000/rag/query', { // TODO: Use environment variable for backend URL
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ query_text: message }),
                });

                if (!response.ok) {
                    throw new Error(`HTTP error! status: ${response.status}`);
                }

                const data = await response.json();
                appendMessage('bot', data.answer_text);
                if (data.sources && data.sources.length > 0) {
                    const sourcesHtml = data.sources.map(src => {
                        return `<p class="source-item">Source: ${src.chapter_title} - ${src.text_snippet}</p>`;
                    }).join('');
                    appendMessage('bot', `<div class="sources-list">${sourcesHtml}</div>`);
                }

            } catch (error) {
                console.error('Error fetching RAG response:', error);
                appendMessage('bot', 'Sorry, I could not get a response from the chatbot.');
            }
        }
    }

    function appendMessage(sender, text) {
        const messageElement = document.createElement('div');
        messageElement.classList.add('message', sender);
        messageElement.innerHTML = text; // Use innerHTML to allow for source formatting
        chatWindow.appendChild(messageElement);
        chatWindow.scrollTop = chatWindow.scrollHeight; // Auto-scroll to latest message
    }
});
