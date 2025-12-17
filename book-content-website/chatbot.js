// Chatbot Component
function Chatbot() {
    const [isOpen, setIsOpen] = React.useState(false);
    const [messages, setMessages] = React.useState([]);
    const [inputMessage, setInputMessage] = React.useState('');
    const [isLoading, setIsLoading] = React.useState(false);
    const [selectedText, setSelectedText] = React.useState('');
    const messagesEndRef = React.useRef(null);
    const chatContainerRef = React.useRef(null);

    // Initialize with welcome message
    React.useEffect(() => {
        if (isOpen && messages.length === 0) {
            setMessages([
                {
                    id: '1',
                    role: 'assistant',
                    content: 'Hello! I\'m your Book Content Assistant. Ask me anything about the book! You can select text from the book and ask questions about it.'
                }
            ]);
        }
    }, [isOpen]);

    // Scroll to bottom when messages change
    React.useEffect(() => {
        scrollToBottom();
    }, [messages]);

    // Handle text selection
    React.useEffect(() => {
        const handleSelection = () => {
            const selectedText = window.getSelection().toString().trim();
            if (selectedText) {
                setSelectedText(selectedText);
            }
        };

        document.addEventListener('mouseup', handleSelection);
        return () => {
            document.removeEventListener('mouseup', handleSelection);
        };
    }, []);

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    };

    const handleSendMessage = (e) => {
        e.preventDefault();
        if (!inputMessage.trim() || isLoading) return;

        let userMessageText = inputMessage.trim();

        // If there's selected text, prepend it to the message
        if (selectedText) {
            userMessageText = `Regarding the selected text: "${selectedText}". ${userMessageText}`;
            setSelectedText(''); // Clear selected text after using it
        }

        const userMessage = {
            id: Date.now().toString(),
            role: 'user',
            content: userMessageText
        };

        // Add user message to chat
        setMessages(prev => [...prev, userMessage]);
        setInputMessage('');
        setIsLoading(true);

        // Simulate bot response after a delay
        setTimeout(() => {
            // Analyze the input to provide more contextually relevant responses
            const lowerCaseInput = userMessageText.toLowerCase();

            // Define book-related keywords and corresponding responses
            const bookResponses = {
                // Physical AI related
                'physical ai|physical artificial intelligence|physical intelligence': [
                    "Physical AI refers to artificial intelligence systems that interact with the physical world through sensing, manipulation, and embodied intelligence. It's a key concept in robotics and autonomous systems.",
                    "Physical AI combines machine learning with physical interaction capabilities, allowing systems to understand and manipulate real-world objects and environments.",
                    "In Physical AI, systems learn to interact with physical materials, forces, and environments, bridging the gap between digital intelligence and physical reality."
                ],
                // Humanoid Robotics related
                'humanoid|humanoid robot|humanoid robotics': [
                    "Humanoid robotics focuses on creating robots with human-like form and capabilities. These robots often have bipedal locomotion, human-like manipulation abilities, and social interaction skills.",
                    "Humanoid robots are designed to operate in human environments and potentially work alongside humans, making them valuable for various applications.",
                    "Humanoid robotics combines biomechanics, control theory, and AI to create robots that can mimic human movements and behaviors."
                ],
                // General book topics
                'robotics|robot|ai|artificial intelligence': [
                    "The book covers various aspects of robotics and AI, including perception, decision-making, control systems, and human-robot interaction.",
                    "Artificial Intelligence in robotics involves machine learning, computer vision, and control algorithms to enable autonomous behavior.",
                    "Robotics integrates multiple disciplines including mechanics, electronics, and software to create intelligent machines."
                ],
                // Advanced topics
                'digital twins|simulation|simulations': [
                    "Digital twins in robotics involve creating virtual models of physical robots to simulate, predict, and optimize their behavior before deployment in real-world scenarios.",
                    "Simulation is crucial for robotics development, allowing testing of algorithms and behaviors in safe, controlled virtual environments before real-world implementation.",
                    "Digital twin technology enables real-time monitoring and predictive maintenance of robotic systems by creating virtual replicas of physical robots."
                ],
                'learning|machine learning|deep learning': [
                    "The book discusses how robots use machine learning to improve their performance through experience, including reinforcement learning and neural networks.",
                    "Machine learning in robotics enables systems to adapt to new situations and improve their capabilities over time.",
                    "Deep learning techniques help robots process sensory information and make complex decisions in dynamic environments."
                ],
                'control|motion|movement': [
                    "The book covers control theory applications in robotics, including motion planning, trajectory optimization, and dynamic control.",
                    "Motion control in robotics involves coordinating multiple actuators to achieve desired movements while maintaining stability.",
                    "Advanced control systems enable robots to perform complex movements and adapt to changing environmental conditions."
                ],
                'embodied|embodiment|embodied ai': [
                    "Embodied AI refers to artificial intelligence systems that interact with the physical world through a body, learning from physical interactions and sensory feedback.",
                    "Embodiment is crucial for AI systems to understand physical laws, object properties, and real-world constraints through direct interaction.",
                    "Embodied AI systems learn more effectively by experiencing the physical world directly rather than just processing abstract data."
                ],
                'perception|computer vision|sensors': [
                    "Robot perception involves processing sensory information from cameras, lidar, touch sensors, and other devices to understand the environment.",
                    "Computer vision enables robots to interpret visual information, recognize objects, and navigate in complex environments.",
                    "Sensory systems in robotics provide crucial feedback for decision-making and safe interaction with the physical world."
                ],
                // Default responses for non-specific queries
                'default': [
                    "Based on the book 'Physical AI & Humanoid Robotics', this topic covers the intersection of artificial intelligence and physical systems.",
                    "The book explores how AI systems can interact with the physical world, particularly in robotics applications.",
                    "Physical AI and humanoid robotics involve complex systems that combine intelligence with physical capabilities.",
                    "This subject area combines machine learning, robotics, and physical system control for advanced applications.",
                    "The book discusses cutting-edge research in AI systems that interact with physical environments."
                ]
            };

            // Find the most relevant response based on keywords
            let selectedResponse = null;

            // Check for specific keywords in the message
            for (const [keywords, responses] of Object.entries(bookResponses)) {
                if (keywords !== 'default') {
                    const keywordList = keywords.split('|');
                    for (const keyword of keywordList) {
                        if (lowerCaseInput.includes(keyword.trim())) {
                            selectedResponse = responses[Math.floor(Math.random() * responses.length)];
                            break;
                        }
                    }
                    if (selectedResponse) break;
                }
            }

            // If no specific keyword matched, use a default response
            if (!selectedResponse) {
                const defaultResponses = bookResponses['default'];
                selectedResponse = defaultResponses[Math.floor(Math.random() * defaultResponses.length)];
            }

            const botMessage = {
                id: `bot-${Date.now()}`,
                role: 'assistant',
                content: selectedResponse
            };

            setMessages(prev => [...prev, botMessage]);
            setIsLoading(false);
        }, 1000);
    };

    const toggleChat = () => {
        setIsOpen(!isOpen);
    };

    const closeChat = () => {
        setIsOpen(false);
    };

    return React.createElement(
        'div',
        { className: 'floating-chatbot' },
        isOpen ? React.createElement(
            'div',
            { className: 'chat-container', ref: chatContainerRef },
            React.createElement(
                'div',
                { className: 'chat-header' },
                React.createElement(
                    'div',
                    { className: 'chat-title' },
                    React.createElement('h3', null, 'Book Assistant')
                ),
                React.createElement(
                    'button',
                    { className: 'close-btn', onClick: closeChat, 'aria-label': 'Close chat' },
                    '×'
                )
            ),
            React.createElement(
                'div',
                { className: 'messages' },
                messages.map((message) =>
                    React.createElement(
                        'div',
                        {
                            key: message.id,
                            className: `message ${message.role === 'user' ? 'user-message' : 'bot-message'}`
                        },
                        React.createElement(
                            'div',
                            { className: 'message-content' },
                            message.content
                        )
                    )
                ),
                isLoading && React.createElement(
                    'div',
                    { className: 'message bot-message' },
                    React.createElement(
                        'div',
                        { className: 'message-content' },
                        React.createElement(
                            'div',
                            { className: 'typing-indicator' },
                            React.createElement('span', null),
                            React.createElement('span', null),
                            React.createElement('span', null)
                        )
                    )
                ),
                React.createElement('div', { ref: messagesEndRef })
            ),
            React.createElement(
                'form',
                { onSubmit: handleSendMessage, className: 'input-form' },
                React.createElement('input', {
                    type: 'text',
                    value: inputMessage,
                    onChange: (e) => setInputMessage(e.target.value),
                    placeholder: selectedText ? 'Ask about selected text...' : 'Ask about the book...',
                    disabled: isLoading,
                    className: 'message-input',
                    onKeyPress: (e) => {
                        if (e.key === 'Enter' && !e.shiftKey) {
                            handleSendMessage(e);
                        }
                    }
                }),
                React.createElement(
                    'button',
                    {
                        type: 'submit',
                        disabled: isLoading || !inputMessage.trim(),
                        className: 'send-btn'
                    },
                    'Send'
                )
            )
        ) : React.createElement(
            'button',
            { className: 'chatbot-icon', onClick: toggleChat, 'aria-label': 'Open chat' },
            '💬'
        )
    );
}