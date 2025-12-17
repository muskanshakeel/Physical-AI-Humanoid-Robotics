// Main App Component
function App() {
    return React.createElement(
        AuthProvider,
        null,
        React.createElement(
            'div',
            { className: 'app' },
            React.createElement(
                'header',
                null,
                React.createElement('h1', { className: 'app-title' }, 'Book Content Assistant'),
                React.createElement(
                    'div',
                    { id: 'auth-container' },
                    React.createElement(AuthButtons)
                )
            ),
            React.createElement(
                'main',
                null,
                React.createElement(
                    'div',
                    { className: 'book-content' },
                    React.createElement('h2', null, 'Physical AI & Humanoid Robotics'),
                    React.createElement('p', null, 'This is a sample book content page. Select text and ask questions about it using the chatbot!'),

                    React.createElement(
                        'div',
                        { className: 'book-section' },
                        React.createElement('h3', null, 'Chapter 1: Introduction to Physical AI'),
                        React.createElement('p', null, 'Physical AI refers to artificial intelligence systems that interact with the physical world through sensing, manipulation, and embodied intelligence. It\'s a key concept in robotics and autonomous systems. Physical AI combines machine learning with physical interaction capabilities, allowing systems to understand and manipulate real-world objects and environments.'),
                        React.createElement('p', null, 'In Physical AI, systems learn to interact with physical materials, forces, and environments, bridging the gap between digital intelligence and physical reality. This approach enables more natural and intuitive human-robot interaction.')
                    ),

                    React.createElement(
                        'div',
                        { className: 'book-section' },
                        React.createElement('h3', null, 'Chapter 2: Humanoid Robotics'),
                        React.createElement('p', null, 'Humanoid robotics focuses on creating robots with human-like form and capabilities. These robots often have bipedal locomotion, human-like manipulation abilities, and social interaction skills. Humanoid robots are designed to operate in human environments and potentially work alongside humans, making them valuable for various applications.'),
                        React.createElement('p', null, 'Humanoid robotics combines biomechanics, control theory, and AI to create robots that can mimic human movements and behaviors. The challenge lies in creating systems that can move and interact as fluidly as humans do.')
                    ),

                    React.createElement(
                        'div',
                        { className: 'book-section' },
                        React.createElement('h3', null, 'Chapter 3: Digital Twins in Robotics'),
                        React.createElement('p', null, 'Digital twins in robotics involve creating virtual models of physical robots to simulate, predict, and optimize their behavior before deployment in real-world scenarios. Simulation is crucial for robotics development, allowing testing of algorithms and behaviors in safe, controlled virtual environments before real-world implementation.'),
                        React.createElement('p', null, 'Digital twin technology enables real-time monitoring and predictive maintenance of robotic systems by creating virtual replicas of physical robots. This approach significantly reduces development time and costs.')
                    )
                )
            )
        ),
        React.createElement(Chatbot)
    );
}

// Render the app
const rootElement = document.getElementById('root');
const root = ReactDOM.createRoot(rootElement);
root.render(React.createElement(App));