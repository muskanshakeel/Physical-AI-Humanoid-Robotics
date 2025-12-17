# Book Content Assistant Website

This is a standalone website with login functionality and a book content chatbot, similar to the RAG chatbot system in the Docusaurus website but as a separate application.

## Features

- **User Authentication**: Login and signup functionality with localStorage-based user management
- **Book Content**: Sample book content about Physical AI & Humanoid Robotics
- **Interactive Chatbot**: AI assistant that can answer questions about the book content
- **Text Selection**: Select text from the book and ask specific questions about it
- **Responsive Design**: Works well on both desktop and mobile devices

## How to Run

1. **Install dependencies**:
   ```bash
   npm install
   ```

2. **Start the server**:
   ```bash
   npm start
   # or
   npx http-server -p 8080
   ```

3. **Open your browser** and navigate to `http://localhost:8080` (or the address shown in the terminal)

## Functionality

- **Authentication**: Users can sign up with a name, email, and password, or log in with existing credentials
- **Book Content**: The website displays sample book content that users can read and select text from
- **Chatbot**: The floating chatbot can answer questions about the book content, with special handling for topics like:
  - Physical AI
  - Humanoid Robotics
  - Digital Twins
  - Machine Learning in Robotics
  - Control Systems
  - Embodied AI
  - And more!

- **Text Selection**: When you select text in the book content and then ask a question in the chatbot, it will reference the selected text in its response

## Technical Implementation

- **Frontend**: Pure HTML/CSS/JS with React (using Babel Standalone for JSX)
- **Authentication**: Client-side authentication using localStorage
- **Chatbot**: Rule-based responses with keyword matching for book-related topics
- **Styling**: CSS for responsive design and modern UI components

## Files Structure

- `index.html` - Main HTML structure
- `styles.css` - All CSS styling
- `auth.js` - Authentication components and context
- `chatbot.js` - Chatbot component with text selection functionality
- `app.js` - Main application component that ties everything together
- `package.json` - Project dependencies and scripts

## Usage

1. **Sign up or log in** using the buttons in the top right corner
2. **Browse the book content** to read about Physical AI & Humanoid Robotics
3. **Select text** in the book content and then ask questions about it in the chatbot
4. **Use the floating chatbot** (bottom right) to ask general questions about the book topics