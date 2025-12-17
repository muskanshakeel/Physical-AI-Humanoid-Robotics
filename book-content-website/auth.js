// Authentication context and components
const AuthContext = React.createContext();

// Auth Provider Component
function AuthProvider({ children }) {
    const [user, setUser] = React.useState(null);
    const [loading, setLoading] = React.useState(true);

    React.useEffect(() => {
        // Check if user is logged in on initial load
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
    }, []);

    const login = (email, password) => {
        // In a real app, you would verify credentials with a backend
        // For this frontend-only implementation, we'll simulate successful login
        // if the email exists in localStorage (user is registered)
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
    };

    const signup = (name, email, password) => {
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
    };

    const logout = () => {
        localStorage.removeItem('user');
        setUser(null);
    };

    const value = {
        user,
        login,
        signup,
        logout,
        loading
    };

    return React.createElement(
        AuthContext.Provider,
        { value: value },
        children
    );
}

// Custom hook to use auth context
function useAuth() {
    const context = React.useContext(AuthContext);
    if (!context) {
        throw new Error('useAuth must be used within an AuthProvider');
    }
    return context;
}

// Authentication Buttons Component
function AuthButtons() {
    const { user, logout } = useAuth();
    const [isLoginModalOpen, setIsLoginModalOpen] = React.useState(false);
    const [isSignupModalOpen, setIsSignupModalOpen] = React.useState(false);
    const [loginForm, setLoginForm] = React.useState({ email: '', password: '' });
    const [signupForm, setSignupForm] = React.useState({
        name: '',
        email: '',
        password: '',
        confirmPassword: ''
    });
    const [error, setError] = React.useState('');

    const handleLogin = (e) => {
        e.preventDefault();
        setError('');

        const result = useAuth().login(loginForm.email, loginForm.password);
        if (!result.success) {
            setError(result.message);
        } else {
            setIsLoginModalOpen(false);
            setLoginForm({ email: '', password: '' });
        }
    };

    const handleSignup = (e) => {
        e.preventDefault();
        setError('');

        if (signupForm.password !== signupForm.confirmPassword) {
            setError('Passwords do not match');
            return;
        }

        const result = useAuth().signup(signupForm.name, signupForm.email, signupForm.password);
        if (!result.success) {
            setError(result.message);
        } else {
            setIsSignupModalOpen(false);
            setSignupForm({ name: '', email: '', password: '', confirmPassword: '' });
        }
    };

    const closeModal = () => {
        setIsLoginModalOpen(false);
        setIsSignupModalOpen(false);
        setError('');
    };

    if (user) {
        return React.createElement(
            'div',
            { className: 'user-info' },
            React.createElement('span', { className: 'welcome-text' }, `Welcome, ${user.name}`),
            React.createElement(
                'button',
                {
                    className: 'auth-btn logout-btn',
                    onClick: logout
                },
                'Logout'
            )
        );
    }

    return React.createElement(
        'div',
        null,
        React.createElement(
            'button',
            {
                className: 'auth-btn login-btn',
                onClick: () => setIsLoginModalOpen(true)
            },
            'Login'
        ),
        React.createElement(
            'button',
            {
                className: 'auth-btn signup-btn',
                onClick: () => setIsSignupModalOpen(true)
            },
            'Sign Up'
        ),

        // Login Modal
        isLoginModalOpen && React.createElement(
            'div',
            { className: 'modal-overlay' },
            React.createElement(
                'div',
                { className: 'modal-content' },
                React.createElement(
                    'div',
                    { className: 'modal-header' },
                    React.createElement('h2', null, 'Login'),
                    React.createElement(
                        'button',
                        { className: 'close-btn', onClick: closeModal },
                        '✖'
                    )
                ),
                React.createElement(
                    'form',
                    { onSubmit: handleLogin, className: 'auth-form' },
                    error && React.createElement('div', { className: 'error-message' }, error),
                    React.createElement(
                        'div',
                        { className: 'form-group' },
                        React.createElement('label', { htmlFor: 'login-email' }, 'Email'),
                        React.createElement('input', {
                            type: 'email',
                            id: 'login-email',
                            value: loginForm.email,
                            onChange: (e) => setLoginForm({ ...loginForm, email: e.target.value }),
                            required: true
                        })
                    ),
                    React.createElement(
                        'div',
                        { className: 'form-group' },
                        React.createElement('label', { htmlFor: 'login-password' }, 'Password'),
                        React.createElement('input', {
                            type: 'password',
                            id: 'login-password',
                            value: loginForm.password,
                            onChange: (e) => setLoginForm({ ...loginForm, password: e.target.value }),
                            required: true
                        })
                    ),
                    React.createElement(
                        'button',
                        { type: 'submit', className: 'submit-btn' },
                        'Login'
                    )
                )
            )
        ),

        // Signup Modal
        isSignupModalOpen && React.createElement(
            'div',
            { className: 'modal-overlay' },
            React.createElement(
                'div',
                { className: 'modal-content' },
                React.createElement(
                    'div',
                    { className: 'modal-header' },
                    React.createElement('h2', null, 'Sign Up'),
                    React.createElement(
                        'button',
                        { className: 'close-btn', onClick: closeModal },
                        '✖'
                    )
                ),
                React.createElement(
                    'form',
                    { onSubmit: handleSignup, className: 'auth-form' },
                    error && React.createElement('div', { className: 'error-message' }, error),
                    React.createElement(
                        'div',
                        { className: 'form-group' },
                        React.createElement('label', { htmlFor: 'signup-name' }, 'Name'),
                        React.createElement('input', {
                            type: 'text',
                            id: 'signup-name',
                            value: signupForm.name,
                            onChange: (e) => setSignupForm({ ...signupForm, name: e.target.value }),
                            required: true
                        })
                    ),
                    React.createElement(
                        'div',
                        { className: 'form-group' },
                        React.createElement('label', { htmlFor: 'signup-email' }, 'Email'),
                        React.createElement('input', {
                            type: 'email',
                            id: 'signup-email',
                            value: signupForm.email,
                            onChange: (e) => setSignupForm({ ...signupForm, email: e.target.value }),
                            required: true
                        })
                    ),
                    React.createElement(
                        'div',
                        { className: 'form-group' },
                        React.createElement('label', { htmlFor: 'signup-password' }, 'Password'),
                        React.createElement('input', {
                            type: 'password',
                            id: 'signup-password',
                            value: signupForm.password,
                            onChange: (e) => setSignupForm({ ...signupForm, password: e.target.value }),
                            required: true
                        })
                    ),
                    React.createElement(
                        'div',
                        { className: 'form-group' },
                        React.createElement('label', { htmlFor: 'signup-confirm-password' }, 'Confirm Password'),
                        React.createElement('input', {
                            type: 'password',
                            id: 'signup-confirm-password',
                            value: signupForm.confirmPassword,
                            onChange: (e) => setSignupForm({ ...signupForm, confirmPassword: e.target.value }),
                            required: true
                        })
                    ),
                    React.createElement(
                        'button',
                        { type: 'submit', className: 'submit-btn' },
                        'Sign Up'
                    )
                )
            )
        )
    );
}