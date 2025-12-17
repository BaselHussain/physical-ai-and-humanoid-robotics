/**
 * AuthContext and useAuth Hook
 *
 * Global authentication state management for the application.
 * Provides signup, signin, signout functions and auth state to all components.
 */

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';

// Backend API base URL (update for production)
const API_BASE_URL = typeof window !== 'undefined' && (window as any).REACT_APP_API_URL
  ? (window as any).REACT_APP_API_URL
  : 'http://localhost:8000';

interface BackgroundProfile {
  programming_experience: string;
  ros2_familiarity: string;
  hardware_access: string;
}

interface User {
  id: number;
  email: string;
  background: BackgroundProfile;
}

interface AuthState {
  user: User | null;
  token: string | null;
  isAuthenticated: boolean;
  isLoading: boolean;
}

interface AuthContextValue extends AuthState {
  signup: (email: string, password: string, background: BackgroundProfile) => Promise<void>;
  signin: (email: string, password: string) => Promise<void>;
  signout: () => void;
  refreshSession: () => Promise<void>;
}

const AuthContext = createContext<AuthContextValue | null>(null);

// Local storage keys
const TOKEN_KEY = 'rag_chatbot_token';
const USER_KEY = 'rag_chatbot_user';

export function AuthProvider({ children }: { children: ReactNode }) {
  const [authState, setAuthState] = useState<AuthState>({
    user: null,
    token: null,
    isAuthenticated: false,
    isLoading: true,
  });

  // Initialize auth state from localStorage on mount
  useEffect(() => {
    const initAuth = async () => {
      const storedToken = localStorage.getItem(TOKEN_KEY);
      const storedUser = localStorage.getItem(USER_KEY);

      if (storedToken && storedUser) {
        try {
          const user = JSON.parse(storedUser);

          // Validate token with backend
          const isValid = await validateToken(storedToken);

          if (isValid) {
            setAuthState({
              user,
              token: storedToken,
              isAuthenticated: true,
              isLoading: false,
            });
          } else {
            // Token invalid, clear storage
            localStorage.removeItem(TOKEN_KEY);
            localStorage.removeItem(USER_KEY);
            setAuthState({
              user: null,
              token: null,
              isAuthenticated: false,
              isLoading: false,
            });
          }
        } catch (error) {
          console.error('Auth initialization error:', error);
          setAuthState({
            user: null,
            token: null,
            isAuthenticated: false,
            isLoading: false,
          });
        }
      } else {
        setAuthState((prev) => ({ ...prev, isLoading: false }));
      }
    };

    initAuth();
  }, []);

  // Validate token with backend
  const validateToken = async (token: string): Promise<boolean> => {
    try {
      const response = await fetch(`${API_BASE_URL}/auth/session`, {
        headers: {
          Authorization: `Bearer ${token}`,
        },
      });

      return response.ok;
    } catch (error) {
      console.error('Token validation error:', error);
      return false;
    }
  };

  // Signup function
  const signup = async (
    email: string,
    password: string,
    background: BackgroundProfile
  ): Promise<void> => {
    try {
      const response = await fetch(`${API_BASE_URL}/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
          background,
        }),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail || 'Signup failed');
      }

      const data = await response.json();

      // Store token and user info
      localStorage.setItem(TOKEN_KEY, data.token);
      localStorage.setItem(
        USER_KEY,
        JSON.stringify({
          id: data.user_id,
          email,
          background: data.background,
        })
      );

      setAuthState({
        user: {
          id: data.user_id,
          email,
          background: data.background,
        },
        token: data.token,
        isAuthenticated: true,
        isLoading: false,
      });
    } catch (error) {
      console.error('Signup error:', error);
      throw error;
    }
  };

  // Signin function
  const signin = async (email: string, password: string): Promise<void> => {
    try {
      const response = await fetch(`${API_BASE_URL}/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email,
          password,
        }),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail || 'Sign in failed');
      }

      const data = await response.json();

      // Store token and user info
      localStorage.setItem(TOKEN_KEY, data.token);
      localStorage.setItem(
        USER_KEY,
        JSON.stringify({
          id: data.user_id,
          email,
          background: data.background,
        })
      );

      setAuthState({
        user: {
          id: data.user_id,
          email,
          background: data.background,
        },
        token: data.token,
        isAuthenticated: true,
        isLoading: false,
      });
    } catch (error) {
      console.error('Signin error:', error);
      throw error;
    }
  };

  // Signout function
  const signout = () => {
    // Clear local storage
    localStorage.removeItem(TOKEN_KEY);
    localStorage.removeItem(USER_KEY);

    // Reset auth state
    setAuthState({
      user: null,
      token: null,
      isAuthenticated: false,
      isLoading: false,
    });

    // Note: Backend signout is optional since JWT is stateless
    // We just remove it from client side
  };

  // Refresh session (check token validity)
  const refreshSession = async (): Promise<void> => {
    const token = authState.token || localStorage.getItem(TOKEN_KEY);

    if (!token) {
      signout();
      return;
    }

    const isValid = await validateToken(token);

    if (!isValid) {
      signout();
      throw new Error('Session expired. Please sign in again.');
    }
  };

  const value: AuthContextValue = {
    ...authState,
    signup,
    signin,
    signout,
    refreshSession,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

// useAuth hook for components
export function useAuth(): AuthContextValue {
  const context = useContext(AuthContext);

  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }

  return context;
}

// Export AuthContext for advanced use cases
export { AuthContext };
