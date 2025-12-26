/**
 * AuthContext and useAuth Hook
 *
 * Global authentication state management for the application.
 * Provides signup, signin, signout functions and auth state to all components.
 *
 * Updated to use Better Auth client SDK for authentication.
 */

import React, { createContext, useContext, ReactNode } from 'react';
import { authClient, useSession as useBetterAuthSession } from '../../lib/auth-client';

/**
 * Background profile for user personalization
 * These fields are stored as custom fields in Better Auth
 */
interface BackgroundProfile {
  programming_experience: string;
  ros2_familiarity: string;
  hardware_access: string;
}

/**
 * Infer User and Session types from Better Auth client
 * This ensures type safety with the Better Auth SDK
 */
type Session = typeof authClient.$Infer.Session;
type User = Session['user'];

interface AuthState {
  user: User | null;
  session: Session | null;
  isAuthenticated: boolean;
  isLoading: boolean;
}

interface AuthContextValue extends AuthState {
  signup: (email: string, password: string, name: string, background: BackgroundProfile) => Promise<void>;
  signin: (email: string, password: string) => Promise<void>;
  signout: () => Promise<void>;
  refreshSession: () => Promise<void>;
}

const AuthContext = createContext<AuthContextValue | null>(null);

export function AuthProvider({ children }: { children: ReactNode }) {
  // Use Better Auth session hook for automatic session management
  const sessionQuery = useBetterAuthSession();

  // Signup function - now uses Better Auth client
  const signup = async (
    email: string,
    password: string,
    name: string,
    background: BackgroundProfile
  ): Promise<void> => {
    try {
      // Call Better Auth signup with additional fields
      // Type assertion needed because Better Auth server has custom fields configured
      // that aren't reflected in the client SDK types (separate project limitation)
      const result = await authClient.signUp.email({
        email,
        password,
        name,
        // Additional fields from Better Auth server config
        programming_experience: background.programming_experience,
        ros2_familiarity: background.ros2_familiarity,
        hardware_access: background.hardware_access,
      } as any);

      // Check for errors
      if (result.error) {
        throw new Error(result.error.message || 'Signup failed');
      }

      // Session is automatically managed by Better Auth
      // The useSession hook will update automatically
    } catch (error) {
      console.error('Signup error:', error);
      throw error;
    }
  };

  // Signin function - now uses Better Auth client
  const signin = async (email: string, password: string): Promise<void> => {
    try {
      // Call Better Auth signin
      const result = await authClient.signIn.email({
        email,
        password,
      });

      // Check for errors
      if (result.error) {
        throw new Error(result.error.message || 'Sign in failed');
      }

      // Session is automatically managed by Better Auth
      // The useSession hook will update automatically
    } catch (error) {
      console.error('Signin error:', error);
      throw error;
    }
  };

  // Signout function - now uses Better Auth client
  const signout = async (): Promise<void> => {
    try {
      await authClient.signOut();
      // Session is automatically cleared by Better Auth
    } catch (error) {
      console.error('Signout error:', error);
      throw error;
    }
  };

  // Refresh session - Better Auth handles this automatically
  const refreshSession = async (): Promise<void> => {
    try {
      // Refetch session from Better Auth
      await sessionQuery.refetch();
    } catch (error) {
      console.error('Session refresh error:', error);
      throw error;
    }
  };

  // Map Better Auth session to our auth state format
  const authState: AuthState = {
    user: sessionQuery.data?.user || null,
    session: sessionQuery.data || null,
    isAuthenticated: !!sessionQuery.data?.user,
    isLoading: sessionQuery.isPending,
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
