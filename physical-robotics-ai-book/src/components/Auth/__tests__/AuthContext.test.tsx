/**
 * Tests for AuthContext and useAuth hook
 *
 * Covers:
 * - AuthProvider initialization and state management
 * - Signup flow with Better Auth client
 * - Signin flow with Better Auth client
 * - Signout flow
 * - Session refresh
 * - Error handling
 */

import React from 'react';
import { render, screen, waitFor, act } from '@testing-library/react';
import { AuthProvider, useAuth } from '../AuthContext';
import * as authClientModule from '../../../lib/auth-client';

// Mock the auth-client module
jest.mock('../../../lib/auth-client', () => ({
  authClient: {
    signUp: {
      email: jest.fn(),
    },
    signIn: {
      email: jest.fn(),
    },
    signOut: jest.fn(),
  },
  useSession: jest.fn(),
}));

// Test component to access auth context
function TestComponent() {
  const auth = useAuth();

  return (
    <div>
      <div data-testid="isAuthenticated">{String(auth.isAuthenticated)}</div>
      <div data-testid="isLoading">{String(auth.isLoading)}</div>
      <div data-testid="userEmail">{auth.user?.email || 'null'}</div>
      <button
        data-testid="signup-btn"
        onClick={() =>
          auth.signup('test@example.com', 'password123', 'Test User', {
            programming_experience: 'intermediate',
            ros2_familiarity: 'basic',
            hardware_access: 'simulation',
          })
        }
      >
        Signup
      </button>
      <button
        data-testid="signin-btn"
        onClick={() => auth.signin('test@example.com', 'password123')}
      >
        Signin
      </button>
      <button data-testid="signout-btn" onClick={() => auth.signout()}>
        Signout
      </button>
      <button data-testid="refresh-btn" onClick={() => auth.refreshSession()}>
        Refresh
      </button>
    </div>
  );
}

describe('AuthContext', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('AuthProvider initialization', () => {
    it('should provide initial unauthenticated state', () => {
      // Mock useSession to return no session
      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: null,
        isPending: false,
        refetch: jest.fn(),
      });

      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      );

      expect(screen.getByTestId('isAuthenticated')).toHaveTextContent('false');
      expect(screen.getByTestId('isLoading')).toHaveTextContent('false');
      expect(screen.getByTestId('userEmail')).toHaveTextContent('null');
    });

    it('should provide authenticated state when session exists', () => {
      // Mock useSession to return active session
      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: {
          user: {
            id: 'user123',
            email: 'test@example.com',
            name: 'Test User',
          },
          session: {
            id: 'session123',
            userId: 'user123',
            expiresAt: new Date(Date.now() + 900000), // 15 minutes from now
          },
        },
        isPending: false,
        refetch: jest.fn(),
      });

      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      );

      expect(screen.getByTestId('isAuthenticated')).toHaveTextContent('true');
      expect(screen.getByTestId('userEmail')).toHaveTextContent('test@example.com');
    });

    it('should show loading state while session is being fetched', () => {
      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: null,
        isPending: true,
        refetch: jest.fn(),
      });

      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      );

      expect(screen.getByTestId('isLoading')).toHaveTextContent('true');
    });
  });

  describe('signup', () => {
    it('should successfully signup a new user', async () => {
      const mockRefetch = jest.fn();
      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: null,
        isPending: false,
        refetch: mockRefetch,
      });

      const mockSignup = authClientModule.authClient.signUp.email as jest.Mock;
      mockSignup.mockResolvedValue({
        data: {
          user: { id: 'user123', email: 'test@example.com' },
        },
        error: null,
      });

      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      );

      await act(async () => {
        screen.getByTestId('signup-btn').click();
      });

      await waitFor(() => {
        expect(mockSignup).toHaveBeenCalledWith({
          email: 'test@example.com',
          password: 'password123',
          name: 'Test User',
          programming_experience: 'intermediate',
          ros2_familiarity: 'basic',
          hardware_access: 'simulation',
        });
      });
    });

    it('should handle signup error', async () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: null,
        isPending: false,
        refetch: jest.fn(),
      });

      const mockSignup = authClientModule.authClient.signUp.email as jest.Mock;
      mockSignup.mockResolvedValue({
        data: null,
        error: { message: 'Email already exists' },
      });

      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      );

      await act(async () => {
        try {
          await screen.getByTestId('signup-btn').click();
        } catch (error) {
          // Expected to throw
        }
      });

      await waitFor(() => {
        expect(consoleErrorSpy).toHaveBeenCalled();
      });

      consoleErrorSpy.mockRestore();
    });
  });

  describe('signin', () => {
    it('should successfully signin a user', async () => {
      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: null,
        isPending: false,
        refetch: jest.fn(),
      });

      const mockSignin = authClientModule.authClient.signIn.email as jest.Mock;
      mockSignin.mockResolvedValue({
        data: {
          user: { id: 'user123', email: 'test@example.com' },
        },
        error: null,
      });

      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      );

      await act(async () => {
        screen.getByTestId('signin-btn').click();
      });

      await waitFor(() => {
        expect(mockSignin).toHaveBeenCalledWith({
          email: 'test@example.com',
          password: 'password123',
        });
      });
    });

    it('should handle signin error', async () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: null,
        isPending: false,
        refetch: jest.fn(),
      });

      const mockSignin = authClientModule.authClient.signIn.email as jest.Mock;
      mockSignin.mockResolvedValue({
        data: null,
        error: { message: 'Invalid credentials' },
      });

      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      );

      await act(async () => {
        try {
          await screen.getByTestId('signin-btn').click();
        } catch (error) {
          // Expected to throw
        }
      });

      await waitFor(() => {
        expect(consoleErrorSpy).toHaveBeenCalled();
      });

      consoleErrorSpy.mockRestore();
    });
  });

  describe('signout', () => {
    it('should successfully signout a user', async () => {
      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: {
          user: { id: 'user123', email: 'test@example.com' },
        },
        isPending: false,
        refetch: jest.fn(),
      });

      const mockSignout = authClientModule.authClient.signOut as jest.Mock;
      mockSignout.mockResolvedValue({});

      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      );

      await act(async () => {
        screen.getByTestId('signout-btn').click();
      });

      await waitFor(() => {
        expect(mockSignout).toHaveBeenCalled();
      });
    });

    it('should handle signout error', async () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: {
          user: { id: 'user123', email: 'test@example.com' },
        },
        isPending: false,
        refetch: jest.fn(),
      });

      const mockSignout = authClientModule.authClient.signOut as jest.Mock;
      mockSignout.mockRejectedValue(new Error('Signout failed'));

      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      );

      await act(async () => {
        try {
          await screen.getByTestId('signout-btn').click();
        } catch (error) {
          // Expected to throw
        }
      });

      await waitFor(() => {
        expect(consoleErrorSpy).toHaveBeenCalled();
      });

      consoleErrorSpy.mockRestore();
    });
  });

  describe('refreshSession', () => {
    it('should successfully refresh session', async () => {
      const mockRefetch = jest.fn().mockResolvedValue({});

      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: {
          user: { id: 'user123', email: 'test@example.com' },
        },
        isPending: false,
        refetch: mockRefetch,
      });

      render(
        <AuthProvider>
          <TestComponent />
        </AuthProvider>
      );

      await act(async () => {
        screen.getByTestId('refresh-btn').click();
      });

      await waitFor(() => {
        expect(mockRefetch).toHaveBeenCalled();
      });
    });
  });

  describe('useAuth hook', () => {
    it('should throw error when used outside AuthProvider', () => {
      // Suppress console.error for this test
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      function InvalidComponent() {
        useAuth(); // This should throw
        return null;
      }

      expect(() => render(<InvalidComponent />)).toThrow(
        'useAuth must be used within an AuthProvider'
      );

      consoleErrorSpy.mockRestore();
    });
  });
});
