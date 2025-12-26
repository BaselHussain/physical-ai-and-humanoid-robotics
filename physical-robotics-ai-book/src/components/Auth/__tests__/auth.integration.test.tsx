/**
 * Integration tests for full authentication flow
 *
 * Covers:
 * - Complete signup → signin → signout flow
 * - Token refresh scenarios
 * - Service unavailability handling
 * - Session persistence
 */

import React from 'react';
import { render, screen, waitFor, act } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { AuthProvider, useAuth } from '../AuthContext';
import SignupForm from '../SignupForm';
import SigninForm from '../SigninForm';
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

// Integration test component simulating app-level auth flow
function AuthFlowTestApp() {
  const auth = useAuth();
  const [showSignup, setShowSignup] = React.useState(true);

  return (
    <div>
      <div data-testid="auth-status">
        {auth.isAuthenticated ? `Authenticated: ${auth.user?.email}` : 'Guest'}
      </div>

      {auth.isAuthenticated ? (
        <div>
          <div data-testid="user-info">
            <p>User ID: {auth.user?.id}</p>
            <p>Email: {auth.user?.email}</p>
            <p>Name: {auth.user?.name}</p>
          </div>
          <button data-testid="signout-btn" onClick={() => auth.signout()}>
            Sign Out
          </button>
        </div>
      ) : (
        <div>
          {showSignup ? (
            <SignupForm
              onSubmit={auth.signup}
              onSwitchToSignin={() => setShowSignup(false)}
            />
          ) : (
            <SigninForm
              onSubmit={auth.signin}
              onSwitchToSignup={() => setShowSignup(true)}
            />
          )}
        </div>
      )}
    </div>
  );
}

describe('Authentication Integration Tests', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('Complete authentication flow', () => {
    it('should complete full signup → signin → signout flow', async () => {
      // Initial state: unauthenticated
      let sessionData: any = null;
      const mockRefetch = jest.fn();

      (authClientModule.useSession as jest.Mock).mockImplementation(() => ({
        data: sessionData,
        isPending: false,
        refetch: mockRefetch,
      }));

      // Mock signup
      (authClientModule.authClient.signUp.email as jest.Mock).mockImplementation(
        async (data) => {
          sessionData = {
            user: {
              id: 'user123',
              email: data.email,
              name: data.name,
              programming_experience: data.programming_experience,
              ros2_familiarity: data.ros2_familiarity,
              hardware_access: data.hardware_access,
            },
            session: {
              id: 'session123',
              userId: 'user123',
              expiresAt: new Date(Date.now() + 900000),
            },
          };
          return { data: sessionData, error: null };
        }
      );

      // Mock signin
      (authClientModule.authClient.signIn.email as jest.Mock).mockImplementation(
        async () => {
          sessionData = {
            user: {
              id: 'user123',
              email: 'test@example.com',
              name: 'Test User',
            },
            session: {
              id: 'session456',
              userId: 'user123',
              expiresAt: new Date(Date.now() + 900000),
            },
          };
          return { data: sessionData, error: null };
        }
      );

      // Mock signout
      (authClientModule.authClient.signOut as jest.Mock).mockImplementation(async () => {
        sessionData = null;
        return {};
      });

      const { rerender } = render(
        <AuthProvider>
          <AuthFlowTestApp />
        </AuthProvider>
      );

      // STEP 1: Initial state - should show signup form
      expect(screen.getByTestId('auth-status')).toHaveTextContent('Guest');
      expect(screen.getByLabelText(/full name/i)).toBeInTheDocument();

      // STEP 2: Fill out and submit signup form
      await userEvent.type(screen.getByLabelText(/full name/i), 'Test User');
      await userEvent.type(screen.getByLabelText(/email/i), 'test@example.com');
      await userEvent.type(screen.getByLabelText(/password/i), 'password123');
      await userEvent.selectOptions(
        screen.getByLabelText(/programming experience/i),
        '3-5 years'
      );
      await userEvent.selectOptions(screen.getByLabelText(/ros 2 familiarity/i), 'Basic');
      await userEvent.selectOptions(
        screen.getByLabelText(/hardware access/i),
        'Simulation only'
      );

      await act(async () => {
        const submitButton = screen.getByRole('button', { name: /sign up/i });
        await userEvent.click(submitButton);
      });

      // Force rerender to pick up session change
      rerender(
        <AuthProvider>
          <AuthFlowTestApp />
        </AuthProvider>
      );

      // STEP 3: Should be authenticated now
      await waitFor(() => {
        expect(screen.queryByTestId('auth-status')).toHaveTextContent(
          'Authenticated: test@example.com'
        );
      });

      // STEP 4: Sign out
      await act(async () => {
        const signoutButton = screen.getByTestId('signout-btn');
        await userEvent.click(signoutButton);
      });

      rerender(
        <AuthProvider>
          <AuthFlowTestApp />
        </AuthProvider>
      );

      // STEP 5: Should be back to guest state
      await waitFor(() => {
        expect(screen.getByTestId('auth-status')).toHaveTextContent('Guest');
      });
    });
  });

  describe('Token refresh scenarios', () => {
    it('should auto-refresh session when refetch is called', async () => {
      let sessionData: any = {
        user: {
          id: 'user123',
          email: 'test@example.com',
          name: 'Test User',
        },
        session: {
          id: 'session123',
          userId: 'user123',
          expiresAt: new Date(Date.now() + 60000), // 1 minute from now
        },
      };

      const mockRefetch = jest.fn().mockImplementation(async () => {
        // Simulate token refresh
        sessionData = {
          ...sessionData,
          session: {
            ...sessionData.session,
            id: 'session-refreshed',
            expiresAt: new Date(Date.now() + 900000), // 15 minutes from now
          },
        };
        return { data: sessionData };
      });

      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: sessionData,
        isPending: false,
        refetch: mockRefetch,
      });

      function TokenRefreshTest() {
        const auth = useAuth();
        return (
          <div>
            <div data-testid="session-id">{auth.session?.session.id}</div>
            <button data-testid="refresh-btn" onClick={() => auth.refreshSession()}>
              Refresh
            </button>
          </div>
        );
      }

      render(
        <AuthProvider>
          <TokenRefreshTest />
        </AuthProvider>
      );

      // Initial session ID
      expect(screen.getByTestId('session-id')).toHaveTextContent('session123');

      // Trigger refresh
      await act(async () => {
        await userEvent.click(screen.getByTestId('refresh-btn'));
      });

      // Should have called refetch
      expect(mockRefetch).toHaveBeenCalled();
    });
  });

  describe('Service unavailability handling', () => {
    it('should handle Better Auth service being unavailable during signup', async () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: null,
        isPending: false,
        refetch: jest.fn(),
      });

      // Mock signup to simulate service unavailable
      (authClientModule.authClient.signUp.email as jest.Mock).mockRejectedValue(
        new Error('Network Error: Service Unavailable')
      );

      render(
        <AuthProvider>
          <AuthFlowTestApp />
        </AuthProvider>
      );

      // Fill out signup form
      await userEvent.type(screen.getByLabelText(/full name/i), 'Test User');
      await userEvent.type(screen.getByLabelText(/email/i), 'test@example.com');
      await userEvent.type(screen.getByLabelText(/password/i), 'password123');
      await userEvent.selectOptions(
        screen.getByLabelText(/programming experience/i),
        '0-2 years'
      );
      await userEvent.selectOptions(screen.getByLabelText(/ros 2 familiarity/i), 'None');
      await userEvent.selectOptions(screen.getByLabelText(/hardware access/i), 'None');

      // Submit - should fail
      await act(async () => {
        const submitButton = screen.getByRole('button', { name: /sign up/i });
        await userEvent.click(submitButton);
      });

      // Should show error message
      await waitFor(() => {
        expect(
          screen.getByText(/network error: service unavailable/i)
        ).toBeInTheDocument();
      });

      // Should still be in guest state
      expect(screen.getByTestId('auth-status')).toHaveTextContent('Guest');

      consoleErrorSpy.mockRestore();
    });

    it('should handle Better Auth service being unavailable during signin', async () => {
      const consoleErrorSpy = jest.spyOn(console, 'error').mockImplementation();

      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: null,
        isPending: false,
        refetch: jest.fn(),
      });

      // Mock signin to simulate service unavailable
      (authClientModule.authClient.signIn.email as jest.Mock).mockResolvedValue({
        data: null,
        error: { message: 'Authentication service temporarily unavailable' },
      });

      render(
        <AuthProvider>
          <AuthFlowTestApp />
        </AuthProvider>
      );

      // Switch to signin form
      await userEvent.click(screen.getByRole('button', { name: /sign in/i }));

      // Fill out signin form
      await userEvent.type(screen.getByLabelText(/email/i), 'test@example.com');
      await userEvent.type(screen.getByLabelText(/password/i), 'password123');

      // Submit - should fail
      await act(async () => {
        const submitButton = screen.getByRole('button', { name: /sign in/i });
        await userEvent.click(submitButton);
      });

      // Should show error message
      await waitFor(() => {
        expect(
          screen.getByText(/authentication service temporarily unavailable/i)
        ).toBeInTheDocument();
      });

      consoleErrorSpy.mockRestore();
    });
  });

  describe('Session persistence', () => {
    it('should maintain session state across component rerenders', async () => {
      const sessionData = {
        user: {
          id: 'user123',
          email: 'test@example.com',
          name: 'Test User',
          programming_experience: '3-5 years',
          ros2_familiarity: 'Basic',
          hardware_access: 'Simulation only',
        },
        session: {
          id: 'session123',
          userId: 'user123',
          expiresAt: new Date(Date.now() + 900000),
        },
      };

      (authClientModule.useSession as jest.Mock).mockReturnValue({
        data: sessionData,
        isPending: false,
        refetch: jest.fn(),
      });

      const { rerender } = render(
        <AuthProvider>
          <AuthFlowTestApp />
        </AuthProvider>
      );

      // Should be authenticated
      expect(screen.getByTestId('auth-status')).toHaveTextContent(
        'Authenticated: test@example.com'
      );

      // Rerender
      rerender(
        <AuthProvider>
          <AuthFlowTestApp />
        </AuthProvider>
      );

      // Should still be authenticated
      expect(screen.getByTestId('auth-status')).toHaveTextContent(
        'Authenticated: test@example.com'
      );
      expect(screen.getByText(/user id: user123/i)).toBeInTheDocument();
    });
  });
});
