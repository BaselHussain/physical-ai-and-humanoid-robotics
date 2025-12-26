/**
 * Better Auth Client Instance
 *
 * Centralized authentication client for the frontend.
 * Connects to the Better Auth microservice running on port 3000.
 */

import { createAuthClient } from 'better-auth/react';
import { inferAdditionalFields } from 'better-auth/client/plugins';

/**
 * Better Auth base URL
 * - Development: http://localhost:3000
 * - Production: Update with deployed Better Auth service URL
 */
const BETTER_AUTH_URL = typeof window !== 'undefined' && (window as any).REACT_APP_BETTER_AUTH_URL
  ? (window as any).REACT_APP_BETTER_AUTH_URL
  : 'http://localhost:3000';

/**
 * Create and export Better Auth client instance
 *
 * This client provides:
 * - Authentication methods (signUp, signIn, signOut)
 * - Session management hooks (useSession)
 * - User info extraction
 * - Automatic token handling
 *
 * The inferAdditionalFields plugin tells TypeScript about custom user fields
 * that are configured on the Better Auth server (programming_experience,
 * ros2_familiarity, hardware_access).
 */
export const authClient = createAuthClient({
  baseURL: BETTER_AUTH_URL,
  plugins: [
    inferAdditionalFields({
      user: {
        programming_experience: {
          type: 'string',
        },
        ros2_familiarity: {
          type: 'string',
        },
        hardware_access: {
          type: 'string',
        },
      },
    }),
  ],
});

/**
 * Export specific methods for convenience
 *
 * Usage:
 * import { signIn, signUp, signOut, useSession } from '@/lib/auth-client';
 */
export const {
  signIn,
  signUp,
  signOut,
  useSession
} = authClient;
