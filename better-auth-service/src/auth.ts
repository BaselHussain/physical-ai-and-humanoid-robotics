import { betterAuth } from 'better-auth';
import { jwt } from 'better-auth/plugins';
import { db } from './database';
import dotenv from 'dotenv';

// Load environment variables
dotenv.config();

/**
 * Better Auth instance with Kysely PostgreSQL adapter,
 * JWT plugin for token generation with custom claims,
 * and custom user metadata for RAG chatbot personalization
 */
export const auth = betterAuth({
  database: db,

  // Secret and base URL from environment
  secret: process.env.BETTER_AUTH_SECRET!,
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:3000',

  // Enable email/password authentication
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production
  },

  // Extend user schema with custom metadata for personalization
  user: {
    additionalFields: {
      // Programming experience level (beginner, intermediate, advanced)
      programming_experience: {
        type: 'string',
        required: false,
        defaultValue: 'beginner',
      },
      // ROS 2 familiarity (none, basic, intermediate, advanced)
      ros2_familiarity: {
        type: 'string',
        required: false,
        defaultValue: 'none',
      },
      // Hardware access (none, simulation, physical)
      hardware_access: {
        type: 'string',
        required: false,
        defaultValue: 'none',
      },
    },
  },

  // Session configuration
  session: {
    expiresIn: 60 * 15, // 15 minutes (access token)
    updateAge: 60 * 1, // Update session every 1 minute
  },

  // Advanced configuration
  advanced: {
    cookiePrefix: 'better-auth',
    crossSubDomainCookies: {
      enabled: true,
    },
  },

  // Plugins
  plugins: [
    // JWT plugin for RS256 asymmetric token signing with JWKS endpoint
    jwt({
      // JWT payload configuration
      jwt: {
        // Define custom JWT payload with user background metadata
        definePayload: ({ user }) => {
          return {
            id: user.id,
            email: user.email,
            name: user.name,
            // Custom claims namespace for user background (RFC 7519 compliant)
            'https://yourdomain.com/claims': {
              programming_experience: user.programming_experience,
              ros2_familiarity: user.ros2_familiarity,
              hardware_access: user.hardware_access,
            },
          };
        },
        // JWT expiration time
        expirationTime: '15m', // 15 minutes to match session
        // Issuer and audience (will be set to baseURL by default)
        issuer: process.env.BETTER_AUTH_URL || 'http://localhost:3000',
        audience: process.env.BETTER_AUTH_URL || 'http://localhost:3000',
      },
      // JWKS configuration for RS256 signing
      jwks: {
        keyPairConfig: {
          alg: 'RS256', // RSA with SHA-256
          modulusLength: 2048, // 2048-bit key
        },
        // Key rotation (optional, for production)
        // rotationInterval: 60 * 60 * 24 * 30, // 30 days
        // gracePeriod: 60 * 60 * 24 * 30, // 30 days
      },
    }),
  ],
});

export type Auth = typeof auth;
