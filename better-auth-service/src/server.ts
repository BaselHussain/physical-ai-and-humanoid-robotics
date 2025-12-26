import express from 'express';
import cors from 'cors';
import { toNodeHandler } from 'better-auth/node';
import { auth } from './auth';
import dotenv from 'dotenv';

// Load environment variables
dotenv.config();

const app = express();
const port = process.env.PORT || 3000;

// Parse CORS origins from environment variable
const corsOrigins = process.env.CORS_ORIGIN
  ? process.env.CORS_ORIGIN.split(',').map((origin) => origin.trim())
  : ['http://localhost:3001', 'http://localhost:3000'];

// CORS configuration
app.use(
  cors({
    origin: (origin, callback) => {
      // Allow requests with no origin (like mobile apps, curl, Postman)
      if (!origin) return callback(null, true);

      // Check if origin matches any of the allowed origins or wildcard patterns
      const isAllowed = corsOrigins.some((allowedOrigin) => {
        // Support wildcard subdomains (e.g., *.yourdomain.com)
        if (allowedOrigin.includes('*')) {
          const pattern = allowedOrigin.replace(/\*/g, '.*');
          const regex = new RegExp(`^${pattern}$`);
          return regex.test(origin);
        }
        return allowedOrigin === origin;
      });

      if (isAllowed) {
        callback(null, true);
      } else {
        callback(new Error(`Origin ${origin} not allowed by CORS`));
      }
    },
    credentials: true, // Allow cookies to be sent
    methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS'],
    allowedHeaders: ['Content-Type', 'Authorization'],
  })
);

// Body parser middleware
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Health check endpoint
app.get('/health', (req, res) => {
  res.json({
    status: 'ok',
    service: 'better-auth-service',
    timestamp: new Date().toISOString(),
  });
});

// Better Auth routes handler
// All Better Auth routes will be available at /api/auth/*
app.use('/api/auth', toNodeHandler(auth));

// JWKS endpoint (public keys for JWT validation)
// This is a dedicated route for easier access
app.get('/.well-known/jwks.json', async (req, res) => {
  try {
    // Forward to Better Auth's JWKS endpoint
    const jwksUrl = `${process.env.BETTER_AUTH_URL || 'http://localhost:3000'}/api/auth/jwks`;
    const response = await fetch(jwksUrl);
    const jwks = await response.json();
    res.json(jwks);
  } catch (error) {
    console.error('Error fetching JWKS:', error);
    res.status(500).json({ error: 'Failed to fetch JWKS' });
  }
});

// Error handling middleware
app.use((err: Error, req: express.Request, res: express.Response, next: express.NextFunction) => {
  console.error('Server error:', err);

  // Handle CORS errors
  if (err.message.includes('not allowed by CORS')) {
    return res.status(403).json({
      error: 'CORS error',
      message: err.message,
    });
  }

  // Generic error response
  res.status(500).json({
    error: 'Internal server error',
    message: process.env.NODE_ENV === 'development' ? err.message : 'An error occurred',
  });
});

// Start server
app.listen(port, () => {
  console.log(`✓ Better Auth service running on port ${port}`);
  console.log(`✓ Environment: ${process.env.NODE_ENV || 'development'}`);
  console.log(`✓ Base URL: ${process.env.BETTER_AUTH_URL || 'http://localhost:3000'}`);
  console.log(`✓ CORS origins: ${corsOrigins.join(', ')}`);
  console.log(`\nEndpoints:`);
  console.log(`  - Health: http://localhost:${port}/health`);
  console.log(`  - Auth API: http://localhost:${port}/api/auth/*`);
  console.log(`  - JWKS: http://localhost:${port}/.well-known/jwks.json`);
  console.log(`\nKey endpoints:`);
  console.log(`  - Sign Up: POST http://localhost:${port}/api/auth/sign-up/email`);
  console.log(`  - Sign In: POST http://localhost:${port}/api/auth/sign-in/email`);
  console.log(`  - Sign Out: POST http://localhost:${port}/api/auth/sign-out`);
  console.log(`  - Get Session: GET http://localhost:${port}/api/auth/get-session`);
});

export default app;
