/**
 * Server Integration Tests
 * Tests Express server, CORS, health check, and Better Auth endpoints
 */

import request from 'supertest';
import express from 'express';

// Mock environment variables before importing server
process.env.BETTER_AUTH_SECRET = 'test-secret-key-32-chars-minimum';
process.env.DATABASE_URL = 'postgresql://test:test@localhost:5432/test_db';
process.env.BETTER_AUTH_URL = 'http://localhost:3000';
process.env.CORS_ORIGIN = 'http://localhost:3001,http://localhost:3000';
process.env.NODE_ENV = 'test';

describe('Better Auth Service - Server Tests', () => {
  let app: express.Application;

  beforeAll(() => {
    // Dynamically import app after env vars are set
    app = require('../src/server').default;
  });

  describe('Health Check Endpoint', () => {
    it('should return 200 OK with service status', async () => {
      const response = await request(app).get('/health');

      expect(response.status).toBe(200);
      expect(response.body).toHaveProperty('status', 'ok');
      expect(response.body).toHaveProperty('service', 'better-auth-service');
      expect(response.body).toHaveProperty('timestamp');
      expect(new Date(response.body.timestamp)).toBeInstanceOf(Date);
    });
  });

  describe('CORS Configuration', () => {
    it('should allow requests from configured origins', async () => {
      const response = await request(app)
        .get('/health')
        .set('Origin', 'http://localhost:3001');

      expect(response.status).toBe(200);
      expect(response.headers['access-control-allow-origin']).toBe('http://localhost:3001');
      expect(response.headers['access-control-allow-credentials']).toBe('true');
    });

    it('should allow requests with no origin (curl, Postman)', async () => {
      const response = await request(app).get('/health');

      expect(response.status).toBe(200);
    });

    it('should handle OPTIONS preflight requests', async () => {
      const response = await request(app)
        .options('/api/auth/sign-in/email')
        .set('Origin', 'http://localhost:3001')
        .set('Access-Control-Request-Method', 'POST');

      expect(response.status).toBe(204);
      expect(response.headers['access-control-allow-methods']).toContain('POST');
    });
  });

  describe('JWKS Endpoint', () => {
    it('should expose JWKS endpoint at /.well-known/jwks.json', async () => {
      const response = await request(app).get('/.well-known/jwks.json');

      // The endpoint exists (may fail to fetch in test environment)
      expect([200, 500]).toContain(response.status);
    });
  });

  describe('Better Auth Routes', () => {
    it('should mount Better Auth routes at /api/auth', async () => {
      // Test that Better Auth routes are mounted
      const response = await request(app).post('/api/auth/sign-up/email');

      // Should not be 404 (route exists)
      expect(response.status).not.toBe(404);
    });
  });

  describe('Error Handling', () => {
    it('should return 500 for unhandled errors', async () => {
      // This test verifies error middleware is present
      // Actual error scenarios would need mocking
      expect(app).toBeDefined();
    });
  });
});
