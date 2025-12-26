/**
 * Better Auth Configuration Tests
 * Tests Auth instance configuration, JWT plugin, and custom user fields
 */

// Mock environment variables before importing auth
process.env.BETTER_AUTH_SECRET = 'test-secret-key-32-chars-minimum';
process.env.DATABASE_URL = 'postgresql://test:test@localhost:5432/test_db';
process.env.BETTER_AUTH_URL = 'http://localhost:3000';
process.env.NODE_ENV = 'test';

describe('Better Auth Configuration Tests', () => {
  let auth: any;

  beforeAll(() => {
    // Dynamically import auth after env vars are set
    auth = require('../src/auth').auth;
  });

  describe('Auth Instance', () => {
    it('should create Better Auth instance', () => {
      expect(auth).toBeDefined();
      expect(auth.options).toBeDefined();
    });

    it('should configure email/password authentication', () => {
      expect(auth.options.emailAndPassword).toBeDefined();
      expect(auth.options.emailAndPassword.enabled).toBe(true);
      expect(auth.options.emailAndPassword.requireEmailVerification).toBe(false);
    });

    it('should set correct base URL from environment', () => {
      expect(auth.options.baseURL).toBe('http://localhost:3000');
    });

    it('should have secret configured', () => {
      expect(auth.options.secret).toBe('test-secret-key-32-chars-minimum');
    });
  });

  describe('Custom User Fields', () => {
    it('should extend user schema with programming_experience field', () => {
      expect(auth.options.user.additionalFields).toBeDefined();
      expect(auth.options.user.additionalFields.programming_experience).toBeDefined();
      expect(auth.options.user.additionalFields.programming_experience.type).toBe('string');
      expect(auth.options.user.additionalFields.programming_experience.defaultValue).toBe('beginner');
    });

    it('should extend user schema with ros2_familiarity field', () => {
      expect(auth.options.user.additionalFields.ros2_familiarity).toBeDefined();
      expect(auth.options.user.additionalFields.ros2_familiarity.type).toBe('string');
      expect(auth.options.user.additionalFields.ros2_familiarity.defaultValue).toBe('none');
    });

    it('should extend user schema with hardware_access field', () => {
      expect(auth.options.user.additionalFields.hardware_access).toBeDefined();
      expect(auth.options.user.additionalFields.hardware_access.type).toBe('string');
      expect(auth.options.user.additionalFields.hardware_access.defaultValue).toBe('none');
    });

    it('should make all custom fields optional', () => {
      const fields = auth.options.user.additionalFields;
      expect(fields.programming_experience.required).toBe(false);
      expect(fields.ros2_familiarity.required).toBe(false);
      expect(fields.hardware_access.required).toBe(false);
    });
  });

  describe('Session Configuration', () => {
    it('should configure session expiration to 15 minutes', () => {
      expect(auth.options.session).toBeDefined();
      expect(auth.options.session.expiresIn).toBe(60 * 15); // 15 minutes in seconds
    });

    it('should configure session update age to 1 minute', () => {
      expect(auth.options.session.updateAge).toBe(60 * 1); // 1 minute in seconds
    });
  });

  describe('Advanced Configuration', () => {
    it('should set cookie prefix', () => {
      expect(auth.options.advanced).toBeDefined();
      expect(auth.options.advanced.cookiePrefix).toBe('better-auth');
    });

    it('should enable cross-subdomain cookies', () => {
      expect(auth.options.advanced.crossSubDomainCookies).toBeDefined();
      expect(auth.options.advanced.crossSubDomainCookies.enabled).toBe(true);
    });
  });

  describe('JWT Plugin Configuration', () => {
    it('should include JWT plugin', () => {
      expect(auth.options.plugins).toBeDefined();
      expect(Array.isArray(auth.options.plugins)).toBe(true);
      expect(auth.options.plugins.length).toBeGreaterThan(0);
    });

    it('should configure JWT with RS256 algorithm', () => {
      // Find JWT plugin in plugins array
      const jwtPlugin = auth.options.plugins.find((p: any) => p.id === 'jwt');
      expect(jwtPlugin).toBeDefined();
      expect(jwtPlugin.options.jwks.keyPairConfig.alg).toBe('RS256');
      expect(jwtPlugin.options.jwks.keyPairConfig.modulusLength).toBe(2048);
    });

    it('should configure JWT expiration to 15 minutes', () => {
      const jwtPlugin = auth.options.plugins.find((p: any) => p.id === 'jwt');
      expect(jwtPlugin).toBeDefined();
      expect(jwtPlugin.options.jwt.expirationTime).toBe('15m');
    });

    it('should set JWT issuer and audience to base URL', () => {
      const jwtPlugin = auth.options.plugins.find((p: any) => p.id === 'jwt');
      expect(jwtPlugin).toBeDefined();
      expect(jwtPlugin.options.jwt.issuer).toBe('http://localhost:3000');
      expect(jwtPlugin.options.jwt.audience).toBe('http://localhost:3000');
    });

    it('should define custom JWT payload with user background', () => {
      const jwtPlugin = auth.options.plugins.find((p: any) => p.id === 'jwt');
      expect(jwtPlugin).toBeDefined();
      expect(jwtPlugin.options.jwt.definePayload).toBeDefined();
      expect(typeof jwtPlugin.options.jwt.definePayload).toBe('function');

      // Test payload generation
      const mockUser = {
        id: 'user-123',
        email: 'test@example.com',
        name: 'Test User',
        programming_experience: 'intermediate',
        ros2_familiarity: 'basic',
        hardware_access: 'simulation',
      };

      const payload = jwtPlugin.options.jwt.definePayload({ user: mockUser });

      expect(payload).toEqual({
        id: 'user-123',
        email: 'test@example.com',
        name: 'Test User',
        'https://yourdomain.com/claims': {
          programming_experience: 'intermediate',
          ros2_familiarity: 'basic',
          hardware_access: 'simulation',
        },
      });
    });

    it('should include standard JWT claims in payload', () => {
      const jwtPlugin = auth.options.plugins.find((p: any) => p.id === 'jwt');
      const mockUser = {
        id: 'user-456',
        email: 'user@test.com',
        name: 'User Name',
        programming_experience: 'beginner',
        ros2_familiarity: 'none',
        hardware_access: 'none',
      };

      const payload = jwtPlugin.options.jwt.definePayload({ user: mockUser });

      expect(payload.id).toBe('user-456');
      expect(payload.email).toBe('user@test.com');
      expect(payload.name).toBe('User Name');
    });

    it('should use custom claims namespace for user background', () => {
      const jwtPlugin = auth.options.plugins.find((p: any) => p.id === 'jwt');
      const mockUser = {
        id: 'user-789',
        email: 'expert@test.com',
        name: 'Expert User',
        programming_experience: 'advanced',
        ros2_familiarity: 'advanced',
        hardware_access: 'physical',
      };

      const payload = jwtPlugin.options.jwt.definePayload({ user: mockUser });

      expect(payload['https://yourdomain.com/claims']).toBeDefined();
      expect(payload['https://yourdomain.com/claims'].programming_experience).toBe('advanced');
      expect(payload['https://yourdomain.com/claims'].ros2_familiarity).toBe('advanced');
      expect(payload['https://yourdomain.com/claims'].hardware_access).toBe('physical');
    });
  });
});
