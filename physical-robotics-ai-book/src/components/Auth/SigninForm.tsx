/**
 * SigninForm Component
 *
 * Login form for existing users.
 * Maps to User Story 2 (Returning User Authentication) from spec.md
 */

import React, { useState } from 'react';
import styles from './Auth.module.css';

interface SigninFormProps {
  onSubmit: (email: string, password: string) => Promise<void>;
  onSwitchToSignup: () => void;
}

export default function SigninForm({ onSubmit, onSwitchToSignup }: SigninFormProps) {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  const [errors, setErrors] = useState<Record<string, string>>({});
  const [isSubmitting, setIsSubmitting] = useState(false);

  // Validation
  const validateEmail = (email: string): boolean => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

  const validateForm = (): boolean => {
    const newErrors: Record<string, string> = {};

    if (!email.trim()) {
      newErrors.email = 'Email is required';
    } else if (!validateEmail(email)) {
      newErrors.email = 'Invalid email format';
    }

    if (!password.trim()) {
      newErrors.password = 'Password is required';
    }

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setIsSubmitting(true);
    setErrors({});

    try {
      await onSubmit(email, password);
    } catch (error: any) {
      // Handle specific error messages from backend
      const errorMessage = error.message || 'Sign in failed. Please try again.';

      if (errorMessage.includes('Invalid') || errorMessage.includes('401')) {
        setErrors({ submit: 'Invalid email or password. Please try again.' });
      } else {
        setErrors({ submit: errorMessage });
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  return (
    <div className={styles.authForm}>
      <h2>Welcome Back</h2>
      <p className={styles.subtitle}>
        Sign in to continue your personalized robotics learning
      </p>

      <form onSubmit={handleSubmit}>
        {/* Email Field */}
        <div className={styles.formGroup}>
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            placeholder="your.email@example.com"
            disabled={isSubmitting}
            className={errors.email ? styles.inputError : ''}
            autoComplete="email"
          />
          {errors.email && <span className={styles.errorText}>{errors.email}</span>}
        </div>

        {/* Password Field */}
        <div className={styles.formGroup}>
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            placeholder="Enter your password"
            disabled={isSubmitting}
            className={errors.password ? styles.inputError : ''}
            autoComplete="current-password"
          />
          {errors.password && <span className={styles.errorText}>{errors.password}</span>}
        </div>

        {/* Submit Error */}
        {errors.submit && (
          <div className={styles.submitError}>{errors.submit}</div>
        )}

        {/* Submit Button */}
        <button
          type="submit"
          className={styles.submitButton}
          disabled={isSubmitting}
        >
          {isSubmitting ? 'Signing In...' : 'Sign In'}
        </button>

        {/* Switch to Sign Up */}
        <div className={styles.switchForm}>
          Don't have an account?{' '}
          <button
            type="button"
            onClick={onSwitchToSignup}
            className={styles.linkButton}
            disabled={isSubmitting}
          >
            Sign Up
          </button>
        </div>
      </form>
    </div>
  );
}
