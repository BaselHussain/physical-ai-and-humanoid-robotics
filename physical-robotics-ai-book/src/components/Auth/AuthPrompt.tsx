/**
 * AuthPrompt Component
 *
 * Displayed to guest users (not authenticated) to encourage sign up/sign in.
 * Maps to User Story 3 (Guest User Restriction) from spec.md
 */

import React from 'react';
import styles from './Auth.module.css';

interface AuthPromptProps {
  onSignup: () => void;
  onSignin: () => void;
  message?: string;
}

export default function AuthPrompt({
  onSignup,
  onSignin,
  message = 'Please sign in to use the personalized chat'
}: AuthPromptProps) {
  return (
    <div className={styles.authPrompt}>
      <h3>🔒 Authentication Required</h3>
      <p>{message}</p>

      <div className={styles.authPromptButtons}>
        <button
          onClick={onSignup}
          className={styles.primaryButton}
          aria-label="Sign up for an account"
        >
          Sign Up
        </button>
        <button
          onClick={onSignin}
          className={styles.secondaryButton}
          aria-label="Sign in to existing account"
        >
          Sign In
        </button>
      </div>
    </div>
  );
}
