/**
 * Swizzled Navbar Content Component
 *
 * Wraps the original Docusaurus Navbar Content and adds custom authentication UI.
 */

import React, { useState } from 'react';
import Content from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type {WrapperProps} from '@docusaurus/types';
import { useAuth } from '../../../components/Auth';
import SignupForm from '../../../components/Auth/SignupForm';
import SigninForm from '../../../components/Auth/SigninForm';
import styles from './NavbarContent.module.css';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  const { user, isAuthenticated, signout, signup, signin } = useAuth();
  const [authView, setAuthView] = useState<'none' | 'signup' | 'signin'>('none');

  return (
    <>
      <Content {...props} />

      {/* Show auth buttons when not authenticated */}
      {!isAuthenticated && (
        <div className={styles.authButtons}>
          <button
            className={styles.signinButton}
            onClick={() => setAuthView('signin')}
            aria-label="Sign in"
          >
            Sign In
          </button>
          <button
            className={styles.signupButton}
            onClick={() => setAuthView('signup')}
            aria-label="Sign up"
          >
            Sign Up
          </button>
        </div>
      )}

      {/* Show user info when authenticated */}
      {isAuthenticated && user && (
        <div className={styles.authInfo}>
          <span className={styles.userName}>{user.name}</span>
          <button
            className={styles.signoutButton}
            onClick={() => signout()}
            aria-label="Sign out"
          >
            Sign Out
          </button>
        </div>
      )}

      {/* Auth modals */}
      {authView === 'signup' && (
        <div className={styles.modalOverlay} onClick={() => setAuthView('none')}>
          <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            <button
              className={styles.modalClose}
              onClick={() => setAuthView('none')}
              aria-label="Close"
            >
              ×
            </button>
            <SignupForm
              onSubmit={async (email, password, name, background) => {
                await signup(email, password, name, background);
                setAuthView('none');
              }}
              onSwitchToSignin={() => setAuthView('signin')}
            />
          </div>
        </div>
      )}

      {authView === 'signin' && (
        <div className={styles.modalOverlay} onClick={() => setAuthView('none')}>
          <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            <button
              className={styles.modalClose}
              onClick={() => setAuthView('none')}
              aria-label="Close"
            >
              ×
            </button>
            <SigninForm
              onSubmit={async (email, password) => {
                await signin(email, password);
                setAuthView('none');
              }}
              onSwitchToSignup={() => setAuthView('signup')}
            />
          </div>
        </div>
      )}
    </>
  );
}
