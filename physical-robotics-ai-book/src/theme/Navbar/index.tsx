import React, {type ReactNode, useState} from 'react';
import Navbar from '@theme-original/Navbar';
import type NavbarType from '@theme/Navbar';
import type {WrapperProps} from '@docusaurus/types';
import styles from './styles.module.css';
import modalStyles from './Content/NavbarContent.module.css';
import {useColorMode} from '@docusaurus/theme-common';
import { useAuth } from '../../components/Auth';
import SignupForm from '../../components/Auth/SignupForm';
import SigninForm from '../../components/Auth/SigninForm';

type Props = WrapperProps<typeof NavbarType>;

export default function NavbarWrapper(props: Props): ReactNode {
  const {colorMode, setColorMode} = useColorMode();
  const { user, isAuthenticated, signout, signup, signin } = useAuth();
  const [authView, setAuthView] = useState<'none' | 'signup' | 'signin'>('none');

  const toggleColorMode = () => {
    setColorMode(colorMode === 'dark' ? 'light' : 'dark');
  };

  const closeMobileMenu = () => {
    const mobileMenu = document.getElementById('mobile-menu');
    if (mobileMenu) {
      mobileMenu.classList.remove(styles.mobileMenuOpen);
    }
  };

  const handleSignOut = async () => {
    await signout();
    closeMobileMenu();
  };

  return (
    <>
      {/* Hidden original navbar for Docusaurus compatibility */}
      <div style={{display: 'none'}}>
        <Navbar {...props} />
      </div>

      {/* Your beautiful custom navbar */}
      <div className={styles.navbarContainer}>
        <nav className={styles.customNavbar}>
          {/* Left Side: Book Title with Gradient */}
          <div className={styles.navbarLeft}>
            <a href="/" className={styles.bookTitle} aria-label="Physical AI & Humanoid Robotics Home">
              Physical AI & Humanoid Robotics
            </a>
          </div>

          {/* Right Side: Auth Buttons + Theme Toggle */}
          <div className={styles.navbarRight}>
            {/* Show auth buttons when not authenticated */}
            {!isAuthenticated && (
              <div className={styles.authButtons}>
                <button
                  className={styles.signInButton}
                  aria-label="Sign In"
                  onClick={() => setAuthView('signin')}
                >
                  Sign In
                </button>
                <button
                  className={styles.signUpButton}
                  aria-label="Sign Up"
                  onClick={() => setAuthView('signup')}
                >
                  Sign Up
                </button>
              </div>
            )}

            {/* Show user info when authenticated */}
            {isAuthenticated && user && (
              <div className={styles.authButtons}>
                <span className={styles.userName}>{user.name}</span>
                <button
                  className={styles.signOutButton}
                  aria-label="Sign Out"
                  onClick={handleSignOut}
                >
                  Sign Out
                </button>
              </div>
            )}

            {/* Theme Toggle Button */}
            <button
              className={styles.themeToggle}
              onClick={toggleColorMode}
              aria-label={`Switch to ${colorMode === 'dark' ? 'light' : 'dark'} mode`}
              title={`Switch to ${colorMode === 'dark' ? 'light' : 'dark'} mode`}
            >
              {colorMode === 'dark' ? '☀️' : '🌙'}
            </button>
          </div>

          {/* Mobile Hamburger Menu - Hidden by default, shown on mobile */}
          <div className={styles.mobileMenuToggle}>
            <button
              className={styles.hamburgerButton}
              aria-label="Toggle mobile menu"
              onClick={() => {
                const mobileMenu = document.getElementById('mobile-menu');
                if (mobileMenu) {
                  mobileMenu.classList.toggle(styles.mobileMenuOpen);
                }
              }}
            >
              ☰
            </button>
          </div>
        </nav>

        {/* Mobile Menu Overlay */}
        <div id="mobile-menu" className={styles.mobileMenu}>
          <div className={styles.mobileMenuContent}>
            {!isAuthenticated && (
              <>
                <button
                  className={styles.signInButtonMobile}
                  aria-label="Sign In"
                  onClick={() => {
                    setAuthView('signin');
                    closeMobileMenu();
                  }}
                >
                  Sign In
                </button>
                <button
                  className={styles.signUpButtonMobile}
                  aria-label="Sign Up"
                  onClick={() => {
                    setAuthView('signup');
                    closeMobileMenu();
                  }}
                >
                  Sign Up
                </button>
              </>
            )}
            {isAuthenticated && user && (
              <>
                <div className={styles.userNameMobile}>{user.name}</div>
                <button
                  className={styles.signOutButtonMobile}
                  aria-label="Sign Out"
                  onClick={handleSignOut}
                >
                  Sign Out
                </button>
              </>
            )}
          </div>
        </div>

        {/* Auth modals with overlay */}
        {authView === 'signup' && (
          <div className={modalStyles.modalOverlay} onClick={() => setAuthView('none')}>
            <div className={modalStyles.modalContent} onClick={(e) => e.stopPropagation()}>
              <button
                className={modalStyles.modalClose}
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
          <div className={modalStyles.modalOverlay} onClick={() => setAuthView('none')}>
            <div className={modalStyles.modalContent} onClick={(e) => e.stopPropagation()}>
              <button
                className={modalStyles.modalClose}
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
      </div>
    </>
  );
}
