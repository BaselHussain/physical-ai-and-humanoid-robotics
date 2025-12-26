import React, {type ReactNode} from 'react';
import Navbar from '@theme-original/Navbar';
import type NavbarType from '@theme/Navbar';
import type {WrapperProps} from '@docusaurus/types';
import styles from './styles.module.css';
import {useColorMode} from '@docusaurus/theme-common';

type Props = WrapperProps<typeof NavbarType>;

export default function NavbarWrapper(props: Props): ReactNode {
  const {colorMode, setColorMode} = useColorMode();

  const toggleColorMode = () => {
    setColorMode(colorMode === 'dark' ? 'light' : 'dark');
  };

  return (
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
          <div className={styles.authButtons}>
            <button
              className={styles.signInButton}
              aria-label="Sign In"
              onClick={() => console.log('Sign In clicked')}
            >
              Sign In
            </button>
            <button
              className={styles.signUpButton}
              aria-label="Sign Up"
              onClick={() => console.log('Sign Up clicked')}
            >
              Sign Up
            </button>
          </div>

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
          <button
            className={styles.signInButtonMobile}
            aria-label="Sign In"
            onClick={() => console.log('Sign In clicked')}
          >
            Sign In
          </button>
          <button
            className={styles.signUpButtonMobile}
            aria-label="Sign Up"
            onClick={() => console.log('Sign Up clicked')}
          >
            Sign Up
          </button>
        </div>
      </div>

      {/* Original Docusaurus Navbar (hidden, for compatibility) */}
      <div style={{display: 'none'}}>
        <Navbar {...props} />
      </div>
    </div>
  );
}
