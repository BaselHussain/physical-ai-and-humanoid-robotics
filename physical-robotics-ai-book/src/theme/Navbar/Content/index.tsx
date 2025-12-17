import React from 'react';
import Content from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type {WrapperProps} from '@docusaurus/types';
import { useAuth } from '@site/src/components/Auth';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  const { isAuthenticated, user, signout, isLoading } = useAuth();

  return (
    <>
      <Content {...props} />
      {/* Add auth controls to the right side of navbar */}
      {!isLoading && isAuthenticated && user && (
        <div style={{
          display: 'flex',
          alignItems: 'center',
          gap: '12px',
          marginLeft: '16px',
          paddingLeft: '16px',
          borderLeft: '1px solid var(--ifm-color-emphasis-300)',
        }}>
          <div style={{
            display: 'flex',
            alignItems: 'center',
            gap: '8px',
            fontSize: '14px',
            color: 'var(--ifm-navbar-link-color)',
          }}>
            <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
              <circle cx="12" cy="7" r="4" />
            </svg>
            <span style={{ fontWeight: 500 }}>
              {user.email}
            </span>
          </div>
          <button
            onClick={signout}
            style={{
              backgroundColor: 'var(--ifm-color-danger)',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              padding: '6px 12px',
              fontSize: '14px',
              cursor: 'pointer',
              fontWeight: 500,
              transition: 'opacity 0.2s',
            }}
            onMouseEnter={(e) => {
              e.currentTarget.style.opacity = '0.8';
            }}
            onMouseLeave={(e) => {
              e.currentTarget.style.opacity = '1';
            }}
            title="Sign out"
          >
            Sign Out
          </button>
        </div>
      )}
    </>
  );
}
