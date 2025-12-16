
import React, { useEffect, useState } from 'react';

interface ChatKitWidgetProps {
  prePopulatedText?: string;
  onClearPrePopulatedText?: () => void;
}

export function ChatKitWidget({ prePopulatedText, onClearPrePopulatedText }: ChatKitWidgetProps = {}) {
  const [isClient, setIsClient] = useState(false);
  const [ChatKitComponents, setChatKitComponents] = useState<any>(null);
  const [error, setError] = useState<string | null>(null);
  const [isMinimized, setIsMinimized] = useState(true); // Start minimized

  useEffect(() => {
    console.log('ChatKitWidget mounted');
    setIsClient(true);

    console.log('Attempting to import @openai/chatkit-react...');
    import('@openai/chatkit-react')
      .then((module) => {
        console.log('ChatKit module loaded successfully:', module);
        setChatKitComponents(module);
      })
      .catch((err) => {
        console.error('Failed to load ChatKit:', err);
        setError(`Failed to load ChatKit: ${err.message}`);
      });
  }, []);

  // Handle pre-populated text from TextSelectionMenu
  useEffect(() => {
    if (prePopulatedText) {
      console.log('Pre-populated text received, opening chat:', prePopulatedText);
      setIsMinimized(false); // Open the chat widget
    }
  }, [prePopulatedText]);

  console.log('ChatKitWidget render - isClient:', isClient, 'error:', error, 'components:', !!ChatKitComponents);

  if (!isClient) {
    console.log('Not client yet, returning null');
    return null;
  }

  if (error) {
    console.log('Showing error state');
    return (
      <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 9999, backgroundColor: '#ff4444', color: 'white', padding: '20px', borderRadius: '8px', maxWidth: '400px' }}>
        <strong>ChatKit Error:</strong> {error}
      </div>
    );
  }

  if (!ChatKitComponents) {
    console.log('Showing loading state');
    return (
      <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 9999, backgroundColor: '#4CAF50', color: 'white', padding: '20px', borderRadius: '8px' }}>
        Loading ChatKit...
      </div>
    );
  }

  console.log('Rendering ChatKitInner with isMinimized:', isMinimized);
  return (
    <ChatKitInner
      ChatKit={ChatKitComponents.ChatKit}
      useChatKit={ChatKitComponents.useChatKit}
      isMinimized={isMinimized}
      onToggleMinimize={() => setIsMinimized(!isMinimized)}
      prePopulatedText={prePopulatedText}
      onClearPrePopulatedText={onClearPrePopulatedText}
    />
  );
}

interface ChatKitInnerProps {
  ChatKit: any;
  useChatKit: any;
  isMinimized: boolean;
  onToggleMinimize: () => void;
  prePopulatedText?: string;
  onClearPrePopulatedText?: () => void;
}

function ChatKitInner({
  ChatKit,
  useChatKit,
  isMinimized,
  onToggleMinimize,
  prePopulatedText,
  onClearPrePopulatedText
}: ChatKitInnerProps) {
  const [error, setError] = useState<string | null>(null);
  const [isReady, setIsReady] = useState(false);

  console.log('ChatKitInner rendering, ChatKit:', !!ChatKit, 'useChatKit:', !!useChatKit, 'isMinimized:', isMinimized);

  const { control, setThreadId, setComposerValue } = useChatKit({
    api: {
      url: 'http://localhost:8000/chatkit',
      domainKey: 'localhost',
    },
    theme: {
      colorScheme: 'light',
      color: { accent: { primary: '#2D8CFF', level: 2 } },
      radius: 'round',
      density: 'compact',
    },
    onReady: () => {
      console.log('ChatKit is ready');
      setIsReady(true);
    },
    onThreadChange: ({ threadId }: any) => {
      console.log('Thread changed:', threadId);
      if (threadId) {
        localStorage.setItem('chatkit_thread_id', threadId);
      } else {
        localStorage.removeItem('chatkit_thread_id');
      }
    },
    onError: ({ error }: any) => {
      console.error('ChatKit onError callback:', error);
      setError(error?.message || 'ChatKit failed to initialize');
    },
  });

  // Restore thread from localStorage when ChatKit is ready
  useEffect(() => {
    if (isReady && setThreadId) {
      const savedThreadId = localStorage.getItem('chatkit_thread_id');
      if (savedThreadId) {
        console.log('Restoring thread:', savedThreadId);
        setThreadId(savedThreadId).catch(err => {
          console.error('Failed to restore thread:', err);
        });
      }
    }
  }, [isReady, setThreadId]);

  // Handle pre-populated text from TextSelectionMenu
  useEffect(() => {
    if (prePopulatedText && isReady && setComposerValue) {
      console.log('Setting composer value with pre-populated text:', prePopulatedText);
      setComposerValue({ text: prePopulatedText })
        .then(() => {
          console.log('Composer value set successfully');
          if (onClearPrePopulatedText) {
            onClearPrePopulatedText();
          }
        })
        .catch((err: any) => {
          console.error('Failed to set composer value:', err);
        });
    }
  }, [prePopulatedText, isReady, setComposerValue, onClearPrePopulatedText]);

  console.log('ChatKitInner - control object:', control);
  console.log('ChatKitInner - error state:', error);

  if (error) {
    console.log('Rendering error UI');
    return (
      <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 9999, backgroundColor: 'white', border: '3px solid red', padding: '20px', borderRadius: '8px', maxWidth: '400px', boxShadow: '0 4px 6px rgba(0,0,0,0.1)' }}>
        <h3 style={{ color: 'red', margin: '0 0 10px 0', fontSize: '18px' }}>ChatKit Error</h3>
        <p style={{ margin: 0, fontSize: '14px', color: '#333' }}>{error}</p>
        <p style={{ margin: '10px 0 0 0', fontSize: '12px', color: '#666' }}>
          Make sure backend is running on port 8000
        </p>
      </div>
    );
  }

  console.log('Rendering ChatKit component with control:', control, 'isMinimized:', isMinimized);

  // Render both minimized icon and full chat widget, toggle visibility with CSS
  return (
    <>
      {/* Minimized chat icon */}
      <div
        onClick={onToggleMinimize}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          zIndex: 9999,
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#2D8CFF',
          color: 'white',
          display: isMinimized ? 'flex' : 'none',
          alignItems: 'center',
          justifyContent: 'center',
          cursor: 'pointer',
          boxShadow: '0 4px 12px rgba(45, 140, 255, 0.4)',
          transition: 'all 0.3s ease',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.transform = 'scale(1.1)';
          e.currentTarget.style.boxShadow = '0 6px 16px rgba(45, 140, 255, 0.5)';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'scale(1)';
          e.currentTarget.style.boxShadow = '0 4px 12px rgba(45, 140, 255, 0.4)';
        }}
        title="Open Chat"
      >
        <svg width="30" height="30" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
      </div>

      {/* Full chat widget - kept mounted for instant appearance */}
      <div style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        zIndex: 9999,
        display: isMinimized ? 'none' : 'block',
      }}>
        {/* Close button - positioned to avoid ChatKit's clock icon */}
        <button
          onClick={onToggleMinimize}
          style={{
            position: 'absolute',
            top: '10px',
            left: '10px',
            zIndex: 10000,
            backgroundColor: 'rgba(0, 0, 0, 0.1)',
            border: 'none',
            borderRadius: '50%',
            width: '32px',
            height: '32px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            cursor: 'pointer',
            transition: 'background-color 0.2s',
          }}
          onMouseEnter={(e) => {
            e.currentTarget.style.backgroundColor = 'rgba(0, 0, 0, 0.2)';
          }}
          onMouseLeave={(e) => {
            e.currentTarget.style.backgroundColor = 'rgba(0, 0, 0, 0.1)';
          }}
          title="Minimize Chat"
        >
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M19 9l-7 7-7-7" />
          </svg>
        </button>

        <ChatKit
          control={control}
          style={{ height: '600px', width: '400px' }}
        />
      </div>
    </>
  );
}
