import React, { useEffect, useState } from 'react';

export function ChatKitWidget() {
  const [isClient, setIsClient] = useState(false);
  const [ChatKitComponents, setChatKitComponents] = useState<any>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    setIsClient(true);
    import('@openai/chatkit-react')
      .then((module) => setChatKitComponents(module))
      .catch((err) => setError(`Failed to load ChatKit: ${err.message}`));
  }, []);

  if (!isClient) return null;

  if (error) {
    return (
      <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 9999, backgroundColor: '#ff4444', color: 'white', padding: '20px', borderRadius: '8px', maxWidth: '400px' }}>
        <strong>ChatKit Error:</strong> {error}
      </div>
    );
  }

  if (!ChatKitComponents) {
    return (
      <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 9999, backgroundColor: '#4CAF50', color: 'white', padding: '20px', borderRadius: '8px' }}>
        Loading ChatKit...
      </div>
    );
  }

  return <ChatKitInner ChatKit={ChatKitComponents.ChatKit} useChatKit={ChatKitComponents.useChatKit} />;
}

function ChatKitInner({ ChatKit, useChatKit }: any) {
  const [error, setError] = useState<string | null>(null);
  const [debugInfo, setDebugInfo] = useState<string>('Initializing...');

  const { control } = useChatKit({
    api: {
      async getClientSecret(existing: string | undefined) {
        try {
          setDebugInfo('Calling backend API...');
          const endpoint = existing ? '/api/chatkit/refresh' : '/api/chatkit/session';
          const body = existing ? JSON.stringify({ token: existing }) : undefined;

          const res = await fetch(`http://localhost:8000${endpoint}`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body,
          });

          if (!res.ok) throw new Error(`HTTP ${res.status}`);
          const secret = (await res.json()).client_secret;
          setDebugInfo('API call successful!');
          return secret;
        } catch (err) {
          const message = err instanceof Error ? err.message : 'Failed to connect';
          setError(`Backend error: ${message}`);
          setDebugInfo(`Error: ${message}`);
          throw err;
        }
      },
    },
    theme: {
      colorScheme: 'light',
      color: { accent: { primary: '#2D8CFF', level: 2 } },
      radius: 'round',
      density: 'compact',
    },
    onError: ({ error }: any) => {
      console.error('ChatKit error:', error);
      setError(error?.message || 'ChatKit failed to initialize');
    },
  });

  if (error) {
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

  return (
    <>
      <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 9999 }}>
        <ChatKit control={control} className="h-[600px] w-[400px]" />
      </div>
      {/* Debug info */}
      <div style={{ position: 'fixed', top: '20px', right: '20px', zIndex: 10000, backgroundColor: '#ffeb3b', color: '#000', padding: '10px', borderRadius: '4px', fontSize: '12px', maxWidth: '300px' }}>
        <strong>Debug:</strong> {debugInfo}
      </div>
    </>
  );
}
