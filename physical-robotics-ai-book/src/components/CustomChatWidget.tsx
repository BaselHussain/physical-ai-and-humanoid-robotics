import React, { useState, useRef, useEffect } from 'react';

interface Message {
  role: 'user' | 'assistant';
  content: string;
}

// Simple markdown to HTML converter
function renderMarkdown(text: string): { __html: string } {
  if (!text) return { __html: '' };

  // Convert headers (### Header -> <h3>Header</h3>)
  let html = text
    .replace(/^### (.*$)/gm, '<h3>$1</h3>')
    .replace(/^## (.*$)/gm, '<h2>$1</h2>')
    .replace(/^# (.*$)/gm, '<h1>$1</h1>')

  // Convert bold (**text** -> <strong>text</strong>)
  html = html.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');

  // Convert italic (*text* -> <em>text</em>)
  html = html.replace(/\*(.*?)\*/g, '<em>$1</em>');

  // Convert code blocks (```lang\ncode\n``` -> <pre><code>code</code></pre>)
  html = html.replace(/```[\s\S]*?\n([\s\S]*?)```/g, '<pre><code>$1</code></pre>');

  // Convert inline code (`code` -> <code>code</code>)
  html = html.replace(/`(.*?)`/g, '<code>$1</code>');

  // Convert links ([text](url) -> <a href="url">text</a>)
  html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2" target="_blank" rel="noopener noreferrer">$1</a>');

  // Convert unordered lists (- item -> <ul><li>item</li></ul>)
  const listRegex = /^(-\s.+)$/gm;
  if (listRegex.test(html)) {
    html = html.replace(/(?:^|\n)-\s(.+?)(?=\n|$)/g, '<li>$1</li>');
    html = html.replace(/(<li>.*<\/li>)/s, '<ul>$1</ul>');
  }

  // Convert tables (simplified)
  html = html.replace(/\n\|.*\|\n\|[-|]+\|\n(\|.*\|\n?)+/g, (match) => {
    const lines = match.trim().split('\n');
    let tableHtml = '<table class="markdown-table">';
    lines.forEach((line, index) => {
      if (line.trim() && !line.includes('|:-') && !line.includes(':-|')) { // Skip separator lines
        const cells = line.split('|').filter(cell => cell !== '');
        if (cells.length > 0) {
          tableHtml += '<tr>';
          cells.forEach(cell => {
            if (cell.trim()) {
              tableHtml += `<td>${cell.trim()}</td>`;
            }
          });
          tableHtml += '</tr>';
        }
      }
    });
    tableHtml += '</table>';
    return tableHtml;
  });

  // Convert newlines to <br> and paragraphs
  html = html.replace(/\n\n/g, '</p><p>');
  html = html.replace(/\n/g, '<br>');

  // Wrap in paragraph tags if not already wrapped
  if (!html.startsWith('<') && !html.includes('<p>')) {
    html = `<p>${html}</p>`;
  } else if (html.startsWith('<p>') && !html.endsWith('</p>')) {
    html += '</p>';
  }

  return { __html: html };
}

export function CustomChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const selectedText = selection?.toString().trim();

      if (selectedText && selectedText.length > 3) {
        setInput(`"${selectedText}"\n\n`);
        setIsOpen(true);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  // Initialize session on mount
  useEffect(() => {
    if (isOpen && !sessionId) {
      fetch('http://localhost:8000/api/session', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
      })
        .then(res => res.json())
        .then(data => setSessionId(data.session_id))
        .catch(err => console.error('Failed to create session:', err));
    }
  }, [isOpen, sessionId]);

  // Auto-scroll to bottom
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const sendMessage = async () => {
    if (!input.trim() || !sessionId || isLoading) return;

    const userMessage: Message = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          session_id: sessionId,
          message: input,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      // Check if the response body is available and readable
      if (!response.body) {
        throw new Error('ReadableStream not available');
      }

      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let assistantContent = '';

      try {
        while (true) {
          const { done, value } = await reader.read();

          // If the stream is done, break the loop
          if (done) break;

          const chunk = decoder.decode(value);
          const lines = chunk.split('\n');

          for (const line of lines) {
            if (line.startsWith('data: ')) {
              try {
                const data = JSON.parse(line.slice(6));
                if (data.type === 'token') {
                  assistantContent += data.delta;
                  setMessages(prev => {
                    const newMessages = [...prev];
                    const lastMsg = newMessages[newMessages.length - 1];
                    if (lastMsg?.role === 'assistant') {
                      lastMsg.content = assistantContent;
                    } else {
                      newMessages.push({ role: 'assistant', content: assistantContent });
                    }
                    return newMessages;
                  });
                } else if (data.type === 'complete') {
                  // Stream completed successfully
                  break;
                } else if (data.type === 'error') {
                  // Handle error from backend
                  setMessages(prev => [...prev, {
                    role: 'assistant',
                    content: data.message || 'An error occurred while processing your request.',
                  }]);
                  break;
                }
              } catch (parseError) {
                console.error('Error parsing SSE data:', parseError);
                // Continue to next line if parsing fails
                continue;
              }
            }
          }
        }
      } catch (readError) {
        // Handle errors during stream reading, including "already finished loading"
        console.error('Error reading stream:', readError);
        if (assistantContent) {
          // If we already have some content, show it with a warning
          setMessages(prev => {
            const newMessages = [...prev];
            const lastMsg = newMessages[newMessages.length - 1];
            if (lastMsg?.role === 'assistant') {
              lastMsg.content = assistantContent + '\n\n(Note: Response may be incomplete due to connection issues)';
            } else {
              newMessages.push({
                role: 'assistant',
                content: assistantContent + '\n\n(Note: Response may be incomplete due to connection issues)'
              });
            }
            return newMessages;
          });
        } else {
          // If no content was received, show error message
          setMessages(prev => [...prev, {
            role: 'assistant',
            content: 'Connection interrupted. Please try again.',
          }]);
        }
      } finally {
        // Ensure the reader is released
        reader.releaseLock();
      }
    } catch (error) {
      console.error('Chat error:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) {
    return (
      <button
        onClick={() => setIsOpen(true)}
        style={{
          position: 'fixed',
          bottom: '20px',
          right: '20px',
          width: '60px',
          height: '60px',
          borderRadius: '50%',
          backgroundColor: '#2D8CFF',
          color: 'white',
          border: 'none',
          cursor: 'pointer',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
          fontSize: '24px',
          zIndex: 9999,
        }}
      >
        💬
      </button>
    );
  }

  return (
    <div style={{
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      width: '400px',
      height: '600px',
      backgroundColor: 'white',
      borderRadius: '12px',
      boxShadow: '0 4px 24px rgba(0,0,0,0.15)',
      display: 'flex',
      flexDirection: 'column',
      zIndex: 9999,
    }}>
      <style>
        {`
          .markdown-table {
            border-collapse: collapse;
            width: 100%;
            margin: 8px 0;
          }
          .markdown-table td {
            border: 1px solid #ddd;
            padding: 4px 8px;
            text-align: left;
          }
          .markdown-table tr:nth-child(even) {
            background-color: #f6f8fa;
          }
          h1, h2, h3 {
            margin: 8px 0;
            line-height: 1.3;
          }
          h1 { font-size: 1.5em; }
          h2 { font-size: 1.3em; }
          h3 { font-size: 1.1em; }
          ul {
            margin: 8px 0;
            padding-left: 20px;
          }
          li {
            margin: 4px 0;
          }
          strong {
            font-weight: bold;
          }
          em {
            font-style: italic;
          }
          code {
            background-color: #f0f0f0;
            padding: 2px 4px;
            border-radius: 3px;
            font-family: monospace;
            font-size: 0.9em;
          }
          pre {
            background-color: #f5f5f5;
            padding: 8px;
            border-radius: 4px;
            overflow-x: auto;
            margin: 8px 0;
          }
          pre code {
            background: none;
            padding: 0;
          }
          p {
            margin: 8px 0;
          }
        `}
      </style>
      {/* Header */}
      <div style={{
        padding: '16px',
        borderBottom: '1px solid #e5e7eb',
        display: 'flex',
        justifyContent: 'space-between',
        alignItems: 'center',
        borderTopLeftRadius: '12px',
        borderTopRightRadius: '12px',
        backgroundColor: '#2D8CFF',
        color: 'white',
      }}>
        <h3 style={{ margin: 0, fontSize: '16px', fontWeight: 600 }}>AI Assistant</h3>
        <button
          onClick={() => setIsOpen(false)}
          style={{
            background: 'none',
            border: 'none',
            color: 'white',
            fontSize: '20px',
            cursor: 'pointer',
            padding: 0,
          }}
        >
          ×
        </button>
      </div>

      {/* Messages */}
      <div style={{
        flex: 1,
        overflowY: 'auto',
        padding: '16px',
        display: 'flex',
        flexDirection: 'column',
        gap: '12px',
      }}>
        {messages.length === 0 && (
          <div style={{ color: '#6b7280', fontSize: '14px', textAlign: 'center', marginTop: '20px' }}>
            Ask me anything about Physical AI and Humanoid Robotics!
          </div>
        )}
        {messages.map((msg, idx) => (
          <div
            key={idx}
            style={{
              alignSelf: msg.role === 'user' ? 'flex-end' : 'flex-start',
              maxWidth: '80%',
              padding: '10px 14px',
              borderRadius: '12px',
              backgroundColor: msg.role === 'user' ? '#2D8CFF' : '#f3f4f6',
              color: msg.role === 'user' ? 'white' : '#1f2937',
              fontSize: '14px',
              lineHeight: '1.5',
            }}
            dangerouslySetInnerHTML={
              msg.role === 'assistant'
                ? renderMarkdown(msg.content)
                : { __html: msg.content }
            }
          />
        ))}
        {isLoading && (
          <div style={{
            alignSelf: 'flex-start',
            padding: '10px 14px',
            borderRadius: '12px',
            backgroundColor: '#f3f4f6',
            color: '#6b7280',
            fontSize: '14px',
          }}>
            Thinking...
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <div style={{
        padding: '16px',
        borderTop: '1px solid #e5e7eb',
        display: 'flex',
        gap: '8px',
      }}>
        <input
          type="text"
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
          placeholder="Type your message..."
          disabled={isLoading}
          style={{
            flex: 1,
            padding: '10px 14px',
            border: '1px solid #e5e7eb',
            borderRadius: '8px',
            fontSize: '14px',
            outline: 'none',
          }}
        />
        <button
          onClick={sendMessage}
          disabled={isLoading || !input.trim()}
          style={{
            padding: '10px 20px',
            backgroundColor: '#2D8CFF',
            color: 'white',
            border: 'none',
            borderRadius: '8px',
            cursor: isLoading ? 'not-allowed' : 'pointer',
            fontSize: '14px',
            opacity: isLoading || !input.trim() ? 0.5 : 1,
          }}
        >
          Send
        </button>
      </div>
    </div>
  );
}
