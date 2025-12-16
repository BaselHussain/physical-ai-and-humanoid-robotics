import React, { useEffect, useState, useCallback } from 'react';

interface TextSelectionMenuProps {
  onAskFromAI: (selectedText: string) => void;
}

export function TextSelectionMenu({ onAskFromAI }: TextSelectionMenuProps) {
  const [menuVisible, setMenuVisible] = useState(false);
  const [menuPosition, setMenuPosition] = useState({ x: 0, y: 0 });
  const [selectedText, setSelectedText] = useState('');

  const handleTextSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection?.toString().trim() || '';

    // Only show menu if text is longer than 3 characters
    if (text.length > 3) {
      const range = selection?.getRangeAt(0);
      const rect = range?.getBoundingClientRect();

      if (rect) {
        // Position menu near the selected text
        setMenuPosition({
          x: rect.left + (rect.width / 2),
          y: rect.bottom + 10, // 10px below selection
        });
        setSelectedText(text);
        setMenuVisible(true);
      }
    } else {
      // Hide menu if selection is too short
      setMenuVisible(false);
    }
  }, []);

  const handleClickOutside = useCallback((e: MouseEvent) => {
    const target = e.target as HTMLElement;

    // Don't hide if clicking on the menu itself
    if (target.closest('.text-selection-menu')) {
      return;
    }

    // Hide menu on any click outside
    setMenuVisible(false);
  }, []);

  const handleCopy = useCallback(async () => {
    try {
      await navigator.clipboard.writeText(selectedText);
      console.log('Text copied to clipboard');
      setMenuVisible(false);
    } catch (error) {
      console.error('Failed to copy text:', error);
      // Fallback: use document.execCommand (deprecated but still works)
      const textArea = document.createElement('textarea');
      textArea.value = selectedText;
      document.body.appendChild(textArea);
      textArea.select();
      document.execCommand('copy');
      document.body.removeChild(textArea);
      setMenuVisible(false);
    }
  }, [selectedText]);

  const handleAskFromAI = useCallback(() => {
    onAskFromAI(selectedText);
    setMenuVisible(false);

    // Clear selection
    window.getSelection()?.removeAllRanges();
  }, [selectedText, onAskFromAI]);

  useEffect(() => {
    // Listen for text selection events
    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('touchend', handleTextSelection);

    // Listen for clicks outside to hide menu
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('touchend', handleTextSelection);
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [handleTextSelection, handleClickOutside]);

  if (!menuVisible) {
    return null;
  }

  return (
    <div
      className="text-selection-menu"
      style={{
        position: 'fixed',
        left: `${menuPosition.x}px`,
        top: `${menuPosition.y}px`,
        transform: 'translateX(-50%)',
        backgroundColor: 'white',
        border: '1px solid #ccc',
        borderRadius: '8px',
        boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
        padding: '8px',
        zIndex: 10000,
        display: 'flex',
        gap: '4px',
      }}
    >
      <button
        onClick={handleAskFromAI}
        style={{
          padding: '8px 16px',
          backgroundColor: '#2D8CFF',
          color: 'white',
          border: 'none',
          borderRadius: '6px',
          cursor: 'pointer',
          fontSize: '14px',
          fontWeight: '500',
          display: 'flex',
          alignItems: 'center',
          gap: '6px',
          transition: 'background-color 0.2s',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.backgroundColor = '#1a7ae8';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.backgroundColor = '#2D8CFF';
        }}
      >
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
        Ask from AI
      </button>

      <button
        onClick={handleCopy}
        style={{
          padding: '8px 16px',
          backgroundColor: '#f5f5f5',
          color: '#333',
          border: '1px solid #ddd',
          borderRadius: '6px',
          cursor: 'pointer',
          fontSize: '14px',
          fontWeight: '500',
          display: 'flex',
          alignItems: 'center',
          gap: '6px',
          transition: 'background-color 0.2s',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.backgroundColor = '#e8e8e8';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.backgroundColor = '#f5f5f5';
        }}
      >
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <rect x="9" y="9" width="13" height="13" rx="2" ry="2" />
          <path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1" />
        </svg>
        Copy
      </button>
    </div>
  );
}
