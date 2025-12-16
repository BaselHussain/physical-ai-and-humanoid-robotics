import React, {type ReactNode, useState, useCallback} from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type {WrapperProps} from '@docusaurus/types';
import { ChatKitWidget } from '@site/src/components/ChatKitWidget';
import { TextSelectionMenu } from '@site/src/components/TextSelectionMenu';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  const [prePopulatedText, setPrePopulatedText] = useState<string | undefined>(undefined);

  // Handler to receive selected text from TextSelectionMenu
  const handleAskFromAI = useCallback((selectedText: string) => {
    console.log('Layout received selected text:', selectedText);
    setPrePopulatedText(selectedText);
  }, []);

  // Handler to clear pre-populated text after it's been used
  const handleClearPrePopulatedText = useCallback(() => {
    console.log('Layout clearing pre-populated text');
    setPrePopulatedText(undefined);
  }, []);

  return (
    <>
      <Layout {...props} />
      <TextSelectionMenu onAskFromAI={handleAskFromAI} />
      <ChatKitWidget
        prePopulatedText={prePopulatedText}
        onClearPrePopulatedText={handleClearPrePopulatedText}
      />
    </>
  );
}
