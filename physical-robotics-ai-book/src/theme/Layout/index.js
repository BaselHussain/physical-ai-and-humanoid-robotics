import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import CustomChatWidgetWidget from '@site/src/components/CustomChatWidgetWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props}>
        {props.children}
        <CustomChatWidgetWidget />
      </OriginalLayout>
    </>
  );
}