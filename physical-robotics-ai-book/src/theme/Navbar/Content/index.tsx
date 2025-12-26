/**
 * Swizzled Navbar Content Component
 *
 * Wraps the original Docusaurus Navbar Content and adds custom authentication UI.
 */

import React from 'react';
import Content from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type {WrapperProps} from '@docusaurus/types';
import NavbarContent from '../../../components/NavbarContent/NavbarContent';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  return (
    <>
      <Content {...props} />
      <NavbarContent />
    </>
  );
}
