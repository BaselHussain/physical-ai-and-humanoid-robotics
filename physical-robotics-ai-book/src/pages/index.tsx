import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageHero from '@site/src/components/HomepageHero';
import ModuleCards from '@site/src/components/ModuleCards';
import FeatureShowcase from '@site/src/components/FeatureShowcase';

import styles from './index.module.css';

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Master ROS 2, Isaac Sim, and Vision-Language-Action Models for Next-Gen Robotics">
      <main>
        <HomepageHero />
        <ModuleCards />
        <FeatureShowcase />
      </main>
    </Layout>
  );
}
