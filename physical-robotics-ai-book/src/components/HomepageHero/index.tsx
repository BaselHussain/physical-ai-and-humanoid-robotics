import React, { JSX } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './styles.module.css';

export default function HomepageHero(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroBackground}></div>
      <div className="container relative z-10">
        <h1 className="text-5xl text-center md:text-6xl font-bold text-white mb-6">
          {siteConfig.title}
        </h1>
        <p className="text-xl md:text-2xl text-gray-100 mb-8 max-w-3xl text-center mx-auto">
          {siteConfig.tagline}
        </p>
        <div className="flex gap-4 justify-center">
          <Link
            className={`${styles.ctaButton} bg-gradient-to-r from-blue-600 to-purple-700 text-white px-8 py-4 rounded-lg text-lg font-semibold hover:shadow-xl transition-all`}
            to="/docs/module-01-physical-ai-intro/">
            Start Reading 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}
