import React, { JSX } from 'react';
import styles from './styles.module.css';

interface Feature {
  title: string;
  icon: string;
  description: string;
}

const features: Feature[] = [
  {
    title: 'RAG Chatbot',
    icon: '💬',
    description: 'Ask questions about any content and get instant, context-aware answers powered by our AI chatbot',
  },
  {
    title: 'Personalized Learning',
    icon: '📈',
    description: 'Track your progress through modules and get tailored recommendations for your learning journey',
  },
];

function FeatureItem({feature, index}: {feature: Feature; index: number}): JSX.Element {
  const isEven = index % 2 === 0;
  
  return (
    <div className={`${styles.featureItem} ${isEven ? styles.featureItemReverse : ''}`}>
      <div className={styles.featureIcon}>
        <span role="img" aria-label={feature.title}>{feature.icon}</span>
      </div>
      <div className={styles.featureContent}>
        <h3 className={styles.featureTitle}>{feature.title}</h3>
        <p className={styles.featureDescription}>{feature.description}</p>
      </div>
    </div>
  );
}

export default function FeatureShowcase(): JSX.Element {
  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <h2 className="text-3xl md:text-4xl font-bold text-center mb-12">
          Enhanced Learning Experience
        </h2>
        <div className={styles.featuresContainer}>
          {features.map((feature, index) => (
            <FeatureItem key={feature.title} feature={feature} index={index} />
          ))}
        </div>
      </div>
    </section>
  );
}
