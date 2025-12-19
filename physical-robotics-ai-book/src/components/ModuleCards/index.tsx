import React, { JSX, useState } from 'react';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

interface ModuleCard {
  id: string;
  icon: string;
  emojiFallback: string;
  title: string;
  description: string;
  link: string;
}

const modules: ModuleCard[] = [
  {
    id: 'physical-ai-intro',
    icon: '/img/module-icons/module-1.svg',
    emojiFallback: '🤖',
    title: 'Physical AI Fundamentals',
    description: 'Introduction to physical AI, humanoid robotics, and foundational concepts',
    link: '/docs/module-01-physical-ai-intro/',
  },
  {
    id: 'ros2-mastery',
    icon: '/img/module-icons/module-2.svg',
    emojiFallback: '⚙️',
    title: 'ROS 2 Mastery',
    description: 'Master Robot Operating System 2 for professional robotics development',
    link: '/docs/module-02-ros2-mastery/',
  },
  {
    id: 'gazebo-simulation',
    icon: '/img/module-icons/module-3.svg',
    emojiFallback: '🎮',
    title: 'Gazebo Simulation',
    description: 'Learn robotics simulation with Gazebo, sensors, and environment building',
    link: '/docs/module-03/intro',
  },
  {
    id: 'isaac-platform',
    icon: '/img/module-icons/module-4.svg',
    emojiFallback: '🧠',
    title: 'NVIDIA Isaac Platform',
    description: 'Advanced simulation and AI with Isaac Sim, perception, and navigation',
    link: '/docs/module-04/intro',
  },
];

function ModuleCardItem({module}: {module: ModuleCard}): JSX.Element {
  const [imgError, setImgError] = useState(false);
  
  return (
    <Link to={module.link} className={styles.card}>
      <div className={styles.iconWrapper}>
        {imgError ? (
          <span className={styles.emojiFallback} role="img" aria-label={module.title}>
            {module.emojiFallback}
          </span>
        ) : (
          <img 
            src={module.icon} 
            alt={module.title}
            className={styles.icon}
            onError={() => setImgError(true)}
          />
        )}
      </div>
      <h3 className={styles.title}>{module.title}</h3>
      <p className={styles.description}>{module.description}</p>
    </Link>
  );
}

export default function ModuleCards(): JSX.Element {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <h2 className="text-3xl md:text-4xl font-bold text-center mb-12">
          Explore Our Modules
        </h2>
        <div className={styles.cardsGrid}>
          {modules.map((module) => (
            <ModuleCardItem key={module.id} module={module} />
          ))}
        </div>
      </div>
    </section>
  );
}
