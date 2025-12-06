import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 01 - Physical AI Introduction',
      items: [
        'module-01-physical-ai-intro/index',
        'module-01-physical-ai-intro/four-pillars-architecture',
        'module-01-physical-ai-intro/hardware-requirements',
        'module-01-physical-ai-intro/ubuntu-ros2-setup',
        'module-01-physical-ai-intro/isaac-sim-installation',
        'module-01-physical-ai-intro/repository-structure',
        'module-01-physical-ai-intro/verification-testing',
        'module-01-physical-ai-intro/next-steps-references',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 02 - The Robotic Nervous System (ROS 2)',
      items: [
        'module-02-ros2-mastery/index',
        'module-02-ros2-mastery/ros2-architecture',
        'module-02-ros2-mastery/workspace-setup',
        'module-02-ros2-mastery/lab01-talker-listener',
        'module-02-ros2-mastery/lab02-custom-messages',
        'module-02-ros2-mastery/lab03-services',
        'module-02-ros2-mastery/lab04-actions',
        'module-02-ros2-mastery/urdf-fundamentals',
        'module-02-ros2-mastery/lab05-humanoid-urdf',
        'module-02-ros2-mastery/tf2-coordinate-frames',
        'module-02-ros2-mastery/lab06-tf2-broadcasting',
        'module-02-ros2-mastery/lab07-launch-files',
        'module-02-ros2-mastery/lab08-rviz-integration',
        'module-02-ros2-mastery/lifecycle-real-time-concepts',
        'module-02-ros2-mastery/next-steps-references',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 03 - The Digital Twin (Gazebo & Unity)',
      items: [
        'module-03/intro',
        'module-03/gazebo-fundamentals',
        'module-03/sensors-and-humanoid',
        'module-03/apartment-worlds-and-unity',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 04 - The AI-Robot Brain (NVIDIA Isaac™ Platform)',
      items: [
        'module-04/intro',
        'module-04/isaac-sim-basics',
        'module-04/perception-and-vslam',
        'module-04/nav2-and-synthetic-data',
      ],
      collapsed: false,
    }
  ],
};

export default sidebars;
