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
  // Manual sidebar structure for the Physical AI & Humanoid Robotics book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Preface',
      items: ['preface'],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1-ros-fundamentals/introduction-to-ros2',
        'module-1-ros-fundamentals/nodes-topics-services',
        'module-1-ros-fundamentals/rclpy-basics',
        'module-1-ros-fundamentals/urdf-for-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/gazebo-simulation',
        'module-2-digital-twin/physics-modeling',
        'module-2-digital-twin/sensor-simulation',
        'module-2-digital-twin/unity-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-ai-robot-brain/isaac-sdk-overview',
        'module-3-ai-robot-brain/perception-pipelines',
        'module-3-ai-robot-brain/vslam-navigation',
        'module-3-ai-robot-brain/nav2-path-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vision-language-action/whisper-integration',
        'module-4-vision-language-action/llm-cognitive-planning',
        'module-4-vision-language-action/voice-to-action',
        'module-4-vision-language-action/openai-whisper-examples',
        'module-4-vision-language-action/llm-cognitive-planning-examples',
        'module-4-vision-language-action/exercises-cognitive-planning',
        'module-4-vision-language-action/assessment-questions',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone-project/autonomous-humanoid',
        'capstone-project/capstone-architecture',
        'capstone-project/implementation-guide',
        'capstone-project/integration-examples',
        'capstone-project/troubleshooting-guide',
        'capstone-project/exercises-system-integration',
        'capstone-project/assessment-capstone',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'interactive-playground',
        'accessibility-features',
        'search-optimization',
        'robotics-glossary',
        'references-further-reading',
        'content-versioning',
        'contributing',
        'reader-feedback',
        'performance-optimization',
        'responsive-design',
        'content-analytics',
        'final-review-editing',
      ],
    },
  ],
};

export default sidebars;
