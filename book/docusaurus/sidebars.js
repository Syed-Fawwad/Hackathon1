// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/chapter-1.1-architecture',
        'module-1-ros2/chapter-1.2-sensor-integration',
        'module-1-ros2/chapter-1.3-action-service-architecture',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/chapter-2.1-gazebo-simulation',
        'module-2-digital-twin/chapter-2.2-unity-visualization',
        'module-2-digital-twin/chapter-2.3-sim-to-real-transfer',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-ai-brain/chapter-3.1-isaac-sim-integration',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/chapter-4.1-vision-language-integration',
        'module-4-vla/chapter-4.2-action-generation',
        'module-4-vla/chapter-4.3-capstone-integration',
      ],
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid',
      items: [
        'capstone/autonomous-humanoid',
      ],
    },
  ],
};

module.exports = sidebars;