import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'ROS 2 Integration',
    Svg: require('@site/static/img/robot.svg').default,
    description: (
      <>
        Core communication framework with node architecture and communication patterns
        for robotic systems.
      </>
    ),
  },
  {
    title: 'AI Perception & Control',
    Svg: require('@site/static/img/robot.svg').default,
    description: (
      <>
        NVIDIA Isaac integration for perception, navigation, and AI inference
        capabilities.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action',
    Svg: require('@site/static/img/robot.svg').default,
    description: (
      <>
        Voice command processing with language understanding and action execution
        for autonomous humanoid robots.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}