import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <h1 className="hero__title">{siteConfig.title}</h1>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                🤖 Start Building Robots
              </Link>
              <Link
                className="button button--outline button--primary button--lg"
                to="/docs/module-1-ros2/chapter-1.1-architecture">
                📚 Read Documentation
              </Link>
            </div>
          </div>
          <div className={styles.heroVisual}>
            <div className={styles.robotIcon}>
              <svg viewBox="0 0 100 100" className={styles.robotSvg}>
                <circle cx="50" cy="35" r="12" fill="white"/>
                <rect x="35" y="47" width="30" height="35" rx="4" fill="white"/>
                <rect x="25" y="30" width="8" height="15" rx="2" fill="var(--robot-blue)"/>
                <rect x="67" y="30" width="8" height="15" rx="2" fill="var(--robot-blue)"/>
                <circle cx="42" cy="33" r="2" fill="var(--robot-orange)"/>
                <circle cx="58" cy="33" r="2" fill="var(--robot-orange)"/>
                <rect x="45" y="38" width="10" height="2" rx="1" fill="var(--robot-green)"/>
              </svg>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Documentation">
      <HomepageHeader />
      <main>
        <section className={styles.featuresSection}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className={styles.featureCard}>
                  <h3>🤖 Advanced AI</h3>
                  <p>
                    Leverage NVIDIA Isaac for perception, navigation, and AI inference
                    capabilities in your humanoid robots.
                  </p>
                </div>
              </div>
              <div className="col col--4">
                <div className={styles.featureCard}>
                  <h3>📡 ROS 2 Integration</h3>
                  <p>
                    Seamless communication framework with node architecture and
                    real-time control systems.
                  </p>
                </div>
              </div>
              <div className="col col--4">
                <div className={styles.featureCard}>
                  <h3>🗣️ Voice Control</h3>
                  <p>
                    Natural language processing with VLA (Vision-Language-Action)
                    systems for intuitive robot control.
                  </p>
                </div>
              </div>
            </div>
          </div>
        </section>

        <HomepageFeatures />

        <section className={styles.ctaSection}>
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2>Ready to Build the Future?</h2>
                <p>
                  Join the revolution in Physical AI and create autonomous humanoid robots
                  that can understand and interact with the world around them.
                </p>
                <Link
                  className="button button--primary button--lg"
                  to="/docs/intro">
                  Start Your Journey
                </Link>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}