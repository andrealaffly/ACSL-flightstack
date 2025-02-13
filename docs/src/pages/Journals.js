import React from 'react';
import Layout from '@theme/Layout';
import styles from './styles.module.css'; // CSS Styling

// Define here the publications
export const publications = [
  {
    id: '2025-pub1',
    authors: 'M. Gramuglia, G. M. Kumar, G. A. Orlando, and A. L\'Afflitto',
    title: 'An Open-Source Framework to Design, Tune, and Fly Nonlinear Control Systems for Autonomous UAVs',
    link: 'https://lafflitto.com/Documents/LAfflitto_Control_System_AIAA_Conference.pdf',
    conference: 'AIAA SciTech Forum, Orlando, FL, January 2025',
  },
  {
    id: '2024-pub3',
    authors: 'M. Gramuglia, G. M. Kumar, and A. L\'Afflitto',
    title: 'Potential and Challenges for a Certified Application of Model Reference Adaptive Control to Aerial Vehicles',
    link: 'https://lafflitto.com/Documents/LAfflitto_Potential_MRAC_Certification.pdf',
    conference: 'IEEE TechDefense, Naples, Italy, November 2024',
  },
  {
    id: '2024-pub2',
    authors: 'M. Gramuglia, G. M. Kumar, and A. L\'Afflitto',
    title: 'Two-Layer Adaptive Funnel MRAC with Applications to the Control of Multi-Rotor UAVs',
    link: 'https://lafflitto.com/Documents/LAfflitto_RoMoCo_2024.pdf',
    conference: 'IEEE RoMoCo, Poznań, Poland, July 2024',
  },
  {
    id: '2024-pub1',
    authors: 'M. Gramuglia, G. M. Kumar, and A. L\'Afflitto',
    title: 'A Hybrid Model Reference Adaptive Control System for Multi-Rotor Unmanned Aerial Vehicles',
    link: 'https://lafflitto.com/Documents/LAfflitto_X8_Hybrid_MRAC_Control_AIAA_Conference.pdf',
    conference: 'AIAA SciTech Forum, Orlando, FL, January 2024',
  },
  {
    id: '2022-pub1',
    authors: 'J. A. Marshall, G. I. Carter, and A. L\'Afflitto',
    title: 'Model Reference Adaptive Control for Prescribed Performance and Longitudinal Control of a Tail-Sitter UAV',
    link: 'https://lafflitto.com/Documents/LAfflitto_MRAC_PP_Quadbiplane.pdf',
    conference: 'AIAA SciTech Forum, San Diego, CA, January 2022',
  },
];

function References() {
  return (
    <Layout title="References and Publications">
      <div className={styles.container}>
        <h1 className={styles.header}>References and Publications</h1>

        <div className={styles.card}>
          <section>
            <h2>📑 Conference Papers and Journals</h2>
            <ol>
              {publications.map((pub, index) => (
                <li key={pub.id} id={pub.id}>
                  <strong>{pub.authors}</strong>
                  <br />
                  <em>{pub.title}</em>,{' '}
                  <a href={pub.link} target="_blank" rel="noopener noreferrer">
                    [PAPER]
                  </a>
                  <br />
                  <span>{pub.conference}</span>
                </li>
              ))}
            </ol>
          </section>
        </div>
      </div>
    </Layout>
  );
}

export default References;
