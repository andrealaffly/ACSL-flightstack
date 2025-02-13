import React from 'react';
import Layout from '@theme/Layout';
import styles from './styles.module.css'; // CSS Styling

// Define here the flightstack publications
export const flightstack_publications = [
  {
    id: '2025-pub1',
    authors: 'M. Gramuglia, G. M. Kumar, G. A. Orlando, and A. L\'Afflitto',
    title: 'An Open-Source Framework to Design, Tune, and Fly Nonlinear Control Systems for Autonomous UAVs',
    link: 'https://lafflitto.com/Documents/LAfflitto_Control_System_AIAA_Conference.pdf',
    conference: 'AIAA SciTech Forum, Orlando, FL, January 2025',
  },
  {
    id: '2024-pub1',
    authors: 'M. Gramuglia, G. M. Kumar, and A. L\'Afflitto',
    title: 'Potential and Challenges for a Certified Application of Model Reference Adaptive Control to Aerial Vehicles',
    link: 'https://lafflitto.com/Documents/LAfflitto_Potential_MRAC_Certification.pdf',
    conference: 'IEEE TechDefense, Naples, Italy, November 2024',
  },
];

// Define here the simulation publications (PyChrono related)
export const simulation_publications = [
  {
    id: '2024-sim3',
    authors: 'M. Gramuglia, G. M. Kumar, and A. L\'Afflitto',
    title: 'Adaptive control for hybrid dynamical systems with user-defined rate of convergence',
    link: 'https://lafflitto.com/Documents/LAfflitto_Two_Layer_Hybrid_MRAC.pdf',
    conference: 'Journal of the Franklin Institute - Vol. 361, no. 9, Apr. 2024, pp. 106854',
  },
  {
    id: '2024-sim2',
    authors: 'M. Gramuglia, G. M. Kumar, and A. L\'Afflitto',
    title: 'Two-Layer Adaptive Funnel MRAC with Applications to the Control of Multi-Rotor UAVs',
    link: 'https://lafflitto.com/Documents/LAfflitto_RoMoCo_2024.pdf',
    conference: 'IEEE RoMoCo, Poznań, Poland, July 2024',
  },
  {
    id: '2024-sim1',
    authors: 'M. Gramuglia, G. M. Kumar, and A. L\'Afflitto',
    title: 'A Hybrid Model Reference Adaptive Control System for Multi-Rotor Unmanned Aerial Vehicles',
    link: 'https://lafflitto.com/Documents/LAfflitto_X8_Hybrid_MRAC_Control_AIAA_Conference.pdf',
    conference: 'AIAA SciTech Forum, Orlando, FL, January 2024',
  },
];

function References() {
  return (
    <Layout title="References and Publications">
      <div className={styles.container}>
        <h1 className={styles.header}>References and Publications</h1>

        <div className={styles.card}>
          <section>
            <h2>📑 Flightstack-related Relevant Journal and Conference Papers</h2>
            <ol className={styles.customList}>
              {flightstack_publications.map((pub, index) => (
                <li key={pub.id} id={pub.id}>
                  <span className={styles.index}>F.{index + 1}</span>  
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

          <section>
            <h2>📑 Simulator-related Relevant Journal and Conference Papers</h2>
            <ol className={styles.customList}>
              {simulation_publications.map((sim, index) => (
                <li key={sim.id} id={sim.id}>
                  <span className={styles.index}>S.{index + 1}</span>  
                  <strong>{sim.authors}</strong>
                  <br />
                  <em>{sim.title}</em>,{' '}
                  <a href={sim.link} target="_blank" rel="noopener noreferrer">
                    [PAPER]
                  </a>
                  <br />
                  <span>{sim.conference}</span>
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