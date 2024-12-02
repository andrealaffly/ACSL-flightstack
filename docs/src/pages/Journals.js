import React from 'react';
import Layout from '@theme/Layout';
import styles from './styles.module.css'; // CSS Styling

export default function References() {
  return (
    <Layout title="References and Publications">
      <div className={styles.container}>
        <h1 className={styles.header}>References and Publications</h1>

        <div className={styles.card}>
          <section>
            <h2>📑 Conference Papers and Journals</h2>
            <ul>
              <li>
                <strong>M. Gramuglia, G. M. Kumar, and A. L'Afflitto</strong><br />
                <em>Two-Layer Adaptive Funnel MRAC with Applications to the Control of Multi-Rotor UAVs</em>,{' '}
                <a
                  href="https://lafflitto.com/Documents/LAfflitto_RoMoCo_2024.pdf"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  [PAPER]
                </a>
                <br />
                <span>IEEE 13th International Workshop on Robot Motion and Control (RoMoCo), Poznań, Poland, July 2024.</span>
              </li>
              <li>
                <strong>M. Gramuglia, G. M. Kumar, and A. L'Afflitto</strong><br />
                <em>A Hybrid Model Reference Adaptive Control System for Multi-Rotor Unmanned Aerial Vehicles</em>,{' '}
                <a
                  href="https://lafflitto.com/Documents/LAfflitto_X8_Hybrid_MRAC_Control_AIAA_Conference.pdf"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  [PAPER]
                </a>
                <br />
                <span>AIAA SciTech Forum, Orlando, FL, January 2024.</span>
              </li>
              <li>
                <strong>J. A. Marshall, G. I. Carter, and A. L'Afflitto</strong><br />
                <em>Model Reference Adaptive Control for Prescribed Performance and Longitudinal Control of a Tail-Sitter UAV</em>,{' '}
                <a
                  href="https://lafflitto.com/Documents/LAfflitto_MRAC_PP_Quadbiplane.pdf"
                  target="_blank"
                  rel="noopener noreferrer"
                >
                  [PAPER]
                </a>
                <br />
                <span>AIAA SciTech Forum, San Diego, CA, January 2022.</span>
              </li>
            </ul>
          </section>
        </div>
      </div>
    </Layout>
  );
}
