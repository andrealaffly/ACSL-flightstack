import React from 'react';
import Layout from '@theme/Layout';
import styles from './styles.module.css'; // Optional CSS styling
import Link from '@docusaurus/Link';

export default function UAVDescription({ imgWidth = '80%', imgHeight = 'auto' }) {
  // Define image size settings here using props
  const imageSize = {
    width: '50%',
    height: '10%',
  };
  const imageSize_two = {
    width: '85%',
    height: '10%',
  };

  return (
    <Layout title="UAVs Description">
      <div className={styles.container}>
        <h1 className={styles.header} style={{ textAlign: 'left', marginLeft: '0' }}>
          UAVs Description
        </h1>

        {/* Updated Description Section with Isolated Links */}
        <div className={styles.description}>
          <p>Our ecosystem comprises:</p>
          <ul>
            <li>
              A flight stack for multi-rotor UAVs{' '}
              <a
                href="https://github.com/andrealaffly/ACSL-flightstack?tab=readme-ov-file"
                style={{ textDecoration: 'underline' }}
              >
                [GitHub]
              </a>
            </li>
            <li>
              A flight stack for fixed-wing UAVs{' '}
              <a
                href="https://github.com/andrealaffly/ACSL-flightstack-winged?tab=readme-ov-file"
                style={{ textDecoration: 'underline' }}
              >
                [GitHub]
              </a>
            </li>
            <li>
              A suite of Matlab® codes to plot logged flight data{' '}
              <a
                href="https://github.com/andrealaffly/ACSL-flightstack-accessories"
                style={{ textDecoration: 'underline' }}
              >
                [GitHub]
              </a>
            </li>
            <li>
              Doxygen documentation of{' '}
              {/* <Link to="pathname:///ACSL-flightstack/documentation/html/index_ONE.html" style={{ textDecoration: 'none' }}> */}
              <Link to="pathname:///documentation/html/index_ONE.html" style={{ textDecoration: 'underline', fontWeight: 'bold' }}>
                multi-rotor
              </Link>{' '}
              UAV flight stack
            </li>
            <li>
              Doxygen documentation of{' '}
              {/* <Link to="pathname:///ACSL-flightstack/documentation/html/index_ONE.html" style={{ textDecoration: 'none' }}> */}
              <Link to="pathname:///documentation/html/index_ONE.html" style={{ textDecoration: 'underline', fontWeight: 'bold' }}>
                winged
              </Link>{' '}
              UAV flight stack
            </li>
            <li>
              A PyChrono-based high-fidelity simulator for UAVs{' '}
              <a
                href="https://github.com/andrealaffly/PyChrono_Wrapper"
                style={{ textDecoration: 'underline' }}
              >
                [GitHub]
              </a>
            </li>
          </ul>
        </div>

        <div className={styles.hardwareGrid}>
          {/* Multi-Rotor UAV Flight Stack */}
          <div className={styles.card}>
            <h2>Multi-Rotor UAV Flight Stack</h2>
            <div
              className={styles.imageWrapper}
              style={{ display: 'flex', justifyContent: 'center' }}
            >
              <img
                src={require('@site/static/img/M_drone.png').default}
                alt="Multi-Rotor Flight Stack"
                className={styles.image}
                style={imageSize} // Apply the image size
              />
            </div>
            <div style={{ marginTop: '20px' }}>
              <p>
                The Advanced Control Systems Lab (ACSL) Flight Stack is a PX4-compatible offboard
                flight stack designed for <strong>multi-rotor</strong> UAVs. This open-source,
                freeware software serves as a shared platform for the UAV control community, enabling
                easier comparison and evaluation of research outcomes.
              </p>
              <hr></hr>
        
              <p>
                This flight stack is tailored for autonomous UAVs featuring collinear propellers,
                such as quadcopters, X8-copters, and hexacopters. It currently supports both
                quadcopters and X8-copters. With adjustments, the software can be adapted to other
                UAV configurations that utilize similar propeller arrangements.
              </p>
            </div>
          </div>

          {/* Winged UAV Flight Stack */}
          <div className={styles.card}>
            <h2>Winged UAV Flight Stack</h2>
            <div
              className={styles.imageWrapper}
              style={{ display: 'flex', justifyContent: 'center' }}
            >
              <img
                src={require('@site/static/img/Copter2.jpg').default}
                alt="Winged Copter"
                className={styles.image}
                style={imageSize_two} // Apply the image size
              />
            </div>
            <div style={{ marginTop: '26px' }}>
              <p>
                The Advanced Control Systems Lab (ACSL) Flight Stack - Winged is a PX4-compatible
                offboard software designed for multi-rotor <strong>winged</strong> UAVs. This
                software aims to provide the UAV control community with an open-source, freeware
                solution that acts as a common platform, enabling easier comparison of research
                outcomes.
              </p>
              <hr></hr>
              <p>
                At the moment, this code is designed for quad-biplanes (QRBPs), which are quadcopters
                with two parallel wings. This architecture currently supports up to 8 motors and, in
                the future, will be improved to support additional classes of fixed-wing aircraft.
              </p>
            </div>
          </div>
        </div>
      </div>
      <div style={{ textAlign: 'left', fontSize: '30px', marginBottom: '26px', marginLeft: '26px' }}>
        <strong>
          All community members are encouraged to use these flight stacks and recommend improvements
          through GitHub.
        </strong>
      </div>
    </Layout>
  );
}
