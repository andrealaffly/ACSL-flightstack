import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css'; // Import CSS styles

// Define the feature list with optional videos and descriptions
const FeatureList = [
  {
    title: 'GitHub',
    imgSrc: '/ACSL-flightstack/img/file.jpg', // Correct path for the image
    description: (
      <>
        We encourage users to make edits and contributions through GitHub.
      </>
    ),
  },
  {
    title: 'Checkout our testing',
    videoSrc: 'https://www.youtube.com/embed/Ykjjg21iAm0?start=0', // Video source
    description: (
      <>
      A video of an X8-Copter being tested in the ACSL. Check out the YouTube channel for additional resources.
      </>
    ),
  },
  {
    title: 'Compatible hardware' ,
    imgSrc: '/ACSL-flightstack/img/Copter2.png', // Correct path for the image
    description: (
      <>
        Compatible with PX4 hardware for seamless integration and operation.
      </>
    ),
  },
];

// Component to render each feature
function Feature({ imgSrc, Svg, title, description, videoSrc }) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        {/* Render the video if provided */}
        {videoSrc && (
  <iframe
    width="400"
    height="400"
    src={videoSrc}
    title={title}
    frameBorder="0"
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
    allowFullScreen
    style={{ borderRadius: '12px', overflow: 'hidden', marginTop: '10px' }} // Adjust the value of '20px' as needed
  ></iframe>
)}


        {/* Render the image or SVG if provided */}
        {imgSrc ? (
          <img src={imgSrc} alt={title} className={styles.featureImg} />
        ) : (
          Svg && <Svg className={styles.featureSvg} role="img" />
        )}
      </div>

      <div className="text--center padding-horiz--md">
        {/* Render the title */}
        <Heading as="h3">{title}</Heading>

        {/* Render the description under the title */}
        <div>{description}</div>
      </div>
    </div>
  );
}

// Main component rendering all features
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
