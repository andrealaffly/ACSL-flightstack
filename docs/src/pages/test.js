import React, { useState } from 'react';
import Layout from '@theme/Layout';
import styles from './styles.module.css'; // Import your custom styles
import { flightstack_publications } from './Journals'; // Import flightstack related publication list
import { simulation_publications } from './Journals'; // Import simulation (PyChrono) related publication list

// Sample data with placeholders for Notes
const data = [
  {
    flightDataGitHubLink: 'https://github.com/andrealaffly/ACSL-flightstack-accessories/tree/main/Flight_Data/2024/06/20240613/PID',
    platform: 'X8-Copter',
    controlTechnique: 'PID',
    stateEst: 'MoCap',
    flightConditions: 'Nominal',
    youtubeVideo: '',
    date: '2024-06-13',
    publicationId: 'N/A',
    notes: 'N/A',
  },
  {
    flightDataGitHubLink: 'https://github.com/andrealaffly/ACSL-flightstack-accessories/tree/main/Flight_Data/2024/07/20240720/MRAC',
    platform: 'X8-Copter',
    controlTechnique: 'MRAC',
    stateEst: 'MoCap',
    flightConditions: 'Nominal',
    youtubeVideo: '',
    date: '2024-07-20',
    publicationId: 'N/A',
    notes: 'N/A',
  },
  {
    flightDataGitHubLink: 'https://github.com/andrealaffly/ACSL-flightstack-accessories/tree/main/Flight_Data/2024/07/20240729/MRAC',
    platform: 'X8-Copter',
    controlTechnique: 'PID & MRAC',
    stateEst: 'MoCap',
    flightConditions: 'Off-Nominal',
    youtubeVideo: '',
    date: '2024-07-29',
    publicationId: '2024-pub1',
    notes: 'Carrying an unknown steady and unsteady payload',
  },
];

// Function to get publication reference dynamically
const getFlightstackPublicationReference = (publicationId) => {
  const index = flightstack_publications.findIndex((pub) => pub.id === publicationId);
  return index !== -1 ? (
    <a href={`/Journals#${publicationId}`} rel="noopener noreferrer">
      [F.{index + 1}]
    </a>
  ) : 'N/A';
};

// Function to get publication reference dynamically
const getSimulationPublicationReference = (publicationId) => {
  const index = simulation_publications.findIndex((pub) => pub.id === publicationId);
  return index !== -1 ? (
    <a href={`/Journals#${publicationId}`} rel="noopener noreferrer">
      [S.{index + 1}]
    </a>
  ) : 'N/A';
};

export default function SearchableTable() {
  const [searchQuery, setSearchQuery] = useState('');

  // Filter data based on search query
  const filteredData = data.filter(
    (item) =>
      item.platform.toLowerCase().includes(searchQuery.toLowerCase()) ||
      item.controlTechnique.toLowerCase().includes(searchQuery.toLowerCase()) ||
      item.stateEst.toLowerCase().includes(searchQuery.toLowerCase()) ||
      item.flightConditions.toLowerCase().includes(searchQuery.toLowerCase()) ||
      item.notes.toLowerCase().includes(searchQuery.toLowerCase())
  );

  return (
    <Layout title="Searchable Table">
      <div className={styles.tableContainer}>
        <h1 className={styles.tableHeader}>Flight tests</h1>

        {/* Search Bar */}
        <input
          type="text"
          placeholder="Search..."
          value={searchQuery}
          onChange={(e) => setSearchQuery(e.target.value)}
          className={styles.searchInput}
        />

        {/* Table */}
        <table className={styles.dataTable}>
          <thead>
            <tr>
              <th>Date</th>
              <th>Control Technique</th>
              <th>State Estimation</th>
              <th>Flight Conditions</th>
              <th>Notes</th>
              <th>Platform</th>
              <th>Flight Data</th>
              <th>Publication</th>
            </tr>
          </thead>
          <tbody>
            {filteredData.length > 0 ? (
              filteredData.map((item, index) => (
                <tr key={index}>
                  <td>{item.date}</td>
                  <td>{item.controlTechnique}</td>
                  <td>{item.stateEst}</td>
                  <td>{item.flightConditions}</td>
                  <td>{item.notes || <div className={styles.notesPlaceholder}><p>Notes coming soon...</p></div>}</td>
                  <td>{item.platform}</td>
                  <td>
                    {item.flightDataGitHubLink ? (
                      <a href={item.flightDataGitHubLink} target="_blank" rel="noopener noreferrer">
                        GitHub
                      </a>
                    ) : (
                      'N/A'
                    )}
                  </td>
                  <td>{getFlightstackPublicationReference(item.publicationId)}</td>
                </tr>
              ))
            ) : (
              <tr>
                <td colSpan="8">No matching records found</td>
              </tr>
            )}
          </tbody>
        </table>
      </div>
    </Layout>
  );
}
