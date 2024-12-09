import React, { useState } from 'react';
import Layout from '@theme/Layout';
import styles from './styles.module.css'; // Import your custom styles

// Sample data with placeholders for Notes
const data = [
  {
    githubLink: 'https://github.com/andrealaffly/ACSL-flightstack-accessories/tree/main/Flight_Data/2024/06/20240613/PID',
    platform: 'X8-Copter',
    controlTechnique: 'PID',
    navSys: 'MoCap',
    flightConditions: 'Nominal',
    youtubeVideo: '',
    date: '2024-06-13',
    publication: 'N/A',
    notes: 'N/A',
  },
  {
    githubLink: 'https://github.com/andrealaffly/ACSL-flightstack-accessories/tree/main/Flight_Data/2024/07/20240720/MRAC',
    platform: 'X8-Copter',
    controlTechnique: 'MRAC',
    navSys: 'MoCap',
    flightConditions: 'Nominal',
    youtubeVideo: '',
    date: '2024-07-20',
    publication: 'N/A',
    notes: 'N/A',
  },
];

export default function SearchableTable() {
  const [searchQuery, setSearchQuery] = useState('');

  // Filter data based on the search query
  const filteredData = data.filter(
    (item) =>
      item.platform.toLowerCase().includes(searchQuery.toLowerCase()) ||
      item.controlTechnique.toLowerCase().includes(searchQuery.toLowerCase()) ||
      item.navSys.toLowerCase().includes(searchQuery.toLowerCase()) ||
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
              <th>Navigation Systems</th>
              <th>Flight Conditions</th>
              <th>Notes</th>
              <th>Platform</th>
              <th>Link to GitHub</th>
              <th>Publication</th>
            </tr>
          </thead>
          <tbody>
            {filteredData.length > 0 ? (
              filteredData.map((item, index) => (
                <tr key={index}>
                  <td>{item.date}</td>
                  <td>{item.controlTechnique}</td>
                  <td>{item.navSys}</td>
                  <td>{item.flightConditions}</td>
                  <td>{item.notes || <div className={styles.notesPlaceholder}><p>Notes coming soon...</p></div>}</td>
                  <td>{item.platform}</td>
                  <td>
                    {item.githubLink ? (
                      <a href={item.githubLink} target="_blank" rel="noopener noreferrer">
                        GitHub
                      </a>
                    ) : (
                      'N/A'
                    )}
                  </td>
                  <td>{item.publication}</td>
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
