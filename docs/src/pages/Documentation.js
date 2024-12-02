import React from 'react';
import Layout from '@theme/Layout';

export default function Documentation() {
  return (
    <Layout title="Documentation">
      <div style={{ height: '100vh', overflow: 'hidden' }}>
        <iframe
          src="/documentation/html/index_ONE.html"
          title="Documentation"
          style={{ width: '100%', height: '100%', border: 'none' }}
        />
      </div>
    </Layout>
  );
}
