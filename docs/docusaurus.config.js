// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// See: https://docusaurus.io/docs/api/docusaurus-config

import { themes as prismThemes } from "prism-react-renderer";

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: "Advanced Control Systems Lab Flight Stacks",
  tagline:
  "A community-lead ecosystem to design, test, and compare control system for multi-rotor and fixed-wing UAVs",
favicon: "img/ACSL_Logo.jpg",

  // Set the production url of your site here
  url: "https://andrealaffly.github.io",
  // url: "https://acslstack.com",

  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: "/ACSL-flightstack/",
  // baseUrl: "/",

  // GitHub pages deployment config.
  organizationName: "andrealaffly", // Usually your GitHub org/user name.
  projectName: "ACSL-flightstack", // Usually your repo name.

  trailingSlash: false,

  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "warn",

  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        //docs: {
         // sidebarPath: require.resolve("./sidebars.js"),
         // editUrl:
          //  "https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/",
       // },
        //blog: {
          //blogSidebarCount: 0,
         // showReadingTime: false,
          
          //feedOptions: {
          //  type: ["rss", "atom"],
          //  xslt: true,
          //},
          //editUrl:
           // "https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/",
          //onInlineTags: "warn",
          //onInlineAuthors: "warn",
          //onUntruncatedBlogPosts: "warn",
       // },
        //theme: {
         // customCss: require.resolve("./src/css/custom.css"),
      //  },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: "img/docusaurus-social-card.jpg",
      navbar: {
        title: "Advanced Control System Lab",
        logo: {
          alt: "My Site Logo",
          src: "img/ACSL_Logo_2.png",
        },
        items: [
          {
            to: "/Description",
            position: "right",
            label: "Description",
          },
          {
            to: "/hardware",
            label: "Compatible Hardware",
            position: "right",
          },
          { to: "/Journals", label: "Publications", position: "right" },
          { to: "/contributors", label: "Contributors", position: "right" },
          { to: "/test", label: "Tests", position: "right" },
          

          {
            
            label: 'Documentation',
            position: 'right',
            items: [
              {
                label: 'Multi-Rotor',
                to: '/Documentation', // Link to the new React page
              },
              {
                label: 'Winged',
                to: '/Documentation', // Link to the new React page
              },
            ]
          },
        ],
      },
      footer: {
        style: "dark",
        
        
        links: [
          
          {
            title: "External Links",
            
            
            items: [
              {
                label: "L'Afflitto Website",
                href: "https://lafflitto.com/",
                
              },
              {
                label: "Winged Copter",
                href: "https://github.com/andrealaffly/ACSL-flightstack-winged?tab=readme-ov-file",
               
              },
              {
                label: "Multi-Rotor",
                href: "https://github.com/andrealaffly/ACSL-flightstack?tab=readme-ov-file://lafflitto.com/",
                
              },
              {
                label: "License",
                href: "https://github.com/andrealaffly/ACSL-flightstack/blob/main/LICENSE.txt",
                
              },
              
            ],
            
          },
        ],
        
        copyright: `Copyright © ${new Date().getFullYear()} Advanced Control Systems Lab, Virginia Tech. Built with Docusaurus.`,
      },
      
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),

  
};

export default config;
