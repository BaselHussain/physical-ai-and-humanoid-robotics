/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx,md,mdx}',
    './docs/**/*.{md,mdx}',
  ],
  darkMode: ['class', '[data-theme="dark"]'], // Docusaurus theme integration
  theme: {
    extend: {
      colors: {
        'robotics-blue': '#2563eb',
        'robotics-green': '#10b981',
        'circuit-gray': '#6b7280',
      },
      backgroundImage: {
        'circuit-pattern': "url('/img/circuit-pattern.svg')",
        'geometric-accent': "url('/img/geometric-accent.svg')",
      },
      transitionTimingFunction: {
        'hover-smooth': 'cubic-bezier(0.4, 0, 0.2, 1)',
      },
    },
  },
  plugins: [],
  corePlugins: {
    preflight: false, // Disable Tailwind's base reset to avoid conflicts with Docusaurus
  },
};
