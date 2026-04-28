import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  docsSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Getting Started',
      items: [
        'getting-started/installation',
        'getting-started/first-project',
        'getting-started/building',
      ],
    },
    {
      type: 'category',
      label: 'Architecture',
      items: [
        'architecture/overview',
        'architecture/microkernel',
        'architecture/ipc',
        'architecture/capabilities',
        'architecture/hal',
      ],
    },
    {
      type: 'category',
      label: 'Guides',
      items: [
        'guides/motor-control',
        'guides/sensor-fusion',
        'guides/navigation',
      ],
    },
    {
      type: 'category',
      label: 'Design',
      items: [
        'design/esp32-rust-rtos-and-sketch-api',
      ],
    },
  ],
  adrSidebar: [
    'decisions/intro',
    {
      type: 'category',
      label: 'Architecture Decisions',
      items: [
        'decisions/0001-rust-over-python',
        'decisions/0002-esp32-chip-selection',
        'decisions/0003-microkernel-architecture',
        'decisions/0004-real-time-requirements',
        'decisions/0005-embassy-async',
      ],
    },
  ],
};

export default sidebars;
