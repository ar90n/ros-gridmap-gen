# ROS GridMap Generator

[![CI](https://github.com/ar90n/ros-gridmap-gen/actions/workflows/ci.yml/badge.svg)](https://github.com/ar90n/ros-gridmap-gen/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Built with vibe coding](https://img.shields.io/badge/built%20with-vibe%20coding-ff69b4)

A browser-based tool for creating and editing ROS occupancy grid maps (PGM + YAML) and Gazebo SDF world files.

## 🚀 Demo

[Live Demo](https://ar90n.github.io/ros-gridmap-gen/)

## ✨ Features

- **Grid Editing**: Create maps on 2×2 to 10×10 grids with intuitive wall placement
- **Origin Setting**: Shift+click to set origin position and orientation (0°→90°→180°→270°)
- **Cost Palette System**: 4 customizable cost values (0-254) with smooth color gradients
- **Cell Cost Assignment**: Click cells to apply cost values, with visual feedback
- **Configurable Parameters**: Adjustable resolution (m/px) and wall thickness (m)
- **Multi-format Export**: PGM, ROS1/ROS2 YAML, and Gazebo SDF world files
- **ZIP Download**: Download selected file formats bundled together
- **URL Sharing**: Save and share complete map state via compressed URL

## 🛠️ Tech Stack

- **Framework**: Svelte 4
- **Build Tool**: Vite
- **Styling**: Tailwind CSS
- **Language**: TypeScript
- **Testing**: Vitest
- **Deployment**: GitHub Pages

## 📦 Development

### Prerequisites
- Node.js 18+
- npm

### Setup
```bash
# Install dependencies
npm install

# Start development server
npm run dev

# Run tests
npm run test

# Build for production
npm run build
```

### Project Structure
```
src/
├── core.ts              # Business logic (Pure functions)
├── state.ts             # State management (Svelte Store + URL persistence)
├── export.ts            # File export functionality (ZIP generation)
├── App.svelte           # Main UI component with cost palette
├── main.ts              # Entry point
├── main.css             # Enhanced styles with animations
└── types/               # Type definitions
    └── jsoncrush.d.ts   # JSONCrush type declarations
```

## 🚀 Deployment

### GitHub Pages
This project automatically deploys to GitHub Pages using GitHub Actions.

1. Enable Pages in your GitHub repository
2. Go to Settings > Pages > Source: "GitHub Actions"
3. Push to `main` branch to trigger automatic deployment

### Manual Deployment
```bash
npm run build
# Upload the contents of the dist folder to any static file hosting
```

## 📋 Usage

1. **Configure Grid**: Choose size (2×2 to 10×10), cell size, resolution, and wall parameters
2. **Place Walls**: Click on grid edges to toggle walls ON/OFF with visual feedback
3. **Set Origin**: Shift+click on cells to set origin, click same cell again to rotate (0°→90°→180°→270°)
4. **Assign Cell Costs**: 
   - Select cost palette (0-254 range) in right panel
   - Click cells to apply cost values
   - Visual color gradient shows cost levels (white=free, red=high cost)
   - Click same cell multiple times to cycle through palette
5. **Export Maps**: Select desired formats (ROS1/ROS2 YAML, SDF) and download as ZIP

## 🎯 Output File Formats

### PGM (Portable Gray Map)
- **Cost-based values**: 0-254 range (inverted: cost 0 → PGM 254, cost 254 → PGM 0)
- **Free Space**: High PGM values (white-ish)
- **Occupied**: Low PGM values (black)
- **Unknown**: 205 (gray, boundary areas)
- **Configurable wall thickness**: Converted from meters to pixels with ceiling expansion

### YAML (ROS Map Server)
- **ROS1 & ROS2 formats**: Compatible with map_server and nav2_map_server
- **Configurable resolution**: Direct meters-per-pixel specification
- **Origin coordinates**: Computed based on selected origin cell and rotation
- **Occupancy thresholds**: Standard values for navigation

### SDF (Gazebo World)
- **Merged wall segments**: Continuous walls combined for efficiency
- **Box models**: Each wall as static collision geometry
- **Configurable parameters**: Wall height (m) and thickness (m)
- **Coordinate alignment**: Matches top-down view input

## 📜 License

MIT License

## 🤝 Contributing

Pull requests and issues are welcome!

## 🔧 Technical Details

### Coordinate Systems
- **State**: Bottom-up (row=0 is bottom) with version migration support
- **DOM**: Top-down (standard web coordinates)
- **PGM**: Bottom-left origin with Y-axis inversion
- **SDF**: Y-axis inverted to match top-down view input

### Cost System
- **4-value palette**: Customizable cost values (0-254)
- **Index-based storage**: Cells reference palette indices for memory efficiency
- **Color interpolation**: Smooth gradient between cost levels
- **ROS compatibility**: Direct mapping to navigation cost values

### URL Persistence
- **JSONCrush compression**: Stores complete state in URL fragment (#s=)
- **Auto-updates**: 150ms debounce for smooth editing
- **Backward compatibility**: Migration system for state versions

### Architecture
- **FDDD principles**: Functional Domain-Driven Design
- **Pure functions**: Immutable core business logic
- **State migration**: Backward compatibility for URL sharing
- **Layered design**: Clear separation between business logic and UI
