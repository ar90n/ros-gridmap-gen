# ROS GridMap Generator

A browser-based tool for creating and editing ROS occupancy grid maps (PGM + YAML) and Gazebo SDF world files.

## 🚀 Demo

[Live Demo](https://ar90n.github.io/ros-gridmap-gen/)

## ✨ Features

- **Grid Editing**: Create maps on 2×2 to 10×10 grids with wall placement
- **Origin Setting**: Shift+click to set origin position and orientation
- **Blocked Cells**: Normal click to mark cells as impassable
- **File Export**: Export as PGM, ROS1/ROS2 YAML, and SDF formats
- **ZIP Download**: Download selected files bundled together
- **URL Sharing**: Save and share map state via URL

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
├── core.ts          # Business logic (Pure functions)
├── state.ts         # State management (Svelte Store)
├── export.ts        # File export functionality
├── App.svelte       # Main UI component
├── main.ts          # Entry point
└── main.css         # Styles
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

1. **Select Grid Size**: Choose from 2×2 to 10×10
2. **Place Walls**: Click on grid edges to toggle walls ON/OFF
3. **Set Origin**: Shift+click on cells to set origin, click same cell again to rotate
4. **Mark Blocked Cells**: Normal click on cells to mark as impassable
5. **Export**: Select desired file formats and download as ZIP

## 🎯 Output File Formats

### PGM (Portable Gray Map)
- **Free Space**: 254 (white-ish)
- **Occupied**: 0 (black)
- **Unknown**: 205 (gray, boundary areas)

### YAML (ROS Map Server)
- Supports both ROS1 and ROS2 formats
- Includes resolution, origin coordinates, and occupancy thresholds

### SDF (Gazebo World)
- Each wall exported as Box model
- Customizable wall height
- Proper coordinate system alignment

## 📜 License

MIT License

## 🤝 Contributing

Pull requests and issues are welcome!

## 🔧 Technical Details

### Coordinate Systems
- **State**: Bottom-up (row=0 is bottom)
- **DOM**: Top-down (standard web coordinates)
- **PGM**: Bottom-left origin (ROS standard)

### URL Persistence
- Uses JSONCrush compression to store state in URL fragment (#s=)
- Auto-updates with 150ms debounce

### Architecture
- Follows Functional Domain-Driven Design (FDDD) principles
- Pure functions in core layer
- Immutable state transformations
- Clear separation between business logic and UI