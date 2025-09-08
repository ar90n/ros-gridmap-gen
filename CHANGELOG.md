# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2025-01-08

### Added
- Interactive grid editing with wall placement (2×2 to 10×10)
- Cost palette system with 4 customizable values (0-254 range)
- Multi-format export: PGM, ROS1/ROS2 YAML, Gazebo SDF world files
- URL state persistence with JSONCrush compression
- Origin setting with rotation support (0°→90°→180°→270°)
- Configurable parameters: cell size, resolution, wall thickness, floor friction
- ZIP download for selected file formats
- Comprehensive physics settings for Gazebo simulation
- Custom floor model with configurable friction properties (0.0+)
- Ambient lighting with performance optimization (no shadows)
- GitHub repository link in header
- Comprehensive test suite with TypeScript support

### Technical Features
- Svelte 4 + Vite build system with TypeScript
- Tailwind CSS styling with responsive design  
- FDDD architecture with pure functions
- State management with reactive stores
- Automatic GitHub Pages deployment
- CI/CD with quality checks (tests, type-check, build, security audit)
- Release automation with semantic versioning