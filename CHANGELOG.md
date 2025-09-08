# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Configurable floor friction coefficient for SDF world files
- Comprehensive physics settings for Gazebo simulation
- Ambient lighting with performance optimization (no shadows)
- Custom floor model with proper friction properties

### Changed  
- Floor thickness increased from 1cm to 10cm for better stability
- Floor surface positioned at z=0 for correct robot placement
- Improved wall surface properties with friction and bounce settings
- Cell click cycling now properly cycles through all 4 palette values (0→1→2→3→0)

### Removed
- Legacy compatibility code and migration functions
- Directional lighting (replaced with ambient for performance)
- Ground plane model (replaced with custom floor)

## [0.1.0] - 2024-XX-XX

### Added
- Initial release of ROS GridMap Generator
- Interactive grid editing with wall placement
- Cost palette system with 4 customizable values (0-254 range)
- Multi-format export: PGM, ROS1/ROS2 YAML, Gazebo SDF
- URL state persistence with JSONCrush compression
- Origin setting with rotation support (0°→90°→180°→270°)
- Configurable parameters: cell size, resolution, wall thickness
- ZIP download for selected file formats
- GitHub Pages deployment
- Comprehensive test suite
- TypeScript support
- Svelte 4 + Vite build system
- Tailwind CSS styling