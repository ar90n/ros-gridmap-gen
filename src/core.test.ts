import { describe, it, expect } from 'vitest';
import {
  makeDefaultState,
  computeOrigin,
  buildPGM,
  buildYamlROS1,
  buildYamlROS2,
  buildSdfWorld,
  buildMVSimWorld,
  type State,
  type Deg
} from './core';

describe('Core Business Logic', () => {
  describe('makeDefaultState', () => {
    it('should create default 5x5 grid state', () => {
      const state = makeDefaultState(5, 5);
      expect(state.rows).toBe(5);
      expect(state.cols).toBe(5);
      expect(state.cellSizeM).toBe(0.5);
      expect(state.resolution).toBe(0.03125); // 0.5m / 16px
      expect(state.wallThicknessM).toBe(0.1); // 100mm default
      expect(state.hEdges).toHaveLength(6); // rows + 1
      expect(state.vEdges).toHaveLength(6); // cols + 1
      expect(state.cellCostPalette).toEqual([0, 30, 80, 150]);
      expect(state.selectedPaletteIndex).toBe(0);
      expect(state.cellCostIndices).toHaveLength(5); // rows
      expect(state.cellCostIndices[0]).toHaveLength(5); // cols
      expect(state.origin).toEqual({ row: 4, col: 4, thetaDeg: 0 });
    });

    it('should create custom sized grid', () => {
      const state = makeDefaultState(3, 4);
      expect(state.rows).toBe(3);
      expect(state.cols).toBe(4);
      expect(state.hEdges).toHaveLength(4); // 3 + 1
      expect(state.vEdges).toHaveLength(5); // 4 + 1
      expect(state.origin).toEqual({ row: 2, col: 3, thetaDeg: 0 });
    });

    it('should initialize all edges as false', () => {
      const state = makeDefaultState(2, 2);
      state.hEdges.forEach(row => {
        row.forEach(edge => expect(edge).toBe(false));
      });
      state.vEdges.forEach(row => {
        row.forEach(edge => expect(edge).toBe(false));
      });
    });
  });

  describe('computeOrigin', () => {
    it('should compute origin for theta=0', () => {
      const state = makeDefaultState(5, 5);
      state.origin = { row: 2, col: 2, thetaDeg: 0 };
      const origin = computeOrigin(state);
      
      const px = 2 * 16 + 8; // col * ppc + ppc/2 = 40
      const py = 2 * 16 + 8; // row * ppc + ppc/2 = 40
      const res = 0.5 / 16; // cellSizeM / ppc
      
      expect(origin.ox).toBeCloseTo(-px * res, 5);
      expect(origin.oy).toBeCloseTo(-py * res, 5);
      expect(origin.theta).toBe(0);
    });

    it('should compute origin for theta=90', () => {
      const state = makeDefaultState(5, 5);
      state.origin = { row: 2, col: 2, thetaDeg: 90 };
      const origin = computeOrigin(state);
      
      expect(origin.theta).toBeCloseTo(Math.PI / 2, 5);
    });

    it('should compute origin for theta=180', () => {
      const state = makeDefaultState(5, 5);
      state.origin = { row: 2, col: 2, thetaDeg: 180 };
      const origin = computeOrigin(state);
      
      expect(origin.theta).toBeCloseTo(Math.PI, 5);
    });

    it('should compute origin for theta=270', () => {
      const state = makeDefaultState(5, 5);
      state.origin = { row: 2, col: 2, thetaDeg: 270 };
      const origin = computeOrigin(state);
      
      expect(origin.theta).toBeCloseTo(3 * Math.PI / 2, 5);
    });
  });

  describe('buildPGM', () => {
    it('should create PGM blob with correct dimensions', () => {
      const state = makeDefaultState(3, 3);
      const blob = buildPGM(state);
      
      expect(blob).toBeInstanceOf(Blob);
      expect(blob.type).toBe('image/x-portable-graymap');
      expect(blob.size).toBeGreaterThan(0);
    });

    it('should render walls correctly for different grid sizes', () => {
      const state1 = makeDefaultState(2, 2);
      const state2 = makeDefaultState(5, 5);
      
      const blob1 = buildPGM(state1);
      const blob2 = buildPGM(state2);
      
      // Larger grid should produce larger file
      expect(blob2.size).toBeGreaterThan(blob1.size);
    });
  });

  describe('buildYamlROS1', () => {
    it('should generate valid ROS1 YAML', () => {
      const state = makeDefaultState(5, 5);
      const yaml = buildYamlROS1(state);
      
      expect(yaml).toContain('image: map.pgm');
      expect(yaml).toContain('resolution:');
      expect(yaml).toContain('origin:');
      expect(yaml).toContain('negate: 0');
      expect(yaml).toContain('occupied_thresh: 0.65');
      expect(yaml).toContain('free_thresh: 0.196');
    });

    it('should use configured resolution', () => {
      const state = makeDefaultState(5, 5);
      state.cellSizeM = 1.0;
      state.resolution = 0.0625; // Set resolution explicitly
      const yaml = buildYamlROS1(state);
      
      expect(yaml).toContain(`resolution: 0.0625`);
    });
  });

  describe('buildYamlROS2', () => {
    it('should generate valid ROS2 YAML', () => {
      const state = makeDefaultState(5, 5);
      const yaml = buildYamlROS2(state);
      
      expect(yaml).toContain('image: map.pgm');
      expect(yaml).toContain('mode: trinary');
      expect(yaml).toContain('resolution:');
      expect(yaml).toContain('origin:');
      expect(yaml).toContain('negate: false');
      expect(yaml).toContain('occupied_thresh: 0.65');
      expect(yaml).toContain('free_thresh: 0.25');
    });
  });

  describe('buildSdfWorld', () => {
    it('should generate valid SDF XML', () => {
      const state = makeDefaultState(2, 2);
      const sdf = buildSdfWorld(state);
      
      expect(sdf).toContain('<sdf version="1.7">');
      expect(sdf).toContain('<world name="map_world">');
      expect(sdf).toContain('<model name="ground_plane"');
      expect(sdf).toContain('</world>');
      expect(sdf).toContain('</sdf>');
    });

    it('should create wall models for edges', () => {
      const state = makeDefaultState(2, 2);
      state.hEdges[1][0] = true;
      state.vEdges[1][0] = true;
      
      const sdf = buildSdfWorld(state);
      
      expect(sdf).toContain('Wall_0');
      expect(sdf).toContain('Wall_');
      expect(sdf).toContain('<model name=');
      expect(sdf).toContain('<static>1</static>');
    });

    it('should position walls relative to origin', () => {
      const state = makeDefaultState(3, 3);
      state.origin = { row: 1, col: 1, thetaDeg: 0 };
      state.hEdges[1][1] = true;
      
      const sdf = buildSdfWorld(state);
      
      expect(sdf).toContain('Wall_0');
      expect(sdf).toContain('<pose>');
    });

    it('should set correct wall thickness', () => {
      const state = makeDefaultState(2, 2);
      state.cellSizeM = 1.0;
      state.hEdges[0][0] = true;
      
      const sdf = buildSdfWorld(state, 0.5, 0.2); // Custom thickness 0.2
      
      expect(sdf).toContain('0.2'); // Check for custom thickness
    });
  });

  describe('buildMVSimWorld', () => {
    it('should generate valid MVSim XML', () => {
      const state = makeDefaultState(2, 2);
      const mvsim = buildMVSimWorld(state);
      
      expect(mvsim).toContain('<?xml version="1.0"?>');
      expect(mvsim).toContain('<mvsim_world>');
      expect(mvsim).toContain('<element class="horizontal_plane">');
      expect(mvsim).toContain('</mvsim_world>');
    });

    it('should create vertical plane elements for walls', () => {
      const state = makeDefaultState(2, 2);
      state.hEdges[1][0] = true;
      state.vEdges[1][0] = true;
      
      const mvsim = buildMVSimWorld(state);
      
      expect(mvsim).toContain('<element class="vertical_plane">');
      expect(mvsim).toContain('<x0>');
      expect(mvsim).toContain('<y0>');
      expect(mvsim).toContain('<height>');
    });

    it('should set correct ground plane dimensions', () => {
      const state = makeDefaultState(3, 3);
      state.cellSizeM = 1.0; // 3x3m grid
      
      const mvsim = buildMVSimWorld(state);
      
      // Should have 1m margin, so -1 to 4
      expect(mvsim).toContain('<x_min>-1</x_min>');
      expect(mvsim).toContain('<y_min>-1</y_min>');
      expect(mvsim).toContain('<x_max>4</x_max>');
      expect(mvsim).toContain('<y_max>4</y_max>');
    });

    it('should set custom wall height', () => {
      const state = makeDefaultState(2, 2);
      state.hEdges[0][0] = true;
      
      const mvsim = buildMVSimWorld(state, 1.5); // Custom height 1.5m
      
      expect(mvsim).toContain('<height>1.5</height>');
    });
  });
});