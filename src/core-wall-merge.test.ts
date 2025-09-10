import { describe, it, expect } from 'vitest';
import { buildSdfWorld, makeDefaultState } from './core';

describe('Wall Merging', () => {
  it('should merge continuous horizontal walls into single segments', () => {
    const state = makeDefaultState(3, 3);
    // Create horizontal wall segment: [1][0], [1][1], [1][2]
    state.hEdges[1][0] = true;
    state.hEdges[1][1] = true;
    state.hEdges[1][2] = true;
    
    const sdf = buildSdfWorld(state, 0.5, 0.1);
    
    // Should contain single merged wall instead of 3 separate walls
    expect(sdf).toContain('wall_horizontal_0');
    expect(sdf).not.toContain('wall_h_1_0');
    expect(sdf).not.toContain('wall_h_1_1');
    expect(sdf).not.toContain('wall_h_1_2');
  });

  it('should merge continuous vertical walls into single segments', () => {
    const state = makeDefaultState(3, 3);
    // Create vertical wall segment: [1][0], [1][1], [1][2]
    state.vEdges[1][0] = true;
    state.vEdges[1][1] = true;
    state.vEdges[1][2] = true;
    
    const sdf = buildSdfWorld(state, 0.5, 0.1);
    
    // Should contain single merged wall instead of 3 separate walls
    expect(sdf).toContain('wall_vertical_0');
    expect(sdf).not.toContain('wall_v_1_0');
    expect(sdf).not.toContain('wall_v_1_1');
    expect(sdf).not.toContain('wall_v_1_2');
  });

  it('should use custom wall thickness', () => {
    const state = makeDefaultState(2, 2);
    state.hEdges[1][0] = true;
    
    const sdf = buildSdfWorld(state, 0.5, 0.2); // 0.2m thickness
    
    // Wall should have 0.2m thickness (horizontal wall: length x thickness x height)
    expect(sdf).toMatch(/<size>[\d.]+ 0\.2 0\.5<\/size>/);
  });

  it('should always include ground plane', () => {
    const state = makeDefaultState(2, 2);
    
    const sdf = buildSdfWorld(state, 0.5, 0.1);
    
    expect(sdf).toContain('model://ground_plane');
    expect(sdf).toContain('<light name="directional_light"');
  });
});