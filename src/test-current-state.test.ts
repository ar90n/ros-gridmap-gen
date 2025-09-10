import { describe, it } from 'vitest';
import { buildSdfWorld, makeDefaultState } from './core';
import fs from 'fs';

describe('Current State Check', () => {
  it('should check what we actually have vs what was requested', () => {
    const state = makeDefaultState(3, 3);
    state.origin = { row: 1, col: 1, thetaDeg: 0 };
    
    state.hEdges[1][1] = true;

    const sdf = buildSdfWorld(state, 0.5, state.wallThicknessM);
    
    fs.writeFileSync('/workspaces/ros-gridmap-gen/current_state.sdf', sdf);
    
    console.log('=== CURRENT STATE ANALYSIS ===');
    
    // Check floor collision
    const floorModel = sdf.match(/<model name="custom_floor"[\s\S]*?<\/model>/);
    const floorHasCollision = floorModel?.[0]?.includes('<collision') || false;
    const floorSizeMatch = sdf.match(/<model name="custom_floor"[\s\S]*?<size>([^<]+)<\/size>/);
    
    console.log('FLOOR ANALYSIS:');
    console.log('  Has collision:', floorHasCollision);
    console.log('  Size:', floorSizeMatch?.[1] || 'not found');
    
    // Calculate expected vs actual
    const mapSize = 3 * 0.5; // 3x3 grid, 0.5m cells = 1.5m
    const expectedFloorSize = mapSize + 2 * (2 * 0.5); // +2 cells each side = +2m total
    console.log('  Map size: 1.5m x 1.5m');
    console.log('  Expected floor: 3.5m x 3.5m (1.5 + 2.0)');
    console.log('  Current margin:', 2 * 0.5, 'm per side');
    
    // Check physics
    const hasPhysics = sdf.includes('<physics');
    const physicsType = sdf.match(/<physics[^>]*type="([^"]+)"/)?.[1];
    
    console.log('\nPHYSICS ANALYSIS:');
    console.log('  Has physics:', hasPhysics);
    console.log('  Physics type:', physicsType || 'none');
    
    // Check static objects
    const staticCount = (sdf.match(/static="true"/g) || []).length;
    console.log('  Static objects count:', staticCount);
    
    console.log('\n📄 File: current_state.sdf');
    console.log('\n❌ Issues to fix:');
    console.log('1. Falling objects (static not working?)');
    console.log('2. Floor margin calculation might be wrong');
  });
});