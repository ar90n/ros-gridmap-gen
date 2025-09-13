#!/usr/bin/env npx ts-node

// Sample map generator for testing examples
// This uses the actual core library functions for consistency

import * as fs from 'fs';
import * as path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

// Import the actual core library functions
import { 
  makeDefaultState, 
  buildPGM, 
  buildYamlROS1, 
  buildYamlROS2, 
  buildSdfWorld, 
  buildSdfWorldIgnition, 
  buildMVSimWorld,
  buildFlatlandWorld,
  type State
} from '../src/core.ts';

// Create sample state using the actual library function
function createSampleState(): State {
  const state = makeDefaultState(4, 4);
  
  // Add some walls for testing
  state.hEdges[1][0] = true;  // horizontal wall
  state.hEdges[1][2] = true;
  state.hEdges[3][1] = true;
  state.hEdges[3][2] = true;
  
  state.vEdges[1][1] = true;  // vertical wall
  state.vEdges[2][1] = true;
  state.vEdges[3][3] = true;
  
  state.cellSizeM = 1.0;
  state.resolution = 0.05;
  
  return state;
}

// Main function using actual library functions
async function main() {
  const mapsDir = path.join(__dirname, 'shared', 'maps');
  
  // Ensure directory exists
  if (!fs.existsSync(mapsDir)) {
    fs.mkdirSync(mapsDir, { recursive: true });
  }
  
  console.log('Generating sample map files using actual core library functions...');
  
  // Create sample state
  const state = createSampleState();
  
  // Generate all files using actual library functions
  const pgmBlob = buildPGM(state);
  const yamlRos1 = buildYamlROS1(state);
  const yamlRos2 = buildYamlROS2(state);
  const wallHeight = 0.5; // 50cm wall height for simulation
  const sdfClassic = buildSdfWorld(state, wallHeight, state.wallThicknessM);
  const sdfIgnition = buildSdfWorldIgnition(state, wallHeight, state.wallThicknessM);
  const mvSimWorldBase = buildMVSimWorld(state, wallHeight, state.wallThicknessM);
  const flatlandWorldBase = buildFlatlandWorld(state);
  
  // Add rover model to Flatland world and change map reference for shared/maps
  const flatlandWorld = flatlandWorldBase
    .replace('map: "map.yaml"', 'map: "map_ros1.yaml"')  // Use ROS1 format for shared/maps
    .replace(
      'models: []',
      `models: 
  - name: rover
    model: "rover_flatland_model.yaml"
    pose: [0.5, 0.5, 0]  # Start position in the world`
    );
  
  // Add robot to MVSim world
  const mvSimWorld = mvSimWorldBase.replace(
    '</mvsim_world>',
    `
  <!-- Include vehicle definitions -->
  <include file="/opt/ros/humble/share/mvsim/definitions/small_robot.vehicle.xml" />
  
  <!-- Add robot to the world -->
  <vehicle name="r1" class="small_robot">
    <init_pose>0.5 0.5 0</init_pose>
    <init_vel>0 0 0</init_vel>
  </vehicle>
  
</mvsim_world>`
  );
  
  // Define Flatland rover model content
  const roverFlatlandModel = `bodies:
  - name: base
    pose: [0, 0, 0]
    type: dynamic
    color: [1, 1, 1, 0.4]

    footprints:
      - type: circle
        radius: 0.3
        density: 1

      - type: polygon
        points: [[-.25, -.05], [-.25, 0.05], [-.15, 0.05], [-.15, -0.05]]
        density: 1

      - type: polygon
        points: [[-.125, -.25], [-.125, -.2], [.125, -.2], [.125, -.25]]
        density: 1

      - type: polygon
        points: [[-.125, .25], [-.125, .2], [.125, .2], [.125, .25]]
        density: 1

plugins:
  - type: DiffDrive
    name: rover_drive
    body: base
    odom_frame_id: odom
    twist_sub: /cmd_vel
    odom_pub: /odom
`;
  
  // Convert PGM Blob to Buffer for Node.js
  const pgmBuffer = Buffer.from(await pgmBlob.arrayBuffer());
  
  // Write files
  fs.writeFileSync(path.join(mapsDir, 'map.pgm'), pgmBuffer);
  fs.writeFileSync(path.join(mapsDir, 'map_ros1.yaml'), yamlRos1);
  fs.writeFileSync(path.join(mapsDir, 'map_ros2.yaml'), yamlRos2);
  fs.writeFileSync(path.join(mapsDir, 'world_classic.sdf'), sdfClassic);
  fs.writeFileSync(path.join(mapsDir, 'world_ignition.sdf'), sdfIgnition);
  fs.writeFileSync(path.join(mapsDir, 'world_mvsim.xml'), mvSimWorld);
  fs.writeFileSync(path.join(mapsDir, 'world_flatland.yaml'), flatlandWorld);
  fs.writeFileSync(path.join(mapsDir, 'rover_flatland_model.yaml'), roverFlatlandModel);
  
  console.log('Sample map files generated in:', mapsDir);
  console.log('Files created:');
  console.log(`  - map.pgm (${state.cols * Math.round(state.cellSizeM / state.resolution)}x${state.rows * Math.round(state.cellSizeM / state.resolution)} pixels, ${state.rows}x${state.cols} cells at ${state.resolution}m/px)`);
  console.log('  - map_ros1.yaml (ROS1 format)');
  console.log('  - map_ros2.yaml (ROS2 format)');
  console.log('  - world_classic.sdf (Gazebo Classic)');
  console.log('  - world_ignition.sdf (Gazebo Ignition)');
  console.log('  - world_mvsim.xml (MVSim with robot)');
  console.log('  - world_flatland.yaml (Flatland world)');
  console.log('  - rover_flatland_model.yaml (Flatland rover model)');
  console.log('');
  console.log('You can now test the simulators:');
  console.log('  docker-compose run --rm gazebo-classic');
  console.log('  docker-compose run --rm gazebo-ignition');
  console.log('  docker-compose run --rm mvsim');
  console.log('  docker-compose run --rm flatland');
}

// Execute if this file is run directly
main().catch(console.error);