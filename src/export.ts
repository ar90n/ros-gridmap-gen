import { buildPGM, buildYamlROS1, buildYamlROS2, buildSdfWorld, buildSdfWorldIgnition, buildMVSimWorld, buildFlatlandWorld, type State } from './core';
import JSZip from 'jszip';

// File download utilities with save dialog
async function downloadBlobWithDialog(blob: Blob, name: string) {
  // Use the File System Access API if available (modern browsers)
  if ('showSaveFilePicker' in window) {
    try {
      const fileHandle = await (window as any).showSaveFilePicker({
        suggestedName: name,
        types: [{
          description: 'ZIP files',
          accept: { 'application/zip': ['.zip'] }
        }]
      });
      const writable = await fileHandle.createWritable();
      await writable.write(blob);
      await writable.close();
      return;
    } catch (err) {
      // User cancelled - don't do anything
      if (err instanceof Error && err.name === 'AbortError') {
        console.log('Save cancelled by user');
        return;
      }
      // Other error occurred, fall back to regular download
      console.warn('Save dialog failed, falling back to regular download:', err);
    }
  }
  
  // Fallback to regular download (only if File System Access API not available)
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 0);
}


// Main export function - now creates a ZIP file
export async function exportAll(state: State, opts: { rosVersion: 'ros1' | 'ros2'; simulatorFormat: 'classic' | 'gazebo' | 'mvsim' | 'flatland'; wallHeight: number }) {
  const zip = new JSZip();
  
  // Always export PGM
  const pgmBlob = buildPGM(state);
  zip.file('map.pgm', pgmBlob);
  
  // Export YAML files based on ROS version selection (unified filename)
  if (opts.rosVersion === 'ros1') {
    zip.file('map.yaml', buildYamlROS1(state));
  } else if (opts.rosVersion === 'ros2') {
    zip.file('map.yaml', buildYamlROS2(state));
  }
  
  // Export simulator format based on selection
  if (opts.simulatorFormat === 'classic') {
    zip.file('world_classic.sdf', buildSdfWorld(state, opts.wallHeight, state.wallThicknessM));
  } else if (opts.simulatorFormat === 'gazebo') {
    zip.file('world_ignition.sdf', buildSdfWorldIgnition(state, opts.wallHeight, state.wallThicknessM));
  } else if (opts.simulatorFormat === 'mvsim') {
    zip.file('world_mvsim.xml', buildMVSimWorld(state, opts.wallHeight, state.wallThicknessM));
  } else if (opts.simulatorFormat === 'flatland') {
    zip.file('world_flatland.yaml', buildFlatlandWorld(state));
  }
  
  // Generate and download ZIP file
  const zipBlob = await zip.generateAsync({ type: 'blob' });
  await downloadBlobWithDialog(zipBlob, 'ros_map_files.zip');
}