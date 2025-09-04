import { buildPGM, buildYamlROS1, buildYamlROS2, buildSdfWorld, type State } from './core';
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
export async function exportAll(state: State, opts: { ros1: boolean; ros2: boolean; sdf: boolean; wallHeight: number }) {
  const zip = new JSZip();
  
  // Always export PGM
  const pgmBlob = buildPGM(state);
  zip.file('map.pgm', pgmBlob);
  
  // Export YAML files based on options
  if (opts.ros2) {
    zip.file('map_ros2.yaml', buildYamlROS2(state));
  }
  if (opts.ros1) {
    zip.file('map_ros1.yaml', buildYamlROS1(state));
  }
  
  // Export SDF world based on option
  if (opts.sdf) {
    zip.file('world.sdf', buildSdfWorld(state, opts.wallHeight, state.wallThicknessM));
  }
  
  // Generate and download ZIP file
  const zipBlob = await zip.generateAsync({ type: 'blob' });
  await downloadBlobWithDialog(zipBlob, 'ros_map_files.zip');
}