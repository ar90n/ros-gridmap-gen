import { writable } from 'svelte/store';
import JSONCrush from 'jsoncrush';
import { makeDefaultState, migrateState, type State, type Deg } from './core';

// URL encoding/decoding functions
const encode = (s: State) => encodeURIComponent(JSONCrush.crush(JSON.stringify(s)));
const decode = (q: string): State | null => {
  try {
    const parsed = JSON.parse(JSONCrush.uncrush(decodeURIComponent(q)));
    return migrateState(parsed);
  } catch {
    return null;
  }
};

// Load state from URL hash
const loadFromHash = (): State | null => {
  const m = location.hash.match(/^#s=(.*)$/);
  return m ? decode(m[1]) : null;
};

// Initialize state from URL or default
export const state = writable<State>(loadFromHash() ?? makeDefaultState(5, 5));
export const shareUrl = writable<string>(location.href);

// Debounced URL update
let debounceTimeout: number | undefined;
state.subscribe((s) => {
  clearTimeout(debounceTimeout);
  debounceTimeout = window.setTimeout(() => {
    const url = `#s=${encode(s)}`;
    history.replaceState(null, '', url);
    shareUrl.set(location.href);
  }, 150);
});

// Action functions (pure state transformations)
export function setGrid(rows: number, cols: number) {
  state.set(makeDefaultState(rows, cols));
}

export function setCellSize(m: number) {
  state.update(s => ({
    ...s,
    cellSizeM: Math.max(0.01, m || 0.5)
  }));
}

export function setResolution(res: number) {
  state.update(s => ({
    ...s,
    resolution: Math.max(0.001, res || 0.03125)
  }));
}

export function setWallThicknessM(m: number) {
  state.update(s => ({
    ...s,
    wallThicknessM: Math.max(0.001, m || 0.03)
  }));
}

export function clickCell(rDom: number, c: number) {
  state.update(s => {
    const rB = s.rows - 1 - rDom; // Convert DOM row to bottom-up row
    
    // Apply selected palette index to cell, or cycle to next palette if already selected
    // This now works for all cells including origin
    const cellCostIndices = s.cellCostIndices.map(row => row.slice());
    const currentIndex = cellCostIndices[rB][c];
    // Always cycle to the next value when clicking a cell
    const newIndex = currentIndex === null ? 0 : (currentIndex + 1) % 4;
    cellCostIndices[rB][c] = newIndex;
    
    return { ...s, cellCostIndices };
  });
}

export function setOrigin(rDom: number, c: number) {
  state.update(s => {
    const rB = s.rows - 1 - rDom; // Convert DOM row to bottom-up row
    
    if (s.origin.row === rB && s.origin.col === c) {
      // Same cell clicked - rotate theta
      const next = ((s.origin.thetaDeg + 90) % 360) as Deg;
      return {
        ...s,
        origin: { ...s.origin, thetaDeg: next }
      };
    }
    
    // Different cell clicked - move origin and reset theta to 0
    // Keep the existing cell cost value unchanged
    return {
      ...s,
      origin: { row: rB, col: c, thetaDeg: 0 }
    };
  });
}

export function toggleEdge(kind: 'top' | 'bottom' | 'left' | 'right', rDom: number, c: number) {
  state.update(s => {
    const rB = s.rows - 1 - rDom; // Convert DOM row to bottom-up row
    
    if (kind === 'top') {
      const hEdges = s.hEdges.map(row => row.slice());
      hEdges[rB + 1][c] = !hEdges[rB + 1][c];
      return { ...s, hEdges };
    } else if (kind === 'bottom') {
      const hEdges = s.hEdges.map(row => row.slice());
      hEdges[rB][c] = !hEdges[rB][c];
      return { ...s, hEdges };
    } else if (kind === 'left') {
      const vEdges = s.vEdges.map(row => row.slice());
      vEdges[c][rB] = !vEdges[c][rB];
      return { ...s, vEdges };
    } else { // right
      const vEdges = s.vEdges.map(row => row.slice());
      vEdges[c + 1][rB] = !vEdges[c + 1][rB];
      return { ...s, vEdges };
    }
  });
}

// New functions for palette management
export function selectPaletteIndex(index: number) {
  state.update(s => ({
    ...s,
    selectedPaletteIndex: Math.max(0, Math.min(3, index))
  }));
}

export function updatePaletteValue(index: number, value: number) {
  state.update(s => {
    const cellCostPalette = [...s.cellCostPalette] as [number, number, number, number];
    cellCostPalette[index] = Math.max(0, Math.min(254, value));
    return { ...s, cellCostPalette };
  });
}