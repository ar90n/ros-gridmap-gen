// Pure business logic - Core layer (FDDD)
// All functions are pure with no side effects

export type Deg = 0 | 90 | 180 | 270;

export interface State {
  version: 3;
  rows: number;
  cols: number;
  cellSizeM: number;
  pixelsPerCell: 16;
  wallThicknessPx: 3;
  hEdges: boolean[][];
  vEdges: boolean[][];
  blockedCells: boolean[][];
  origin: { row: number; col: number; thetaDeg: Deg };
}

// Pure function to create default state
export function makeDefaultState(rows: number, cols: number): State {
  const hEdges = Array.from({ length: rows + 1 }, () => 
    Array(cols).fill(false)
  );
  const vEdges = Array.from({ length: cols + 1 }, () => 
    Array(rows).fill(false)
  );
  const blockedCells = Array.from({ length: rows }, () => 
    Array(cols).fill(false)
  );
  
  return {
    version: 3,
    rows,
    cols,
    cellSizeM: 0.5,
    pixelsPerCell: 16,
    wallThicknessPx: 3,
    hEdges,
    vEdges,
    blockedCells,
    origin: { row: rows - 1, col: cols - 1, thetaDeg: 0 }
  };
}

// Pure function to compute origin transformation
export function computeOrigin(state: State): { ox: number; oy: number; theta: number } {
  const ppc = state.pixelsPerCell;
  const res = state.cellSizeM / ppc;
  const px = state.origin.col * ppc + ppc / 2;
  const py = state.origin.row * ppc + ppc / 2;
  const rad = (state.origin.thetaDeg * Math.PI) / 180;
  const cos = Math.cos(rad);
  const sin = Math.sin(rad);
  
  const ox = -(cos * px - sin * py) * res;
  const oy = -(sin * px + cos * py) * res;
  
  return { ox, oy, theta: rad };
}

// Pure function to build PGM image data
export function buildPGM(state: State): Blob {
  const borderSize = 2; // 境界の厚さ（セル単位）
  const cellPx = state.pixelsPerCell;
  const W = (state.cols + 2 * borderSize) * cellPx;
  const H = (state.rows + 2 * borderSize) * cellPx;
  const free = 254;      // Free space (white-ish)
  const occ = 0;         // Occupied walls (black)
  const unknown = 205;   // Unknown boundary areas (gray)
  const buf = new Uint8Array(W * H);
  
  // 全体を境界（グレー：unknown）で初期化
  buf.fill(unknown);
  
  // マップ領域を自由空間（白）で塗りつぶし
  for (let r = 0; r < state.rows; r++) {
    for (let c = 0; c < state.cols; c++) {
      const startX = (c + borderSize) * cellPx;
      // Y軸反転: PGM画像のY座標を反転させる
      const startY = ((state.rows - 1 - r) + borderSize) * cellPx;
      
      // セルが侵入不可でなければ自由空間に
      if (!state.blockedCells[r][c]) {
        for (let y = startY; y < startY + cellPx; y++) {
          for (let x = startX; x < startX + cellPx; x++) {
            buf[y * W + x] = free;
          }
        }
      }
    }
  }
  
  const t = state.wallThicknessPx;
  const half = Math.floor((t - 1) / 2);
  
  // Draw horizontal edges (walls between rows)
  for (let r = 0; r <= state.rows; r++) {
    for (let c = 0; c < state.cols; c++) {
      if (state.hEdges[r][c]) {
        // Center the wall on the edge between cells
        // hEdges[r]のrはすでに上から下のインデックスなので反転なし
        const y0 = (r + borderSize) * cellPx;
        const x0 = (c + borderSize) * cellPx;
        for (let dy = -half; dy <= half; dy++) {
          const y = y0 + dy;
          if (y < 0 || y >= H) continue;
          for (let x = x0; x < x0 + cellPx; x++) {
            buf[y * W + x] = occ;
          }
        }
      }
    }
  }
  
  // Draw vertical edges (walls between columns)
  for (let xIdx = 0; xIdx <= state.cols; xIdx++) {
    for (let r = 0; r < state.rows; r++) {
      if (state.vEdges[xIdx][r]) {
        // Center the wall on the edge between cells
        const x0 = (xIdx + borderSize) * cellPx;
        // vEdges[xIdx][r]のrもすでに上から下のインデックスなので反転なし
        const y0 = (r + borderSize) * cellPx;
        for (let dx = -half; dx <= half; dx++) {
          const x = x0 + dx;
          if (x < 0 || x >= W) continue;
          for (let y = y0; y < y0 + cellPx; y++) {
            buf[y * W + x] = occ;
          }
        }
      }
    }
  }
  
  const header = `P5\n${W} ${H}\n255\n`;
  return new Blob(
    [new TextEncoder().encode(header), buf],
    { type: 'image/x-portable-graymap' }
  );
}

// Pure function to build ROS2 YAML
export function buildYamlROS2(state: State): string {
  const { ox, oy, theta } = computeOrigin(state);
  const res = state.cellSizeM / state.pixelsPerCell;
  
  return `image: map.pgm
mode: trinary
resolution: ${res}
origin: [${ox}, ${oy}, ${theta}]
negate: false
occupied_thresh: 0.65
free_thresh: 0.25
`;
}

// Pure function to build ROS1 YAML
export function buildYamlROS1(state: State): string {
  const { ox, oy, theta } = computeOrigin(state);
  const res = state.cellSizeM / state.pixelsPerCell;
  
  return `image: map.pgm
resolution: ${res}
origin: [${ox}, ${oy}, ${theta}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
`;
}

// Helper function for SDF box model (walls - white color)
function modelWall(name: string, pose: string, size: string): string {
  return `    <model name="${name}" static="true">
      <pose>${pose}</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>${size}</size></box></geometry></collision>
        <visual name="vis">
          <geometry><box><size>${size}</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.5 0.5 0.5 1</specular>
          </material>
        </visual>
      </link>
    </model>`;
}

// Helper function for SDF box model (floor - default gray)
function modelBox(name: string, pose: string, size: string): string {
  return `    <model name="${name}" static="true">
      <pose>${pose}</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>${size}</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>${size}</size></box></geometry></visual>
      </link>
    </model>`;
}

// Wall segment for merging
interface WallSegment {
  type: 'horizontal' | 'vertical';
  x: number;
  y: number;
  length: number;
  thickness: number;
  height: number;
}

// Pure function to merge continuous walls
function mergeWalls(state: State, thickness: number, height: number): WallSegment[] {
  const segments: WallSegment[] = [];
  const cellSize = state.cellSizeM;
  const oxC = (state.origin.col + 0.5) * cellSize;
  const oyC = (state.origin.row + 0.5) * cellSize;
  
  // Process horizontal walls - merge continuous segments
  for (let r = 0; r <= state.rows; r++) {
    let segmentStart = -1;
    for (let c = 0; c <= state.cols; c++) {
      if (c < state.cols && state.hEdges[r][c]) {
        if (segmentStart === -1) segmentStart = c;
      } else if (segmentStart !== -1) {
        // End of segment - align with cell boundaries
        const length = (c - segmentStart) * cellSize;
        const x = (segmentStart + (c - segmentStart) / 2) * cellSize - oxC;
        const y = r * cellSize - oyC; // Wall center at exact cell boundary
        segments.push({
          type: 'horizontal',
          x, y,
          length,
          thickness,
          height
        });
        segmentStart = -1;
      }
    }
    // Handle segment that extends to the end
    if (segmentStart !== -1) {
      const length = (state.cols - segmentStart) * cellSize;
      const x = (segmentStart + (state.cols - segmentStart) / 2) * cellSize - oxC;
      const y = r * cellSize - oyC;
      segments.push({
        type: 'horizontal',
        x, y,
        length,
        thickness,
        height
      });
    }
  }
  
  // Process vertical walls - merge continuous segments
  for (let xIdx = 0; xIdx <= state.cols; xIdx++) {
    let segmentStart = -1;
    for (let r = 0; r <= state.rows; r++) {
      if (r < state.rows && state.vEdges[xIdx][r]) {
        if (segmentStart === -1) segmentStart = r;
      } else if (segmentStart !== -1) {
        // End of segment - align with cell boundaries
        const length = (r - segmentStart) * cellSize;
        const x = xIdx * cellSize - oxC; // Wall center at exact cell boundary
        const y = (segmentStart + (r - segmentStart) / 2) * cellSize - oyC;
        segments.push({
          type: 'vertical',
          x, y,
          length,
          thickness,
          height
        });
        segmentStart = -1;
      }
    }
    // Handle segment that extends to the end
    if (segmentStart !== -1) {
      const length = (state.rows - segmentStart) * cellSize;
      const x = xIdx * cellSize - oxC;
      const y = (segmentStart + (state.rows - segmentStart) / 2) * cellSize - oyC;
      segments.push({
        type: 'vertical',
        x, y,
        length,
        thickness,
        height
      });
    }
  }
  
  return segments;
}

// Pure function to build SDF world
export function buildSdfWorld(state: State, wallHeight: number = 0.5, wallThickness: number = 0.03): string {
  const height = wallHeight; // m
  const thickness = wallThickness; // m (use user-specified thickness)
  const oxC = (state.origin.col + 0.5) * state.cellSizeM;
  const oyC = (state.origin.row + 0.5) * state.cellSizeM;
  const yaw90 = Math.PI / 2;
  
  const out: string[] = [];
  out.push('<sdf version="1.7">');
  out.push('  <world name="map_world">');
  
  // Always include default ground plane
  out.push('    <include><uri>model://ground_plane</uri></include>');
  
  // Get merged wall segments
  const wallSegments = mergeWalls(state, thickness, height);
  
  // Add merged wall segments
  wallSegments.forEach((segment, index) => {
    const yaw = segment.type === 'vertical' ? yaw90 : 0;
    const size = segment.type === 'horizontal' 
      ? `${segment.length} ${segment.thickness} ${segment.height}`
      : `${segment.thickness} ${segment.length} ${segment.height}`;
    
    out.push(modelWall(
      `wall_${segment.type}_${index}`,
      `${segment.x} ${segment.y} ${segment.height / 2} 0 0 ${yaw}`,
      size
    ));
  });
  
  out.push('  </world>');
  out.push('</sdf>');
  
  return out.join('\n');
}