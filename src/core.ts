// Pure business logic - Core layer (FDDD)
// All functions are pure with no side effects

export type Deg = 0 | 90 | 180 | 270;

export interface State {
  version: 4;
  rows: number;
  cols: number;
  cellSizeM: number;
  resolution: number; // meters per pixel
  wallThicknessM: number; // configurable wall thickness in meters
  floorFriction: number; // friction coefficient for floor (0.0+, typical: 0.8-1.5)
  hEdges: boolean[][];
  vEdges: boolean[][];
  // Cost palette system
  cellCostPalette: [number, number, number, number]; // 4 cost values (0-254)
  cellCostIndices: (number | null)[][]; // Each cell references palette index (0-3) or null
  selectedPaletteIndex: number; // Currently selected palette index (0-3)
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
  const cellCostIndices = Array.from({ length: rows }, () => 
    Array(cols).fill(0)  // Initialize all cells with palette index 0 (first palette)
  );
  
  return {
    version: 4,
    rows,
    cols,
    cellSizeM: 0.5,
    resolution: 0.03125, // default: 0.5m / 16px = 0.03125 m/px
    wallThicknessM: 0.03, // 30mm default
    floorFriction: 0.8, // default floor friction coefficient
    hEdges,
    vEdges,
    cellCostPalette: [0, 30, 80, 150], // Low cost (free), Medium low, Medium high, High cost
    cellCostIndices,
    selectedPaletteIndex: 0,
    origin: { row: rows - 1, col: cols - 1, thetaDeg: 0 }
  };
}


// Pure function to compute origin transformation
export function computeOrigin(state: State): { ox: number; oy: number; theta: number } {
  const ppc = state.cellSizeM / state.resolution;
  const px = state.origin.col * ppc + ppc / 2;
  const py = state.origin.row * ppc + ppc / 2;
  const rad = (state.origin.thetaDeg * Math.PI) / 180;
  const cos = Math.cos(rad);
  const sin = Math.sin(rad);
  
  const ox = -(cos * px - sin * py) * state.resolution;
  const oy = -(sin * px + cos * py) * state.resolution;
  
  return { ox, oy, theta: rad };
}

// Pure function to build PGM image data
export function buildPGM(state: State): Blob {
  const borderSize = 2; // 境界の厚さ（セル単位）
  const cellPx = state.cellSizeM / state.resolution;
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
      
      // セルのコスト値を取得（パレットインデックスから）
      const costIndex = state.cellCostIndices[r][c];
      const costValue = costIndex !== null ? state.cellCostPalette[costIndex] : 0; // Default to cost 0 (free)
      // Convert cost to PGM pixel: cost 0 (free) -> PGM 254 (white), cost 254 (occupied) -> PGM 0 (black)
      const cellCost = 254 - costValue;
      
      for (let y = startY; y < startY + cellPx; y++) {
        for (let x = startX; x < startX + cellPx; x++) {
          buf[y * W + x] = cellCost;
        }
      }
    }
  }
  
  const wallThicknessPx = state.wallThicknessM / state.resolution;
  
  // Draw horizontal edges (walls between rows)
  for (let r = 0; r <= state.rows; r++) {
    for (let c = 0; c < state.cols; c++) {
      if (state.hEdges[r][c]) {
        // Center the wall on the edge between cells
        // hEdges[r]のrはすでに上から下のインデックスなので反転なし
        const y0 = (r + borderSize) * cellPx; // Wall center position
        const x0 = (c + borderSize) * cellPx;
        
        // Calculate exact wall boundaries and expand fractional pixels
        const wallHalfThickness = wallThicknessPx / 2;
        const yStart = Math.floor(y0 - wallHalfThickness);
        const yEnd = Math.ceil(y0 + wallHalfThickness);
        
        for (let y = yStart; y < yEnd; y++) {
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
        const x0 = (xIdx + borderSize) * cellPx; // Wall center position
        // vEdges[xIdx][r]のrもすでに上から下のインデックスなので反転なし
        const y0 = (r + borderSize) * cellPx;
        
        // Calculate exact wall boundaries and expand fractional pixels
        const wallHalfThickness = wallThicknessPx / 2;
        const xStart = Math.floor(x0 - wallHalfThickness);
        const xEnd = Math.ceil(x0 + wallHalfThickness);
        
        for (let x = xStart; x < xEnd; x++) {
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
  
  return `image: map.pgm
mode: trinary
resolution: ${state.resolution}
origin: [${ox}, ${oy}, ${theta}]
negate: false
occupied_thresh: 0.65
free_thresh: 0.25
`;
}

// Pure function to build ROS1 YAML
export function buildYamlROS1(state: State): string {
  const { ox, oy, theta } = computeOrigin(state);
  
  return `image: map.pgm
resolution: ${state.resolution}
origin: [${ox}, ${oy}, ${theta}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
`;
}

// Helper function for SDF box model (walls - white color with friction)
function modelWall(name: string, pose: string, size: string): string {
  return `    <model name="${name}" static="true">
      <pose>${pose}</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>${size}</size></box></geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
            <bounce>
              <restitution_coefficient>0.0</restitution_coefficient>
              <threshold>1000.0</threshold>
            </bounce>
            <contact>
              <ode>
                <kp>10000000.0</kp>
                <kd>1.0</kd>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="vis">
          <geometry><box><size>${size}</size></box></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>1 1 1 1</specular>
            <emissive>0 0 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>`;
}

// Helper function for SDF floor model (gray with configurable friction)
function modelFloor(name: string, pose: string, size: string, friction: number = 0.8): string {
  return `    <model name="${name}" static="true">
      <pose>${pose}</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>${size}</size></box></geometry>
          <surface>
            <friction>
              <ode>
                <mu>${friction}</mu>
                <mu2>${friction}</mu2>
              </ode>
            </friction>
            <bounce>
              <restitution_coefficient>0.1</restitution_coefficient>
              <threshold>1000.0</threshold>
            </bounce>
            <contact>
              <ode>
                <kp>10000000.0</kp>
                <kd>1.0</kd>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="vis">
          <geometry><box><size>${size}</size></box></geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>0 0 0 1</emissive>
          </material>
        </visual>
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
  const oyC = ((state.rows - 1 - state.origin.row) + 0.5) * cellSize;
  
  // Process horizontal walls - merge continuous segments
  for (let r = 0; r <= state.rows; r++) {
    let segmentStart = -1;
    for (let c = 0; c <= state.cols; c++) {
      if (c < state.cols && state.hEdges[r][c]) {
        if (segmentStart === -1) segmentStart = c;
      } else if (segmentStart !== -1) {
        // End of segment - create wall with accurate endpoints and thickness margin
        const segmentEnd = c - 1; // Last column index in segment
        const baseLength = (segmentEnd - segmentStart + 1) * cellSize; // Exact segment length
        const length = baseLength + thickness; // Add thickness margin (thickness/2 * 2)
        
        // Position: center between segment start and end cells
        const startX = segmentStart * cellSize;
        const endX = (segmentEnd + 1) * cellSize;
        const x = (startX + endX) / 2 - oxC; // Center of segment span
        const y = (state.rows - r) * cellSize - oyC; // Wall at row boundary (Y-axis inverted)
        
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
      const segmentEnd = state.cols - 1; // Last column index
      const baseLength = (segmentEnd - segmentStart + 1) * cellSize;
      const length = baseLength + thickness; // Add thickness margin
      
      const startX = segmentStart * cellSize;
      const endX = (segmentEnd + 1) * cellSize;
      const x = (startX + endX) / 2 - oxC;
      const y = (state.rows - r) * cellSize - oyC;
      
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
        // End of segment - create wall with accurate endpoints and thickness margin
        const segmentEnd = r - 1; // Last row index in segment
        const baseLength = (segmentEnd - segmentStart + 1) * cellSize; // Exact segment length
        const length = baseLength + thickness; // Add thickness margin (thickness/2 * 2)
        
        // Position: center between segment start and end cells
        const startY = (state.rows - segmentEnd - 1) * cellSize;
        const endY = (state.rows - segmentStart) * cellSize;
        const x = xIdx * cellSize - oxC; // Wall at column boundary
        const y = (startY + endY) / 2 - oyC; // Center of segment span (Y-axis inverted)
        
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
      const segmentEnd = state.rows - 1; // Last row index
      const baseLength = (segmentEnd - segmentStart + 1) * cellSize;
      const length = baseLength + thickness; // Add thickness margin
      
      const startY = (state.rows - segmentEnd - 1) * cellSize;
      const endY = (state.rows - segmentStart) * cellSize;
      const x = xIdx * cellSize - oxC;
      const y = (startY + endY) / 2 - oyC;
      
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
  
  const out: string[] = [];
  out.push('<sdf version="1.7">');
  out.push('  <world name="map_world">');
  
  // Physics settings
  out.push('    <physics name="default_physics" default="1" type="ode">');
  out.push('      <gravity>0 0 -9.8066</gravity>');
  out.push('      <ode>');
  out.push('        <solver>');
  out.push('          <type>quick</type>');
  out.push('          <iters>150</iters>');
  out.push('          <sor>1.4</sor>');
  out.push('        </solver>');
  out.push('        <constraints>');
  out.push('          <cfm>1e-5</cfm>');
  out.push('          <erp>0.2</erp>');
  out.push('          <contact_max_correcting_vel>2000.0</contact_max_correcting_vel>');
  out.push('          <contact_surface_layer>0.01</contact_surface_layer>');
  out.push('        </constraints>');
  out.push('      </ode>');
  out.push('      <max_step_size>0.004</max_step_size>');
  out.push('      <real_time_factor>1.0</real_time_factor>');
  out.push('      <real_time_update_rate>250</real_time_update_rate>');
  out.push('    </physics>');
  out.push('');
  
  // Ambient lighting only (no shadows for performance)
  out.push('    <scene>');
  out.push('      <ambient>0.8 0.8 0.8 1.0</ambient>');
  out.push('      <background>0.7 0.7 0.7 1.0</background>');
  out.push('      <shadows>false</shadows>');
  out.push('    </scene>');
  out.push('');
  
  // Custom floor instead of ground_plane
  const mapWidth = state.cols * state.cellSizeM;
  const mapHeight = state.rows * state.cellSizeM;
  const floorThickness = 0.1; // 10cm thick floor for stability
  const floorSize = `${mapWidth + 1} ${mapHeight + 1} ${floorThickness}`;
  const floorPose = `0 0 ${-floorThickness / 2} 0 0 0`; // Floor surface at z=0
  
  out.push(modelFloor('custom_floor', floorPose, floorSize, state.floorFriction));
  out.push('');
  
  // Get merged wall segments
  const wallSegments = mergeWalls(state, thickness, height);
  
  // Add merged wall segments
  wallSegments.forEach((segment, index) => {
    let yaw, size;
    
    if (segment.type === 'horizontal') {
      // Horizontal walls: extend along X-axis, thin along Y-axis
      yaw = 0; // No rotation needed
      size = `${segment.length} ${segment.thickness} ${segment.height}`;
    } else {
      // Vertical walls: extend along Y-axis, thin along X-axis  
      yaw = 0; // No rotation - use size to define orientation
      size = `${segment.thickness} ${segment.length} ${segment.height}`;
    }
    
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