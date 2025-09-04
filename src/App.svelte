<script lang="ts">
  import { state, shareUrl, setGrid, setCellSize, setResolution, setWallThicknessM, clickCell, setOrigin, selectPaletteIndex, updatePaletteValue } from './state';
  import { exportAll } from './export';

  let ros1 = true;
  let ros2 = true;
  let sdf = true;
  let wallHeight = 0.5;
  let helpExpanded = false;
  
  const gridSizes = [2, 3, 4, 5, 6, 7, 8, 9, 10];
  
  // Direct binding approach
  
  // Color function with interpolation between key points
  function getCostColor(costValue) {
    const value = Math.max(0, Math.min(254, costValue));
    
    // 補間のための基準点
    const points = [
      { cost: 0, r: 255, g: 255, b: 255 },    // 白
      { cost: 30, r: 255, g: 217, b: 217 },   // 薄い赤
      { cost: 80, r: 255, g: 130, b: 130 },   // ピンク赤
      { cost: 150, r: 180, g: 0, b: 0 },      // 濃い赤
      { cost: 254, r: 139, g: 0, b: 0 }       // 最も濃い赤
    ];
    
    // 線形補間
    for (let i = 0; i < points.length - 1; i++) {
      if (value >= points[i].cost && value <= points[i + 1].cost) {
        const t = (value - points[i].cost) / (points[i + 1].cost - points[i].cost);
        const r = Math.round(points[i].r + (points[i + 1].r - points[i].r) * t);
        const g = Math.round(points[i].g + (points[i + 1].g - points[i].g) * t);
        const b = Math.round(points[i].b + (points[i + 1].b - points[i].b) * t);
        return `rgb(${r}, ${g}, ${b})`;
      }
    }
    
    // 254以上の場合
    return 'rgb(139, 0, 0)';
  }
  
  const onExport = async () => {
    await exportAll($state, { ros1, ros2, sdf, wallHeight });
  };
  
  // Calculate grid layout
  $: pixelsPerCell = Math.round($state.cellSizeM / $state.resolution);
  $: cellDisplayPx = Math.min(64, Math.max(32, pixelsPerCell)); // Clamp display size for UI
  $: gridWidth = $state.cols * 72 + 8;
  $: gridHeight = $state.rows * 72 + 8;
  
  function toggleWall(type, row, col) {
    state.update(s => {
      if (type === 'h') {
        const hEdges = s.hEdges.map(r => r.slice());
        hEdges[row][col] = !hEdges[row][col];
        return { ...s, hEdges };
      } else {
        const vEdges = s.vEdges.map(r => r.slice());
        vEdges[col][row] = !vEdges[col][row];
        return { ...s, vEdges };
      }
    });
  }
</script>

<div class="app-container">
  <!-- Header -->
  <header class="app-header">
    <div class="app-title">
      <span class="text-3xl">🗺️</span>
      <span class="text-gradient">ROS GridMap Generator</span>
    </div>
    <div class="app-version">v1.0</div>
  </header>

  <!-- Toolbar -->
  <div class="toolbar">
    <div class="tool-group">
      <label class="tool-label">Grid Size</label>
      <select 
        class="control-select" 
        value={$state.rows}
        on:change={(e) => setGrid(parseInt(e.currentTarget.value), $state.cols)}
      >
        {#each gridSizes as size}
          <option value={size}>{size}</option>
        {/each}
      </select>
      <span class="text-slate-400">×</span>
      <select 
        class="control-select"
        value={$state.cols}
        on:change={(e) => setGrid($state.rows, parseInt(e.currentTarget.value))}
      >
        {#each gridSizes as size}
          <option value={size}>{size}</option>
        {/each}
      </select>
    </div>

    <div class="tool-group">
      <label class="tool-label">Cell Size</label>
      <input 
        class="control-input w-16"
        type="text" 
        bind:value={$state.cellSizeM}
        on:change={(e) => setCellSize(parseFloat(e.currentTarget.value))}
      />
      <span class="text-slate-400">m</span>
    </div>

    <div class="tool-group">
      <label class="tool-label">Resolution</label>
      <input 
        class="control-input w-20"
        type="text" 
        bind:value={$state.resolution}
        on:change={(e) => setResolution(parseFloat(e.currentTarget.value))}
      />
      <span class="text-slate-400">m/px</span>
    </div>

    <div class="tool-group">
      <label class="tool-label">Wall Thickness</label>
      <input 
        class="control-input w-16"
        type="text" 
        bind:value={$state.wallThicknessM}
        on:change={(e) => setWallThicknessM(parseFloat(e.currentTarget.value))}
      />
      <span class="text-slate-400">m</span>
    </div>

    <div class="tool-group">
      <label class="tool-label">Wall Height</label>
      <input 
        class="control-input w-16" 
        type="text" 
        bind:value={wallHeight}
      />
      <span class="text-slate-400">m</span>
    </div>

    <div class="tool-group">
      <label class="tool-label">Export</label>
      <label class="flex items-center gap-2 text-slate-300">
        <input class="control-checkbox" type="checkbox" bind:checked={ros1} />
        ROS1
      </label>
      <label class="flex items-center gap-2 text-slate-300">
        <input class="control-checkbox" type="checkbox" bind:checked={ros2} />
        ROS2
      </label>
      <label class="flex items-center gap-2 text-slate-300">
        <input class="control-checkbox" type="checkbox" bind:checked={sdf} />
        SDF
      </label>
    </div>

    <div class="tool-group ml-auto">
      <button class="btn btn-primary" on:click={onExport}>
        <span>📥</span>
        Export Files
      </button>
      <button class="btn btn-secondary" on:click={() => setGrid($state.rows, $state.cols)}>
        <span>🔄</span>
        Clear Grid
      </button>
    </div>
  </div>

  <!-- Main Content -->
  <main class="main-content">
    <div class="content-layout">
      <div class="grid-viewport animate-float-up">
        <div class="grid-container" style={`width: ${gridWidth}px; height: ${gridHeight}px;`}>
        
        <!-- Horizontal Walls -->
        {#each Array($state.rows + 1) as _, hRow}
          {#each Array($state.cols) as _, hCol}
            {@const isActive = $state.hEdges[hRow]?.[hCol] || false}
            <div
              class="wall-hitbox wall-hitbox-horizontal"
              style={`
                left: ${hCol * 72 + 4}px;
                top: ${hRow * 72 - 8}px;
              `}
              role="button"
              tabindex="0"
              on:click={() => toggleWall('h', hRow, hCol)}
              on:keydown={(e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault();
                  toggleWall('h', hRow, hCol);
                }
              }}
            >
              <div
                class="wall wall-horizontal"
                class:active={isActive}
                title={`H-Wall: row=${hRow}, col=${hCol}, top=${hRow * 72}px`}
              ></div>
            </div>
          {/each}
        {/each}

        <!-- Vertical Walls -->
        {#each Array($state.rows) as _, vRow}
          {#each Array($state.cols + 1) as _, vCol}
            {@const isActive = $state.vEdges[vCol]?.[vRow] || false}
            <div
              class="wall-hitbox wall-hitbox-vertical"
              style={`
                left: ${vCol * 72 - 8}px;
                top: ${vRow * 72 + 4}px;
              `}
              role="button"
              tabindex="0"
              on:click={() => toggleWall('v', vRow, vCol)}
              on:keydown={(e) => {
                if (e.key === 'Enter' || e.key === ' ') {
                  e.preventDefault();
                  toggleWall('v', vRow, vCol);
                }
              }}
            >
              <div
                class="wall wall-vertical"
                class:active={isActive}
                title={`V-Wall: row=${vRow}, col=${vCol}, top=${vRow * 72 + 8}px`}
              ></div>
            </div>
          {/each}
        {/each}

        <!-- Grid Cells -->
        {#each Array($state.rows) as _, cRow}
          {#each Array($state.cols) as _, cCol}
            {@const rB = $state.rows - 1 - cRow}
            {@const isOrigin = $state.origin.row === rB && $state.origin.col === cCol}
            {@const costIndex = $state.cellCostIndices[rB]?.[cCol]}
            {@const hasCustomCost = costIndex !== null}
            {@const costValue = hasCustomCost ? $state.cellCostPalette[costIndex] : 254}
            {@const cellColor = getCostColor(costValue)}
            <div
              class="grid-cell"
              class:origin-cell={isOrigin}
              class:has-cost={hasCustomCost}
              style={`
                left: ${cCol * 72 + 8}px;
                top: ${cRow * 72 + 8}px;
                ${hasCustomCost ? `background-color: ${cellColor}; border-color: ${cellColor};` : ''}
              `}
            >
              <div 
                class="cell-click-area"
                role="button"
                tabindex="0"
                on:click={(e) => {
                  if (e.shiftKey) {
                    setOrigin(cRow, cCol);
                  } else {
                    clickCell(cRow, cCol);
                  }
                }}
                on:keydown={(e) => {
                  if (e.key === 'Enter' || e.key === ' ') {
                    e.preventDefault();
                    if (e.shiftKey) {
                      setOrigin(cRow, cCol);
                    } else {
                      clickCell(cRow, cCol);
                    }
                  }
                }}
              >
                {#if isOrigin}
                  <div class="origin">
                    <div class="origin-arrow">
                      {{0: '→', 90: '↑', 180: '←', 270: '↓'}[$state.origin.thetaDeg]}
                    </div>
                  </div>
                {/if}
              </div>
            </div>
          {/each}
        {/each}
        
        </div>
      </div>

      <!-- Cost Palette -->
      <div class="cost-palette">
        <h3 class="palette-title">Cost Palette</h3>
        <div class="palette-grid">
          {#each $state.cellCostPalette as cost, index (index)}
            {@const isSelected = $state.selectedPaletteIndex === index}
            <div 
              class="palette-item"
              class:selected={isSelected}
              class:cost-0={index === 0}
              class:cost-1={index === 1}
              class:cost-2={index === 2}
              class:cost-3={index === 3}
              on:click={() => selectPaletteIndex(index)}
              role="button"
              tabindex="0"
            >
              <div class="palette-preview" style="background-color: {getCostColor(cost)}"></div>
              <div class="palette-info">
                <input 
                  class="palette-input"
                  type="number"
                  min="0"
                  max="254"
                  bind:value={$state.cellCostPalette[index]}
                  on:focus={() => selectPaletteIndex(index)}
                />
              </div>
            </div>
          {/each}
        </div>
        <div class="palette-help">
          <p class="text-xs text-slate-400">Click to select, then click cells to apply cost</p>
        </div>
      </div>
    </div>
  </main>

  <!-- Status Bar -->
  <footer class="status-bar">
    <span class="status-label">Share URL</span>
    <input class="status-url" readonly bind:value={$shareUrl} />
    <button 
      class="btn btn-ghost"
      on:click={async () => {
        try {
          await navigator.clipboard.writeText($shareUrl);
          // Could add toast notification here
        } catch {
          // Handle error gracefully
        }
      }}
    >
      <span>📋</span>
      Copy
    </button>
  </footer>

  <!-- Help Section -->
  <div class="help-section">
    <button 
      class="help-toggle"
      on:click={() => helpExpanded = !helpExpanded}
    >
      <span class="help-toggle-icon" class:expanded={helpExpanded}>❓</span>
      <span>How to Use</span>
      <span class="help-expand-icon" class:expanded={helpExpanded}>▼</span>
    </button>
    
    {#if helpExpanded}
      <div class="help-content animate-float-up">
        <div class="help-grid">
          <div class="help-item">
            <span class="help-icon">🧱</span>
            <div class="help-text">
              <strong>Add/Remove Walls</strong>
              <p>Click on cell borders (gray lines) to toggle walls between cells</p>
            </div>
          </div>
          
          <div class="help-item">
            <span class="help-icon">🎨</span>
            <div class="help-text">
              <strong>Set Cell Costs</strong>
              <p>Select a cost palette (right panel), then click cells to apply cost. Colors: White=Free, Red=High Cost</p>
            </div>
          </div>
          
          <div class="help-item">
            <span class="help-icon">🎯</span>
            <div class="help-text">
              <strong>Set Origin Point</strong>
              <p>Shift+Click on a cell to set origin (green arrow). Shift+Click again to rotate (0°→90°→180°→270°)</p>
            </div>
          </div>
          
          <div class="help-item">
            <span class="help-icon">🎛️</span>
            <div class="help-text">
              <strong>Cost Palette</strong>
              <p>Customize 4 cost values (0-254) in right panel. Lower values = safer paths, Higher values = costly paths</p>
            </div>
          </div>
          
          <div class="help-item">
            <span class="help-icon">🔄</span>
            <div class="help-text">
              <strong>Cycle Cell Costs</strong>
              <p>Click same cell multiple times to cycle through cost palette values (0→30→80→150→0)</p>
            </div>
          </div>
          
          <div class="help-item">
            <span class="help-icon">📥</span>
            <div class="help-text">
              <strong>Export Maps</strong>
              <p>Choose format (ROS1/ROS2/SDF), set wall properties, then click Export Files to download ZIP</p>
            </div>
          </div>
        </div>
      </div>
    {/if}
  </div>
</div>

<style>
  .origin-cell {
    @apply ring-2 ring-emerald-400;
    box-shadow: 0 0 20px rgb(16 185 129 / 0.3);
  }
</style>