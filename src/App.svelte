<script lang="ts">
  import { state, shareUrl, setGrid, setCellSize, clickCell, setOrigin } from './state';
  import { exportAll } from './export';

  let ros1 = true;
  let ros2 = true;
  let sdf = true;
  let wallHeight = 0.5;
  
  const gridSizes = [2, 3, 4, 5, 6, 7, 8, 9, 10];
  
  const onExport = async () => {
    await exportAll($state, { ros1, ros2, sdf, wallHeight });
  };
  
  // Calculate grid layout
  $: gridWidth = $state.cols * 64 + ($state.cols + 1) * 8;
  $: gridHeight = $state.rows * 64 + ($state.rows + 1) * 8;
  
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
        class="control-input"
        type="number" 
        step="0.01" 
        bind:value={$state.cellSizeM}
        on:change={(e) => setCellSize(parseFloat(e.currentTarget.value))}
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
      <div class="flex items-center gap-2 text-slate-300" class:opacity-50={!sdf}>
        <label class="text-xs">Wall Height</label>
        <input 
          class="control-input w-16 text-xs" 
          type="number" 
          step="0.1" 
          min="0.1"
          max="10"
          bind:value={wallHeight}
          disabled={!sdf}
        />
        <span class="text-xs text-slate-400">m</span>
      </div>
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
    <div class="grid-viewport animate-float-up">
      <div class="grid-container" style={`width: ${gridWidth}px; height: ${gridHeight}px;`}>
        
        <!-- Horizontal Walls -->
        {#each Array($state.rows + 1) as _, hRow}
          {#each Array($state.cols) as _, hCol}
            {@const isActive = $state.hEdges[hRow]?.[hCol] || false}
            <div
              class="wall wall-horizontal"
              class:active={isActive}
              style={`
                left: ${hCol * 72 + 8}px;
                top: ${hRow * 72}px;
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
            ></div>
          {/each}
        {/each}

        <!-- Vertical Walls -->
        {#each Array($state.rows) as _, vRow}
          {#each Array($state.cols + 1) as _, vCol}
            {@const isActive = $state.vEdges[vCol]?.[vRow] || false}
            <div
              class="wall wall-vertical"
              class:active={isActive}
              style={`
                left: ${vCol * 72}px;
                top: ${vRow * 72 + 8}px;
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
            ></div>
          {/each}
        {/each}

        <!-- Grid Cells -->
        {#each Array($state.rows) as _, cRow}
          {#each Array($state.cols) as _, cCol}
            {@const rB = $state.rows - 1 - cRow}
            {@const isOrigin = $state.origin.row === rB && $state.origin.col === cCol}
            {@const isBlocked = $state.blockedCells[rB]?.[cCol] || false}
            <div
              class="grid-cell"
              class:origin-cell={isOrigin}
              class:blocked={isBlocked}
              style={`
                left: ${cCol * 72 + 8}px;
                top: ${cRow * 72 + 8}px;
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
</div>

<style>
  .origin-cell {
    @apply ring-2 ring-emerald-400;
    box-shadow: 0 0 20px rgb(16 185 129 / 0.3);
  }
</style>