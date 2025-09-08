/// <reference types="vite/client" />
import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';
import { readFileSync } from 'fs';

const pkg = JSON.parse(readFileSync('./package.json', 'utf-8'));

export default defineConfig({
  plugins: [svelte()],
  base: './',
  root: process.cwd(),
  define: {
    __APP_VERSION__: JSON.stringify(pkg.version)
  },
  build: {
    rollupOptions: {
      input: './index.html'
    },
    outDir: 'dist',
    assetsDir: 'assets'
  },
  // @ts-ignore - Vitest configuration
  test: {
    globals: true,
    environment: 'jsdom'
  }
});