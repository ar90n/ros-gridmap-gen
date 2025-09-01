/// <reference types="vite/client" />
import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

export default defineConfig({
  plugins: [svelte()],
  base: './',
  root: process.cwd(),
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