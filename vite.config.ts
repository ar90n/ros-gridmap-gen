/// <reference types="vite/client" />
import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';

export default defineConfig({
  plugins: [svelte()],
  base: process.env.NODE_ENV === 'production' ? '/ros-gridmap-gen/' : '/',
  build: {
    outDir: 'dist',
    assetsDir: 'assets'
  },
  // @ts-ignore - Vitest configuration
  test: {
    globals: true,
    environment: 'jsdom'
  }
});