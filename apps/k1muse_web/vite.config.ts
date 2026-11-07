import { defineConfig } from "vite";
import { fileURLToPath } from "node:url";
export default defineConfig({
  base: "./",
  build: {
    outDir: "../../ros2_ws/src/k1muse_web/web",
    emptyOutDir: true,
    rollupOptions: { input: {
      console: fileURLToPath(new URL("./index.html", import.meta.url)),
      pharmacy: fileURLToPath(new URL("./pharmacy.html", import.meta.url))
    } }
  }
});
