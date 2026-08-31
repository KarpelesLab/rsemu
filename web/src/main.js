// The only script tag on the page. Everything else is a module Vite bundles
// in — including `rsemu.js`, which stays plain ESM so `check.mjs` can import
// the same file under node without a build step.

import { createApp } from "vue";
import App from "./App.vue";
import "./styles.css";

createApp(App).mount("#app");
