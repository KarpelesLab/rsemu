#!/usr/bin/env node
// Assert every installed npm package is permissively licensed.
//
//   node scripts/licenses.mjs [--list]
//
// `CLAUDE.md`'s provenance rule is about the Rust crate, but the reason behind
// it does not stop at the language boundary: rsemu is MIT, this page is part of
// rsemu, and the built site ships whatever is in `node_modules` inside its
// bundle. A copyleft package reaching `dist/` would be a licence violation on
// the published site, so it is checked rather than assumed.
//
// This reads the `license` field of every package.json under node_modules. It
// is not a substitute for reading a LICENSE file when adding a dependency — it
// is the thing that notices when a *transitive* dependency changes underneath.

import { readdirSync, readFileSync, existsSync } from "node:fs";
import { join } from "node:path";

/** SPDX identifiers we are allowed to bundle. Anything else is a decision. */
const ALLOWED = new Set([
  "MIT",
  "MIT-0",
  "ISC",
  "BSD-2-Clause",
  "BSD-3-Clause",
  "Apache-2.0",
  "0BSD",
  "Unlicense",
  "CC0-1.0",
  "BlueOak-1.0.0",
  "Python-2.0",
]);

/** Split `(MIT OR Apache-2.0)` and friends into the identifiers it offers. */
function terms(spdx) {
  return spdx
    .replace(/[()]/g, " ")
    .split(/\s+(?:OR|AND|WITH)\s+/i)
    .map((t) => t.trim())
    .filter(Boolean);
}

const packages = [];
function walk(dir) {
  if (!existsSync(dir)) return;
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    if (!entry.isDirectory() || entry.name === ".bin") continue;
    const path = join(dir, entry.name);
    // A scope directory (@vue) holds packages rather than being one.
    if (entry.name.startsWith("@")) {
      walk(path);
      continue;
    }
    const manifest = join(path, "package.json");
    if (existsSync(manifest)) {
      const json = JSON.parse(readFileSync(manifest, "utf8"));
      const license =
        json.license ??
        (Array.isArray(json.licenses) ? json.licenses.map((l) => l.type).join(" OR ") : null);
      packages.push({ name: json.name ?? entry.name, version: json.version, license });
    }
    walk(join(path, "node_modules"));
  }
}
walk(new URL("../node_modules", import.meta.url).pathname);

if (packages.length === 0) {
  console.error("no packages found — run `npm ci` first");
  process.exit(1);
}

packages.sort((a, b) => a.name.localeCompare(b.name));
const bad = [];
for (const p of packages) {
  const okay = p.license && terms(p.license).some((t) => ALLOWED.has(t));
  if (!okay) bad.push(p);
  if (process.argv.includes("--list") || !okay) {
    console.log(`  ${okay ? "ok  " : "BAD "} ${(p.license ?? "(none)").padEnd(16)} ${p.name}@${p.version}`);
  }
}

console.log(`${packages.length} packages, ${bad.length} not permissively licensed`);
if (bad.length > 0) {
  console.error("a non-permissive package would ship inside dist/ — see CLAUDE.md");
  process.exit(1);
}
