// Assemble the app from the repository, so bs_pose_editor.html stays the only source of the UI.
//
//   node scripts/prepare.mjs            -> app/  (editor, 3D model, three.js served locally, no CDN)
//   node scripts/prepare.mjs --backend  -> also build/backend/ (Python bridge + calibration tools + UF2s)
import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, '..');           // ELECTRON_GUI
const REPO = path.resolve(ROOT, '..');
const APP = path.join(ROOT, 'app');
const THREE = path.join(ROOT, 'node_modules', 'three');
const CDN = 'https://cdn.jsdelivr.net/npm/three@0.160.0/';
// three.js files the editor imports (each of them only imports 'three' itself).
const THREE_FILES = [
  'build/three.module.js',
  'examples/jsm/controls/OrbitControls.js',
  'examples/jsm/controls/TransformControls.js',
  'examples/jsm/renderers/CSS2DRenderer.js',
  'examples/jsm/loaders/OBJLoader.js',
];
const MODEL = 'docs/valve-index-lighthousebasestation-gen2';

function copy(from, to) {
  fs.mkdirSync(path.dirname(to), { recursive: true });
  fs.cpSync(from, to, { recursive: true });
}

fs.rmSync(APP, { recursive: true, force: true });
let html = fs.readFileSync(path.join(REPO, 'bs_pose_editor.html'), 'utf8');
if (!html.includes(CDN)) throw new Error(`bs_pose_editor.html no longer imports three.js from ${CDN}: update prepare.mjs`);
const version = JSON.parse(fs.readFileSync(path.join(THREE, 'package.json'), 'utf8')).version;
if (!CDN.includes(`three@${version}/`)) throw new Error(`node_modules/three is ${version}, the editor uses ${CDN}`);
html = html.replaceAll(CDN, './vendor/three/');
for (const file of THREE_FILES) {
  if (!html.includes(file.startsWith('build/') ? file : path.posix.basename(file))) {
    throw new Error(`the editor does not import ${file} any more: update THREE_FILES`);
  }
}
const extra = [...html.matchAll(/three\/addons\/([\w/]+\.js)/g)].map(m => 'examples/jsm/' + m[1])
  .filter(f => !THREE_FILES.includes(f));
if (extra.length) throw new Error(`the editor imports three.js files not vendored: ${extra.join(', ')} — add them to THREE_FILES`);

fs.mkdirSync(APP, { recursive: true });
fs.writeFileSync(path.join(APP, 'bs_pose_editor.html'), html);
for (const file of THREE_FILES) copy(path.join(THREE, file), path.join(APP, 'vendor', 'three', file));
for (const sub of ['source/lh_basestation_valve_gen2.obj', 'textures/lh_basestation_valve_gen2.png']) {
  copy(path.join(REPO, MODEL, sub), path.join(APP, MODEL, sub));
}
// Start-up screen and logos (ELECTRON_GUI/ui, ELECTRON_GUI/assets).
copy(path.join(ROOT, 'ui', 'splash.html'), path.join(APP, 'splash.html'));
for (const name of fs.readdirSync(path.join(ROOT, 'assets'))) copy(path.join(ROOT, 'assets', name), path.join(APP, 'assets', name));
console.log(`app/ ready: editor + start-up screen + three.js ${version} (local) + base-station model`);

if (process.argv.includes('--backend')) {
  // Same file set as the Windows bundle (utils/make_windows_bundle.py FILES): bridge, Bitcraze
  // calibration tools, reference run, firmware UF2s + headers. Built without its zip / copies.
  const out = path.join(ROOT, 'build', 'backend');
  fs.rmSync(out, { recursive: true, force: true });
  const python = process.env.LH2_PYTHON || (process.platform === 'win32' ? 'py' : 'python3');
  const code = 'import sys; from pathlib import Path; sys.path.insert(0, sys.argv[1]); ' +
               'import make_windows_bundle as m; m.build(Path(sys.argv[2]))';
  execFileSync(python, [...(python === 'py' ? ['-3'] : []), '-c', code, path.join(REPO, 'utils'), out], { stdio: 'inherit' });
  // The desktop app has its own launcher and data folder: drop the browser-bundle helpers.
  for (const name of ['avvia.bat', 'LEGGIMI.txt', 'requirements.txt', 'bs_pose_editor.html', 'docs']) {
    fs.rmSync(path.join(out, name), { recursive: true, force: true });
  }
  console.log(`build/backend/ ready (${out})`);
}
