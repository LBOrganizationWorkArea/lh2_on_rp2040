// LightHouse Studio — Electron main process.
// Starts the Python backend (bridge), then shows bs_pose_editor.html from app/ through the
// privileged app:// scheme (ES modules, fetch and localStorage work as on http).
'use strict';

const { app, BrowserWindow, Menu, dialog, net, protocol, shell } = require('electron');
const fs = require('fs');
const path = require('path');
const { pathToFileURL } = require('url');
const { startBackend, stopBackend } = require('./backend');

const APP_DIR = path.join(__dirname, '..', 'app');  // filled by scripts/prepare.mjs
const EDITOR = 'bs_pose_editor.html';
const SPLASH_MIN_MS = 8000;  // the start-up screen stays at least this long, even if the backend is faster

protocol.registerSchemesAsPrivileged([
  { scheme: 'app', privileges: { standard: true, secure: true, supportFetchAPI: true, corsEnabled: true } },
]);

let win = null;
let backend = null;
const userDir = () => path.join(app.getPath('documents'), 'LH2');
// Installed app: a writable user folder. Development (npm start): the repository's calib_runs, as before.
const runsDir = () => app.isPackaged ? path.join(userDir(), 'calib_runs')
  : path.join(__dirname, '..', '..', 'utils', 'calibration', 'calib_runs');
const logFile = () => path.join(app.getPath('userData'), 'logs', 'backend.log');

// Calibration runs shipped with the app (the reference run used by the simulation) are copied
// once into the user's folder; runs already there are never touched.
function seedRuns() {
  const shipped = app.isPackaged
    ? path.join(process.resourcesPath, 'backend', 'utils', 'calibration', 'calib_runs')
    : path.join(__dirname, '..', '..', 'utils', 'calibration', 'calib_runs');
  if (!app.isPackaged) return;
  fs.mkdirSync(runsDir(), { recursive: true });
  if (!fs.existsSync(shipped)) return;
  for (const name of fs.readdirSync(shipped)) {
    const target = path.join(runsDir(), name);
    if (!fs.existsSync(target)) fs.cpSync(path.join(shipped, name), target, { recursive: true });
  }
}

function page(title, body) {
  return 'data:text/html;charset=utf-8,' + encodeURIComponent(`<!doctype html><meta charset="utf-8"><title>${title}</title>
<style>body{background:#181818;color:#ddd;font:15px system-ui,'Segoe UI',sans-serif;display:flex;align-items:center;justify-content:center;height:100vh;margin:0}
.box{max-width:720px;padding:24px}h1{font-size:20px;color:#fff}pre{background:#111;border:1px solid #333;padding:10px;white-space:pre-wrap;font-size:12px;max-height:50vh;overflow:auto}
.spin{width:28px;height:28px;border:3px solid #2f5f8a;border-top-color:#8adcf0;border-radius:50%;animation:s 1s linear infinite;margin-bottom:14px}@keyframes s{to{transform:rotate(360deg)}}</style>
<div class="box">${body}</div>`);
}

function buildMenu() {
  const template = [
    {
      label: 'File',
      submenu: [
        { label: 'Open calibration sessions folder', click: () => shell.openPath(runsDir()) },
        { label: 'Open firmware folder (UF2)', enabled: app.isPackaged,
          click: () => shell.openPath(path.join(process.resourcesPath, 'backend', 'firmware')) },
        { type: 'separator' },
        { role: 'quit' },
      ],
    },
    { label: 'View', submenu: [{ role: 'reload' }, { role: 'togglefullscreen' }, { type: 'separator' },
      { role: 'resetZoom' }, { role: 'zoomIn' }, { role: 'zoomOut' }] },
    {
      label: 'Help',
      submenu: [
        { label: 'Backend log', click: () => shell.openPath(logFile()) },
        { label: 'Restart backend', click: () => restartBackend() },
        { role: 'toggleDevTools' },
        { type: 'separator' },
        { label: 'About LightHouse Studio', click: () => dialog.showMessageBox(win, {
          type: 'info', title: 'LightHouse Studio',
          message: `LightHouse Studio ${app.getVersion()}`,
          detail: `Electron ${process.versions.electron} · backend: ${backend ? `${backend.source}, ${backend.url}` : 'not running'}\n` +
                  `Sessions: ${runsDir()}\nLog: ${logFile()}`,
        }) },
      ],
    },
  ];
  Menu.setApplicationMenu(Menu.buildFromTemplate(template));
}

async function launch() {
  await win.loadURL(`app://lh2/splash.html?v=${encodeURIComponent('v' + app.getVersion())}`);  // ui/splash.html
  const started = Date.now();
  let lastLine = '', status = 'Starting the backend…';
  const show = (line = lastLine) => {
    lastLine = line;
    if (!win || win.isDestroyed()) return;
    const args = [status, Math.round((Date.now() - started) / 1000), line.slice(0, 300)].map(v => JSON.stringify(v)).join(',');
    win.webContents.executeJavaScript(`window.splash && window.splash(${args})`).catch(() => {});
  };
  const ticker = setInterval(() => show(), 1000);  // the elapsed time keeps moving between log lines
  try {
    seedRuns();
    backend = await startBackend({ packaged: app.isPackaged, resourcesPath: process.resourcesPath,
                                   runsDir: runsDir(), logFile: logFile(), onLine: show });
    backend.process.on('exit', code => {
      if (!app.isQuitting && win && !win.isDestroyed()) {
        dialog.showMessageBox(win, { type: 'error', title: 'LightHouse Studio',
          message: `The backend stopped (code ${code}).`, detail: `Help → Restart backend, or see ${logFile()}` });
      }
    });
    // Check the backend answers before showing the editor (the log explains it if not).
    status = 'Checking the backend…';
    show('');
    const probe = await net.fetch(`${backend.url}/api/lh2/status`, { method: 'POST', signal: AbortSignal.timeout(30000) });
    if (!probe.ok) throw new Error(`the backend answered ${probe.status} to the first status request`);
    await probe.json();
    status = 'Backend ready — opening the editor…';
    show('');
    await new Promise(r => setTimeout(r, Math.max(0, SPLASH_MIN_MS - (Date.now() - started))));
    clearInterval(ticker);
    win.loadURL(`app://lh2/${EDITOR}?bridge=${encodeURIComponent('app://lh2')}`);
  } catch (error) {
    clearInterval(ticker);
    const esc = s => String(s).replace(/[&<>]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
    let tail = '';
    try { tail = fs.readFileSync(logFile(), 'utf8').slice(-3000); } catch (_e) { /* no log yet */ }
    win.loadURL(page('LightHouse Studio — error', `<h1>The backend could not start</h1>${esc(error.message.split('\n')[0])}
      <pre>${esc(error.message.split('\n').slice(1).join('\n'))}</pre>Backend log (${esc(logFile())}):<pre>${esc(tail)}</pre>
      Help → Restart backend to try again.<br><br><span style="color:#7d8792">LightHouse Studio — made by Davide Lentini &amp; Eduardo Gonzalez</span>`));
  }
}

async function restartBackend() {
  if (backend) { stopBackend(backend.process); backend = null; }
  await new Promise(r => setTimeout(r, 2000));  // let the UDP port go
  launch();
}

if (!app.requestSingleInstanceLock()) {
  app.quit();  // a second copy would fight the first one for the drone's UDP port
} else {
  app.on('second-instance', () => { if (win) { if (win.isMinimized()) win.restore(); win.focus(); } });
  app.whenReady().then(() => {
    protocol.handle('app', async request => {
      const url = new URL(request.url);
      // The page talks to the backend through its own origin (app://lh2/api/...): the app forwards
      // the call, so no cross-origin / private-network rule can block it.
      if (url.pathname.startsWith('/api/')) {
        if (!backend) return new Response(JSON.stringify({ error: 'backend not running' }), { status: 503 });
        const body = ['GET', 'HEAD'].includes(request.method) ? undefined : await request.arrayBuffer();
        try {
          return await net.fetch(backend.url + url.pathname + url.search, { method: request.method, body });
        } catch (error) {
          return new Response(JSON.stringify({ error: `backend unreachable: ${error.message}` }), { status: 502 });
        }
      }
      const rel = decodeURIComponent(url.pathname).replace(/^\/+/, '');
      const file = path.normalize(path.join(APP_DIR, rel));
      if (!file.startsWith(APP_DIR)) return new Response('forbidden', { status: 403 });
      return net.fetch(pathToFileURL(file).toString());
    });
    buildMenu();
    win = new BrowserWindow({
      width: 1500, height: 920, minWidth: 900, minHeight: 600, backgroundColor: '#181818',
      title: 'LightHouse Studio', show: true, icon: path.join(APP_DIR, 'assets', 'LB.ico'),
      webPreferences: { contextIsolation: true, nodeIntegration: false, sandbox: true },
    });
    // Links to the outside world open in the normal browser, never inside the app.
    win.webContents.setWindowOpenHandler(({ url }) => { shell.openExternal(url); return { action: 'deny' }; });
    launch();
  });
  app.on('before-quit', () => { app.isQuitting = true; if (backend) stopBackend(backend.process); });
  app.on('window-all-closed', () => app.quit());
}
