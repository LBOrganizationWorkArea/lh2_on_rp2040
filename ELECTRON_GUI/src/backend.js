// Python backend of LH2 Studio: the same bridge the browser editor uses
// (utils/user_interface/display_real_time_windows.py), started and stopped by the app.
//
//  - packaged app: resources/python/python.exe (embedded Python) + resources/backend/utils/...
//  - development:  LH2_PYTHON, else the repository .venv, else python3 / py, + the repository scripts
'use strict';

const { spawn } = require('child_process');
const fs = require('fs');
const net = require('net');
const path = require('path');

const REPO = path.resolve(__dirname, '..', '..');
const BRIDGE_REL = path.join('utils', 'user_interface', 'display_real_time_windows.py');

function locate(packaged, resourcesPath) {
  if (packaged) {
    const python = path.join(resourcesPath, 'python', process.platform === 'win32' ? 'python.exe' : 'bin/python3');
    return { python, bridge: path.join(resourcesPath, 'backend', BRIDGE_REL), source: 'bundled' };
  }
  const candidates = [
    process.env.LH2_PYTHON,
    path.join(REPO, '.venv', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python'),
  ].filter(Boolean);
  const python = candidates.find(p => fs.existsSync(p)) || (process.platform === 'win32' ? 'py' : 'python3');
  return { python, bridge: path.join(REPO, BRIDGE_REL), source: 'repository' };
}

// First free TCP port from `start` (8051 may be taken by a bridge started with avvia.bat).
function freePort(start = 8051, tries = 50) {
  return new Promise((resolve, reject) => {
    const attempt = port => {
      if (port >= start + tries) return reject(new Error(`no free port in ${start}..${start + tries - 1}`));
      const server = net.createServer();
      server.once('error', () => attempt(port + 1));
      server.once('listening', () => server.close(() => resolve(port)));
      server.listen(port, '127.0.0.1');
    };
    attempt(start);
  });
}

/**
 * Start the bridge. Resolves { url, process } once it prints "READY <url>".
 * @param {object} opts  packaged, resourcesPath, runsDir, logFile, timeoutMs
 */
async function startBackend({ packaged, resourcesPath, runsDir, logFile, timeoutMs = 60000 }) {
  const { python, bridge, source } = locate(packaged, resourcesPath);
  if (!fs.existsSync(bridge)) throw new Error(`backend script not found: ${bridge}`);
  const port = await freePort();
  fs.mkdirSync(path.dirname(logFile), { recursive: true });
  const log = fs.createWriteStream(logFile, { flags: 'w' });
  log.write(`[LH2 Studio] ${new Date().toISOString()} starting ${source} backend\n  ${python} ${bridge} --http-port ${port}\n  sessions: ${runsDir}\n\n`);

  const pythonArgs = python === 'py' ? ['-3', bridge] : [bridge];
  const udp = process.env.LH2_UDP_PORT ? ['--udp-port', process.env.LH2_UDP_PORT] : [];  // tests: keep off 14550
  const child = spawn(python, [...pythonArgs, '--http-port', String(port), ...udp, '--exit-with-stdin'], {
    cwd: path.dirname(bridge),
    env: { ...process.env, LH2_RUNS_DIR: runsDir, PYTHONUNBUFFERED: '1', PYTHONUTF8: '1' },
    stdio: ['pipe', 'pipe', 'pipe'],  // stdin stays open: the bridge exits when it closes (app gone)
    windowsHide: true,
  });

  return new Promise((resolve, reject) => {
    let settled = false, tail = '';
    const finish = (fn, value) => { if (!settled) { settled = true; clearTimeout(timer); fn(value); } };
    const onData = chunk => {
      const text = chunk.toString();
      log.write(text);
      tail = (tail + text).slice(-4000);
      const ready = tail.match(/READY (http:\/\/127\.0\.0\.1:\d+)/);
      if (ready) finish(resolve, { url: ready[1], process: child, port, python, source, log: logFile });
    };
    child.stdout.on('data', onData);
    child.stderr.on('data', onData);
    child.on('error', error => finish(reject, new Error(`could not start Python (${python}): ${error.message}`)));
    child.on('exit', code => {
      log.write(`\n[LH2 Studio] backend exited with code ${code}\n`);
      finish(reject, new Error(`the backend stopped (code ${code}) before it was ready:\n${tail.slice(-1500)}`));
    });
    const timer = setTimeout(() => finish(reject, new Error(`the backend did not start within ${timeoutMs / 1000} s:\n${tail.slice(-1500)}`)), timeoutMs);
  });
}

function stopBackend(child) {
  if (!child || child.exitCode !== null) return;
  try { child.stdin.end(); } catch (_e) { /* already closed */ }  // asks it to exit (--exit-with-stdin)
  setTimeout(() => { if (child.exitCode === null) child.kill(); }, 1500);
}

module.exports = { startBackend, stopBackend, locate, freePort };
