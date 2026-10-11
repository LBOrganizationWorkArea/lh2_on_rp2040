// Build build/python-win/: the official "embeddable" CPython for Windows plus the packages in
// requirements-win.txt, installed from win_amd64 wheels. Works from Windows, Linux or WSL (it only
// downloads and unpacks), so the Windows app can be packaged anywhere.
//
//   node scripts/fetch-python-win.mjs        (re-run with --force to rebuild)
import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, '..');
const OUT = path.join(ROOT, 'build', 'python-win');
const PY_VERSION = '3.11.9';  // last 3.11 release with an embeddable Windows build
const EMBED_URL = `https://www.python.org/ftp/python/${PY_VERSION}/python-${PY_VERSION}-embed-amd64.zip`;
const host = process.env.LH2_PYTHON || (process.platform === 'win32' ? 'py' : 'python3');
const hostArgs = host === 'py' ? ['-3'] : [];
const python = (...args) => execFileSync(host, [...hostArgs, ...args], { stdio: 'inherit' });

if (fs.existsSync(path.join(OUT, 'python.exe')) && !process.argv.includes('--force')) {
  console.log(`build/python-win/ already present (use --force to rebuild)`);
  process.exit(0);
}
fs.rmSync(OUT, { recursive: true, force: true });
fs.mkdirSync(OUT, { recursive: true });
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'lh2-pywin-'));

console.log(`downloading ${EMBED_URL}`);
const response = await fetch(EMBED_URL);
if (!response.ok) throw new Error(`download failed: ${response.status}`);
const embedZip = path.join(tmp, 'embed.zip');
fs.writeFileSync(embedZip, Buffer.from(await response.arrayBuffer()));

const wheels = path.join(tmp, 'wheels');
python('-m', 'pip', 'download', '--only-binary=:all:', '--platform', 'win_amd64',
       '--python-version', '3.11', '--implementation', 'cp',
       '-r', path.join(ROOT, 'requirements-win.txt'), '-d', wheels);

// Unpack Python, then every wheel into Lib/site-packages (a wheel is a zip laid out for it;
// <name>.data/{purelib,platlib} goes to site-packages too).
const unpack = `
import pathlib, shutil, sys, zipfile
out, embed, wheels = map(pathlib.Path, sys.argv[1:4])
zipfile.ZipFile(embed).extractall(out)
site = out / "Lib" / "site-packages"
site.mkdir(parents=True, exist_ok=True)
for wheel in sorted(wheels.glob("*.whl")):
    zipfile.ZipFile(wheel).extractall(site)
    print("installed", wheel.name)
for data in site.glob("*.data"):
    for kind in ("purelib", "platlib"):
        if (data / kind).is_dir():
            shutil.copytree(data / kind, site, dirs_exist_ok=True)
    shutil.rmtree(data)
for package in ("numpy", "scipy"):  # their test suites are ~26 MB and never imported
    for tests in sorted((site / package).rglob("tests"), reverse=True):
        if tests.is_dir():
            shutil.rmtree(tests)
`;
python('-c', unpack, OUT, embedZip, wheels);

// The embeddable build ignores PYTHONPATH and the script folder: sys.path comes only from
// python311._pth. Add site-packages and the backend's script folders (relative to python.exe:
// resources/python -> resources/backend/...), so the calibration scripts can import each other.
const pth = fs.readdirSync(OUT).find(name => /^python\d+\._pth$/.test(name));
if (!pth) throw new Error('python*._pth not found in the embeddable package');
const stdlibZip = fs.readdirSync(OUT).find(name => /^python\d+\.zip$/.test(name));
fs.writeFileSync(path.join(OUT, pth), [
  stdlibZip, '.', 'Lib\\site-packages',
  '..\\backend', '..\\backend\\utils\\calibration', '..\\backend\\utils\\user_interface',
  'import site', '',
].join('\r\n'));
fs.rmSync(tmp, { recursive: true, force: true });
console.log(`build/python-win/ ready: CPython ${PY_VERSION} + ${fs.readFileSync(path.join(ROOT, 'requirements-win.txt'), 'utf8').split('\n').filter(l => l && !l.startsWith('#')).join(', ')}`);
