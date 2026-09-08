// Suite Blurobot. Sengaja TERPISAH dari `node tests/run.js`:
//
//   * empat gerbang XML repo utama (xsd/instr/rungwire/declared) tidak ada urusannya
//     dengan kinematik, dan suite yang gagal karena tetangganya bikin orang berhenti
//     membaca hasilnya,
//   * suite ini butuh project mesin `.smc2` yang TIDAK ada di repo - jadi dia harus
//     boleh SKIP, sementara suite repo utama tidak boleh.
//
//   node blurobot/tests/run.js
'use strict';
const { spawnSync } = require('child_process');
const fs = require('fs');
const path = require('path');

const suites = fs.readdirSync(__dirname)
  .filter(f => f.endsWith('.test.js'))
  .sort()
  .map(f => [f.replace(/\.test\.js$/, ''), path.join(__dirname, f)]);

let failed = 0;
for (const [name, file] of suites) {
  const r = spawnSync(process.execPath, [file], { encoding: 'utf8' });
  const out = (r.stdout || '') + (r.stderr || '');
  console.log('=== ' + name);
  process.stdout.write(out.endsWith('\n') || !out ? out : out + '\n');
  if (r.status !== 0) failed++;
}

console.log(failed ? '\nGAGAL: ' + failed + ' suite' : '\nSEMUA SUITE LULUS (' + suites.length + ')');
process.exit(failed ? 1 : 0);
