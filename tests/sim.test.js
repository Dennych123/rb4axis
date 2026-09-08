// Program simulasi (blurobot/sim/) - yang bisa diperiksa TANPA membuka Studio.
//
// Yang paling mahal di alur ini bukan rumus yang salah, tapi satu nama yang salah
// ketik: Studio baru mengeluh setelah project dibuat, FB ditempel, tabel variabel
// ditempel, dan Build dijalankan - satu putaran penuh untuk satu huruf. Jadi tes
// ini mengadu SETIAP nama yang disebut P_SIM_ROBOT.st ke dua tabel variabel yang
// ikut ditempel, plus memastikan blok init masih sama dengan robot.config.json.
'use strict';
const fs = require('fs');
const path = require('path');
const { spawnSync } = require('child_process');

const SIM = path.join(__dirname, '..', 'sim');
const ST = fs.readFileSync(path.join(SIM, 'P_SIM_ROBOT.st'), 'utf8');
const cfg = JSON.parse(fs.readFileSync(path.join(SIM, 'robot.config.json'), 'utf8'));

let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

// ------------------------------------------------- config masih jadi sumbernya
const r = spawnSync(process.execPath, [path.join(__dirname, '..', 'tools', 'gen_sim.js'), '--check'],
  { encoding: 'utf8' });
chk('sim/ sesuai robot.config.json', r.status === 0,
    ((r.stdout || '') + (r.stderr || '')).trim());

chk('SIM_DT = periode task config', ST.includes('SIM_DT := ' + (cfg.task.periode_ms / 1000) + ';'),
    'kalau ST dan config beda, kecepatan di layar bukan yang diminta');
chk('langkah jog = 10 seperti mesin', cfg.jog.langkah === 10,
    'JOG_WORLD_STEP di ladder asli tetap 10 karena cabang penumbuhnya digerbang DONE yang selalu TRUE');

// ------------------------------------------------------- tidak ada MC_ sama sekali
chk('tanpa instruksi MC_*', !/\bMC_[A-Za-z]/.test(ST),
    'project sim sengaja tanpa axis: tidak ada setting motion yang bisa salah');

// --------------------------------------------------- semua nama harus dideklarasi
// Kata kunci ST, konstanta bahasa, dan fungsi standar yang dipakai berkas ini.
// Daftar sengaja PENDEK dan eksplisit: daftar panjang yang menampung apa saja
// bikin tes ini berhenti menangkap salah ketik, yang justru satu-satunya tugasnya.
const KATA = new Set([
  'IF', 'THEN', 'ELSE', 'ELSIF', 'END_IF', 'FOR', 'TO', 'DO', 'END_FOR',
  'CASE', 'OF', 'END_CASE', 'AND', 'OR', 'NOT', 'TRUE', 'FALSE',
  'ABS', 'SQRT', 'SIN', 'COS', 'ATAN', 'REAL_TO_LREAL',
  'EXECUTE', 'ROBOT_POS_INPUT', 'ROBOT_POS_OUTPUT',
  'ROBOT_POS_WORLD_OUTPUT', 'ROBOT_POS_JOINT_OUTPUT'
]);

const kolom0 = f => fs.readFileSync(path.join(SIM, f), 'utf8').split('\n')
  .filter(Boolean).map(l => l.split('\t')[0]);
const glob = kolom0('GlobalVariables.tsv');
const lokal = kolom0('ProgramVariables.tsv');
const dideklarasi = new Set(glob.concat(lokal));

// Komentar dibuang dulu - komentar di berkas ini menyebut nama variabel yang
// memang tidak selalu ada (nama di project ASLI, misalnya JOG_WORLD_STEP).
const kode = ST.split('\n').map(l => l.replace(/\/\/.*$/, '')).join('\n');
const tidakAda = [...new Set(kode.match(/[A-Za-z_][A-Za-z0-9_]*/g) || [])]
  .filter(n => !KATA.has(n) && !dideklarasi.has(n));
chk('tiap nama di ST ada di salah satu tabel variabel', tidakAda.length === 0,
    tidakAda.length ? 'belum dideklarasi: ' + tidakAda.join(' ') : '');

// Kebalikannya: variabel yang dideklarasi tapi tidak pernah dipakai. Bukan galat -
// tapi baris yang ditempel ke Studio tanpa ada yang memakainya biasanya sisa dari
// rancangan yang berubah, dan Studio TIDAK akan memberitahu.
//
// External milik FB DIKECUALIKAN: yang menuntutnya FB, bukan program ini.
// RAD_TO_DEGREE contohnya - cuma dipakai di dalam INVERSE_KINEMATIC, dan kalau
// baris itu dibuang dari tabel global, yang gagal justru FB-nya.
const EXT_FB = new Set(fs.readFileSync(path.join(__dirname, '..', 'extract', 'variables.tsv'), 'utf8')
  .split('\n').filter(Boolean).map(l => l.split('\t')[0]));
const tidakDipakai = glob.concat(lokal)
  .filter(n => !EXT_FB.has(n) && !new RegExp('\\b' + n + '\\b').test(kode));
chk('tidak ada variabel yang dideklarasi tapi nganggur', tidakDipakai.length === 0,
    tidakDipakai.join(' '));

// ------------------------------------------------------------ bentuk tabelnya
const g = fs.readFileSync(path.join(SIM, 'GlobalVariables.tsv'), 'utf8').split('\n').filter(Boolean);
chk('GlobalVariables.tsv 8 kolom, tanpa judul',
    g.every(l => l.split('\t').length === 8) && !/^Name\t/i.test(g[0]));
chk('external kedua FB ikut di tabel global',
    ['PI', 'DEGREE_TO_RAD', 'RAD_TO_DEGREE', 'ROBOT_L2_LREAL', 'PD1300_000']
      .every(n => glob.includes(n)),
    'FB menuntutnya sebagai VAR_EXTERNAL - kurang satu, Build gagal di FB, bukan di program');

// FB instance harus bertipe FB-nya sendiri; salah tipe di sini = (DefinitionError)
// yang pesannya tidak menyebut sebabnya.
const p = fs.readFileSync(path.join(SIM, 'ProgramVariables.tsv'), 'utf8').split('\n').filter(Boolean);
chk('FK1/IK1 bertipe FB-nya',
    p.some(l => l.startsWith('FK1\tFORWARD_KINEMATIC\t')) && p.some(l => l.startsWith('IK1\tINVERSE_KINEMATIC\t')));

// Konstanta project TIDAK boleh ikut ditulis init sim: di tabel mereka Constant,
// dan Studio menolak assignment ke variabel Constant waktu Build.
chk('init sim tidak menulis ke PI/DEGREE_TO_RAD/RAD_TO_DEGREE',
    !/\b(PI|DEGREE_TO_RAD|RAD_TO_DEGREE)\s*:=/.test(kode));

console.log(fail ? 'GAGAL ' + fail : 'LULUS');
process.exit(fail ? 1 : 0);
