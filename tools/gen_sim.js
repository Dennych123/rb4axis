#!/usr/bin/env node
// robot.config.json -> blok init di P_SIM_ROBOT.st + dua tabel variabel.
//
//   node blurobot/tools/gen_sim.js            tulis
//   node blurobot/tools/gen_sim.js --check    cuma periksa, exit 1 kalau sudah basi
//
// Kenapa dibangkitkan dan bukan diketik dua kali: angka yang sama muncul di config
// DAN di literal ST. Disalin tangan, dua-duanya kelihatan benar sampai ada yang
// mengubah salah satunya - dan yang jalan di simulator selalu yang di ST, sementara
// yang dibaca orang (dan halaman viz) yang di config. Sekarang config-nya sumber,
// dan tests/sim.test.js menolak kalau ST-nya tidak lagi cocok.
//
// Yang diganti CUMA daerah di antara dua penanda di dalam blok IF NOT SIM_INIT_DONE.
// Sisanya - logika jog, penjaga jangkauan, motion model - ditulis tangan dan tidak
// pernah disentuh generator.
'use strict';
const fs = require('fs');
const path = require('path');

const SIM = path.join(__dirname, '..', 'sim');
const CFG = path.join(SIM, 'robot.config.json');
const ST = path.join(SIM, 'P_SIM_ROBOT.st');
const GTSV = path.join(SIM, 'GlobalVariables.tsv');
const PTSV = path.join(SIM, 'ProgramVariables.tsv');
const TAGS = path.join(__dirname, '..', 'bridge', 'tags.json');
const EXTRACT_TSV = path.join(__dirname, '..', 'extract', 'variables.tsv');

const AWAL = '\t// <<< DIBANGKITKAN dari robot.config.json oleh tools/gen_sim.js - jangan sunting tangan';
const AKHIR = '\t// >>> DIBANGKITKAN';

// LREAL di ST butuh titik desimal: `400` itu literal INT dan Studio menolak
// menugaskannya ke LREAL tanpa konversi. Angka bulat dari JSON hilang ".0"-nya
// waktu di-String(), jadi ditambahkan kembali di sini.
function lreal(x) {
  const s = String(x);
  return /[.eE]/.test(s) ? s : s + '.0';
}

// ------------------------------------------------------- variabel global sim
// Satu daftar, tiga keluaran: tabel yang ditempel ke Studio, daftar tag yang
// dibaca bridge OPC UA, dan (lewat tipenya) apa yang boleh ditulis halaman.
// Daftar kedua yang ditulis tangan pasti drift - dan drift-nya diam: halaman
// menulis ke tag yang tidak ada, bridge menjawab OK, dan tidak ada yang bergerak.
//
// Kolom ketiga = arah dari sudut pandang HALAMAN:
//   'R'  halaman cuma membaca      'W'  halaman menulis      'RW' dua-duanya
const GLOBAL_SIM = [
  ['SIM_INIT_DONE', 'BOOL', 'R', 'sudah diinisialisasi dari config'],
  ['SIM_RESET', 'BOOL', 'W', 'tulis TRUE: paksa init ulang dari nilai config'],
  ['SIM_DT', 'LREAL', 'R', 'periode task primer detik - HARUS sama dengan setelan task di Studio'],
  ['SIM_HEARTBEAT', 'UDINT', 'R', 'naik tiap scan - bukti program sim benar-benar jalan'],

  ['SIM_JOINT_POS', 'ARRAY[0..3] OF LREAL', 'R', 'posisi sumbu sekarang: X mm, theta1-3 derajat'],
  ['SIM_JOINT_CMD', 'ARRAY[0..3] OF LREAL', 'R', 'target sumbu yang dikejar motion model'],
  ['SIM_JOINT_OUT', 'ARRAY[0..3] OF REAL', 'R', 'keluaran joint FORWARD_KINEMATIC'],
  ['SIM_WORLD_POS', 'ARRAY[0..3] OF REAL', 'R', 'pose world hasil FK: X Y Z theta_EE - sudah termasuk tool'],
  ['SIM_WORLD_CMD', 'ARRAY[0..3] OF LREAL', 'RW', 'target move point: X Y Z theta_EE'],

  ['SIM_JOG_MODE', 'INT', 'RW', '0 joint / 1 world / 2 tool'],
  ['SIM_JOG_HOLD', 'BOOL', 'RW', 'TRUE: tahan-jalan (tambahan viz). FALSE: satu langkah per tekan, seperti mesin'],
  ['SIM_JOG_STEP', 'LREAL', 'RW', 'besar satu langkah jog - mm atau derajat'],
  ['SIM_JOG_P', 'ARRAY[0..3] OF BOOL', 'W', 'tombol jog arah plus per sumbu'],
  ['SIM_JOG_N', 'ARRAY[0..3] OF BOOL', 'W', 'tombol jog arah minus per sumbu'],

  ['SIM_MOVE_EXEC', 'BOOL', 'W', 'tepi naik: jalankan move ke SIM_WORLD_CMD'],
  ['SIM_MOVE_DONE', 'BOOL', 'R', 'semua sumbu sudah sampai target'],
  ['SIM_HOME_EXEC', 'BOOL', 'W', 'tepi naik: kembali ke pose home'],
  ['SIM_HOME', 'ARRAY[0..3] OF LREAL', 'RW', 'pose home per sumbu'],
  ['SIM_VEL', 'ARRAY[0..3] OF LREAL', 'RW', 'kecepatan maksimum per sumbu - mm/s atau derajat/s'],
  ['SIM_VEL_W', 'ARRAY[0..3] OF LREAL', 'RW', 'kecepatan jog world/tool - mm/s untuk X Y Z, derajat/s untuk theta_EE'],
  ['SIM_BUSY', 'BOOL', 'R', 'masih ada sumbu yang bergerak'],

  ['SIM_LIMIT', 'ARRAY[0..7] OF BOOL', 'R', 'soft limit yang KENA - pasangan min/max per sumbu'],
  ['SIM_ERROR', 'BOOL', 'R', 'perintah terakhir ditolak penjaga jangkauan'],
  ['SIM_ERROR_ID', 'INT', 'R', '0 tidak ada, 1 di luar jangkauan, 2 Y3 <= 0 (ATAN kehilangan kuadran)']
];

// ------------------------------------------------------ variabel lokal program
const LOKAL = [
  ['FK1', 'FORWARD_KINEMATIC', 'instance FB dari extract/ - JANGAN diganti isinya'],
  ['IK1', 'INVERSE_KINEMATIC', 'instance FB dari extract/ - JANGAN diganti isinya'],
  ['i', 'INT', 'pencacah FOR'],
  ['AX', 'INT', 'sumbu yang tombolnya sedang ditekan, -1 kalau tidak ada'],
  ['DELTA', 'LREAL', 'besar langkah jog scan ini, sudah bertanda'],
  ['POSE_REQ', 'ARRAY[0..3] OF LREAL', 'pose world yang diminta, sebelum diperiksa penjaga'],
  ['NEED_IK', 'BOOL', 'permintaan scan ini perlu inverse kinematic'],
  ['REACH_OK', 'BOOL', 'pose diminta lolos penjaga jangkauan'],
  ['TH', 'LREAL', 'theta_EE radian'],
  ['TH_TOOL', 'LREAL', 'sudut tool polar'],
  ['R_TOOL', 'LREAL', 'jarak tool polar'],
  ['Y3', 'LREAL', 'titik pergelangan sebelum tool dan L4'],
  ['Z3', 'LREAL', 'titik pergelangan sebelum tool dan L4'],
  ['RR', 'LREAL', 'jarak bahu ke pergelangan'],
  ['STEP_M', 'LREAL', 'langkah motion model satu scan'],
  ['D', 'LREAL', 'sisa jarak ke target'],
  ['LAST_P', 'ARRAY[0..3] OF BOOL', 'keadaan tombol plus scan sebelumnya'],
  ['LAST_N', 'ARRAY[0..3] OF BOOL', 'keadaan tombol minus scan sebelumnya'],
  ['EDGE_P', 'ARRAY[0..3] OF BOOL', 'tepi naik tombol plus'],
  ['EDGE_N', 'ARRAY[0..3] OF BOOL', 'tepi naik tombol minus'],
  ['LAST_MOVE', 'BOOL', 'keadaan SIM_MOVE_EXEC scan sebelumnya'],
  ['LAST_HOME', 'BOOL', 'keadaan SIM_HOME_EXEC scan sebelumnya'],
  ['EDGE_MOVE', 'BOOL', 'tepi naik SIM_MOVE_EXEC'],
  ['EDGE_HOME', 'BOOL', 'tepi naik SIM_HOME_EXEC']
];

function blokInit(cfg) {
  const L = [];
  const t = s => '\t' + s;
  L.push(t('ROBOT_L1_LREAL := ' + lreal(cfg.link.L1) + ';'));
  L.push(t('ROBOT_L2_LREAL := ' + lreal(cfg.link.L2) + ';'));
  L.push(t('ROBOT_L3_LREAL := ' + lreal(cfg.link.L3) + ';'));
  L.push(t('ROBOT_L4_LREAL := ' + lreal(cfg.link.L4) + ';'));
  // ROBOT_TOOL_X TIDAK diikutkan: tidak ada satu rumus pun yang membacanya.
  L.push(t('ROBOT_TOOL_Y_LREAL := ' + lreal(cfg.tool.Y) + ';'));
  L.push(t('ROBOT_TOOL_Z_LREAL := ' + lreal(cfg.tool.Z) + ';'));
  L.push('');
  cfg.offset.nilai.forEach((v, i) => {
    L.push(t('ROBOT_ROBOT_ORG_OFFSET_LREAL[' + i + '] := ' + lreal(v) + ';'));
  });
  L.push('');
  for (let i = 0; i <= 7; i++) {
    const k = 'PD1300_00' + i;
    L.push(t(k + ' := ' + lreal(cfg.limit[k]) + ';'));
  }
  L.push('');
  cfg.jog.sumbu.forEach((v, i) => L.push(t('SIM_VEL[' + i + '] := ' + lreal(v) + ';')));
  L.push('');
  cfg.jog.world.forEach((v, i) => L.push(t('SIM_VEL_W[' + i + '] := ' + lreal(v) + ';')));
  L.push('');
  cfg.home.sumbu.forEach((v, i) => L.push(t('SIM_HOME[' + i + '] := ' + lreal(v) + ';')));
  L.push('');
  L.push(t('SIM_DT := ' + lreal(cfg.task.periode_ms / 1000) + ';'));
  L.push(t('SIM_JOG_STEP := ' + lreal(cfg.jog.langkah) + ';'));
  return L;
}

function stBaru(lama, cfg) {
  const baris = lama.split('\n');
  const a = baris.findIndex(l => l.trimEnd() === AWAL.trimEnd());
  const b = baris.findIndex(l => l.trimEnd() === AKHIR.trimEnd());
  if (a < 0 || b < 0 || b < a) {
    console.error('GAGAL: penanda blok init tidak ketemu di P_SIM_ROBOT.st');
    process.exit(2);
  }
  return baris.slice(0, a + 1).concat(blokInit(cfg), baris.slice(b)).join('\n');
}

// Kolom tabel Global Variable Studio - sama persis dengan TSV_HEAD di
// js/gen_all.js dan dengan extract/variables.tsv. TANPA baris judul.
function barisGlobal(n, t, cmt) {
  return [n, t, '', '', 'False', 'False', 'Do not publish', cmt || ''].join('\t');
}

function main() {
  const cek = process.argv.includes('--check');
  const cfg = JSON.parse(fs.readFileSync(CFG, 'utf8'));

  const stLama = fs.readFileSync(ST, 'utf8');
  const st = stBaru(stLama, cfg);

  // Variabel global sim + external yang dituntut kedua FB. Yang kedua disalin apa
  // adanya dari extract/ supaya tipe, retain dan Constant-nya tidak pernah
  // ditebak ulang di sini.
  const ext = fs.existsSync(EXTRACT_TSV)
    ? fs.readFileSync(EXTRACT_TSV, 'utf8').split('\n').filter(Boolean) : [];
  if (!ext.length) {
    console.error('GAGAL: extract/variables.tsv kosong - jalanin dulu: node blurobot/tools/extract.js');
    process.exit(2);
  }
  const gtsv = GLOBAL_SIM.map(r => barisGlobal(r[0], r[1], r[3])).concat(ext).join('\n') + '\n';

  // Tabel variabel PROGRAM: kolomnya lebih sedikit (tidak ada Network Publish).
  const ptsv = LOKAL.map(r => [r[0], r[1], '', 'False', 'False', r[2]].join('\t')).join('\n') + '\n';

  // Daftar tag buat bridge. Dibangkitkan dari daftar yang SAMA dengan tabel Studio,
  // jadi tag yang ada di halaman pasti ada di controller. Yang boleh DITULIS
  // dibatasi di sini, bukan di halaman: nama yang datang dari browser tidak pernah
  // dipakai mentah buat menulis ke PLC.
  //
  // External milik FB (ROBOT_L*, ROBOT_TOOL_*, PD1300_*) ikut dibaca supaya viz
  // menggambar dimensi yang benar-benar dipegang controller. Yang Constant
  // (PI, DEGREE_TO_RAD, RAD_TO_DEGREE) dilewati - tidak ada gunanya di layar.
  const KONSTAN = /^(PI|DEGREE_TO_RAD|RAD_TO_DEGREE)$/;
  const extRow = ext.map(l => l.split('\t')).filter(c => !KONSTAN.test(c[0]));
  const tag = n => ({ nama: n[0], tipe: n[1], komen: n[3] || '' });
  const tags = {
    _catatan: 'DIBANGKITKAN tools/gen_sim.js - jangan sunting tangan. '
      + 'Path OPC UA = prefix + nama.',
    endpoint: 'opc.tcp://127.0.0.1:4840',
    prefix: 'GlobalVars.',
    baca: GLOBAL_SIM.filter(r => r[2].indexOf('R') >= 0).map(tag)
      .concat(extRow.map(c => ({ nama: c[0], tipe: c[1], komen: 'external FB' }))),
    tulis: GLOBAL_SIM.filter(r => r[2].indexOf('W') >= 0).map(tag)
      .concat(extRow.map(c => ({ nama: c[0], tipe: c[1], komen: 'external FB' })))
  };
  const tagsJson = JSON.stringify(tags, null, 2) + '\n';

  const berkas = [[ST, st], [GTSV, gtsv], [PTSV, ptsv], [TAGS, tagsJson]];
  if (cek) {
    const basi = berkas.filter(([p, isi]) => !fs.existsSync(p) || fs.readFileSync(p, 'utf8') !== isi);
    if (basi.length) {
      console.error('BASI: ' + basi.map(x => path.basename(x[0])).join(', ')
        + '  -> jalanin: node blurobot/tools/gen_sim.js');
      process.exit(1);
    }
    console.log('OK  sim/ sudah sesuai robot.config.json');
    return;
  }
  for (const [p, isi] of berkas) fs.writeFileSync(p, isi, 'utf8');
  console.log('OK  ' + GLOBAL_SIM.length + ' global sim + ' + ext.length + ' external FB, '
    + LOKAL.length + ' variabel program -> sim/, ' + tags.baca.length + ' tag baca / '
    + tags.tulis.length + ' tag tulis -> bridge/tags.json');
}

main();
