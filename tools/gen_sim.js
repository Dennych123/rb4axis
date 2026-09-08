#!/usr/bin/env node
// robot.config.json -> blok init di PRG_SIM_ROBOT.st + dua tabel variabel.
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
const ST = path.join(SIM, 'PRG_SIM_ROBOT.st');
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
  ['SIM_ERROR', 'BOOL', 'R', 'perintah terakhir ditolak'],
  ['SIM_ERROR_ID', 'INT', 'R', '0 tidak ada, 1 di luar jangkauan, 2 terlalu dekat, 3 singular, 4 soft limit'],
  ['SIM_ELBOW_UP', 'BOOL', 'RW',
    'FALSE = cabang elbow yang sama dengan mesin. TRUE = cermin, cuma ada di FB V2'],

  ['SIM_GRIP_CMD', 'BOOL', 'RW', 'TRUE tutup, FALSE buka'],
  ['SIM_GRIP_POS', 'LREAL', 'R', 'bukaan sekarang - jarak antar jari, mm'],
  ['SIM_GRIP_STROKE', 'LREAL', 'RW', 'bukaan penuh, mm'],
  ['SIM_GRIP_VEL', 'LREAL', 'RW', 'kecepatan jari, mm/s'],
  ['SIM_GRIP_OPEN', 'BOOL', 'R', 'jari sudah terbuka penuh'],
  ['SIM_GRIP_CLOSED', 'BOOL', 'R', 'jari sudah menutup rapat'],
  ['SIM_GRIP_LEN', 'LREAL', 'R', 'panjang gripper - sudah ikut di ROBOT_TOOL_Y_LREAL, ini buat viz']
];

// ------------------------------------------------------ variabel lokal program
// FB yang dipakai sim itu yang V2 - yang sudah dibetulkan. Yang V1 (di extract/)
// tetap ada sebagai catatan apa yang jalan di mesin, dan tidak di-instance di sini:
// instance yang tidak dipakai tetap memakan memori dan bikin orang mengira dua-duanya
// ikut menentukan gerakan.
//
// Penjaga jangkauan yang dulu di program (REACH_OK, Y3, Z3, RR, R_TOOL, TH_TOOL)
// SUDAH PINDAH ke dalam INVERSE_KINEMATIC_V2. Dua penjaga untuk satu hal pasti
// berbeda pendapat suatu hari, dan yang di luar tidak pernah tahu rumus di dalam.
const LOKAL = [
  ['FK2', 'FORWARD_KINEMATIC_V2', 'instance FB kinematik maju yang sudah dibetulkan'],
  ['IK2', 'INVERSE_KINEMATIC_V2', 'instance FB kinematik balik yang sudah dibetulkan'],
  ['i', 'INT', 'pencacah FOR'],
  ['AX', 'INT', 'sumbu yang tombolnya sedang ditekan, -1 kalau tidak ada'],
  ['DELTA', 'LREAL', 'besar langkah jog scan ini, sudah bertanda'],
  ['POSE_REQ', 'ARRAY[0..3] OF LREAL', 'pose world yang diminta'],
  ['NEED_IK', 'BOOL', 'permintaan scan ini perlu inverse kinematic'],
  ['TH', 'LREAL', 'theta_EE radian, buat jog mode tool'],
  ['STEP_M', 'LREAL', 'langkah motion model satu scan'],
  ['D', 'LREAL', 'sisa jarak ke target'],
  ['GRIP_TARGET', 'LREAL', 'bukaan gripper yang dituju'],
  // Penampung keluaran FB. Studio MENOLAK mengindeks array milik instance FB
  // ("Cannot use an element of array or a member of structure for the reference of
  // function block instance variables"), jadi arraynya disalin UTUH dulu ke sini.
  // Anggota skalar seperti IK2.DONE tidak kena aturan itu.
  ['IK_OUT', 'ARRAY[0..3] OF LREAL', 'salinan ROBOT_POS_OUTPUT milik IK2'],
  ['FK_WORLD', 'ARRAY[0..3] OF REAL', 'salinan ROBOT_POS_WORLD_OUTPUT milik FK2'],
  ['FK_JOINT', 'ARRAY[0..3] OF REAL', 'salinan ROBOT_POS_JOINT_OUTPUT milik FK2'],
  ['LAST_P', 'ARRAY[0..3] OF BOOL', 'keadaan tombol plus scan sebelumnya'],
  ['LAST_N', 'ARRAY[0..3] OF BOOL', 'keadaan tombol minus scan sebelumnya'],
  ['EDGE_P', 'ARRAY[0..3] OF BOOL', 'tepi naik tombol plus'],
  ['EDGE_N', 'ARRAY[0..3] OF BOOL', 'tepi naik tombol minus'],
  ['LAST_MOVE', 'BOOL', 'keadaan SIM_MOVE_EXEC scan sebelumnya'],
  ['LAST_HOME', 'BOOL', 'keadaan SIM_HOME_EXEC scan sebelumnya'],
  ['EDGE_MOVE', 'BOOL', 'tepi naik SIM_MOVE_EXEC'],
  ['EDGE_HOME', 'BOOL', 'tepi naik SIM_HOME_EXEC']
];

// ------------------------------------------- tabel variabel kedua FB V2
// Ditulis di sini, bukan diketik ulang di Studio: susunan pin FB itu yang dicocokkan
// Studio ke pemanggilnya, dan satu nama yang meleset menghasilkan (DefinitionError)
// yang pesannya tidak menyebut pin mana.
const FB_EXT_UMUM = [
  ['ROBOT_L1_LREAL', 'LREAL'], ['ROBOT_L2_LREAL', 'LREAL'],
  ['ROBOT_L3_LREAL', 'LREAL'], ['ROBOT_L4_LREAL', 'LREAL'],
  ['ROBOT_TOOL_Y_LREAL', 'LREAL'], ['ROBOT_TOOL_Z_LREAL', 'LREAL'],
  ['PI', 'LREAL'], ['DEGREE_TO_RAD', 'LREAL']
];

const FB_V2 = [
  {
    nama: 'FORWARD_KINEMATIC_V2',
    IN: [['EXECUTE', 'BOOL'], ['ROBOT_POS_INPUT', 'ARRAY[0..3] OF LREAL']],
    OUT: [['DONE', 'BOOL'], ['ROBOT_POS_JOINT_OUTPUT', 'ARRAY[0..3] OF REAL'],
          ['ROBOT_POS_WORLD_OUTPUT', 'ARRAY[0..3] OF REAL'],
          ['WORLD_LREAL', 'ARRAY[0..3] OF LREAL']],
    VAR: [['TY', 'LREAL'], ['TZ', 'LREAL'], ['R_TOOL', 'LREAL'], ['TH_TOOL', 'LREAL'],
          ['A1', 'LREAL'], ['A2', 'LREAL'], ['A3', 'LREAL'],
          ['WY', 'LREAL'], ['WZ', 'LREAL'], ['I', 'INT']],
    EXT: FB_EXT_UMUM
  },
  {
    nama: 'INVERSE_KINEMATIC_V2',
    IN: [['EXECUTE', 'BOOL'], ['ROBOT_POS_INPUT', 'ARRAY[0..3] OF LREAL'], ['ELBOW_UP', 'BOOL']],
    OUT: [['DONE', 'BOOL'], ['ERROR', 'BOOL'], ['ERROR_ID', 'INT'],
          ['ROBOT_POS_OUTPUT', 'ARRAY[0..3] OF LREAL'],
          ['LIMIT_HIT', 'ARRAY[0..7] OF BOOL'], ['LIMIT_ANY', 'BOOL']],
    VAR: [['TY', 'LREAL'], ['TZ', 'LREAL'], ['R_TOOL', 'LREAL'], ['TH_TOOL', 'LREAL'],
          ['TH_EE', 'LREAL'], ['Y_EE', 'LREAL'], ['Z_EE', 'LREAL'],
          ['Y3', 'LREAL'], ['Z3', 'LREAL'], ['R', 'LREAL'],
          ['BETA', 'LREAL'], ['GAMMA', 'LREAL'], ['ALFA', 'LREAL'],
          ['THETA_RAD1', 'LREAL'], ['THETA_RAD2', 'LREAL'], ['THETA_RAD3', 'LREAL'],
          ['THETA_DEGREE1', 'LREAL'], ['THETA_DEGREE2', 'LREAL'], ['THETA_DEGREE3', 'LREAL'],
          ['I', 'INT']],
    EXT: FB_EXT_UMUM.concat([
      ['RAD_TO_DEGREE', 'LREAL'], ['ROBOT_ROBOT_ORG_OFFSET_LREAL', 'ARRAY[0..4] OF LREAL'],
      ['PD1300_000', 'REAL'], ['PD1300_001', 'REAL'], ['PD1300_002', 'REAL'], ['PD1300_003', 'REAL'],
      ['PD1300_004', 'REAL'], ['PD1300_005', 'REAL'], ['PD1300_006', 'REAL'], ['PD1300_007', 'REAL']
    ])
  }
];

function blokInit(cfg) {
  const L = [];
  const t = s => '\t' + s;
  L.push(t('ROBOT_L1_LREAL := ' + lreal(cfg.link.L1) + ';'));
  L.push(t('ROBOT_L2_LREAL := ' + lreal(cfg.link.L2) + ';'));
  L.push(t('ROBOT_L3_LREAL := ' + lreal(cfg.link.L3) + ';'));
  L.push(t('ROBOT_L4_LREAL := ' + lreal(cfg.link.L4) + ';'));
  L.push('');
  // ROBOT_TOOL_X TIDAK diikutkan: tidak ada satu rumus pun yang membacanya.
  //
  // Panjang gripper dijumlahkan DI SINI, satu kali, supaya TCP berada di ujung jari
  // dan FK/IK memang menghitung sampai situ. Kalau dijumlahkan lagi di tempat lain
  // (viz, config, atau rung), lengannya panjang dua kali gripper - dan itu tidak
  // kelihatan salah di layar, cuma meleset beberapa sentimeter.
  L.push(t('// tool.Y (' + lreal(cfg.tool.Y) + ') + gripper.panjang ('
    + lreal(cfg.gripper.panjang) + ') - TCP di ujung jari'));
  L.push(t('ROBOT_TOOL_Y_LREAL := ' + lreal(cfg.tool.Y + cfg.gripper.panjang) + ';'));
  L.push(t('ROBOT_TOOL_Z_LREAL := ' + lreal(cfg.tool.Z) + ';'));
  L.push('');
  L.push(t('SIM_GRIP_LEN := ' + lreal(cfg.gripper.panjang) + ';'));
  L.push(t('SIM_GRIP_STROKE := ' + lreal(cfg.gripper.stroke) + ';'));
  L.push(t('SIM_GRIP_VEL := ' + lreal(cfg.gripper.kecepatan) + ';'));
  L.push(t('SIM_GRIP_POS := ' + lreal(cfg.gripper.bukaan_awal) + ';'));
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
    console.error('GAGAL: penanda blok init tidak ketemu di PRG_SIM_ROBOT.st');
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

  // Tabel variabel tiap FB V2, satu berkas per FB. Ditempel ke tabel variabel FB-nya
  // di Studio; VAR_INPUT/VAR_OUTPUT ditulis dengan urutan Ord yang sama dengan
  // urutan pin di sini, karena itu yang menentukan bentuk kotaknya di ladder.
  const fbTsv = FB_V2.map(fb => {
    const b = [];
    const blok = (grup, list) => list.forEach(r => b.push([r[0], r[1], '', grup].join('\t')));
    blok('VAR_INPUT', fb.IN);
    blok('VAR_OUTPUT', fb.OUT);
    blok('VAR', fb.VAR);
    blok('VAR_EXTERNAL', fb.EXT);
    return [path.join(SIM, fb.nama + '.vars.tsv'), b.join('\n') + '\n'];
  });

  const berkas = [[ST, st], [GTSV, gtsv], [PTSV, ptsv], [TAGS, tagsJson]].concat(fbTsv);
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
  // Berkas import ikut basi begitu tabel variabel berubah, dan basinya DIAM: XML
  // lama tetap sah, tetap lolos XSD, cuma isinya project yang sudah tidak ada.
  console.log('    lanjut:  node blurobot/tools/gen_xml.js   (BlurobotSim.xml ikut basi)');
}

main();
