// Port JS kinematik (blurobot/web/kin.js) diadu ke sifat-sifat ST aslinya.
//
// Tes di sini punya dua tugas yang beda dan gampang tertukar:
//
//   * yang MENJAGA  - round-trip IK->FK, konstanta, penjaga jangkauan. Gagal = port
//     JS-nya melenceng dari ST, dan mode offline halaman bakal menggambar robot lain.
//   * yang MENCATAT - tiga cacat di ST asli (DONE selalu TRUE, ATAN kehilangan
//     kuadran, ACOS tanpa penjaga) plus FK yang tidak menyentuh EXECUTE/DONE.
//     Tes ini SENGAJA menuntut perilaku yang salah. Kalau nanti ada yang
//     "memperbaiki" rumusnya, yang mengadu duluan tes ini - dan itu memang mau,
//     karena sim yang lebih benar dari mesinnya menjawab pertanyaan yang salah.
'use strict';
const fs = require('fs');
const path = require('path');

const K = require(path.join(__dirname, '..', 'web', 'kin.js'));
const CFGFILE = path.join(__dirname, '..', 'sim', 'robot.config.json');
const TSV = path.join(__dirname, '..', 'extract', 'variables.tsv');

let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

const raw = JSON.parse(fs.readFileSync(CFGFILE, 'utf8'));
const cfg = {
  L1: raw.link.L1, L2: raw.link.L2, L3: raw.link.L3, L4: raw.link.L4,
  toolY: raw.tool.Y, toolZ: raw.tool.Z,
  offset: raw.offset.nilai,
  limit: [raw.limit.PD1300_000, raw.limit.PD1300_001, raw.limit.PD1300_002, raw.limit.PD1300_003,
          raw.limit.PD1300_004, raw.limit.PD1300_005, raw.limit.PD1300_006, raw.limit.PD1300_007]
};

// --------------------------------------------------------- konstanta = project
// Konstanta di kin.js harus SAMA dengan yang di tabel global project, bukan
// Math.PI. Kalau beda, tes yang membandingkan hasil PLC dengan hasil JS jadi
// berisik terus dan lama-lama diabaikan.
if (fs.existsSync(TSV)) {
  const iv = {};
  for (const line of fs.readFileSync(TSV, 'utf8').split('\n')) {
    const c = line.split('\t');
    if (c[0]) iv[c[0]] = c[2];
  }
  chk('PI = nilai awal project', String(K.KIN_PI) === iv.PI, K.KIN_PI + ' vs ' + iv.PI);
  chk('DEGREE_TO_RAD = nilai awal project', String(K.KIN_DEGREE_TO_RAD) === iv.DEGREE_TO_RAD,
      K.KIN_DEGREE_TO_RAD + ' vs ' + iv.DEGREE_TO_RAD);
  chk('RAD_TO_DEGREE = nilai awal project', String(K.KIN_RAD_TO_DEGREE) === iv.RAD_TO_DEGREE,
      K.KIN_RAD_TO_DEGREE + ' vs ' + iv.RAD_TO_DEGREE);
  chk('konstanta BUKAN Math.PI', K.KIN_PI !== Math.PI, 'beda di digit ke-10, dan itu disengaja');
} else {
  console.log('  SKIP  extract/variables.tsv belum ada - jalanin dulu: node blurobot/tools/extract.js');
}

// ------------------------------------------------------------- round-trip
// Elbow-down (theta2 negatif) karena itu cabang yang dipilih rumus IK: rad2 =
// BETA - PI selalu di (-PI, 0). Pose elbow-up TIDAK bisa dihasilkan IK ini sama
// sekali - bukan bug tes, memang begitu robotnya diprogram.
let rtMax = 0, rtN = 0, rtSkip = 0;
for (const t1 of [20, 45, 70, 100]) {
  for (const t2 of [-100, -70, -40]) {
    for (const t3 of [-30, 0, 30]) {
      const j = [123.5, t1, t2, t3];
      const fk = K.forwardKinematic(j, cfg);
      const rc = K.reachable(fk.world, cfg);
      if (!rc.ok || rc.y3 < 0) { rtSkip++; continue; }   // y3<0 = cabang ATAN yang cacat
      const ik = K.inverseKinematic(fk.world, cfg);
      rtN++;
      for (let i = 0; i < 4; i++) rtMax = Math.max(rtMax, Math.abs(ik.joint[i] - j[i]));
    }
  }
}
chk('round-trip FK->IK menutup (' + rtN + ' pose, ' + rtSkip + ' dilewati)',
    rtN >= 20 && rtMax < 5e-3, 'selisih terbesar ' + rtMax.toExponential(2) + ' (toleransi 5e-3, '
    + 'batasnya pembulatan REAL 32-bit di keluaran FK)');

// Sumbu 0 itu LINEAR dan lewat apa adanya - kalau ini pernah "diperbaiki" jadi
// sudut, seluruh viz 3D salah sumbu tanpa satu pun tes lain yang mengeluh.
const lin = K.forwardKinematic([250, 45, -60, 10], cfg);
chk('sumbu 0 = X linear, lewat apa adanya', Math.abs(lin.world[0] - 250) < 1e-6, 'X=' + lin.world[0]);
chk('theta_EE = jumlah tiga sudut', Math.abs(lin.world[3] - (45 - 60 + 10)) < 1e-4, lin.world[3]);

// --------------------------------------------------------------- tool offset
const tool0 = K.toolPolar({ toolY: 0, toolZ: 80 });
chk('toolY=0 kena penjaga bagi-nol (R=|Z|, theta ~ 90 derajat)',
    Math.abs(tool0.r - 80) < 1e-6 && Math.abs(tool0.theta - Math.PI / 2) < 1e-9,
    'r=' + tool0.r + ' theta=' + tool0.theta);
const tanpaTool = K.forwardKinematic([0, 0, 0, 0], { L1: 400, L2: 300, L3: 250, L4: 100, toolY: 1e-15, toolZ: 0 });
chk('tanpa tool: Y = L2+L3+L4 pada semua sudut nol', Math.abs(tanpaTool.world[1] - 650) < 1e-3, tanpaTool.world[1]);

// ------------------------------------------------- YANG MENCATAT CACAT ASLI
// 1. FORWARD_KINEMATIC tidak pernah menulis DONE, dan tidak melihat EXECUTE.
chk('CACAT: FK.DONE tidak pernah TRUE', lin.done === false,
    'di ST, DONE memang tidak disebut sama sekali di badan FB');

// 2. INVERSE_KINEMATIC.DONE TRUE walau soft limit ditembus.
const jauh = K.forwardKinematic([9999, 45, -60, 0], cfg);   // X jauh di luar PD1300_001
const ikJauh = K.inverseKinematic(jauh.world, cfg);
chk('CACAT: IK.DONE tetap TRUE saat batas ditembus', ikJauh.done === true,
    'IF/ELSIF/ELSE menulis DONE:=TRUE di SEMUA cabang');
chk('batas yang kena tetap terbaca lewat limits[] (tambahan port JS, bukan dari ST)',
    ikJauh.limits[1] === true, 'X > PD1300_001 - 1');

// 3. ATAN kehilangan kuadran: pose dengan Y3 < 0 tidak bisa dikembalikan.
let ketemuY3Negatif = false, salahnya = 0;
for (const t1 of [150, 170]) {
  const j = [0, t1, -40, 0];
  const fk = K.forwardKinematic(j, cfg);
  const rc = K.reachable(fk.world, cfg);
  if (!(rc.ok && rc.y3 < 0)) continue;
  ketemuY3Negatif = true;
  const ik = K.inverseKinematic(fk.world, cfg);
  salahnya = Math.max(salahnya, Math.abs(ik.joint[1] - j[1]));
}
chk('CACAT: pose dengan Y3 < 0 meleset (ATAN, bukan ATAN2)',
    ketemuY3Negatif && salahnya > 100, 'selisih sudut 1 sampai ' + salahnya.toFixed(1) + ' derajat');
chk('atan2Fix() memang membetulkan kuadrannya - dan SENGAJA tidak dipakai sim',
    Math.abs(K.atan2Fix(-1, -1) - (-3 * Math.PI / 4)) < 1e-12);

// 4. ACOS tanpa penjaga: di luar jangkauan jadi NaN di JS (error runtime di PLC).
const luar = K.inverseKinematic([0, 5000, 5000, 0], cfg);
chk('CACAT: di luar jangkauan, IK menghasilkan NaN', Number.isNaN(luar.joint[1]),
    'di PLC ini error runtime, bukan angka salah');
chk('reachable() menangkapnya SEBELUM IK dipanggil', K.reachable([0, 5000, 5000, 0], cfg).ok === false,
    'penjaga ada di pemanggil, FB-nya tetap identik dengan yang di mesin');
chk('reachable() meloloskan titik yang memang terjangkau',
    K.reachable(K.forwardKinematic([0, 45, -60, 0], cfg).world, cfg).ok === true);

console.log(fail ? 'GAGAL ' + fail : 'LULUS');
process.exit(fail ? 1 : 0);
