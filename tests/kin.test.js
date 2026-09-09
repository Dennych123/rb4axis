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

// ===========================================================================
// V2 - versi yang DIPERBAIKI (blurobot/sim/*_V2.st, dicerminkan di kin.js).
//
// Tes di atas MENUNTUT perilaku yang salah karena itu yang jalan di mesin. Tes di
// bawah menuntut yang benar. Dua-duanya harus hijau berbarengan: kalau yang atas
// ikut hijau cuma karena V1 diam-diam diganti V2, cacatnya berhenti terdokumentasi
// dan tidak ada lagi yang bisa diadu ke project mesin.
// ===========================================================================
console.log('  --- V2');

// Panjang gripper ikut TCP, sama seperti yang ditulis gen_sim ke ROBOT_TOOL_Y.
const cfg2 = Object.assign({}, cfg, { toolY: raw.tool.Y + raw.gripper.panjang,
                                      gripLen: raw.gripper.panjang });

// 1. Round-trip di SELURUH kuadran, termasuk yang di V1 meleset 180 derajat.
let r2 = 0, n2 = 0, dilewati = 0;
for (const t1 of [15, 45, 90, 135, 170]) {
  for (const t2 of [-140, -100, -60, -25]) {
    for (const t3 of [-40, 0, 40]) {
      const j = [80, t1, t2, t3];
      const fk = K.forwardKinematicV2(j, cfg2);
      const ik = K.inverseKinematicV2(fk.worldL, cfg2, false);
      if (ik.errorId === 1 || ik.errorId === 2 || ik.errorId === 3) { dilewati++; continue; }
      n2++;
      for (let i = 0; i < 4; i++) r2 = Math.max(r2, Math.abs(ik.joint[i] - j[i]));
    }
  }
}
// Toleransi 1e-5, dan batas itu BUKAN kelonggaran: DEGREE_TO_RAD dan RAD_TO_DEGREE
// di project mesin dipotong 9 angka, jadi perkaliannya bukan 1 melainkan
// 1 - 2.98e-8. Round-trip sudut 170 derajat karena itu tidak mungkin lebih rapat
// dari ~5e-6 - berapa pun benarnya rumusnya. Menuntut lebih rapat berarti menuntut
// konstanta yang lain, dan konstanta itu yang dipakai PLC.
const BATAS_KONSTANTA = 1e-5;
chk('V2: round-trip menutup di semua kuadran (' + n2 + ' pose, ' + dilewati + ' di luar jangkauan)',
    n2 >= 40 && r2 < BATAS_KONSTANTA, 'selisih terbesar ' + r2.toExponential(2)
    + ' - lantainya pembulatan konstanta project, bukan rumusnya');
chk('konstanta project memang bukan invers eksak',
    Math.abs(K.KIN_DEGREE_TO_RAD * K.KIN_RAD_TO_DEGREE - 1) > 1e-9,
    'D2R x R2D - 1 = ' + (K.KIN_DEGREE_TO_RAD * K.KIN_RAD_TO_DEGREE - 1).toExponential(3)
    + ' - itu sebabnya toleransi round-trip tidak bisa 1e-9');

// 2. Kuadran: pose yang di V1 meleset TEPAT 180 derajat, di V2 kembali utuh.
const jY3 = [0, 165, -40, 0];
const fkY3 = K.forwardKinematicV2(jY3, cfg2);
const ikY3v1 = K.inverseKinematic(fkY3.worldL, cfg2);
const ikY3v2 = K.inverseKinematicV2(fkY3.worldL, cfg2, false);
chk('V2: pose Y3 < 0 kembali benar', Math.abs(ikY3v2.joint[1] - 165) < BATAS_KONSTANTA,
    'V1 di pose yang sama: ' + ikY3v1.joint[1].toFixed(1) + ' derajat');
chk('V1 dan V2 memang BEDA di pose itu', Math.abs(ikY3v1.joint[1] - ikY3v2.joint[1]) > 100,
    'ini bukti perbaikannya nyata, bukan dua nama untuk rumus yang sama');
// atanKuadran sama dengan atan2 MODULO 2 PI, bukan sama persis: jangkauannya
// (-PI/2, 3PI/2] sementara atan2 (-PI, PI]. Bedanya disengaja dan harus dicatat,
// karena berpengaruh ke SOFT LIMIT: pose yang menjangkau ke belakang menghasilkan
// theta1 sekitar +200 derajat, bukan -160. Dua-duanya pose yang sama, tapi yang
// dibandingkan ke PD1300_003 (maks 180) angkanya - jadi batas atas kena, batas
// bawah tidak. Kalau nanti perlu -160, normalisasinya ditambahkan SEKALI di FB dan
// batasnya ikut ditinjau, bukan ditambal di pemanggil.
chk('atanKuadran = Math.atan2 modulo 2 PI di keempat kuadran',
    [[1, 1], [1, -1], [-1, -1], [-1, 1], [1, 0], [-1, 0]].every(([z, y]) => {
      const d = Math.abs(K.atanKuadran(z, y) - Math.atan2(z, y));
      return d < 1e-8 || Math.abs(d - 2 * Math.PI) < 1e-8;
    }),
    'dibangun dari ATAN karena ATAN2 tidak ada di daftar instruksi W560');
chk('jangkauan atanKuadran (-PI/2, 3PI/2] - bukan jangkauan atan2',
    K.atanKuadran(-1, -1) > Math.PI && Math.atan2(-1, -1) < 0,
    'theta1 buat pose ke belakang keluar sebagai +200 derajat, bukan -160');

// 3. Di luar jangkauan: DITOLAK, bukan NaN.
const jauh2 = K.inverseKinematicV2([0, 5000, 5000, 0], cfg2, false);
chk('V2: di luar jangkauan -> DONE FALSE + ERROR_ID 1', !jauh2.done && jauh2.errorId === 1);
// Titik yang terlalu DEKAT: pergelangan cuma 10 mm dari bahu, sementara lengan
// terpendek yang bisa dibentuk |L2-L3| = 50 mm. Pose sebelumnya ([0,0,L1,0]) salah
// dipakai buat ini - pergelangannya justru jatuh 240 mm di belakang bahu, masih
// terjangkau, jadi yang dilaporkan soft limit (4), bukan terlalu dekat (2).
const dekat = K.inverseKinematicV2([0, cfg2.toolY + cfg2.L4 + 10, cfg2.L1, 0], cfg2, false);
chk('V2: terlalu dekat -> ERROR_ID 2', !dekat.done && dekat.errorId === 2, 'ERROR_ID ' + dekat.errorId);

// 4. DONE sekarang BERARTI sesuatu - itu perbaikan yang paling menentukan.
const luarBatas = K.forwardKinematicV2([9999, 45, -60, 0], cfg2);
const ikBatas = K.inverseKinematicV2(luarBatas.worldL, cfg2, false);
chk('V2: soft limit ditembus -> DONE FALSE, ERROR_ID 4',
    !ikBatas.done && ikBatas.errorId === 4 && ikBatas.limits[1] === true);
chk('V2: sudutnya tetap dikeluarkan walau ditolak', isFinite(ikBatas.joint[1]),
    'pemanggil bisa menunjukkan "seharusnya ke sini, ditolak batas"');
const dalamBatas = K.inverseKinematicV2(K.forwardKinematicV2([100, 45, -60, 0], cfg2).worldL, cfg2, false);
chk('V2: pose sah -> DONE TRUE', dalamBatas.done && dalamBatas.errorId === 0);

// 5. FK V2 menghormati EXECUTE dan menulis DONE.
chk('V2: FK EXECUTE=false -> DONE FALSE', K.forwardKinematicV2([0, 45, -60, 0], cfg2, false).done === false);
chk('V2: FK EXECUTE=true -> DONE TRUE', K.forwardKinematicV2([0, 45, -60, 0], cfg2).done === true);

// 6. Elbow up: cabang cermin yang di V1 tidak pernah ada.
const poseE = K.forwardKinematicV2([0, 60, -80, 20], cfg2).worldL;
const eDown = K.inverseKinematicV2(poseE, cfg2, false);
const eUp = K.inverseKinematicV2(poseE, cfg2, true);
chk('V2: elbow down = cabang mesin (theta2 negatif)', eDown.joint[2] < 0, eDown.joint[2].toFixed(1));
chk('V2: elbow up = cermin (theta2 positif)', eUp.joint[2] > 0, eUp.joint[2].toFixed(1));
chk('V2: dua-duanya sampai ke pose yang SAMA', (() => {
  const a = K.forwardKinematicV2(eDown.joint, cfg2).worldL;
  const b = K.forwardKinematicV2(eUp.joint, cfg2).worldL;
  return a.every((v, i) => Math.abs(v - b[i]) < 1e-6);
})(), 'kalau tidak, cabang cerminnya salah rumus - bukan solusi lain, tapi pose lain');

// 7. Tool tanpa panjang: V2 tidak lagi mengarang sudut 90 derajat.
const t0 = K.toolPolarV2({ toolY: 0, toolZ: 0 });
chk('V2: tool panjang nol -> sudutnya nol, bukan 90 derajat', t0.r === 0 && t0.theta === 0,
    'V1 memaksa Y jadi 1e-15, dan itu bikin THETA_TOOL melompat ke ~90');
chk('V2: tool arah -Y dibedakan dari +Y',
    Math.abs(K.toolPolarV2({ toolY: -50, toolZ: 0 }).theta - K.KIN_PI) < 1e-8,
    'ATAN polos memberi 0 untuk dua-duanya');


// ===========================================================================
// 8. PEMBANDING: DH dan Jacobian
//
// Mesin memakai rumus geometri tertutup. Buku teks memakai DH dan Jacobian. Kalau
// dua-duanya benar, jawabannya sama - dan itu yang diuji di sini, bukan bahwa
// kodenya ada.
// ===========================================================================
const cfgJ = Object.assign({}, cfg2, { limit: raw ? undefined : undefined });

// --- FK: rumus mesin vs perkalian matriks DH
let bedaDH = 0;
const posUji = [[0, 0, 0, 0], [100, 45, -74.3, -60.7], [-300, 90, -90, -90],
                [250, 120, -140, 60], [0, 170, -150, 110]];
for (const j of posUji) {
  const a = K.fkSteps(j, cfg2).worldL;
  const b = K.fkDH(j, cfg2).world;
  for (let i = 0; i < 4; i++) bedaDH = Math.max(bedaDH, Math.abs(a[i] - b[i]));
}
// Toleransinya 1e-4, bukan nol, dan sebabnya ada isinya: DH memakai PI/2 utuh untuk
// sudut offsetnya, sementara rumus mesin memakai 90 x DEGREE_TO_RAD yang dipotong 9
// angka. Sudut yang sama, pembulatan yang beda - lantai yang sama dengan round-trip.
chk('DH memberi pose yang sama dengan rumus mesin (' + posUji.length + ' pose)', bedaDH < 1e-4,
    'selisih terbesar ' + bedaDH.toExponential(2) + ' (lantai konstanta)');
chk('DH bukan kebetulan: pose nol lurus ke +Y', (() => {
  const w = K.fkDH([0, 0, 0, 0], cfg2).world;
  return Math.abs(w[1] - (cfg2.L2 + cfg2.L3 + cfg2.L4 + cfg2.toolY)) < 1e-4
      && Math.abs(w[2] - cfg2.L1) < 1e-4;
})());
chk('tabel DH punya enam baris, semua alpha nol (rantai sebidang)', (() => {
  const t = K.dhTable([0, 30, -40, 10], cfg2);
  return t.length === 6 && t.every(b => b.alpha === 0);
})(), 'alpha bukan nol berarti ada sendi yang tidak sebidang - lengan ini tidak begitu');

// --- Jacobian diadu ke beda-hingga FK. Ini yang membuktikan turunannya benar;
// mencocokkannya ke rumus yang ditulis tangan cuma membuktikan dua tulisan sama.
let bedaJ = 0;
for (const j of [[0, 45, -74.3, -60.7], [0, 90, -90, -90], [0, 20, -30, 15]]) {
  const J = K.jacobian(j, cfg2).J;
  const h = 1e-6;                                  // radian
  for (let c = 0; c < 3; c++) {
    const jp = j.slice(), jm = j.slice();
    jp[c + 1] += h * K.KIN_RAD_TO_DEGREE;
    jm[c + 1] -= h * K.KIN_RAD_TO_DEGREE;
    const wp = K.fkSteps(jp, cfg2).worldL, wm = K.fkSteps(jm, cfg2).worldL;
    bedaJ = Math.max(bedaJ,
      Math.abs((wp[1] - wm[1]) / (2 * h) - J[0][c]),
      Math.abs((wp[2] - wm[2]) / (2 * h) - J[1][c]));
  }
}
chk('Jacobian = turunan FK yang sebenarnya (beda-hingga)', bedaJ < 1e-3,
    'selisih terbesar ' + bedaJ.toExponential(2) + ' mm/rad');

// det J untuk lengan sebidang ini punya bentuk pendek. Kalau ini merah, bukan
// rumusnya yang salah - Jacobian-nya yang tidak menggambarkan lengan ini.
let bedaDet = 0;
for (const t2 of [-140, -74.3, -20, 20, 90]) {
  const d = K.jacobian([0, 45, t2, 10], cfg2).det;
  bedaDet = Math.max(bedaDet, Math.abs(d - cfg2.L2 * cfg2.L3 * Math.sin(t2 * K.KIN_DEGREE_TO_RAD)));
}
chk('det J = L2 L3 sin(theta2)', bedaDet < 1e-6, bedaDet.toExponential(2));
chk('siku lurus = singular (det J nol)',
    Math.abs(K.jacobian([0, 45, 0, 10], cfg2).det) < 1e-9,
    'di situ ujung tool tidak bisa lagi bergerak ke segala arah');

// --- IK: tertutup vs Jacobian
const poseIK = K.fkSteps([0, 45, -74.3, -60.7], cfg2).worldL;
const ikTutup = K.inverseKinematicV2(poseIK, cfg2, false);
const ikIter = K.ikJacobian(poseIK, [0, 40, -70, -55], cfg2);
chk('IK Jacobian sampai ke jawaban yang sama dengan IK tertutup',
    ikIter.done && [1, 2, 3].every(i => Math.abs(ikIter.joint[i] - ikTutup.joint[i]) < 1e-3),
    ikIter.iter + ' putaran, sisa ' + ikIter.sisa.toExponential(1));
chk('IK Jacobian butuh BEBERAPA putaran, IK tertutup nol', ikIter.iter >= 1 && ikIter.iter <= 20,
    ikIter.iter + ' - itu harga yang dibayar untuk cara yang berlaku di rantai apa pun');

// Tebakan awal MENENTUKAN cabang mana yang ketemu, dan itu perbedaan yang paling
// penting antara dua cara ini. IK tertutup memilih cabangnya lewat parameter yang
// eksplisit (elbowUp). Yang iteratif jatuh ke cabang yang paling dekat dengan
// tebakannya - jadi jawabannya ikut berubah kalau posisi awal robot berubah, tanpa
// ada yang meminta.
chk('tebakan menentukan CABANG yang ketemu', (() => {
  const atas = K.inverseKinematicV2(poseIK, cfg2, true);          // cabang cermin
  const r = K.ikJacobian(poseIK, atas.joint, cfg2);
  if (!r.done) return false;
  // Berangkat dari cabang cermin, berakhir di cabang cermin - bukan di cabang mesin.
  return Math.abs(r.joint[2] - atas.joint[2]) < 1e-2
      && Math.abs(r.joint[2] - ikTutup.joint[2]) > 10;
})(), 'yang tertutup memilih cabang lewat parameter; yang iteratif ikut tebakannya');

// --- Lurus atau tidak. Ini klaim yang dipakai panel, jadi dijaga di sini.
const gerakUji = { dt: 0.004, vel: [900, 90, 90, 120], acc: [1800, 240, 240, 320] };
const pA = K.fkSteps([0, 45, -74.3, -60.7], cfg2).worldL;
const pB = [pA[0], pA[1], pA[2] + 150, pA[3]];
const jA = K.inverseKinematicV2(pA, cfg2, false).joint;
const jB = K.inverseKinematicV2(pB, cfg2, false).joint;
const lintJ = K.pathJoint(jA, jB, cfg2, gerakUji);
const lintL = K.pathLine(pA, pB, cfg2, 80, false, false);
const tA = { x: pA[0], y: pA[1], z: pA[2] }, tB = { x: pB[0], y: pB[1], z: pB[2] };
const devJ = K.deviasiLurus(lintJ.titik, tA, tB);
const devL = K.deviasiLurus(lintL.titik, tA, tB);
chk('gerak sumbu MELENGKUNG (bukan garis lurus)', devJ.maks > 5,
    'simpangan terbesar ' + devJ.maks.toFixed(1) + ' mm pada perpindahan 150 mm');
chk('gerak lurus memang lurus', devL.maks < 0.01, devL.maks.toExponential(1) + ' mm');
chk('gerak sumbu tetap SAMPAI di tujuan',
    [1, 2, 3].every(i => Math.abs(lintJ.sudut[lintJ.sudut.length - 1][i] - jB[i]) < 1e-9),
    'melengkung di tengah jalan itu wajar; tidak sampai itu cacat');
chk('lintasan lurus melaporkan titik yang tidak punya solusi', (() => {
  // Garis lurus antara dua pose yang sah bisa keluar dari ruang kerja di tengahnya.
  // Yang dituntut di sini: gagalnya DIHITUNG, bukan diam-diam dilewati.
  const jauh = K.pathLine(pA, [pA[0], pA[1] + 900, pA[2], pA[3]], cfg2, 40, false, false);
  return jauh.gagal > 0;
})());

// Motion model cuma ADA SATU salinan di JS, dan dia di kin.js. Halaman memakainya
// lewat sini; salinan kedua di halaman pasti berbeda suatu hari, dan bedanya terbaca
// seperti PLC-nya yang salah.
chk('langkahSumbu ada di kin.js', typeof K.langkahSumbu === 'function');
chk('halaman TIDAK punya salinan motion model sendiri', (() => {
  const r = require('fs').readFileSync(
    require('path').join(__dirname, '..', 'web', 'robot.js'), 'utf8');
  return !/function langkahSumbu\(/.test(r);
})());

console.log(fail ? 'GAGAL ' + fail : 'LULUS');
process.exit(fail ? 1 : 0);
