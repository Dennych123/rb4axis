// Port JS dari FORWARD_KINEMATIC / INVERSE_KINEMATIC milik project mesin.
//
// Dipakai dua tempat: tes (blurobot/tests/kin.test.js) dan mode offline halaman
// viz - kalau simulator mati, lengan di layar tetap bergerak dan bedanya ditandai
// di layar, bukan diam-diam.
//
// ATURAN BERKAS INI: SETIA, BUKAN BENAR. Rumusnya disalin baris per baris dari
// blurobot/extract/*.st, termasuk tiga hal yang di ST-nya cacat:
//
//   1. INVERSE_KINEMATIC.DONE selalu TRUE - cabang IF/ELSIF/ELSE pemeriksa soft
//      limit menulis TRUE di SEMUA cabang, jadi hasil pemeriksaannya tidak pernah
//      sampai ke siapa pun. Di sini `done` juga selalu true; batas yang KENA
//      dilaporkan terpisah lewat `limits`, yang TIDAK ada di ST.
//   2. ALFA := ATAN(Z3/Y3) - ATAN, bukan ATAN2. Pose dengan Y3 < 0 meleset 180
//      derajat. Ditiru apa adanya; atan2Fix() ada buat MEMBANDINGKAN, bukan buat
//      menggantikan.
//   3. ACOS tanpa penjaga jangkauan - titik di luar L2+L3 bikin argumennya > 1.
//      Di ST hasilnya error runtime; di JS jadi NaN. Yang menjaga adalah
//      PEMANGGIL (reachable()), bukan fungsi ini.
//
// FORWARD_KINEMATIC juga tidak menyentuh EXECUTE maupun DONE sama sekali: dia
// menghitung tiap kali dipanggil, dan DONE-nya tidak pernah ditulis.
//
// Konstanta diambil dari tabel global project (extract/variables.tsv), BUKAN dari
// Math.PI. Beda di digit ke-10 memang kecil, tapi tes yang membandingkan hasil PLC
// dengan hasil JS jadi tidak bisa dipakai kalau konstantanya sendiri sudah beda.
'use strict';

const KIN_PI = 3.141592654;
const KIN_DEGREE_TO_RAD = 0.017453292;
const KIN_RAD_TO_DEGREE = 57.29577951;

// LREAL_TO_REAL di ST = pembulatan ke float 32 bit. Math.fround melakukan persis
// itu. Tanpa ini, nilai yang dibaca dari PLC (REAL) selalu beda tipis dari nilai
// JS (double) dan tesnya berisik tanpa sebab.
function toREAL(x) { return Math.fround(x); }

// Nilai tool dipakai dua kali dengan penjaga bagi-nol yang sama. Di ST penjaga itu
// MENULIS BALIK ke variabel globalnya (ROBOT_TOOL_Y_LREAL := 0.000000000000001) -
// efek samping yang bertahan sesudah FB selesai. Di sini nilainya cuma dipakai
// lokal; kalau ada yang perlu meniru efek sampingnya, dia harus menulis sendiri.
function toolPolar(cfg) {
  const y = cfg.toolY === 0 ? 0.000000000000001 : cfg.toolY;
  return { r: Math.sqrt(y * y + cfg.toolZ * cfg.toolZ), theta: Math.atan(cfg.toolZ / y) };
}

/**
 * FORWARD_KINEMATIC: sudut sumbu -> pose world.
 * @param {number[]} pos  [X mm, theta1 deg, theta2 deg, theta3 deg]
 * @param {object} cfg    {L1,L2,L3,L4,toolY,toolZ}
 * @returns {{joint:number[], world:number[], done:boolean}}
 */
function forwardKinematic(pos, cfg) {
  const deg = [pos[0], pos[1], pos[2], pos[3]];
  const rad = deg.map(d => d * KIN_DEGREE_TO_RAD);

  // Sumbu 0 lewat apa adanya: itu sumbu LINEAR (rel X), bukan sudut. Rantai
  // planarnya cuma sumbu 1-3, di bidang Y-Z.
  const fk = [
    deg[0],
    cfg.L2 * Math.cos(rad[1])
      + cfg.L3 * Math.cos(rad[1] + rad[2])
      + cfg.L4 * Math.cos(rad[1] + rad[2] + rad[3]),
    cfg.L1
      + cfg.L2 * Math.sin(rad[1])
      + cfg.L3 * Math.sin(rad[1] + rad[2])
      + cfg.L4 * Math.sin(rad[1] + rad[2] + rad[3]),
    deg[1] + deg[2] + deg[3]
  ];

  const tool = toolPolar(cfg);
  const thetaEe = rad[1] + rad[2] + rad[3];

  return {
    joint: deg.map(toREAL),
    world: [
      toREAL(fk[0]),
      toREAL(fk[1] + tool.r * Math.cos(tool.theta + thetaEe)),
      toREAL(fk[2] + tool.r * Math.sin(tool.theta + thetaEe)),
      toREAL(fk[3])
    ],
    done: false          // ST tidak pernah menulis DONE di FB ini
  };
}

/**
 * INVERSE_KINEMATIC: pose world -> sudut sumbu.
 * @param {number[]} pos  [X mm, Y mm, Z mm, theta_EE deg]
 * @param {object} cfg    {L1..L4, toolY, toolZ, offset[5], limit[8]}
 * @returns {{joint:number[], done:boolean, limits:boolean[], reachable:boolean}}
 */
function inverseKinematic(pos, cfg) {
  const tool = toolPolar(cfg);
  const thEe = pos[3] * KIN_DEGREE_TO_RAD;

  const yEe = pos[1] - tool.r * Math.cos(tool.theta + thEe);
  const zEe = pos[2] - tool.r * Math.sin(tool.theta + thEe);

  const y3 = yEe - cfg.L4 * Math.cos(thEe);
  const z3 = (zEe - cfg.L1) - cfg.L4 * Math.sin(thEe);

  const r = Math.sqrt(y3 * y3 + z3 * z3);
  const beta = Math.acos((cfg.L2 * cfg.L2 + cfg.L3 * cfg.L3 - r * r) / (2 * cfg.L2 * cfg.L3));
  const gamma = Math.acos((r * r + cfg.L2 * cfg.L2 - cfg.L3 * cfg.L3) / (2 * cfg.L2 * r));
  const alfa = Math.atan(z3 / y3);                 // ATAN, bukan ATAN2 - lihat kepala berkas

  const rad1 = alfa + gamma;
  const rad2 = beta - KIN_PI;
  const rad3 = thEe - rad1 - rad2;

  const d1 = rad1 * KIN_RAD_TO_DEGREE;
  const d2 = rad2 * KIN_RAD_TO_DEGREE;
  const d3 = rad3 * KIN_RAD_TO_DEGREE;

  const off = cfg.offset || [0, 0, 0, 0, 0];
  const joint = [pos[0] - off[0], d1 - off[1], d2 - off[2], d3 - off[3]];

  // Bagian ini ADA di ST tapi hasilnya dibuang (DONE ditulis TRUE di semua cabang).
  // Di sini dilaporkan supaya sim dan halaman punya sesuatu yang bisa dipakai.
  const L = cfg.limit || [];
  const limits = [
    pos[0] < (L[0] + 1), pos[0] > (L[1] - 1),
    d1 < (L[2] + 1), d1 > (L[3] - 1),
    d2 < (L[4] + 1), d2 > (L[5] - 1),
    d3 < (L[6] + 1), d3 > (L[7] - 1)
  ];

  return { joint, done: true, limits, reachable: isFinite(beta) && isFinite(gamma) && isFinite(alfa) };
}

/**
 * Penjaga yang ADA DI PEMANGGIL, bukan di dalam FB: titik di luar jangkauan bikin
 * argumen ACOS > 1 dan di PLC itu error runtime, bukan hasil yang salah. Program
 * sim memanggil ini sebelum IK, jadi FB-nya tetap identik dengan yang di mesin.
 */
function reachable(pos, cfg) {
  const tool = toolPolar(cfg);
  const thEe = pos[3] * KIN_DEGREE_TO_RAD;
  const y3 = (pos[1] - tool.r * Math.cos(tool.theta + thEe)) - cfg.L4 * Math.cos(thEe);
  const z3 = ((pos[2] - tool.r * Math.sin(tool.theta + thEe)) - cfg.L1) - cfg.L4 * Math.sin(thEe);
  const r = Math.sqrt(y3 * y3 + z3 * z3);
  return {
    ok: y3 !== 0 && r <= (cfg.L2 + cfg.L3) && r >= Math.abs(cfg.L2 - cfg.L3) && r > 0,
    r, y3, z3
  };
}

/** Bandingan: rumus yang BENAR untuk ALFA. Tidak dipakai sim - buat tes saja. */
function atan2Fix(z3, y3) { return Math.atan2(z3, y3); }

/**
 * Titik-titik rantai dalam koordinat PLC (X, Y, Z) - buat menggambar lengannya.
 *
 * Ini SATU-SATUNYA sumber posisi buat viz 3D. Menggambar dari sudut sendi langsung
 * di halaman berarti rumus rantai ditulis dua kali, dan yang kedua bebas melenceng
 * tanpa satu pun tes yang mengeluh - gambarnya tetap tampak wajar, cuma menceritakan
 * robot yang lain. Titik terakhir di sini WAJIB sama dengan world hasil
 * forwardKinematic(); tests/viz.test.js yang menjaganya.
 *
 * @returns {{x:number,y:number,z:number}[]} kereta, bahu, siku, pergelangan, ujung L4, tool
 */
function chainPoints(pos, cfg) {
  const a1 = pos[1] * KIN_DEGREE_TO_RAD;
  const a2 = a1 + pos[2] * KIN_DEGREE_TO_RAD;
  const a3 = a2 + pos[3] * KIN_DEGREE_TO_RAD;
  const tool = toolPolar(cfg);
  const p = [{ x: pos[0], y: 0, z: 0 }];
  p.push({ x: pos[0], y: 0, z: cfg.L1 });
  const tambah = (r, sudut) => {
    const t = p[p.length - 1];
    p.push({ x: t.x, y: t.y + r * Math.cos(sudut), z: t.z + r * Math.sin(sudut) });
  };
  tambah(cfg.L2, a1);
  tambah(cfg.L3, a2);
  tambah(cfg.L4, a3);
  tambah(tool.r, tool.theta + a3);
  return p;
}

if (typeof module !== 'undefined') {
  module.exports = { forwardKinematic, inverseKinematic, reachable, atan2Fix, toolPolar, toREAL,
                     chainPoints,
                     KIN_PI, KIN_DEGREE_TO_RAD, KIN_RAD_TO_DEGREE };
}
