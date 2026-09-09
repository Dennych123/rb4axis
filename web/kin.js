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

// ===========================================================================
// V2 - versi yang DIPERBAIKI. Cerminan baris per baris dari
// blurobot/sim/FORWARD_KINEMATIC_V2.st dan INVERSE_KINEMATIC_V2.st.
//
// Kuadran dibetulkan dengan ATAN + koreksi eksplisit, BUKAN Math.atan2 - persis
// seperti ST-nya, yang juga tidak boleh memakai ATAN2 karena instruksi itu tidak
// ada di daftar 353 instruksi W560. Kalau di sini dipakai Math.atan2 dan di sana
// konstruksi manual, dua-duanya "benar" tapi hasilnya bisa beda di tepi kuadran -
// dan perbandingan PLC vs JS berhenti berarti.
// ===========================================================================

/** Sudut polar dengan kuadran benar, dibangun dari ATAN saja. */
function atanKuadran(z, y) {
  if (y > 0) return Math.atan(z / y);
  if (y < 0) return Math.atan(z / y) + KIN_PI;
  return z >= 0 ? KIN_PI / 2 : -KIN_PI / 2;
}

/** Tool polar tanpa efek samping. R=0 berarti tidak ada arah, bukan 90 derajat. */
function toolPolarV2(cfg) {
  const r = Math.sqrt(cfg.toolY * cfg.toolY + cfg.toolZ * cfg.toolZ);
  return { r, theta: r === 0 ? 0 : atanKuadran(cfg.toolZ, cfg.toolY) };
}

/**
 * FORWARD_KINEMATIC_V2.
 * @returns {{joint:number[], world:number[], worldL:number[], done:boolean}}
 *          world = REAL (dibulatkan 32-bit, seperti yang dibaca HMI/OPC UA),
 *          worldL = LREAL penuh (yang dipakai round-trip supaya tidak berisik)
 */
function forwardKinematicV2(pos, cfg, execute) {
  if (execute === false) return { joint: [0, 0, 0, 0], world: [0, 0, 0, 0], worldL: [0, 0, 0, 0], done: false };
  const s = fkSteps(pos, cfg);
  return { joint: pos.map(toREAL), world: s.worldL.map(toREAL), worldL: s.worldL, done: true };
}

/**
 * FK, dengan SEMUA nilai antaranya ikut keluar - buat panel yang menjelaskan cara
 * kerjanya sambil menunjukkan angkanya bergerak.
 *
 * Ini BUKAN salinan kedua rumusnya: forwardKinematicV2 memanggil fungsi ini, jadi yang
 * ditampilkan panel benar-benar angka yang dipakai menggambar. Panel yang menghitung
 * sendiri pasti melenceng suatu hari, dan melencengnya diam - penjelasan yang salah
 * lebih berbahaya daripada tidak ada penjelasan.
 */
function fkSteps(pos, cfg) {
  const tool = toolPolarV2(cfg);
  const a1 = pos[1] * KIN_DEGREE_TO_RAD;
  const a2 = a1 + pos[2] * KIN_DEGREE_TO_RAD;
  const a3 = a2 + pos[3] * KIN_DEGREE_TO_RAD;

  // Sumbangan tiap ruas dipisah supaya panel bisa menunjukkan "L2 menyumbang sekian,
  // L3 sekian" - itu yang bikin rumusnya berhenti terlihat seperti mantra.
  const c = [cfg.L2 * Math.cos(a1), cfg.L3 * Math.cos(a2), cfg.L4 * Math.cos(a3)];
  const sn = [cfg.L2 * Math.sin(a1), cfg.L3 * Math.sin(a2), cfg.L4 * Math.sin(a3)];
  const wy = c[0] + c[1] + c[2];
  const wz = cfg.L1 + sn[0] + sn[1] + sn[2];
  const thTool = tool.theta + a3;

  const worldL = [
    pos[0],
    wy + tool.r * Math.cos(thTool),
    wz + tool.r * Math.sin(thTool),
    pos[1] + pos[2] + pos[3]
  ];
  return {
    a1: a1, a2: a2, a3: a3, cos: c, sin: sn, wy: wy, wz: wz,
    toolR: tool.r, toolTheta: tool.theta, thTool: thTool,
    dy: tool.r * Math.cos(thTool), dz: tool.r * Math.sin(thTool),
    worldL: worldL
  };
}

/**
 * INVERSE_KINEMATIC_V2.
 * @param {boolean} elbowUp  false = cabang yang sama dengan mesin
 * @returns {{joint:number[], done:boolean, error:boolean, errorId:number,
 *            limits:boolean[], limitAny:boolean}}
 *          errorId: 1 di luar jangkauan, 2 terlalu dekat, 3 singular, 4 soft limit
 */
function inverseKinematicV2(pos, cfg, elbowUp) {
  const s = ikSteps(pos, cfg, elbowUp);
  if (!s.done) {
    return { joint: [NaN, NaN, NaN, NaN], done: false, error: true, errorId: s.errorId,
             limits: [false, false, false, false, false, false, false, false], limitAny: false };
  }
  return { joint: s.joint, done: !s.limitAny, error: s.limitAny, errorId: s.limitAny ? 4 : 0,
           limits: s.limits, limitAny: s.limitAny };
}

/**
 * IK dengan seluruh nilai antaranya - pasangan fkSteps, dan sumber tunggal yang sama:
 * inverseKinematicV2 memanggil fungsi ini.
 *
 * Nama yang dikeluarkan sengaja PERSIS nama di INVERSE_KINEMATIC_V2.st (Y3, Z3, R,
 * BETA, GAMMA, ALFA), supaya panel penjelas dan berkas ST bisa dibaca berdampingan
 * tanpa menerjemahkan apa pun.
 *
 * done=false berarti pose ditolak SEBELUM ACOS; nilai yang sudah sempat dihitung tetap
 * dikembalikan supaya panel bisa menunjukkan DI MANA gagalnya, bukan cuma bahwa gagal.
 */
function ikSteps(pos, cfg, elbowUp) {
  const tool = toolPolarV2(cfg);
  const thEe = pos[3] * KIN_DEGREE_TO_RAD;

  // Mundur dari TCP: buang tool, lalu buang L4 - sisanya titik pergelangan, dan dari
  // situ soalnya tinggal segitiga dua sisi (L2, L3) dengan alas R.
  const y3 = (pos[1] - tool.r * Math.cos(tool.theta + thEe)) - cfg.L4 * Math.cos(thEe);
  const z3 = ((pos[2] - tool.r * Math.sin(tool.theta + thEe)) - cfg.L1) - cfg.L4 * Math.sin(thEe);
  const r = Math.sqrt(y3 * y3 + z3 * z3);
  const dasar = { thEe: thEe, y3: y3, z3: z3, r: r, toolR: tool.r, toolTheta: tool.theta,
                  jangkauMaks: cfg.L2 + cfg.L3, jangkauMin: Math.abs(cfg.L2 - cfg.L3) };

  // Jangkauan diperiksa DULU. Sesudah ACOS tidak menolong: yang meledak ACOS-nya.
  if (r > cfg.L2 + cfg.L3) return Object.assign({ done: false, errorId: 1 }, dasar);
  if (r < Math.abs(cfg.L2 - cfg.L3)) return Object.assign({ done: false, errorId: 2 }, dasar);
  if (r === 0) return Object.assign({ done: false, errorId: 3 }, dasar);

  const beta = Math.acos((cfg.L2 * cfg.L2 + cfg.L3 * cfg.L3 - r * r) / (2 * cfg.L2 * cfg.L3));
  const gamma = Math.acos((r * r + cfg.L2 * cfg.L2 - cfg.L3 * cfg.L3) / (2 * cfg.L2 * r));
  const alfa = atanKuadran(z3, y3);

  const rad1 = elbowUp ? alfa - gamma : alfa + gamma;
  const rad2 = elbowUp ? KIN_PI - beta : beta - KIN_PI;
  const rad3 = thEe - rad1 - rad2;

  const d1 = rad1 * KIN_RAD_TO_DEGREE, d2 = rad2 * KIN_RAD_TO_DEGREE, d3 = rad3 * KIN_RAD_TO_DEGREE;
  const off = cfg.offset || [0, 0, 0, 0, 0];
  const joint = [pos[0] - off[0], d1 - off[1], d2 - off[2], d3 - off[3]];

  const L = cfg.limit || [];
  const limits = [
    joint[0] < (L[0] + 1), joint[0] > (L[1] - 1),
    d1 < (L[2] + 1), d1 > (L[3] - 1),
    d2 < (L[4] + 1), d2 > (L[5] - 1),
    d3 < (L[6] + 1), d3 > (L[7] - 1)
  ];

  // Sudutnya tetap dikeluarkan walau batas ditembus - yang ditahan cuma DONE, jadi
  // pemanggil bisa menunjukkan "seharusnya ke sini, ditolak batas".
  return Object.assign({ done: true, errorId: 0, beta: beta, gamma: gamma, alfa: alfa,
                         elbowUp: !!elbowUp, rad1: rad1, rad2: rad2, rad3: rad3,
                         joint: joint, limits: limits, limitAny: limits.some(Boolean) }, dasar);
}

/**
 * Titik-titik rantai dalam koordinat PLC (X, Y, Z) - buat menggambar lengannya.
 *
 * Ini SATU-SATUNYA sumber posisi buat viz 3D. Menggambar dari sudut sendi langsung
 * di halaman berarti rumus rantai ditulis dua kali, dan yang kedua bebas melenceng
 * tanpa satu pun tes yang mengeluh - gambarnya tetap tampak wajar, cuma menceritakan
 * robot yang lain. Titik terakhir di sini WAJIB sama dengan world hasil
 * forwardKinematic(); tests/viz.test.js yang menjaganya.
 *
 * Titiknya TUJUH: kereta, bahu, siku, pergelangan, ujung L4, pangkal gripper, TCP.
 * Pangkal gripper dihitung mundur dari TCP sejauh `cfg.gripLen` - panjang gripper
 * SUDAH termasuk di cfg.toolY (dijumlahkan sekali waktu gen_sim menulis
 * ROBOT_TOOL_Y_LREAL), jadi menambahkannya lagi di sini bikin lengan panjang dua
 * kali gripper. Tanpa gripper (gripLen 0) titik ke-6 dan ke-7 berimpit.
 *
 * @returns {{x:number,y:number,z:number}[]}
 */
function chainPoints(pos, cfg) {
  const a1 = pos[1] * KIN_DEGREE_TO_RAD;
  const a2 = a1 + pos[2] * KIN_DEGREE_TO_RAD;
  const a3 = a2 + pos[3] * KIN_DEGREE_TO_RAD;
  const tool = toolPolarV2(cfg);
  const p = [{ x: pos[0], y: 0, z: 0 }];
  p.push({ x: pos[0], y: 0, z: cfg.L1 });
  const tambah = (r, sudut) => {
    const t = p[p.length - 1];
    p.push({ x: t.x, y: t.y + r * Math.cos(sudut), z: t.z + r * Math.sin(sudut) });
  };
  tambah(cfg.L2, a1);
  tambah(cfg.L3, a2);
  tambah(cfg.L4, a3);

  const gripLen = Math.min(cfg.gripLen || 0, tool.r);
  tambah(tool.r - gripLen, tool.theta + a3);     // pangkal gripper
  tambah(gripLen, tool.theta + a3);              // TCP = ujung jari
  return p;
}

/**
 * Titik ujung dua jari gripper, buat digambar. Jari membuka SEPANJANG SUMBU X -
 * arah rel - bukan di bidang lengan.
 *
 * Itu bukan pilihan gambar: gripper turun TEGAK dari atas (theta_EE -90) dan
 * menjepit sisi KIRI-KANAN produk, sisi yang sama yang dipegang mesin aslinya.
 * Jari yang membuka di bidang Y-Z menjepit sisi depan-belakang, dan dari kamera
 * mana pun itu tetap terlihat seperti "menjepit" - salah yang tidak mengeluh.
 *
 * Sumbu X juga satu-satunya arah yang TIDAK ikut berputar bersama lengan: rantai
 * 1-3 seluruhnya planar di Y-Z, jadi bukaan jari tetap sejajar rel di pose mana pun.
 *
 * @param {number} bukaan  jarak antar jari (mm). Menutup berhenti di lebar produk,
 *                         bukan di 0 - lihat gripper.tutup di robot.config.json.
 */
function gripperPoints(pos, cfg, bukaan) {
  const p = chainPoints(pos, cfg);
  const pangkal = p[5], tcp = p[6];
  const h = bukaan / 2;
  return {
    pangkal, tcp,
    jari: [1, -1].map(s => ({
      atas: { x: pangkal.x + s * h, y: pangkal.y, z: pangkal.z },
      ujung: { x: tcp.x + s * h, y: tcp.y, z: tcp.z }
    }))
  };
}

/**
 * Penjaga tabrakan - CERMINAN dari blok penjaga di PRG_SIM_ROBOT.st, dipakai
 * halaman waktu simulator mati.
 *
 * Yang menentukan tetap PLC: waktu tersambung, halaman cuma menampilkan SIM_COLLIDE
 * yang dihitung di sana. Fungsi ini supaya mode offline tidak jadi mode yang bisa
 * menembus mesin - kalau bisa, orang belajar bahwa menembus mesin itu wajar.
 *
 * Badan mesin = kotak: lebar (arah rel) x dalam, dari lantai sampai PERMUKAAN
 * stasiun. Permukaannya sendiri bukan tabrakan - produk memang diletakkan di situ,
 * jadi batasnya sedikit di bawahnya.
 *
 * @param {{x,y,z,theta}} pose  pose TCP yang mau diperiksa (theta derajat)
 * @param {object} cfg          {gripLen, gripPos, gripJari}
 * @param {Array}  stasiun      [{x,y,z}]
 * @param {object} kotak        {lebar, dalam, margin}
 * @returns {{hit:boolean, st:number}}  st: indeks stasiun, -1 lantai atau tidak ada
 */
function collideCheck(pose, cfg, stasiun, kotak) {
  const mx = kotak.lebar / 2 + kotak.margin + (cfg.gripPos || 0) / 2 + (cfg.gripJari || 0);
  const my = kotak.dalam / 2 + kotak.margin;
  const th = pose[3] * KIN_DEGREE_TO_RAD;
  const g = cfg.gripLen || 0;
  const titik = [
    { x: pose[0], y: pose[1], z: pose[2] },
    { x: pose[0], y: pose[1] - g * Math.cos(th), z: pose[2] - g * Math.sin(th) }
  ];
  for (const t of titik) {
    if (t.z < 0) return { hit: true, st: -1 };
    for (let i = 0; i < stasiun.length; i++) {
      const s = stasiun[i];
      if (Math.abs(t.x - s.x) < mx && Math.abs(t.y - s.y) < my && t.z < s.z - 2) {
        return { hit: true, st: i };
      }
    }
  }
  return { hit: false, st: -1 };
}

/**
 * Nilai dari PLC -> array JS biasa. TIGA bentuk masuk ke sini, dan itu bukan
 * kerapian yang bisa ditawar:
 *
 *   [1,2,3]              BOOL array - node-opcua memberi Array biasa
 *   Float64Array         LREAL/REAL array - typed array
 *   {"0":1,"1":2}        typed array yang sudah lewat JSON (SSE) - OBJEK, tanpa
 *                        length, jadi Array.from() mengembalikan [] KOSONG
 *
 * Bentuk ketiga itu yang menghapus seluruh lengan dari layar: sudut sendi jadi
 * undefined, chainPoints menghasilkan NaN, dan yang tersisa cuma rel dan kereta -
 * tanpa satu pun galat di konsol, karena NaN bukan error.
 */
function keArray(v, panjang) {
  const n = panjang || 0;
  if (v == null) return new Array(n).fill(0);
  if (Array.isArray(v)) return v.slice();
  if (ArrayBuffer.isView(v)) return Array.prototype.slice.call(v);
  if (typeof v === 'object') {
    const kunci = Object.keys(v).filter(k => /^\d+$/.test(k)).map(Number).sort((a, b) => a - b);
    if (kunci.length) return kunci.map(k => v[k]);
  }
  return new Array(n).fill(0);
}

if (typeof module !== 'undefined') {
  module.exports = { forwardKinematic, inverseKinematic, reachable, atan2Fix, toolPolar, toREAL,
                     forwardKinematicV2, inverseKinematicV2, toolPolarV2, atanKuadran,
                     fkSteps, ikSteps, chainPoints, gripperPoints, collideCheck, keArray,
                     KIN_PI, KIN_DEGREE_TO_RAD, KIN_RAD_TO_DEGREE };
}
