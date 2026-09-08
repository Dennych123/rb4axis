// Rantai yang DIGAMBAR harus rantai yang DIHITUNG.
//
// Kelas kegagalan yang dijaga di sini: gambar 3D yang tampak wajar tapi menceritakan
// robot lain. Tidak ada yang mengeluh - tidak ada error, tidak ada rung merah, tidak
// ada tag yang salah - dan orang mengambil keputusan dari gambar itu. Persis kelas
// yang sama dengan "rung tergambar wajar tapi menjalankan rangkaian lain" di reader.
//
// Karena itu titik ujung chainPoints() diadu ke keluaran forwardKinematic(), dan
// halaman viz WAJIB memakai chainPoints() - bukan menghitung sendiri dari sudut.
'use strict';
const fs = require('fs');
const path = require('path');
const K = require(path.join(__dirname, '..', 'web', 'kin.js'));

const raw = JSON.parse(fs.readFileSync(path.join(__dirname, '..', 'sim', 'robot.config.json'), 'utf8'));
// toolY SUDAH termasuk panjang gripper - dijumlahkan sekali, sama seperti yang
// ditulis gen_sim.js ke ROBOT_TOOL_Y_LREAL dan sama seperti yang dibaca halaman.
const cfg = { L1: raw.link.L1, L2: raw.link.L2, L3: raw.link.L3, L4: raw.link.L4,
              toolY: raw.tool.Y + raw.gripper.panjang, toolZ: raw.tool.Z,
              gripLen: raw.gripper.panjang };

let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

// ------------------------------------------------ ujung rantai == hasil FK
// Yang diadu FK V2 (yang dipakai sim DAN halaman), bukan V1.
let maks = 0, n = 0;
for (const x of [-200, 0, 350]) {
  for (const t1 of [0, 30, 90, 135]) {
    for (const t2 of [-120, -60, 0]) {
      for (const t3 of [-45, 0, 45]) {
        const j = [x, t1, t2, t3];
        const p = K.chainPoints(j, cfg);
        const w = K.forwardKinematicV2(j, cfg).worldL;
        n++;
        maks = Math.max(maks,
          Math.abs(p[6].x - w[0]), Math.abs(p[6].y - w[1]), Math.abs(p[6].z - w[2]));
      }
    }
  }
}
chk('TCP = world hasil FK V2 (' + n + ' pose)', maks < 1e-9,
    'selisih terbesar ' + maks.toExponential(2) + ' mm');

// ------------------------------------------------------- bentuk rantainya
const p = K.chainPoints([250, 0, 0, 0], cfg);
chk('7 titik: kereta, bahu, siku, pergelangan, ujung L4, pangkal gripper, TCP', p.length === 7);
chk('kereta ikut sumbu 0 (prismatik, bukan sudut)', p[0].x === 250 && p[0].y === 0 && p[0].z === 0,
    JSON.stringify(p[0]));
chk('seluruh rantai di satu bidang X yang sama', p.every(t => t.x === 250),
    'sumbu 1-3 planar - kalau salah satu titik menyimpang di X, ada rotasi yang salah sumbu');
chk('bahu setinggi L1', Math.abs(p[1].z - cfg.L1) < 1e-9, 'z=' + p[1].z);
chk('pada semua sudut nol, rantai lurus ke +Y',
    Math.abs(p[4].y - (cfg.L2 + cfg.L3 + cfg.L4)) < 1e-9 && Math.abs(p[4].z - cfg.L1) < 1e-9,
    'y=' + p[4].y);

// Panjang ruas harus TETAP berapa pun sudutnya - kalau berubah, ada titik yang
// dihitung dari sudut kumulatif yang salah.
function jarak(a, b) { return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z); }
let bedaPanjang = 0;
for (const t1 of [10, 80, 160]) {
  for (const t2 of [-100, -30]) {
    const q = K.chainPoints([0, t1, t2, 25], cfg);
    bedaPanjang = Math.max(bedaPanjang,
      Math.abs(jarak(q[1], q[2]) - cfg.L2),
      Math.abs(jarak(q[2], q[3]) - cfg.L3),
      Math.abs(jarak(q[3], q[4]) - cfg.L4));
  }
}
chk('panjang ruas tetap di semua pose', bedaPanjang < 1e-9, bedaPanjang.toExponential(2));

// ------------------------------------------------------------------ gripper
// Panjang gripper sudah termasuk di toolY. Kalau ada yang menjumlahkannya LAGI di
// chainPoints, jarak pangkal->TCP jadi dua kali panjangnya - dan di layar itu cuma
// terlihat seperti lengan yang sedikit lebih panjang.
const pg = K.chainPoints([0, 45, -60, 10], cfg);
chk('pangkal->TCP = panjang gripper, bukan dua kalinya',
    Math.abs(jarak(pg[5], pg[6]) - raw.gripper.panjang) < 1e-9,
    jarak(pg[5], pg[6]).toFixed(3) + ' vs ' + raw.gripper.panjang);
chk('tanpa gripper, pangkal dan TCP berimpit', (() => {
  const q = K.chainPoints([0, 45, -60, 10], Object.assign({}, cfg, { gripLen: 0 }));
  return jarak(q[5], q[6]) < 1e-12;
})());

const g = K.gripperPoints([0, 30, -70, 20], cfg, 60);
chk('dua jari', g.jari.length === 2);
chk('jarak antar jari = bukaan yang diminta',
    Math.abs(jarak(g.jari[0].ujung, g.jari[1].ujung) - 60) < 1e-9,
    jarak(g.jari[0].ujung, g.jari[1].ujung).toFixed(4));
chk('jari sepanjang gripper dan sejajar',
    Math.abs(jarak(g.jari[0].atas, g.jari[0].ujung) - raw.gripper.panjang) < 1e-9
    && Math.abs(jarak(g.jari[1].atas, g.jari[1].ujung) - raw.gripper.panjang) < 1e-9);
chk('jari membuka TEGAK LURUS arah tool', (() => {
  // Hasil kali titik arah tool dengan arah bukaan harus nol; kalau tidak, jari
  // membuka miring dan TCP di tengah-tengahnya bukan lagi titik yang dihitung FK.
  const arah = { y: g.tcp.y - g.pangkal.y, z: g.tcp.z - g.pangkal.z };
  const buka = { y: g.jari[0].ujung.y - g.jari[1].ujung.y, z: g.jari[0].ujung.z - g.jari[1].ujung.z };
  return Math.abs(arah.y * buka.y + arah.z * buka.z) < 1e-9;
})());
chk('menutup rapat: kedua jari berimpit di TCP', (() => {
  const q = K.gripperPoints([0, 30, -70, 20], cfg, 0);
  return jarak(q.jari[0].ujung, q.jari[1].ujung) < 1e-12
    && jarak(q.jari[0].ujung, q.tcp) < 1e-12;
})());

// -------------------------------------------- halaman memakai fungsi itu, bukan salinannya
const robot = fs.readFileSync(path.join(__dirname, '..', 'web', 'robot.js'), 'utf8');
chk('robot.js menggambar dari chainPoints()', /chainPoints\(/.test(robot));
chk('robot.js tidak menghitung rantai sendiri',
    !/Math\.(cos|sin)\s*\([^)]*joint/.test(robot),
    'rumus rantai kedua di halaman = gambar yang bebas melenceng dari FK');

// Pemetaan sumbu PLC->three cuma di satu tempat, dan urutannya (x, z, y) itu yang
// bikin Z PLC jadi ketinggian. Ketukar: lengan rebah, rel berdiri.
chk('pemetaan sumbu PLC->three ada di satu fungsi', /function ke3\(p\)[\s\S]*?p\.x, p\.z, p\.y/.test(robot));

// ------------------------------------------------ nilai PLC -> array yang bisa dipakai
// SUDAH KEJADIAN: lengan hilang dari layar begitu PLC tersambung, base dan rel tetap
// ada. Sebabnya bentuk nilainya, bukan kinematiknya - array LREAL/REAL datang sebagai
// Float64Array, dan lewat JSON (SSE) berubah jadi objek {"0":..,"1":..} TANPA length.
// `Array.from` atas objek begitu mengembalikan [] kosong, sudut sendi jadi undefined,
// chainPoints menghasilkan NaN, dan tidak ada satu pun galat di konsol karena NaN
// bukan error.
const lewatJson = v => JSON.parse(JSON.stringify(v));
chk('keArray: typed array (LREAL dari node-opcua)',
    K.keArray(new Float64Array([0, 90, -90, 0])).join() === '0,90,-90,0');
chk('keArray: typed array yang sudah lewat JSON - bentuk yang bikin lengan hilang',
    K.keArray(lewatJson(new Float64Array([0, 90, -90, 0]))).join() === '0,90,-90,0',
    'Array.from atas bentuk ini memberi [] kosong');
chk('keArray: array biasa (BOOL) lewat apa adanya',
    K.keArray([false, true, false, false]).length === 4);
chk('keArray: nilai hilang jadi array nol sepanjang yang diminta',
    K.keArray(undefined, 4).join() === '0,0,0,0',
    'lebih baik lengan di pose nol daripada NaN yang menghapusnya dari layar');
chk('rantai tetap tergambar dari bentuk objek', (() => {
  const j = K.keArray(lewatJson(new Float64Array([100, 45, -60, 10])), 4);
  return K.chainPoints(j, cfg).every(p => isFinite(p.x) && isFinite(p.y) && isFinite(p.z));
})());

// Halaman TIDAK boleh memakai Array.from atas nilai dari stream - itu yang dulu salah.
const robotSrc = fs.readFileSync(path.join(__dirname, '..', 'web', 'robot.js'), 'utf8');
chk('robot.js memakai keArray, bukan Array.from, buat nilai dari PLC',
    !/Array\.from\(v[.[]/.test(robotSrc), 'Array.from(v.SIM_...) mengembalikan [] untuk objek');

// ------------------------------------------------ tiap pose stasiun harus terjangkau
// Stasiun yang tidak terjangkau baru ketahuan waktu sekuensnya berhenti di langkah 11
// dan robotnya diam - tanpa galat, karena IK memang menolak dengan benar. Diperiksa
// di sini: pose permukaan DAN pose approach, keduanya, lewat IK yang sama dengan PLC.
const siklus = raw.siklus;
chk('config punya enam stasiun', siklus.stasiun.length === 6, siklus.stasiun.map(s => s.nama).join(' '));
const takTerjangkau = [];
for (const s of siklus.stasiun) {
  for (const [label, z] of [['pose', s.z], ['approach', s.z + siklus.approach]]) {
    const ik = K.inverseKinematicV2([s.x, s.y, z, s.theta], cfg, false);
    if (!ik.done) takTerjangkau.push(s.nama + ' ' + label + ' (errId ' + ik.errorId + ')');
  }
}
chk('12 pose stasiun+approach terjangkau dan di dalam soft limit', takTerjangkau.length === 0,
    takTerjangkau.join(', '));
chk('stasiun ada di dalam rel', siklus.stasiun.every(s =>
      s.x > raw.limit.PD1300_000 && s.x < raw.limit.PD1300_001),
    'rel ' + raw.limit.PD1300_000 + '..' + raw.limit.PD1300_001);
// Beda tinggi ICC vs DW: PULUHAN mm, bukan ratusan. Mesin di sel nyata berdiri di
// satu lantai dengan tinggi meja yang hampir sama; beda 300 mm memaksa lengan
// mengambil pose yang tidak pernah terjadi di sana, dan viz-nya jadi meyakinkan
// untuk sel yang salah.
chk('beda tinggi ICC vs DW 25..40 mm', (() => {
  const icc = siklus.stasiun.filter(s => s.tipe === 1).map(s => s.z);
  const dw = siklus.stasiun.filter(s => s.tipe === 2).map(s => s.z);
  if (icc.length !== 2 || dw.length !== 2) return false;
  const beda = Math.abs(icc[0] - dw[0]);
  return beda >= 25 && beda <= 40 && icc[0] === icc[1] && dw[0] === dw[1];
})(), 'ICC z=' + siklus.stasiun.filter(s => s.tipe === 1)[0].z
    + ' DW z=' + siklus.stasiun.filter(s => s.tipe === 2)[0].z);

// ------------------------------------------------- gripper menyumpit dari atas
chk('tiap stasiun diambil dari ATAS (theta_EE -90)',
    siklus.stasiun.every(s => s.theta === -90),
    'theta 0 = tool menjulur mendatar; gripper mendekat dari samping dan menembus badan mesin');

// Jari membuka sepanjang SUMBU X - arah rel - jadi yang dijepit sisi kiri-kanan
// produk. Kalau bukaannya di bidang Y-Z, yang dijepit sisi depan-belakang: dari
// kamera mana pun tetap terlihat "menjepit", cuma bukan sisi yang benar.
const gx = K.gripperPoints([0, 45, -74.3, -60.7], cfg, 120);
chk('jari membuka sepanjang sumbu X (arah rel)',
    Math.abs(gx.jari[0].ujung.x - gx.jari[1].ujung.x - 120) < 1e-9
    && Math.abs(gx.jari[0].ujung.y - gx.jari[1].ujung.y) < 1e-9
    && Math.abs(gx.jari[0].ujung.z - gx.jari[1].ujung.z) < 1e-9,
    'dx=' + (gx.jari[0].ujung.x - gx.jari[1].ujung.x).toFixed(2));
chk('bukaan menjepit = lebar PCB', raw.gripper.tutup === siklus.pcb.panjang,
    'jari yang menutup ke 0 menembus produk yang sedang dipegangnya');
chk('bukaan penuh lebih lebar dari produk', raw.gripper.stroke > siklus.pcb.panjang);

// -------------------------------------------- rel cuma dilewati dalam pose jalan
// Pose home dipakai sekuenser sebagai pose JALAN. Kalau dia tidak lebih tinggi dari
// mesin tertinggi, robot menyapu tiap mesin yang dilewatinya - dan di layar itu
// mulus, karena tidak ada yang menghitung tabrakan waktu menggambar.
const tertinggi = Math.max(...siklus.stasiun.map(s => s.z));
const poseJalan = K.forwardKinematicV2(raw.home.sumbu, cfg).worldL;
chk('pose jalan (home) lebih tinggi dari mesin tertinggi',
    poseJalan[2] > tertinggi + 100,
    'TCP z=' + poseJalan[2].toFixed(0) + ' vs mesin ' + tertinggi);

// ------------------------------------------------------------------ tabrakan
// collideCheck di kin.js itu cerminan penjaga di PRG_SIM_ROBOT.st, dipakai halaman
// waktu simulator mati. Yang diuji di sini bukan rumusnya, tapi bahwa kotaknya
// SEPADAN dengan sel: pose kerja tidak boleh dianggap menabrak, dan pose di dalam
// badan mesin tidak boleh lolos.
const kotak = siklus.mesin;
const cfgTab = Object.assign({}, cfg, { gripPos: raw.gripper.stroke, gripJari: raw.gripper.tebal_jari });
const salahTolak = [];
for (const s of siklus.stasiun) {
  for (const [label, z] of [['pose', s.z], ['approach', s.z + siklus.approach]]) {
    if (K.collideCheck([s.x, s.y, z, s.theta], cfgTab, siklus.stasiun, kotak).hit) {
      salahTolak.push(s.nama + ' ' + label);
    }
  }
}
chk('pose kerja TIDAK dianggap tabrakan', salahTolak.length === 0,
    salahTolak.join(', ') + ' - penjaga yang menolak pose kerjanya sendiri bikin sel mati total');

const dalamMesin = siklus.stasiun[1];
chk('titik di dalam badan mesin ketahuan', (() => {
  const t = K.collideCheck([dalamMesin.x, dalamMesin.y, dalamMesin.z - 80, -90], cfgTab,
                           siklus.stasiun, kotak);
  return t.hit && t.st === 1;
})());
chk('menembus lantai ketahuan',
    K.collideCheck([0, 300, -5, -90], cfgTab, siklus.stasiun, kotak).st === -1);

// Lengan yang menyusuri rel dalam pose jalan harus bebas di SELURUH panjang rel -
// itu yang bikin langkah "lipat dulu, baru geser" ada gunanya.
let sapuan = 0;
for (let x = raw.limit.PD1300_000; x <= raw.limit.PD1300_001; x += 25) {
  if (K.collideCheck([x, poseJalan[1], poseJalan[2], poseJalan[3]], cfgTab,
                     siklus.stasiun, kotak).hit) sapuan++;
}
chk('pose jalan bebas di sepanjang rel', sapuan === 0, sapuan + ' titik menabrak');

// Halaman menggambar stasiun dari tag PLC, bukan dari config: kalau keduanya beda,
// yang salah harus kelihatan sebagai gripper turun di sebelah mesin - bukan tersembunyi
// karena gambar dan sekuens membaca angka yang berbeda.
chk('robot.js membaca pose stasiun dari tag PLC', /v\.SIM_ST_X/.test(robot));

// --------------------------------------------- panjang ruas = jarak, bukan kelipatan
// SUDAH KEJADIAN: `kotak(58, 1, 46)` menaruh tebal 1 di y dan kedalaman 46 di z,
// sementara ruasKe menyetel scale.z = panjang. Hasilnya tiap ruas tergambar 46 KALI
// lebih panjang - lengan memanjang keluar layar seperti rel raksasa - dan tidak satu
// pun angka di panel berubah, karena kinematiknya memang benar.
//
// Yang menutup celahnya bukan "ingat urutan argumen", tapi skala yang dihitung
// TERHADAP kedalaman geometrinya sendiri.
chk('ruasKe membagi dengan kedalaman geometri, bukan menganggapnya 1',
    /geometry\.parameters[\s\S]{0,120}?scale\.z\s*=\s*Math\.max\(1, panjang\)\s*\/\s*dasar/.test(robot),
    'kalau ini merah, panjang ruas jadi kelipatan ukuran kotaknya');
chk('rel juga diskalakan terhadap lebar geometrinya',
    /rel\.scale\.x\s*=\s*span\s*\/\s*\(bagian\.rel\.geometry\.parameters\.width/.test(robot));

// Sabuk pengaman kedua: geometri yang dipakai ruasKe ditulis dengan kedalaman 1,
// jadi angka skalanya sama dengan milimeter dan gampang dibaca waktu di-debug.
const geomRuas = (robot.match(/kotak\(\s*\d+\s*,\s*\d+\s*,\s*1\s*,/g) || []).length;
chk('geometri ruas ditulis dengan kedalaman 1 (' + geomRuas + ' buah)', geomRuas >= 6,
    'tiang + 3 lengan + badan gripper + 2 jari');

console.log(fail ? 'GAGAL ' + fail : 'LULUS');
process.exit(fail ? 1 : 0);
