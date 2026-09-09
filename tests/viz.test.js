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

// ------------------------------------------------------- gerakan yang digambar
// Yang membuat gerakan terlihat mulus di antara dua kabar dari PLC (~50 ms) bukan
// mengejar posisi terakhir - itu selalu tertinggal, dan makin cepat sumbunya makin
// jauh tertinggal - tapi meramalkannya dari KECEPATAN yang memang sedang dipakai PLC.
chk('gambar diramal dari kecepatan PLC, bukan sekadar mengejar posisi',
    /st\.jointPlc\[i\] \+ \(st\.jointVel\[i\] \|\| 0\) \* umur/.test(robot));
// Ramalan tanpa batas = lengan terbang menjauh waktu kabarnya berhenti datang, dan di
// layar itu terlihat seperti robot yang kabur, bukan seperti sambungan yang putus.
chk('ramalan dibatasi', /RAMAL_MAKS = 0\.\d+/.test(robot)
    && /Math\.min\(\(jam\(\) - st\.tSampel\) \/ 1000, RAMAL_MAKS\)/.test(robot));
// Panel disusun ulang tiap kabar = puluhan elemen dibuang dan dibuat lagi 20x per
// detik, di tengah frame. Yang tersendat justru animasi 3D-nya, bukan panelnya.
chk('panel tidak menyusun ulang innerHTML tiap kabar dari PLC', (() => {
  // Komentar dibuang dulu: catatan di dalam fungsi itu MENYEBUT innerHTML justru untuk
  // menjelaskan kenapa tidak dipakai, dan mencocokkan teks mentah bikin tes ini
  // menjawab pertanyaan yang lain.
  const badan = robot.slice(robot.indexOf('function panelTampil'), robot.indexOf('var NAMA_STATE'))
    .replace(/\/\/.*/g, '');
  return !/innerHTML/.test(badan);
})(), 'panelTampil() harus mengganti teks di elemen yang sudah ada');
chk('panel digambar ter-throttle dari putar(), bukan dari tiap pesan SSE',
    /t - panelTerakhir > \d+/.test(robot) && /perluPanel = true;/.test(robot)
    && !/es\.onmessage[\s\S]{0,2000}?panelTampil\(\);/.test(robot),
    'kabar SSE cuma menandai; yang menggambar putar()');

// ------------------------------------------------------------------ fisika
// Rapier dipakai untuk SATU hal: ke mana benda yang jatuh mendarat. Batas itu yang
// diuji di sini, bukan fisikanya - fisikanya milik Rapier.
//
// Kenapa batasnya penting: fisika di browser jalan mengikuti frame, dan frame-nya
// berubah menurut komputernya. PLC jalan 4 ms tetap. Begitu ada interlock atau sensor
// yang bergantung pada fisika browser, hasilnya berhenti bisa diulang - dan simulasi
// yang hasilnya berubah menurut laptopnya tidak bisa dipakai membuktikan apa pun.
chk('fisika cuma dipanggil dari jalur benda JATUH',
    /function mulaiJatuh\(\)[\s\S]{0,400}?fisikaJatuhkan\(/.test(robot),
    'satu-satunya pintu masuk fisika');
chk('PCB yang dipegang dan yang di stasiun TIDAK lewat fisika', (() => {
  const blok = robot.slice(robot.indexOf('// ---------------------------------------------------------- penutup berengsel'),
                           robot.indexOf('function panelTampil'));
  return !/fisika/.test(blok);
})(), 'posisi produk yang dipegang/diletakkan ditentukan PLC, bukan solver');
chk('aktuator tidak punya badan dinamis', !/RigidBodyDesc\.dynamic[\s\S]{0,200}?(jari|cover|lengan)/.test(robot),
    'aktuator disetir PLC; membuatnya dynamic berarti solver ikut memutuskan posisinya');
chk('fisika tidak pernah menulis ke PLC', (() => {
  const i = robot.indexOf('function fisikaBangun');
  const j = robot.indexOf('function mulaiJatuh');
  return !/kirim\(/.test(robot.slice(i, j));
})(), 'satu arah: PLC memutuskan, fisika menggambar akibatnya');

// Langkah waktu TETAP. Memakai selisih waktu frame bikin hasil di laptop cepat berbeda
// dengan di laptop lambat.
chk('langkah fisika tetap, pakai akumulator',
    /dunia\.timestep = 1 \/ 120/.test(robot)
    && /while \(fisika\.sisa >= fisika\.dunia\.timestep/.test(robot));
chk('jumlah langkah per frame dibatasi', /&& n < \d+\)/.test(robot),
    'tanpa batas, satu frame yang telat memicu ratusan langkah dan halamannya membeku');

// Satuan: PLC dan gambar milimeter, solver meter. Solver rigid body dirancang untuk
// angka sekitar 1; dibiarkan dalam mm, tumpukan bergetar dan benda tipis tembus lantai.
chk('konversi satuan mm->m ada di SATU tempat', /var SK = 0\.001;/.test(robot));

// Halaman WAJIB tetap jalan tanpa Rapier: mesin di pabrik sering tanpa internet.
chk('ada jalan mundur kalau Rapier tidak termuat',
    /if \(fisika && fisikaJatuhkan\(/.test(robot) && /st\.jatuh = \{ p: tcp/.test(robot));
chk('dunia dibangun waktu Rapier mengabari, bukan waktu halaman dimuat',
    /addEventListener\('rapier-siap'/.test(robot));

// Collider mesin lahir dari data yang SAMA dengan yang digambar dan yang dipakai
// penjaga tabrakan. Kotak keempat yang berdiri sendiri = PCB memantul di tempat yang
// mesinnya tidak ada.
chk('collider stasiun dibangun dari st.stasiun + st.mesin',
    /function fisikaStasiun[\s\S]{0,900}?st\.mesin\.lebar[\s\S]{0,200}?st\.mesin\.dalam/.test(robot));

const html = fs.readFileSync(path.join(__dirname, '..', 'web', 'index.html'), 'utf8');
chk('Rapier dimuat lokal dulu, baru CDN dengan versi DIPATOK',
    /'\.\/rapier3d-compat\.js'/.test(html) && /rapier3d-compat@\d+\.\d+\.\d+/.test(html),
    'versi tanpa patokan bikin halaman ini bisa rusak sendiri suatu hari');

// ------------------------------------- panel penjelas memakai rumus yang SAMA
// Panel yang menjelaskan kinematik sambil menghitung sendiri adalah cara paling halus
// untuk berbohong: gambarnya benar, angkanya benar, penjelasannya salah - dan yang
// membacanya justru orang yang belum tahu mana yang benar.
//
// Karena itu fkSteps/ikSteps yang dipakai panel adalah fungsi yang DIPANGGIL
// forwardKinematicV2/inverseKinematicV2, bukan sepupunya.
let bedaFK = 0, bedaIK = 0;
for (const j of [[0, 45, -74, -61], [-300, 90, -90, -90], [200, 20, -30, 10]]) {
  const a = K.forwardKinematicV2(j, cfg).worldL;
  const b = K.fkSteps(j, cfg).worldL;
  for (let i = 0; i < 4; i++) bedaFK = Math.max(bedaFK, Math.abs(a[i] - b[i]));

  const pose = a.slice();
  const ik1 = K.inverseKinematicV2(pose, cfg, false);
  const ik2 = K.ikSteps(pose, cfg, false);
  if (ik1.done !== ik2.done) bedaIK = Infinity;
  if (ik1.done) for (let i = 0; i < 4; i++) bedaIK = Math.max(bedaIK, Math.abs(ik1.joint[i] - ik2.joint[i]));
}
chk('fkSteps = forwardKinematicV2 (bit per bit)', bedaFK === 0, String(bedaFK));
chk('ikSteps = inverseKinematicV2 (bit per bit)', bedaIK === 0, String(bedaIK));

// Nilai antara yang ditampilkan harus KONSISTEN dengan hasilnya, bukan sekadar ada:
// R itu jarak ke pergelangan, dan pergelangan itu titik ke-3 rantai.
{
  const j = [0, 45, -74, -61];
  const w = K.fkSteps(j, cfg).worldL;
  const ik = K.ikSteps(w, cfg, false);
  const rantai = K.chainPoints(ik.joint, cfg);
  const rDariRantai = Math.hypot(rantai[3].y - rantai[1].y, rantai[3].z - rantai[1].z);
  // Toleransinya 1e-4, bukan 1e-9, dan itu BUKAN kelonggaran: jalannya lewat
  // derajat (IK memberi derajat, chainPoints mengubahnya balik ke radian), dan
  // DEGREE_TO_RAD project dipotong 9 angka - perkaliannya 1 - 2.98e-8. Di R ~440 mm
  // itu ~6e-6 mm. Menuntut lebih rapat berarti menuntut konstanta yang lain.
  chk('R yang ditampilkan = jarak bahu ke pergelangan', Math.abs(ik.r - rDariRantai) < 1e-4,
      ik.r.toFixed(6) + ' vs ' + rDariRantai.toFixed(6) + ' (lantai konstanta ~6e-6)');
  // Yang dibuktikan: ketiga sudut yang ditampilkan panel memang sudut segitiga yang
  // SAMA. RAD_TO_DEGREE yang dipotong 9 angka menggeser 180 derajat sebesar ~5e-6,
  // jadi toleransinya di situ - lantai yang sama dengan round-trip.
  chk('segitiga IK tertutup: beta + gamma + sudut ketiga = 180',
      Math.abs((ik.beta + ik.gamma + Math.acos((rDariRantai * rDariRantai + cfg.L3 * cfg.L3
        - cfg.L2 * cfg.L2) / (2 * cfg.L3 * rDariRantai))) * K.KIN_RAD_TO_DEGREE - 180) < 1e-4);
  chk('pose yang di luar jangkauan ditolak SEBELUM ACOS, dengan R tetap dilaporkan', (() => {
    const jauh = K.ikSteps([0, 2000, 400, -90], cfg, false);
    return !jauh.done && jauh.errorId === 1 && isFinite(jauh.r) && jauh.beta === undefined;
  })(), 'panel harus bisa menunjukkan DI MANA gagalnya, bukan cuma bahwa gagal');
}

// Halaman TIDAK boleh punya trigonometri kinematik sendiri. Cover berengsel boleh
// memakai konversi derajat->radian; yang dilarang acos/atan/asin, karena itu isi
// rumus IK-nya.
chk('robot.js tidak memakai acos/atan/asin sama sekali',
    !/Math\.(acos|atan|atan2|asin)\s*\(/.test(robot),
    'rumus kinematik tempatnya di kin.js - satu sumber untuk PLC, tes, dan panel');
chk('panel penjelas dibangun dari fkSteps/ikSteps',
    /fkSteps\(/.test(robot) && /ikSteps\(/.test(robot));

// Slider mengirim waktu DILEPAS (onchange), bukan tiap piksel (oninput): satu tulis
// per gerakan mouse membanjiri sesi OPC UA yang sama yang membaca puluhan tag.
chk('slider jog mengirim waktu dilepas, bukan tiap piksel', (() => {
  const blok = robot.slice(robot.indexOf('function bikinSlider'), robot.indexOf('function sliderTampil'));
  // Badan oninput dipotong tepat di onchange berikutnya. Tanpa itu, pemeriksaan
  // "oninput tidak mengirim" ikut membaca isi onchange yang memang mengirim - dan tes
  // ini gagal justru waktu kodenya benar.
  const oninput = blok.slice(blok.indexOf('oninput ='), blok.indexOf('onchange ='));
  return /onchange = function[\s\S]{0,160}?sliderKirim/.test(blok) && !/sliderKirim/.test(oninput);
})());

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
// Gripper sudah menggantung tegak di pose jalan, jadi turun ke stasiun tinggal
// menurunkan - bukan memutar dulu di atas mesin, yang mengayunkan jari melewati
// badan mesin dalam perjalanannya.
chk('pose jalan sudah menghadap bawah (theta_EE -90)', poseJalan[3] === -90, 'thEE=' + poseJalan[3]);
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

// ------------------------------------------------------------ penutup mesin
// Penutup itu badan yang bergerak di ruang yang sama dengan gripper. Yang menjaga
// keduanya tidak bertabrakan bukan penjaga tabrakan (penutup tidak ada di sana),
// tapi URUTAN: robot cuma turun sesudah penutupnya terbuka penuh.
chk('config punya penutup dengan sudut dan kecepatan',
    siklus.mesin.cover && siklus.mesin.cover.sudut > 0 && siklus.mesin.cover.kecepatan > 0);
chk('penutup terbuka lebih tinggi dari PCB yang dijepit',
    siklus.mesin.cover.sudut >= 45,
    'sudut kecil = penutup masih menutupi jalan masuk gripper walau "terbuka"');
chk('halaman menggambar penutup dari tag PLC, bukan dari config',
    /v\.SIM_ST_COVER/.test(robot) && /st\.stCover\[cv\]/.test(robot));
chk('penutup diputar pada ENGSEL, bukan di tengahnya',
    /pivot\.rotation\.x/.test(robot) && /cb\.tutup\.position\.set\(0, teb \/ 2, -dalam \/ 2\)/.test(robot),
    'kotak yang diputar di tengahnya menembus meja tiap kali membuka');


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
