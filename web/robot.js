// Viz 3D Blurobot: scene three.js + panel kontrol + sambungan ke bridge.
//
// Dua sumber gerakan, dan halaman SELALU memberi tahu yang mana sedang dipakai:
//
//   PLC     - nilai datang dari simulator NX lewat bridge (SSE). Ini yang sebenarnya.
//   OFFLINE - simulator mati, halaman menjalankan kin.js + motion model yang sama di
//             browser. Berguna buat melihat bentuk gerakan, TAPI bukan bukti program
//             PLC-nya benar. Ditandai kuning di layar, bukan diam-diam menggantikan.
//
// Kinematik yang dipakai halaman ini V2 - sama dengan yang dipakai program sim.
// Fungsi V1 di kin.js sengaja masih ada buat tes cacat aslinya, dan TIDAK dipakai
// di sini: kalau halaman memakai rumus yang beda dari PLC, mode offline dan mode
// PLC menggambar dua robot berbeda tanpa ada yang mengeluh.
//
// Semua posisi datang dari chainPoints()/gripperPoints() di kin.js. Halaman tidak
// menghitung rantai sendiri - rumus kedua di sini bebas melenceng dari FK sambil
// tetap menggambar lengan yang tampak wajar.
'use strict';

var TAG = { joint: 'SIM_JOINT_POS', world: 'SIM_WORLD_POS', limit: 'SIM_LIMIT',
            beat: 'SIM_HEARTBEAT', err: 'SIM_ERROR', errId: 'SIM_ERROR_ID' };

var ERR_TEKS = { 0: '', 1: 'di luar jangkauan (R > L2+L3)', 2: 'terlalu dekat (R < |L2-L3|)',
                 3: 'singular (R = 0)', 4: 'soft limit ditembus' };

var st = {
  plc: false, bridge: false,
  joint: [0, 90, -90, 0],         // yang DIGAMBAR - dihaluskan waktu tersambung PLC
  jointPlc: [0, 90, -90, 0],      // yang TERAKHIR DIBACA dari PLC - itu yang di panel
  jointVel: [0, 0, 0, 0],
  world: [0, 0, 0, 0],
  cmd: [0, 90, -90, 0],
  limit: [false, false, false, false, false, false, false, false],
  beat: 0, err: false, errId: 0, elbowUp: false,
  grip: { pos: 180, stroke: 180, tutup: 120, jari: 14, len: 90, cmd: false, vel: 160 },
  auto: false, langkah: 0, siklus: 0, tujuan: 0, part: 0,
  jobSrc: -1, jobDst: -1,
  // panel: selector, emergency, dan syarat home. Semuanya dibaca dari PLC - halaman
  // TIDAK menyimpulkan sendiri boleh-tidaknya jalan, karena kesimpulannya bisa beda
  // dari yang dipakai PLC memutuskan.
  selAuto: false, estop: false, homed: true, stopReq: false, state: 0, abortId: 0,
  drop: null, jatuh: null,        // pencacah produk jatuh + animasi jatuhnya
  collide: false, collideSt: -1,
  approach: 140,
  stasiun: [],                    // {nama,tipe,x,y,z,theta,proses}
  stState: [0, 0, 0, 0, 0, 0], stTimer: [0, 0, 0, 0, 0, 0],
  mesin: { lebar: 300, dalam: 320, margin: 12 },
  pcb: { panjang: 120, lebar: 80, tebal: 8 },
  dim: { L1: 400, L2: 300, L3: 250, L4: 100, toolY: 140, toolZ: 0,
         offset: [0, 0, 0, 0, 0], limitv: [-500, 500, -90, 180, -150, 0, -120, 120] },
  vel: [900, 90, 90, 120], acc: [1800, 240, 240, 320],
  velW: [100, 100, 100, 30], step: 10, mode: 0, hold: false
};

var el = function (id) { return document.getElementById(id); };
var f2 = function (x) { return (typeof x === 'number' && isFinite(x)) ? x.toFixed(2) : '-'; };

function cfgKin() {
  return { L1: st.dim.L1, L2: st.dim.L2, L3: st.dim.L3, L4: st.dim.L4,
           toolY: st.dim.toolY, toolZ: st.dim.toolZ, gripLen: st.grip.len,
           gripPos: st.grip.pos, gripJari: st.grip.jari,
           offset: st.dim.offset, limit: st.dim.limitv };
}

// ------------------------------------------------------------------ bridge
function post(url, body) {
  return fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' },
                      body: JSON.stringify(body) }).then(function (r) { return r.json(); });
}

// Status DITANYAKAN ke bridge, tidak ditebak dari location.protocol: halaman ini
// juga bisa dibuka dari file:// sementara bridge-nya jalan, dan tebakan dari
// protokol bikin alat yang siap dipakai kelihatan mati.
function ping() {
  fetch('/api/ping').then(function (r) { return r.json(); }).then(function (j) {
    st.bridge = true;
    st.plc = j.plc && j.plc.sambung;
    statusTampil(j.plc && j.plc.pesan);
  }).catch(function () { st.bridge = false; st.plc = false; statusTampil('bridge tidak menjawab'); });
}

function statusTampil(pesan) {
  var s = el('status');
  if (st.plc) { s.className = 'ok'; s.textContent = 'PLC tersambung - angka di layar dari simulator NX'; }
  else if (st.bridge) {
    s.className = 'off';
    s.textContent = 'PLC putus - halaman pakai kinematik JS (bukan bukti program PLC benar). '
      + (pesan ? String(pesan).split('\n')[0] : '');
  } else {
    s.className = 'bad';
    s.textContent = 'bridge mati - jalankan: node blurobot/bridge/bridge.js';
  }
}

function stream() {
  var es = new EventSource('/api/stream');
  es.onmessage = function (e) {
    var j = JSON.parse(e.data);
    st.bridge = true;
    st.plc = j.plc.sambung;
    statusTampil(j.plc.pesan);
    var v = j.nilai || {};
    if (st.plc) {
      // keArray, BUKAN Array.from: array LREAL/REAL datang sebagai typed array dan
      // lewat JSON berubah jadi objek {"0":..} tanpa length - Array.from() atas objek
      // begitu mengembalikan [] kosong, dan lengannya lenyap dari layar tanpa galat.
      // Yang datang dari PLC ditaruh di jointPlc, BUKAN langsung di joint: yang
      // digambar dikejar ke situ tiap frame (lihat haluskan()). Nilai PLC-nya sendiri
      // tidak diubah - panel menampilkan yang ini, jadi angka di layar tetap angka
      // simulator, bukan angka hasil penghalusan.
      if (v[TAG.joint]) { st.jointPlc = keArray(v[TAG.joint], 4); st.cmd = st.jointPlc.slice(); }
      if (v.SIM_JOINT_VEL) st.jointVel = keArray(v.SIM_JOINT_VEL, 4);
      if (v[TAG.world]) st.world = keArray(v[TAG.world], 4);
      if (v[TAG.limit]) st.limit = keArray(v[TAG.limit], 8);
      if (v[TAG.beat] !== undefined) st.beat = v[TAG.beat];
      st.err = !!v[TAG.err]; st.errId = v[TAG.errId] || 0;
      if (v.SIM_ELBOW_UP !== undefined) st.elbowUp = !!v.SIM_ELBOW_UP;
      if (v.SIM_GRIP_POS !== undefined) st.grip.pos = v.SIM_GRIP_POS;
      if (v.SIM_GRIP_STROKE) st.grip.stroke = v.SIM_GRIP_STROKE;
      if (v.SIM_GRIP_TUTUP !== undefined) st.grip.tutup = v.SIM_GRIP_TUTUP;
      if (v.SIM_GRIP_JARI) st.grip.jari = v.SIM_GRIP_JARI;
      if (v.SIM_GRIP_LEN) st.grip.len = v.SIM_GRIP_LEN;
      if (v.SIM_GRIP_CMD !== undefined) st.grip.cmd = !!v.SIM_GRIP_CMD;
      if (v.ROBOT_L1_LREAL) {
        st.dim.L1 = v.ROBOT_L1_LREAL; st.dim.L2 = v.ROBOT_L2_LREAL;
        st.dim.L3 = v.ROBOT_L3_LREAL; st.dim.L4 = v.ROBOT_L4_LREAL;
        st.dim.toolY = v.ROBOT_TOOL_Y_LREAL; st.dim.toolZ = v.ROBOT_TOOL_Z_LREAL;
      }
      if (v.SIM_AUTO !== undefined) st.auto = !!v.SIM_AUTO;
      if (v.SIM_CYCLE_STEP !== undefined) st.langkah = v.SIM_CYCLE_STEP;
      if (v.SIM_CYCLE_COUNT !== undefined) st.siklus = v.SIM_CYCLE_COUNT;
      if (v.SIM_TARGET_ST !== undefined) st.tujuan = v.SIM_TARGET_ST;
      if (v.SIM_PART_STATE !== undefined) st.part = v.SIM_PART_STATE;
      // Jatuhnya produk datang sebagai PENCACAH, bukan pulsa: bridge mengambil sampel
      // tiap 50 ms, dan pulsa satu scan (4 ms) lewat begitu saja tanpa pernah terlihat.
      // Pesan pertama cuma menyelaraskan angkanya - tanpa itu, membuka halaman sesudah
      // ada produk jatuh menampilkan animasi jatuh yang tidak sedang terjadi.
      if (v.SIM_DROP_COUNT !== undefined) {
        if (st.drop !== null && v.SIM_DROP_COUNT > st.drop) mulaiJatuh();
        st.drop = v.SIM_DROP_COUNT;
      }
      if (v.SIM_JOB_SRC !== undefined) st.jobSrc = v.SIM_JOB_SRC;
      if (v.SIM_JOB_DST !== undefined) st.jobDst = v.SIM_JOB_DST;
      if (v.SIM_SEL_AUTO !== undefined) st.selAuto = !!v.SIM_SEL_AUTO;
      if (v.SIM_ESTOP !== undefined) st.estop = !!v.SIM_ESTOP;
      if (v.SIM_HOMED !== undefined) st.homed = !!v.SIM_HOMED;
      if (v.SIM_STOP_REQ !== undefined) st.stopReq = !!v.SIM_STOP_REQ;
      if (v.SIM_STATE !== undefined) st.state = v.SIM_STATE;
      if (v.SIM_ABORT_ID !== undefined) st.abortId = v.SIM_ABORT_ID;
      if (v.SIM_COLLIDE !== undefined) st.collide = !!v.SIM_COLLIDE;
      if (v.SIM_COLLIDE_ST !== undefined) st.collideSt = v.SIM_COLLIDE_ST;
      if (v.SIM_ST_STATE) st.stState = keArray(v.SIM_ST_STATE, 6);
      if (v.SIM_ST_TIMER) st.stTimer = keArray(v.SIM_ST_TIMER, 6);
      if (v.SIM_ST_W) st.mesin.lebar = v.SIM_ST_W;
      if (v.SIM_ST_D) st.mesin.dalam = v.SIM_ST_D;
      if (v.SIM_ST_MARGIN !== undefined) st.mesin.margin = v.SIM_ST_MARGIN;
      if (v.SIM_APPROACH) st.approach = v.SIM_APPROACH;
      // Pose stasiun dibaca dari PLC, bukan dari config: yang digambar harus yang
      // benar-benar dikejar sekuenser. Kalau keduanya beda, yang salah ketahuan
      // sebagai gripper yang turun di sebelah mesin - bukan sebagai angka.
      if (v.SIM_ST_X) {
        const sx = keArray(v.SIM_ST_X), sy = keArray(v.SIM_ST_Y),
              sz = keArray(v.SIM_ST_Z), stt = keArray(v.SIM_ST_T),
              tp = keArray(v.SIM_ST_TIPE), pr = keArray(v.SIM_ST_PROSES);
        st.stasiun = sx.map((x, i) => ({
          nama: (st.stasiun[i] && st.stasiun[i].nama) || ('ST' + i),
          tipe: tp[i] || 0, x: x, y: sy[i], z: sz[i], theta: stt[i] || 0, proses: pr[i] || 0
        }));
      }
      if (v.SIM_VEL) st.vel = keArray(v.SIM_VEL, 4);
      if (v.SIM_ACC) st.acc = keArray(v.SIM_ACC, 4);
      if (v.SIM_VEL_W) st.velW = keArray(v.SIM_VEL_W, 4);
      for (var i = 0; i < 8; i++) {
        var k = 'PD1300_00' + i;
        if (v[k] !== undefined) st.dim.limitv[i] = v[k];
      }
    }
    panelTampil();
  };
  es.onerror = function () { st.bridge = false; st.plc = false; statusTampil(null); };
}

function kirim(nama, nilai, indeks) {
  if (!st.plc) return Promise.resolve({ offline: true });
  return post('/api/write', { nama: nama, nilai: nilai, indeks: indeks })
    .then(function (j) { if (j.err) el('err').textContent = j.err; return j; });
}

// ------------------------------------------------------- kinematik offline
function offlineMinta(pose) {
  // Penjaga tabrakan yang SAMA dengan di PLC (collideCheck di kin.js). Tanpa ini,
  // mode offline jadi mode yang boleh menembus mesin - dan yang memakainya belajar
  // bahwa menembus mesin itu wajar sampai simulatornya dinyalakan.
  var tab = collideCheck(pose, cfgKin(), st.stasiun, st.mesin);
  if (tab.hit) {
    st.err = true; st.errId = 5;
    el('err').textContent = 'ditolak: ' + (tab.st >= 0 && st.stasiun[tab.st]
      ? 'menabrak ' + st.stasiun[tab.st].nama : 'menembus lantai');
    return;
  }
  var ik = inverseKinematicV2(pose, cfgKin(), st.elbowUp);
  st.err = !ik.done;
  st.errId = ik.errorId;
  el('err').textContent = ik.done ? '' : ('ditolak: ' + (ERR_TEKS[ik.errorId] || ik.errorId));
  if (ik.done) st.cmd = ik.joint;
}

// Profil trapesium, CERMINAN motion model di PRG_SIM_ROBOT.st: dipercepat sampai
// SIM_VEL lalu direm tepat waktu (v = sqrt(2*a*s)). Kalau di sini dipakai model
// yang lebih sederhana, mode offline dan mode PLC bergerak dengan bentuk yang beda -
// dan bedanya terbaca seperti PLC-nya yang salah.
function langkahSumbu(pos, cmd, vel, vmax, acc, dt) {
  var d = cmd - pos;
  var vt = Math.min(vmax, Math.sqrt(2 * acc * Math.abs(d)));
  if (d < 0) vt = -vt;
  var dv = acc * dt;
  if (Math.abs(vt - vel) <= dv) vel = vt;
  else vel += (vt > vel ? dv : -dv);
  var s = vel * dt;
  if (Math.abs(d) <= Math.abs(s)) return { pos: cmd, vel: 0, jalan: false };
  return { pos: pos + s, vel: vel, jalan: true };
}

var tSebelum = 0;
function offlineStep(t) {
  var dt = Math.min((t - tSebelum) / 1000, 0.1);
  tSebelum = t;
  if (!dt) return;
  if (st.plc) { haluskan(dt); return; }

  for (var i = 0; i < 4; i++) {
    var r = langkahSumbu(st.joint[i], st.cmd[i], st.jointVel[i], st.vel[i], st.acc[i], dt);
    st.joint[i] = r.pos;
    st.jointVel[i] = r.vel;
  }
  st.jointPlc = st.joint.slice();

  // Gripper: model yang sama dengan di ST - dan menutupnya berhenti di lebar produk,
  // bukan di nol.
  var target = st.grip.cmd ? st.grip.tutup : st.grip.stroke;
  var dg = target - st.grip.pos;
  var lg = st.grip.vel * dt;
  st.grip.pos = (Math.abs(dg) <= lg) ? target : st.grip.pos + (dg > 0 ? lg : -lg);

  var fk = forwardKinematicV2(st.joint, cfgKin());
  st.world = fk.world;
  var ik = inverseKinematicV2(fk.worldL, cfgKin(), st.elbowUp);
  st.limit = ik.limits;
  var tab = collideCheck(st.world, cfgKin(), st.stasiun, st.mesin);
  st.collide = tab.hit;
  st.collideSt = tab.st;
}

// Penghalusan gambar waktu tersambung PLC. Bridge mengirim tiap ~50 ms sementara
// layar menggambar tiap ~16 ms, jadi tanpa ini tiap tiga frame menampilkan angka
// yang sama lalu melompat - dan yang terlihat patah-patah walau sumbunya bergerak
// mulus di simulator.
//
// Yang dihaluskan CUMA yang digambar. Panel tetap menampilkan st.jointPlc apa adanya,
// jadi angka di layar selalu angka simulator. Ketinggalannya paling banyak satu
// sampel: begitu nilai berhenti berubah, gambar mengejarnya sampai persis.
var TAU = 0.05;
function haluskan(dt) {
  var a = 1 - Math.exp(-dt / TAU);
  for (var i = 0; i < 4; i++) {
    var d = st.jointPlc[i] - st.joint[i];
    st.joint[i] = (Math.abs(d) < 1e-6) ? st.jointPlc[i] : st.joint[i] + d * a;
  }
}

// ------------------------------------------------------------------- scene
var scene, cam, renderer, bagian = {};
var orbit = { theta: -0.85, phi: 0.55, jarak: 1900, tX: 0, tY: 380 };

var MAT = {};
function bahan(warna, metal, kasar) {
  return new THREE.MeshStandardMaterial({ color: warna, metalness: metal, roughness: kasar });
}

function kotak(w, h, d, mat) {
  var m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.castShadow = true;
  m.receiveShadow = true;
  return m;
}

// Silinder sendi: sumbu putarnya sumbu X (semua sendi revolute berputar di bidang
// Y-Z), jadi silinder bawaan three yang berdiri di Y diputar 90 derajat pada Z.
function silinder(r, panjang, mat) {
  var m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, panjang, 24), mat);
  m.rotation.z = Math.PI / 2;
  m.castShadow = true;
  return m;
}

function bikinScene() {
  if (typeof THREE === 'undefined') {
    el('view').innerHTML = '<p style="padding:20px">three.js tidak termuat. Kalau mesin ini offline, '
      + 'taruh salinan <code>three.min.js</code> di folder <code>blurobot/web/</code>.</p>';
    return false;
  }
  var cv = el('cv');
  renderer = new THREE.WebGLRenderer({ canvas: cv, antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0d1117);
  scene.fog = new THREE.Fog(0x0d1117, 2600, 6000);
  cam = new THREE.PerspectiveCamera(45, 1, 10, 12000);

  MAT.rangka = bahan(0x64748b, 0.55, 0.55);
  MAT.rel = bahan(0x3f4854, 0.7, 0.4);
  MAT.kereta = bahan(0x0ea5e9, 0.5, 0.45);
  MAT.lengan1 = bahan(0x38bdf8, 0.35, 0.5);
  MAT.lengan2 = bahan(0x22c55e, 0.35, 0.5);
  MAT.lengan3 = bahan(0xf59e0b, 0.35, 0.5);
  MAT.sendi = bahan(0xe2e8f0, 0.8, 0.3);
  MAT.gripper = bahan(0xef4444, 0.6, 0.35);
  MAT.jari = bahan(0xfca5a5, 0.4, 0.5);

  // Cahaya: satu langit lembut supaya sisi gelap tidak jadi hitam pekat, satu
  // terarah yang melempar bayangan. Tanpa bayangan, kedalaman hilang total dan
  // lengan yang di depan tidak bisa dibedakan dari yang di belakang.
  scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1f2937, 0.85));
  var sun = new THREE.DirectionalLight(0xffffff, 1.05);
  sun.position.set(900, 1500, 900);
  sun.castShadow = true;
  sun.shadow.mapSize.set(1024, 1024);
  var c = sun.shadow.camera;
  // Kotak bayangan harus MELIPUTI seluruh sel, bukan cuma lengannya. Rel 3 m dengan
  // stasiun di +-1400: batas 1600 memotong bayangan mesin di kedua ujung, dan yang
  // kelihatan cuma "mesin ujung kok kelihatan melayang".
  c.left = -2200; c.right = 2200; c.top = 2200; c.bottom = -2200; c.near = 100; c.far = 5200;
  scene.add(sun);
  // Lampu isi dari sisi berlawanan. Posisinya DISETEL, bukan lewat translateX:
  // cahaya terarah menyinari dari position ke target, dan light yang posisinya
  // masih (0,0,0) arahnya tidak tentu - lampunya ada tapi tidak menerangi apa pun.
  var isi = new THREE.DirectionalLight(0x93c5fd, 0.3);
  isi.position.set(-900, 600, -700);
  scene.add(isi);

  // Lantai penerima bayangan + grid berskala. Grid 100 mm per petak, jadi ukuran
  // benda bisa dikira-kira dari layar tanpa melihat panel.
  var lantai = new THREE.Mesh(new THREE.PlaneGeometry(6000, 4000),
    new THREE.MeshStandardMaterial({ color: 0x161b22, roughness: 0.95, metalness: 0 }));
  lantai.rotation.x = -Math.PI / 2;
  lantai.position.y = -30;
  lantai.receiveShadow = true;
  scene.add(lantai);
  var grid = new THREE.GridHelper(4000, 40, 0x3b4657, 0x232b36);
  grid.position.y = -29;
  scene.add(grid);

  // Rel + dua penahan ujung. Panjangnya mengikuti soft limit sumbu 0, jadi batas
  // kerjanya jadi benda yang kelihatan, bukan cuma angka di panel.
  bagian.rel = kotak(1, 26, 150, MAT.rel);
  scene.add(bagian.rel);
  bagian.stopA = kotak(24, 90, 170, MAT.rangka);
  bagian.stopB = kotak(24, 90, 170, MAT.rangka);
  scene.add(bagian.stopA);
  scene.add(bagian.stopB);

  bagian.kereta = kotak(150, 74, 190, MAT.kereta);
  scene.add(bagian.kereta);

  // Tiang bahu (L1) + tiga ruas lengan. Ruasnya kotak pipih, sendinya silinder
  // supaya arah putarnya kelihatan sebagai benda.
  bagian.tiang = kotak(84, 84, 1, MAT.rangka);
  scene.add(bagian.tiang);

  bagian.lengan = [
    kotak(58, 46, 1, MAT.lengan1),
    kotak(50, 40, 1, MAT.lengan2),
    kotak(42, 34, 1, MAT.lengan3)
  ];
  bagian.lengan.forEach(function (m) { scene.add(m); });

  bagian.sendi = [silinder(38, 76, MAT.sendi), silinder(32, 66, MAT.sendi),
                  silinder(27, 58, MAT.sendi), silinder(22, 50, MAT.sendi)];
  bagian.sendi.forEach(function (m) { scene.add(m); });

  // Gripper: badan + dua jari yang bergerak menjauh/mendekat.
  bagian.gripBadan = kotak(56, 56, 1, MAT.gripper);
  scene.add(bagian.gripBadan);
  bagian.jari = [kotak(26, 14, 1, MAT.jari), kotak(26, 14, 1, MAT.jari)];
  bagian.jari.forEach(function (m) { scene.add(m); });

  // Enam badan stasiun + pelat atasnya. Dibuat SEKALI dengan ukuran satuan lalu
  // diskalakan tiap frame - membangun ulang mesh tiap kali pose berubah bikin
  // pengumpul sampah bekerja terus dan animasinya tersendat.
  MAT.icc = bahan(0xf59e0b, 0.35, 0.6);
  MAT.dw = bahan(0x14b8a6, 0.35, 0.6);
  MAT.wip = bahan(0x64748b, 0.3, 0.7);
  MAT.plat = bahan(0xcbd5e1, 0.6, 0.4);
  MAT.pcb = bahan(0x16a34a, 0.1, 0.8);
  // PCB yang sedang DIPROSES dibedakan warnanya dari yang sudah selesai: dua ICC
  // yang satu masih menghitung dan satu sudah menunggu diambil kelihatan sama persis
  // kalau warnanya sama, dan justru itu yang mau dilihat dari buffer.
  MAT.pcbProses = bahan(0x0e7490, 0.1, 0.8);
  MAT.tabrak = bahan(0xdc2626, 0.3, 0.6);
  MAT.pcbJatuh = bahan(0xb91c1c, 0.1, 0.8);
  bagian.stasiun = [];
  for (var s = 0; s < 6; s++) {
    var badan = kotak(1, 1, 1, MAT.wip);
    var plat = kotak(1, 1, 1, MAT.plat);
    scene.add(badan);
    scene.add(plat);
    bagian.stasiun.push({ badan: badan, plat: plat });
  }

  // Nama stasiun ditulis di layar. Warna saja tidak cukup: ICC 1 dan ICC 2 warnanya
  // sama, dan yang perlu dibaca justru "gripper turun di ICC 1 atau ICC 2".
  for (var t = 0; t < 6; t++) {
    var lab = labelSprite('');
    scene.add(lab);
    bagian.stasiun[t].label = lab;
  }

  // PCB: SATU per stasiun plus satu yang dipegang gripper. Dulu cuma satu kotak,
  // dan itu cukup selama cuma ada satu produk di seluruh sel. Sekarang ICC 1 dan
  // ICC 2 memang harus bisa berisi bersamaan - itu inti buffer-nya - jadi satu kotak
  // berarti produk kedua tidak pernah kelihatan.
  bagian.pcbSt = [];
  for (var q = 0; q < 6; q++) {
    var kp = kotak(1, 1, 1, MAT.pcb);
    scene.add(kp);
    bagian.pcbSt.push(kp);
  }
  bagian.pcbHold = kotak(1, 1, 1, MAT.pcb);
  scene.add(bagian.pcbHold);
  // Produk yang jatuh punya kotaknya sendiri: yang dipegang HILANG di scan yang sama
  // (PLC sudah menyatakan gripper kosong), jadi memakai kotak yang sama berarti
  // jatuhnya tidak pernah tergambar.
  bagian.pcbJatuh = kotak(1, 1, 1, MAT.pcbJatuh);
  bagian.pcbJatuh.visible = false;
  scene.add(bagian.pcbJatuh);

  // Penanda TCP: bola kecil di ujung jari. Itu titik yang dijanjikan FK/IK, jadi
  // kalau dia tidak berimpit dengan angka world di panel, ada yang salah.
  bagian.tcp = new THREE.Mesh(new THREE.SphereGeometry(11, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0xfde047 }));
  scene.add(bagian.tcp);

  pasangOrbit(cv);
  return true;
}

// Orbit ditulis sendiri, bukan OrbitControls: OrbitControls bukan bagian dari build
// inti three.js, jadi memakainya berarti satu unduhan CDN lagi yang bisa gagal
// sendiri - untuk lima baris matematika.
function pasangOrbit(cv) {
  var seret = false, lx = 0, ly = 0;
  cv.addEventListener('mousedown', function (e) { seret = true; lx = e.clientX; ly = e.clientY; });
  window.addEventListener('mouseup', function () { seret = false; });
  window.addEventListener('mousemove', function (e) {
    if (!seret) return;
    orbit.theta -= (e.clientX - lx) * 0.008;
    orbit.phi = Math.max(0.05, Math.min(1.45, orbit.phi - (e.clientY - ly) * 0.006));
    lx = e.clientX; ly = e.clientY;
  });
  cv.addEventListener('wheel', function (e) {
    e.preventDefault();
    orbit.jarak = Math.max(500, Math.min(6000, orbit.jarak * (1 + e.deltaY * 0.001)));
  }, { passive: false });
}

// Label teks di scene 3D: tekstur kanvas di atas sprite. three.js inti tidak punya
// teks sama sekali, dan pemuat font tambahan berarti satu unduhan CDN lagi yang bisa
// gagal sendiri - untuk enam kata.
function labelSprite(teks) {
  var cv = document.createElement('canvas');
  cv.width = 256; cv.height = 64;
  var sp = new THREE.Sprite(new THREE.SpriteMaterial({
    map: new THREE.CanvasTexture(cv), depthTest: false, transparent: true
  }));
  sp.scale.set(320, 80, 1);
  sp.userData.cv = cv;
  sp.userData.teks = null;
  if (teks) tulisLabel(sp, teks);
  return sp;
}

function tulisLabel(sp, teks) {
  if (sp.userData.teks === teks) return;      // menggambar ulang tiap frame = tekstur baru tiap frame
  sp.userData.teks = teks;
  var cv = sp.userData.cv, g = cv.getContext('2d');
  g.clearRect(0, 0, cv.width, cv.height);
  g.fillStyle = 'rgba(15,18,22,0.75)';
  g.fillRect(0, 8, cv.width, 48);
  g.font = 'bold 30px system-ui, sans-serif';
  g.fillStyle = '#e5e7eb';
  g.textAlign = 'center';
  g.textBaseline = 'middle';
  g.fillText(teks, cv.width / 2, 32);
  sp.material.map.needsUpdate = true;
}

function ukur() {
  var v = el('view');
  if (!renderer) return;
  renderer.setSize(v.clientWidth, v.clientHeight, false);
  cam.aspect = v.clientWidth / Math.max(1, v.clientHeight);
  cam.updateProjectionMatrix();
}

// PLC (X, Y, Z) -> three (x, y, z). PLC Z itu KETINGGIAN; three pakai Y-up, jadi
// Z PLC jadi y three dan Y PLC jadi z three. Ketukar, lengan tergambar rebah dan
// rel-nya berdiri - gambar yang tetap "masuk akal" sampai angkanya dibandingkan.
function ke3(p) { return new THREE.Vector3(p.x, p.z, p.y); }

// Satu ruas = benda antara dua titik: ditaruh di tengahnya, dipanjangkan sepanjang
// jaraknya, lalu diputar menghadap ujungnya. lookAt() mengarahkan +z lokal ke sasaran.
//
// Skalanya DIBAGI kedalaman geometrinya sendiri, bukan dianggap 1. Menganggapnya 1
// itu aturan tak tertulis yang harus diingat di tiap pemanggilan `new BoxGeometry`,
// dan sekali urutan argumennya tertukar - tebal 1 mendarat di y, kedalaman jadi 46 -
// ruasnya tergambar 46 KALI lebih panjang. Bentuk gagalnya: lengan memanjang keluar
// layar seperti rel raksasa, dan tidak ada satu pun angka di panel yang berubah,
// karena kinematiknya memang benar. Sudah kejadian.
function ruasKe(mesh, a, b) {
  var panjang = a.distanceTo(b);
  var dasar = (mesh.geometry.parameters && mesh.geometry.parameters.depth) || 1;
  mesh.position.copy(a).add(b).multiplyScalar(0.5);
  mesh.scale.z = Math.max(1, panjang) / dasar;
  mesh.lookAt(b);
  mesh.visible = panjang > 0.5;
}

// Produk jatuh: dilepas dari TCP terakhir lalu dipercepat gravitasi sampai lantai,
// diam sebentar, lalu hilang. Jatuhnya BUKAN keadaan PLC - di sana produknya sudah
// tidak ada begitu gripper terbuka. Ini gambar dari sebuah kejadian, dan lamanya
// tidak boleh dipakai menyimpulkan apa pun tentang sel.
var G = 9810;                                   // mm/s2
function mulaiJatuh() {
  if (!renderer) return;                        // three.js tidak termuat - tidak ada yang digambar
  var g = gripperPoints(st.joint, cfgKin(), st.grip.pos);
  st.jatuh = { p: ke3(g.tcp), t: (typeof performance !== 'undefined' ? performance.now() : Date.now()) };
}

function gambarJatuh(pcb) {
  var m = bagian.pcbJatuh;
  if (!st.jatuh) { m.visible = false; return; }
  var dt = ((typeof performance !== 'undefined' ? performance.now() : Date.now()) - st.jatuh.t) / 1000;
  if (dt > 1.6) { st.jatuh = null; m.visible = false; return; }
  var lantai = -30 + pcb.tebal / 2;
  var y = Math.max(lantai, st.jatuh.p.y - 0.5 * G * dt * dt);
  m.visible = true;
  m.scale.set(pcb.panjang, pcb.tebal, pcb.lebar);
  m.position.set(st.jatuh.p.x, y, st.jatuh.p.z);
  m.rotation.z = Math.min(0.5, dt * 1.2);       // miring waktu mendarat - jelas bukan diletakkan
}

function gambar() {
  if (!renderer) return;
  var d = st.dim;
  var cfg = cfgKin();
  var titik = chainPoints(st.joint, cfg);
  var g = gripperPoints(st.joint, cfg, st.grip.pos);

  var span = Math.max(200, d.limitv[1] - d.limitv[0]);
  var tengah = (d.limitv[0] + d.limitv[1]) / 2;
  bagian.rel.scale.x = span / (bagian.rel.geometry.parameters.width || 1);
  bagian.rel.position.set(tengah, -14, 0);
  bagian.stopA.position.set(d.limitv[0], 18, 0);
  bagian.stopB.position.set(d.limitv[1], 18, 0);

  bagian.kereta.position.set(st.joint[0], 22, 0);

  ruasKe(bagian.tiang, ke3(titik[0]), ke3(titik[1]));
  for (var i = 0; i < 3; i++) ruasKe(bagian.lengan[i], ke3(titik[i + 1]), ke3(titik[i + 2]));
  for (var k = 0; k < 4; k++) bagian.sendi[k].position.copy(ke3(titik[k + 1]));

  ruasKe(bagian.gripBadan, ke3(titik[4]), ke3(g.pangkal));
  for (var j = 0; j < 2; j++) ruasKe(bagian.jari[j], ke3(g.jari[j].atas), ke3(g.jari[j].ujung));
  bagian.tcp.position.copy(ke3(g.tcp));

  // ------------------------------------------------------------ stasiun + PCB
  var pcb = st.pcb;
  for (var s = 0; s < bagian.stasiun.length; s++) {
    var b = bagian.stasiun[s], sd = st.stasiun[s];
    if (!sd) {
      b.badan.visible = false;
      b.plat.visible = false;
      bagian.pcbSt[s].visible = false;
      if (b.label) b.label.visible = false;
      continue;
    }
    b.badan.visible = true;
    b.plat.visible = true;
    // Yang sedang ditabrak dimerahkan. Tanpa itu, "SIM_COLLIDE menyala" cuma satu
    // lampu di panel dan yang bertanya berikutnya adalah "menabrak yang mana".
    b.badan.material = (st.collide && st.collideSt === s) ? MAT.tabrak
      : (sd.tipe === 1) ? MAT.icc : (sd.tipe === 2) ? MAT.dw : MAT.wip;
    // Badan berdiri dari lantai sampai permukaan stasiun: tinggi mesin ITU YANG
    // membedakan ICC dari DW di layar, dan tingginya datang dari pose stasiun -
    // bukan angka terpisah yang bisa melenceng dari tempat gripper turun.
    // LANTAI ada di y = -30 (tiga), permukaan stasiun di y = sd.z. Badannya harus
    // menjangkau keduanya; memakai sd.z sebagai tinggi bikin mesinnya menggantung
    // 30 mm di atas lantai - kecil, dan justru karena kecil tidak pernah ditanyakan.
    // Ukuran badan mesin dibaca dari tag PLC - kotak yang SAMA dengan yang dipakai
    // penjaga tabrakan di sana. Angka gambar sendiri berarti gripper bisa berhenti
    // di udara atau menembus kotak, dan dua-duanya terbaca sebagai bug yang lain.
    var tinggi = Math.max(20, sd.z + 30);
    b.badan.scale.set(st.mesin.lebar, tinggi, st.mesin.dalam);
    b.badan.position.set(sd.x, sd.z - tinggi / 2, sd.y);
    b.plat.scale.set(st.mesin.lebar + 20, 16, st.mesin.dalam + 20);
    b.plat.position.set(sd.x, sd.z, sd.y);
    if (b.label) {
      // Sisa waktu proses ikut di label - itu yang menjelaskan kenapa robot pergi
      // ke ICC yang satunya dan bukan ke yang ini.
      var sisa = st.stTimer[s] > 0.05 ? '  ' + st.stTimer[s].toFixed(0) + 's'
        : (st.stState[s] === 2 && sd.tipe !== 0 && sd.tipe !== 3 ? '  siap' : '');
      tulisLabel(b.label, sd.nama + sisa);
      b.label.visible = true;
      b.label.position.set(sd.x, sd.z + 150, sd.y);
    }

    // PCB di stasiun. WIP IN selalu berisi (stok), WIP OUT selalu kosong (produk
    // sudah keluar dari sel) - itu yang dipegang PLC di SIM_ST_STATE, bukan tebakan
    // halaman.
    var kp = bagian.pcbSt[s];
    kp.visible = st.stState[s] > 0;
    kp.material = (st.stState[s] === 1) ? MAT.pcbProses : MAT.pcb;
    kp.scale.set(pcb.panjang, pcb.tebal, pcb.lebar);
    kp.position.set(sd.x, sd.z + pcb.tebal / 2 + 8, sd.y);
  }

  bagian.pcbHold.scale.set(pcb.panjang, pcb.tebal, pcb.lebar);
  bagian.pcbHold.visible = st.part === 1;
  // Dipegang: duduk tepat di TCP, dijepit kedua jari di sisi kiri-kanannya.
  if (st.part === 1) bagian.pcbHold.position.copy(ke3(g.tcp));
  // Yang dipegang waktu gripper dibuka di MANUAL jatuh ke lantai. Yang digambar cuma
  // jatuhnya; yang menyatakan produknya hilang tetap PLC (SIM_PART_STATE + SIM_DROP_COUNT).
  gambarJatuh(pcb);

  orbit.tY = d.L1 * 0.8;
  cam.position.set(
    orbit.tX + orbit.jarak * Math.cos(orbit.phi) * Math.sin(orbit.theta),
    orbit.tY + orbit.jarak * Math.sin(orbit.phi),
    orbit.jarak * Math.cos(orbit.phi) * Math.cos(orbit.theta));
  cam.lookAt(orbit.tX, orbit.tY, 0);
  renderer.render(scene, cam);
}

// -------------------------------------------------------------------- panel
var NAMA_JOINT = ['X (rel)', 'theta 1', 'theta 2', 'theta 3'];
var NAMA_WORLD = ['X', 'Y', 'Z', 'theta_EE'];
var NAMA_LIMIT = ['X min', 'X maks', 't1 min', 't1 maks', 't2 min', 't2 maks', 't3 min', 't3 maks'];

function bikinJog() {
  var w = el('jog');
  w.innerHTML = '';
  for (var i = 0; i < 4; i++) {
    (function (i) {
      var row = document.createElement('div');
      row.className = 'jogrow';
      var lbl = document.createElement('span');
      lbl.className = 'lbl';
      lbl.id = 'jogl' + i;
      row.appendChild(lbl);
      [['-', 'SIM_JOG_N'], ['+', 'SIM_JOG_P']].forEach(function (b) {
        var t = document.createElement('button');
        t.textContent = b[0];
        // Tekan dan lepas dikirim TERPISAH: mode tahan-jalan butuh tombolnya
        // benar-benar bertahan ON di PLC, bukan satu pulsa per klik.
        t.onmousedown = function () { jogTekan(b[1], i, true); };
        t.onmouseup = function () { jogTekan(b[1], i, false); };
        t.onmouseleave = function () { jogTekan(b[1], i, false); };
        row.appendChild(t);
      });
      w.appendChild(row);
    })(i);
  }
}

function jogTekan(tag, i, on) {
  if (st.plc) { kirim(tag, on, i); return; }
  if (!on) return;
  var arah = (tag === 'SIM_JOG_P') ? 1 : -1;
  var dl = arah * st.step;
  if (st.mode === 0) { st.cmd[i] += dl; return; }
  var pose = st.world.slice();
  if (st.mode === 1) pose[i] += dl;
  else {
    var th = pose[3] * KIN_DEGREE_TO_RAD;
    if (i === 0) pose[0] += dl;
    else if (i === 1) { pose[1] += dl * Math.cos(th); pose[2] += dl * Math.sin(th); }
    else if (i === 2) { pose[1] -= dl * Math.sin(th); pose[2] += dl * Math.cos(th); }
    else pose[3] += dl;
  }
  offlineMinta(pose);
}

function panelTampil() {
  for (var i = 0; i < 4; i++) {
    // Angka PLC apa adanya, BUKAN yang sudah dihaluskan buat digambar. Panel yang
    // menampilkan angka hasil interpolasi berarti tidak ada lagi tempat untuk
    // membandingkan layar dengan simulator.
    el('j' + i).textContent = f2(st.jointPlc[i]);
    el('w' + i).textContent = f2(st.world[i]);
    var l = el('jogl' + i);
    if (l) l.textContent = (st.mode === 0) ? NAMA_JOINT[i] : NAMA_WORLD[i];
  }
  var h = '';
  for (var k = 0; k < 8; k++) {
    h += '<span class="lamp' + (st.limit[k] ? ' on' : '') + '"></span>' + NAMA_LIMIT[k]
       + (k % 2 ? '<br>' : ' &nbsp; ');
  }
  el('limits').innerHTML = h;
  el('beat').textContent = 'heartbeat ' + st.beat
    + (st.err ? '   -   ' + (ERR_TEKS[st.errId] || ('error ' + st.errId)) : '');

  // Siklusnya jalan DI PLC. Waktu offline tombolnya dimatikan, bukan dijalankan
  // sendiri di halaman: sekuens kedua di JS berarti dua sumber kebenaran, dan yang
  // di layar bakal terlihat benar justru waktu yang di PLC salah.
  var bisaRun = st.plc && st.selAuto && st.homed && !st.estop && !st.auto;
  el('runBtn').disabled = !bisaRun;
  el('runBtn').className = bisaRun ? 'act' : '';
  el('cstopBtn').disabled = !st.plc || !st.auto || st.stopReq;
  // Home disorot begitu dia jadi satu-satunya tombol yang berguna. Sesudah berhenti
  // total, Autorun mati dan panel bilang "PERLU HOME" - tombolnya harus ikut menunjuk
  // dirinya sendiri, bukan menunggu orang mencarinya.
  el('home').disabled = st.plc && (st.estop || st.auto);
  el('home').className = (!st.homed && !st.estop) ? 'act' : '';
  el('selAuto').checked = st.selAuto;
  el('selAuto').disabled = !st.plc;
  var eb = el('estopBtn');
  eb.textContent = st.estop ? 'Lepas E-STOP' : 'E-STOP';
  eb.className = st.estop ? 'act' : 'bahaya';
  eb.disabled = !st.plc;

  el('cState').textContent = NAMA_STATE[st.state] || st.state;
  el('cStep').textContent = st.langkah;
  el('cJob').textContent = (st.jobSrc >= 0 && st.stasiun[st.jobSrc] && st.stasiun[st.jobDst])
    ? st.stasiun[st.jobSrc].nama + ' → ' + st.stasiun[st.jobDst].nama : 'menganggur';
  var sd = st.stasiun[st.tujuan];
  el('cTuju').textContent = sd ? sd.nama : st.tujuan;
  el('cPart').textContent = st.part === 1 ? 'di gripper' : 'kosong';
  el('cCount').textContent = st.siklus;
  var bawa = st.part === 1 && st.jobDst >= 0 && st.stasiun[st.jobDst];
  el('cSebab').textContent = !st.homed
    ? (NAMA_ABORT[st.abortId] || 'berhenti') + ' - tekan Home dulu'
      + (bawa ? ' (produk tetap dipegang, lanjut ke ' + st.stasiun[st.jobDst].nama
                + ' sesudah Autorun)' : '')
    : st.stopReq ? 'cycle stop: selesaikan pekerjaan ini dulu'
    : bawa && !st.auto ? 'produk di gripper - Autorun melanjutkan antar ke '
        + st.stasiun[st.jobDst].nama + '. Buka gripper = produk JATUH'
    : (st.drop ? st.drop + ' produk jatuh' : '-');

  // Tabel stasiun: keadaan + sisa waktu tiap mesin. Ini yang menjawab "kenapa
  // robotnya diam" - biasanya karena kedua ICC masih menghitung.
  var ht = '';
  for (var s = 0; s < st.stasiun.length; s++) {
    var x = st.stasiun[s];
    ht += '<tr><td class="k">' + x.nama + '</td><td class="v">' + (NAMA_STST[st.stState[s]] || '-')
       + '</td><td class="v">' + (st.stTimer[s] > 0.05 ? f2(st.stTimer[s]) + ' s' : '-')
       + '</td></tr>';
  }
  el('stTabel').innerHTML = ht;
  el('cTabrak').textContent = !st.collide ? 'bebas'
    : (st.collideSt >= 0 && st.stasiun[st.collideSt] ? 'menyentuh ' + st.stasiun[st.collideSt].nama
       : 'menyentuh lantai');
  el('cTabrak').className = st.collide ? 'v bad' : 'v';

  el('gripPos').textContent = f2(st.grip.pos) + ' / ' + f2(st.grip.stroke) + ' mm';
  var gb = el('gripBtn');
  gb.textContent = st.grip.cmd ? 'Buka' : 'Tutup';
  gb.className = st.grip.cmd ? '' : 'act';

  var d = st.dim;
  el('dims').innerHTML =
    '<tr><td class="k">L1</td><td class="v">' + f2(d.L1) + '</td>'
    + '<td class="k">L2</td><td class="v">' + f2(d.L2) + '</td></tr>'
    + '<tr><td class="k">L3</td><td class="v">' + f2(d.L3) + '</td>'
    + '<td class="k">L4</td><td class="v">' + f2(d.L4) + '</td></tr>'
    + '<tr><td class="k">tool Y</td><td class="v">' + f2(d.toolY) + '</td>'
    + '<td class="k">tool Z</td><td class="v">' + f2(d.toolZ) + '</td></tr>'
    + '<tr><td class="k">gripper</td><td class="v">' + f2(st.grip.len) + '</td>'
    + '<td class="k">stroke</td><td class="v">' + f2(st.grip.stroke) + '</td></tr>';
}

var NAMA_STATE = ['manual', 'sedang home', 'auto siap', 'jalan', 'stop di akhir siklus',
                  'PERLU HOME', 'EMERGENCY'];
var NAMA_ABORT = { 0: 'berhenti', 1: 'emergency ditekan', 2: 'selector diubah saat jalan',
                   3: 'menabrak' };
var NAMA_STST = { 0: 'kosong', 1: 'proses', 2: 'siap diambil' };

// Tombol mengirim TEPI, bukan keadaan: PLC yang memutuskan boleh atau tidak
// (selector, home, emergency), dan halaman tidak pernah menulis SIM_AUTO langsung.
// Kalau halaman yang menyalakan, syaratnya ditegakkan di browser - tempat yang tidak
// dijalankan simulator, dan tidak ikut waktu tombolnya ditekan dari HMI sungguhan.
function pulsa(nama) {
  return kirim(nama, false).then(function () { return kirim(nama, true); })
    .then(function () { return kirim(nama, false); });
}

function pasangKontrol() {
  el('runBtn').onclick = function () { pulsa('SIM_AUTORUN'); };
  el('cstopBtn').onclick = function () { pulsa('SIM_CYCLE_STOP'); };
  el('selAuto').onchange = function () {
    st.selAuto = this.checked;
    kirim('SIM_SEL_AUTO', st.selAuto);
  };
  el('estopBtn').onclick = function () {
    st.estop = !st.estop;
    kirim('SIM_ESTOP', st.estop);
    panelTampil();
  };
  el('mode').onchange = function () {
    st.mode = +this.value;
    kirim('SIM_JOG_MODE', st.mode);
    panelTampil();
  };
  el('hold').onchange = function () { st.hold = this.checked; kirim('SIM_JOG_HOLD', st.hold); };
  el('elbow').onchange = function () { st.elbowUp = this.checked; kirim('SIM_ELBOW_UP', st.elbowUp); };
  el('stepSet').onclick = function () {
    st.step = +el('step').value || 10;
    kirim('SIM_JOG_STEP', st.step);
  };
  el('gripBtn').onclick = function () {
    st.grip.cmd = !st.grip.cmd;
    kirim('SIM_GRIP_CMD', st.grip.cmd);
    panelTampil();
  };
  el('here').onclick = function () {
    el('tx').value = st.world[0].toFixed(1); el('ty').value = st.world[1].toFixed(1);
    el('tz').value = st.world[2].toFixed(1); el('tt').value = st.world[3].toFixed(1);
  };
  el('move').onclick = function () {
    var pose = [+el('tx').value, +el('ty').value, +el('tz').value, +el('tt').value];
    if (pose.some(function (x) { return !isFinite(x); })) { el('err').textContent = 'isi keempat angkanya'; return; }
    el('err').textContent = '';
    if (!st.plc) { offlineMinta(pose); return; }
    // SIM_WORLD_CMD dulu, BARU tepi naik SIM_MOVE_EXEC. Terbalik, PLC membaca
    // target lama - dan lengannya ke tempat yang benar cuma kalau targetnya
    // kebetulan belum berubah.
    kirim('SIM_WORLD_CMD', pose)
      .then(function () { return kirim('SIM_MOVE_EXEC', false); })
      .then(function () { return kirim('SIM_MOVE_EXEC', true); });
  };
  el('home').onclick = function () {
    if (!st.plc) { st.cmd = st.homeJoint.slice(); return; }
    pulsa('SIM_HOME_EXEC');
  };
  el('reset').onclick = function () { kirim('SIM_RESET', true); };
}

function muatConfig() {
  return fetch('/api/config').then(function (r) { return r.json(); }).then(function (c) {
    st.dim.L1 = c.link.L1; st.dim.L2 = c.link.L2; st.dim.L3 = c.link.L3; st.dim.L4 = c.link.L4;
    // Sama seperti gen_sim.js: panjang gripper dijumlahkan ke tool SATU kali.
    // Halaman tidak boleh menjumlahkannya lagi waktu menggambar.
    st.dim.toolY = c.tool.Y + c.gripper.panjang;
    st.dim.toolZ = c.tool.Z;
    st.dim.offset = c.offset.nilai;
    st.dim.limitv = [c.limit.PD1300_000, c.limit.PD1300_001, c.limit.PD1300_002, c.limit.PD1300_003,
                     c.limit.PD1300_004, c.limit.PD1300_005, c.limit.PD1300_006, c.limit.PD1300_007];
    st.vel = c.jog.sumbu; st.acc = c.jog.akselerasi;
    st.velW = c.jog.world; st.step = c.jog.langkah;
    st.grip = { pos: c.gripper.bukaan_awal, stroke: c.gripper.stroke, tutup: c.gripper.tutup,
                jari: c.gripper.tebal_jari, len: c.gripper.panjang,
                cmd: false, vel: c.gripper.kecepatan };
    if (c.siklus) {
      st.stasiun = c.siklus.stasiun.map(x => ({ nama: x.nama, tipe: x.tipe, x: x.x, y: x.y,
                                                z: x.z, theta: x.theta, proses: x.proses }));
      st.approach = c.siklus.approach;
      st.mesin = c.siklus.mesin;
      st.pcb = c.siklus.pcb;
      // Waktu offline, WIP IN tetap berisi dan sisanya kosong - sama dengan yang
      // dipaksakan PLC tiap scan. Halaman tidak menjalankan sekuensnya, jadi isi
      // stasiun tidak akan berubah sendiri; itu memang yang mau ditunjukkan.
      st.stState = c.siklus.stasiun.map(x => (x.tipe === 0 ? 2 : 0));
      st.stTimer = c.siklus.stasiun.map(function () { return 0; });
    }
    st.homeJoint = c.home.sumbu.slice();
    st.joint = c.home.sumbu.slice(); st.jointPlc = c.home.sumbu.slice();
    st.cmd = c.home.sumbu.slice();
    el('step').value = st.step;
    // Kamera diatur dari UKURAN robot, bukan angka tetap: ganti L1..L4 di config
    // jadi dua kali lipat, dan angka tetap bikin lengannya keluar layar.
    // Kamera mundur sejauh yang PALING BESAR: jangkauan lengan atau panjang rel.
    // Sel 3 meter tidak muat di jarak yang pas buat lengan 1 meter, dan yang di luar
    // layar tidak kelihatan hilang - cuma tidak ada.
    var jangkau = c.link.L1 + c.link.L2 + c.link.L3 + c.link.L4 + c.gripper.panjang;
    var rel = c.limit.PD1300_001 - c.limit.PD1300_000;
    orbit.jarak = Math.max(jangkau * 2.1, rel * 1.15);
  }).catch(function () { /* bridge mati: pakai bawaan di st */ });
}

function putar(t) {
  offlineStep(t);
  gambar();
  requestAnimationFrame(putar);
}

bikinJog();
pasangKontrol();
muatConfig().then(function () {
  panelTampil();
  if (bikinScene()) { ukur(); requestAnimationFrame(putar); }
  window.addEventListener('resize', ukur);
  ping();
  stream();
  setInterval(ping, 5000);
});
