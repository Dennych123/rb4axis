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

var ERR_TEKS = { 0: '', 1: 'out of reach (R > L2+L3)', 2: 'too close (R < |L2-L3|)',
                 3: 'singular (R = 0)', 4: 'soft limit exceeded', 5: 'would hit a machine' };

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
  ctRun: 0, ctLast: 0, ctAvg: 0, ctN: 0,
  selAuto: false, estop: false, homed: true, stopReq: false, state: 0, abortId: 0,
  drop: null, jatuh: null,        // pencacah produk jatuh + animasi jatuhnya
  tSampel: 0,                     // kapan nilai sumbu terakhir datang - dasar ramalan
  collide: false, collideSt: -1,
  approach: 140,
  stasiun: [],                    // {nama,tipe,x,y,z,theta,proses}
  stState: [0, 0, 0, 0, 0, 0], stTimer: [0, 0, 0, 0, 0, 0],
  mesin: { lebar: 300, dalam: 320, margin: 12, cover: { sudut: 80, kecepatan: 110, tebal: 18 } },
  stCover: [0, 0, 0, 0, 0, 0], coverSudut: 80,
  frames: true, tri: true, ikSrc: 0, metode: 2, showPath: true,
  dt: 0.004,                      // periode task - dipakai simulasi lintasan
  uji: null,                      // hasil uji lintasan terakhir (ramalan browser)
  plcUji: [null, null],           // hasil UKURAN PLC: [0] gerak sumbu, [1] gerak lurus
  lineVel: 250, moveMode: 0, devMax: 0, devMode: 0, moveT: 0, lineAbort: 0,
  lineAktif: false, devTrace: null, tungguPlc: -1,
  pcb: { panjang: 120, lebar: 80, tebal: 8 },
  dim: { L1: 400, L2: 300, L3: 250, L4: 100, toolY: 140, toolZ: 0,
         offset: [0, 0, 0, 0, 0], limitv: [-500, 500, -90, 180, -150, 0, -120, 120] },
  vel: [900, 90, 90, 120], acc: [1800, 240, 240, 320], ovr: 100,
  velW: [100, 100, 100, 30], step: 10, mode: 0, hold: false
};

var el = function (id) { return document.getElementById(id); };
var seretOvr = false;               // slider speed sedang diseret - jangan ditimpa nilai PLC
var perluPanel = false;             // ada nilai baru yang belum tergambar di panel
var panelTerakhir = 0;
var selTerakhir = null;             // posisi selector waktu kartu manual terakhir diatur
var jam = function () {
  return (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
};
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
  if (st.plc) { s.className = 'ok'; s.textContent = 'PLC connected - values on screen come from the NX simulator'; }
  else if (st.bridge) {
    s.className = 'off';
    s.textContent = 'PLC offline - page is running its own JS kinematics (not proof the PLC program is right). '
      + (pesan ? String(pesan).split('\n')[0] : '');
  } else {
    s.className = 'bad';
    s.textContent = 'bridge down - run: node bridge/bridge.js';
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
      if (v[TAG.joint]) {
        st.jointPlc = keArray(v[TAG.joint], 4);
        st.cmd = st.jointPlc.slice();
        st.tSampel = jam();
      }
      if (v.SIM_JOINT_VEL) st.jointVel = keArray(v.SIM_JOINT_VEL, 4);
      // LREAL kalau ada, REAL kalau tidak. Yang REAL sudah dibulatkan ke 32 bit, dan
      // halaman memakai pose ini sebagai dasar slider world dan panel penjelas - dua
      // tempat yang angkanya dibaca orang sampai tiga desimal.
      if (v.SIM_WORLD_POS_L) st.world = keArray(v.SIM_WORLD_POS_L, 4);
      else if (v[TAG.world]) st.world = keArray(v[TAG.world], 4);
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
      if (v.SIM_CT_RUN !== undefined) st.ctRun = v.SIM_CT_RUN;
      if (v.SIM_CT_LAST !== undefined) st.ctLast = v.SIM_CT_LAST;
      if (v.SIM_CT_AVG10 !== undefined) st.ctAvg = v.SIM_CT_AVG10;
      if (v.SIM_CT_N !== undefined) st.ctN = v.SIM_CT_N;
      if (v.SIM_MOVE_MODE !== undefined) st.moveMode = v.SIM_MOVE_MODE;
      if (v.SIM_LINE_VEL) st.lineVel = v.SIM_LINE_VEL;
      if (v.SIM_LINE_ACTIVE !== undefined) st.lineAktif = !!v.SIM_LINE_ACTIVE;
      if (v.SIM_LINE_ABORT !== undefined) st.lineAbort = v.SIM_LINE_ABORT;
      if (v.SIM_DEV_MAX !== undefined) st.devMax = v.SIM_DEV_MAX;
      if (v.SIM_DEV_MODE !== undefined) st.devMode = v.SIM_DEV_MODE;
      if (v.SIM_MOVE_T !== undefined) st.moveT = v.SIM_MOVE_T;
      if (v.SIM_DEV_TRACE) st.devTrace = keArray(v.SIM_DEV_TRACE, 50);
      // Hasil PLC dipanen waktu perpindahannya SELESAI. Dipanen tiap kabar, yang
      // tersimpan kurva setengah jalan - dan kurva setengah jalan terbaca seperti
      // gerakan yang berhenti di tengah.
      if (st.tungguPlc >= 0 && v.SIM_MOVE_DONE && !st.lineAktif && st.devTrace) {
        st.plcUji[st.tungguPlc] = { trace: st.devTrace.slice(), maks: st.devMax,
                                    waktu: st.moveT, abort: st.lineAbort };
        st.tungguPlc = -1;
        gambarGrafik();
        ujiHasilTampil();
      }
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
      if (v.SIM_ST_COVER) st.stCover = keArray(v.SIM_ST_COVER, 6);
      if (v.SIM_COVER_SUDUT) st.coverSudut = v.SIM_COVER_SUDUT;
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
        if (refStasiun.length !== st.stasiun.length) bikinPanelStatis();
      }
      if (v.SIM_VEL) st.vel = keArray(v.SIM_VEL, 4);
      if (v.SIM_ACC) st.acc = keArray(v.SIM_ACC, 4);
      // Override dibaca BALIK dari PLC: dia yang menjepitnya ke 1..100, dan slider yang
      // tetap menunjukkan angka yang dikirim halaman berarti layar memperlihatkan
      // setelan yang tidak dipakai siapa pun.
      if (v.SIM_SPEED_OVR !== undefined && !seretOvr) {
        st.ovr = v.SIM_SPEED_OVR;
        el('ovr').value = Math.round(st.ovr);
      }
      if (v.SIM_VEL_W) st.velW = keArray(v.SIM_VEL_W, 4);
      for (var i = 0; i < 8; i++) {
        var k = 'PD1300_00' + i;
        if (v[k] !== undefined) st.dim.limitv[i] = v[k];
      }
    }
    // Panel TIDAK digambar di sini. Pesan datang ~20x per detik dan tiap panggilan
    // menyentuh puluhan elemen; menggambarnya tiap pesan memaksa layout ulang di tengah
    // frame, dan yang terasa justru animasi 3D-nya yang tersendat - bukan panelnya.
    // Ditandai saja, lalu digambar paling sering 8x per detik dari putar().
    perluPanel = true;
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
    el('err').textContent = 'rejected: ' + (tab.st >= 0 && st.stasiun[tab.st]
      ? 'would hit ' + st.stasiun[tab.st].nama : 'would go through the floor');
    return;
  }
  var ik = inverseKinematicV2(pose, cfgKin(), st.elbowUp);
  st.err = !ik.done;
  st.errId = ik.errorId;
  el('err').textContent = ik.done ? '' : ('rejected: ' + (ERR_TEKS[ik.errorId] || ik.errorId));
  if (ik.done) st.cmd = ik.joint;
}

// Motion model (profil trapesium) TIDAK ditulis di sini lagi - dia di kin.js sebagai
// langkahSumbu(), dipakai bersama oleh mode offline halaman ini DAN oleh pembanding
// lintasan. Dua salinan pasti berbeda suatu hari, dan bedanya terbaca seperti
// PLC-nya yang salah.

var tSebelum = 0;
function offlineStep(t) {
  var dt = Math.min((t - tSebelum) / 1000, 0.1);
  tSebelum = t;
  if (!dt) return;
  if (st.plc) { haluskan(dt); return; }

  for (var i = 0; i < 4; i++) {
    var r = langkahSumbu(st.joint[i], st.cmd[i], st.jointVel[i],
                         st.vel[i] * st.ovr / 100, st.acc[i], dt);
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

// Penghalusan gambar waktu tersambung PLC. Bridge mengirim tiap ~50 ms sementara layar
// menggambar tiap ~16 ms: tanpa apa-apa, tiga frame menampilkan angka yang sama lalu
// melompat.
//
// Yang dipakai KECEPATAN sumbu dari PLC (SIM_JOINT_VEL), bukan sekadar mengejar posisi
// terakhir. Bedanya menentukan:
//
//   mengejar posisi  - gambar selalu TERTINGGAL, dan makin cepat sumbunya makin jauh
//                      tertinggal. Waktu sumbu berhenti mendadak, gambar masih meluncur.
//   memakai kecepatan - di antara dua sampel, posisi diramal dari kecepatan yang memang
//                      sedang dipakai PLC. Yang digambar lanjut bergerak dengan kecepatan
//                      yang BENAR, bukan menunggu kabar berikutnya.
//
// Ramalannya dibatasi 120 ms. Kalau kabar berhenti datang (bridge putus, simulator
// dijeda), lengan yang diramal terus akan terbang menjauh - dan yang di layar terlihat
// seperti robot yang kabur, bukan seperti sambungan yang putus.
//
// Koreksinya tetap ada tapi cepat (tau 25 ms): ramalan tidak pernah persis, dan tanpa
// koreksi selisihnya menumpuk sampai gambar dan angka panel bercerita beda.
//
// Panel tetap menampilkan st.jointPlc apa adanya - itu yang bikin layar masih bisa
// diadu ke simulator.
var TAU = 0.025;
var RAMAL_MAKS = 0.12;
function haluskan(dt) {
  var umur = Math.min((jam() - st.tSampel) / 1000, RAMAL_MAKS);
  var a = 1 - Math.exp(-dt / TAU);
  for (var i = 0; i < 4; i++) {
    var target = st.jointPlc[i] + (st.jointVel[i] || 0) * umur;
    var d = target - st.joint[i];
    st.joint[i] = (Math.abs(d) < 1e-6) ? target : st.joint[i] + d * a;
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
    el('view').innerHTML = '<p style="padding:20px">three.js did not load. If this machine is offline, '
      + 'drop a copy of <code>three.min.js</code> into the <code>web/</code> folder.</p>';
    return false;
  }
  var cv = el('cv');
  renderer = new THREE.WebGLRenderer({ canvas: cv, antialias: true });
  // Dibatasi 1.5, bukan 2. Di layar 4K, pixel ratio 2 berarti empat kali lipat piksel
  // yang harus dibayangi dan digambar tiap frame - dan yang hilang justru kehalusan
  // gerakan, yang lebih kelihatan daripada tepi yang sedikit lebih tajam.
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
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

  // Penutup berengsel tiap mesin. Dibuat sebagai PIVOT di garis engsel dengan kotak
  // penutup sebagai anaknya, bukan kotak yang diputar sendiri: memutar kotak berarti
  // memutarnya di tengah, dan penutup yang berputar di tengahnya menembus meja tiap
  // kali membuka.
  MAT.cover = new THREE.MeshStandardMaterial({ color: 0x93c5fd, metalness: 0.3, roughness: 0.35,
                                               transparent: true, opacity: 0.55 });
  bagian.cover = [];
  for (var cvi = 0; cvi < 6; cvi++) {
    var pivot = new THREE.Object3D();
    var tutup = kotak(1, 1, 1, MAT.cover);
    pivot.add(tutup);
    scene.add(pivot);
    bagian.cover.push({ pivot: pivot, tutup: tutup });
  }

  // Sumbu koordinat: satu triad per kerangka (world zero, kereta, bahu, siku,
  // pergelangan, TCP). Merah/hijau/biru = X/Y/Z, dan itu yang bikin "koordinat nol"
  // berhenti jadi angka di tabel.
  bagian.axes = [];
  for (var ax = 0; ax < 6; ax++) {
    var h = new THREE.AxesHelper(ax === 0 ? 220 : 110);
    h.material.depthTest = false;
    scene.add(h);
    bagian.axes.push(h);
  }

  // Segitiga IK: bahu -> siku -> pergelangan -> balik ke bahu. Sisi terakhir itu R,
  // dan justru sisi itu yang tidak ada bendanya - dia jarak, bukan batang.
  var gTri = new THREE.BufferGeometry();
  gTri.setAttribute('position', new THREE.BufferAttribute(new Float32Array(12), 3));
  bagian.tri = new THREE.Line(gTri, new THREE.LineBasicMaterial({ color: 0xfacc15, linewidth: 2 }));
  bagian.tri.frustumCulled = false;
  scene.add(bagian.tri);
  bagian.triLabel = [labelSprite('L2'), labelSprite('L3'), labelSprite('R')];
  bagian.triLabel.forEach(function (l) { l.scale.set(150, 38, 1); scene.add(l); });

  // Dua lintasan pembanding: kuning = gerak sumbu, cyan = gerak lurus. Digambar di
  // tempatnya supaya "melengkung" berhenti jadi kata dan jadi bentuk.
  bagian.pathJ = new THREE.Line(new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0xf59e0b }));
  bagian.pathL = new THREE.Line(new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ color: 0x22d3ee }));
  bagian.pathJ.frustumCulled = false;
  bagian.pathL.frustumCulled = false;
  bagian.pathJ.visible = false;
  bagian.pathL.visible = false;
  scene.add(bagian.pathJ);
  scene.add(bagian.pathL);

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
        : (st.stState[s] === 2 && sd.tipe !== 0 && sd.tipe !== 3 ? '  ready' : '');
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

  // ---------------------------------------------------------- penutup berengsel
  for (var cv = 0; cv < bagian.cover.length; cv++) {
    var cb = bagian.cover[cv], cd = st.stasiun[cv];
    var punya = cd && (cd.tipe === 1 || cd.tipe === 2);
    cb.pivot.visible = !!punya;
    if (!punya) continue;
    var dalam = st.mesin.dalam * 0.82, teb = st.mesin.cover.tebal;
    cb.tutup.scale.set(st.mesin.lebar, teb, dalam);
    // Anak digeser ke DEPAN engsel, jadi yang terangkat tepi depannya - engsel di
    // belakang, seperti tutup kotak.
    cb.tutup.position.set(0, teb / 2, -dalam / 2);
    cb.pivot.position.set(cd.x, cd.z + 10, cd.y + st.mesin.dalam / 2);
    cb.pivot.rotation.x = (st.stCover[cv] || 0) * Math.PI / 180;
  }

  // ------------------------------------------------------------ kerangka + segitiga
  var titikFrame = [
    { x: 0, y: 0, z: 0 },                       // world zero: tengah rel, di lantai
    titik[0], titik[1], titik[2], titik[3], g.tcp
  ];
  for (var fr = 0; fr < bagian.axes.length; fr++) {
    bagian.axes[fr].visible = st.frames;
    if (st.frames) bagian.axes[fr].position.copy(ke3(titikFrame[fr]));
  }
  bagian.tri.visible = st.tri;
  bagian.triLabel.forEach(function (l) { l.visible = st.tri; });
  if (st.tri) {
    var pos = bagian.tri.geometry.attributes.position;
    var tigaTitik = [titik[1], titik[2], titik[3], titik[1]];
    for (var tp = 0; tp < 4; tp++) {
      var v3 = ke3(tigaTitik[tp]);
      pos.setXYZ(tp, v3.x, v3.y, v3.z);
    }
    pos.needsUpdate = true;
    bagian.tri.geometry.computeBoundingSphere();
    var tengah = function (a, b) {
      var va = ke3(a), vb = ke3(b);
      return va.add(vb).multiplyScalar(0.5);
    };
    bagian.triLabel[0].position.copy(tengah(titik[1], titik[2]));
    bagian.triLabel[1].position.copy(tengah(titik[2], titik[3]));
    bagian.triLabel[2].position.copy(tengah(titik[3], titik[1]));
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
var NAMA_JOINT = ['X (rail)', 'theta 1', 'theta 2', 'theta 3'];
var NAMA_WORLD = ['X', 'Y', 'Z', 'theta_EE'];
var NAMA_LIMIT = ['X min', 'X max', 't1 min', 't1 max', 't2 min', 't2 max', 't3 min', 't3 max'];

// Bagian panel yang bentuknya tidak pernah berubah dibangun SEKALI, dan panelTampil()
// cuma mengganti teks dan kelasnya. Yang dulu: innerHTML disusun ulang tiap kabar dari
// PLC (~20x per detik) - browser membuang lalu membuat lagi puluhan elemen di tengah
// frame, dan yang terasa tersendat justru animasi 3D-nya, bukan panelnya.
var refStasiun = [];
var refLamp = [];
var refDim = [];
function bikinPanelStatis() {
  var w = el('limits');
  w.innerHTML = '';
  refLamp = [];
  for (var k = 0; k < 8; k++) {
    var lamp = document.createElement('span');
    lamp.className = 'lamp';
    w.appendChild(lamp);
    w.appendChild(document.createTextNode(NAMA_LIMIT[k]));
    w.appendChild(k % 2 ? document.createElement('br') : document.createTextNode('  '));
    refLamp.push(lamp);
  }

  // Tabel dimensi: nilainya berubah cuma waktu config diganti, tapi dulu ikut disusun
  // ulang tiap gambaran panel.
  var dm = el('dims');
  dm.innerHTML = '';
  refDim = [];
  [['L1', 'L2'], ['L3', 'L4'], ['tool Y', 'tool Z'], ['gripper', 'stroke']].forEach(function (pas) {
    var tr = document.createElement('tr');
    pas.forEach(function (nama) {
      var k = document.createElement('td');
      k.className = 'k';
      k.textContent = nama;
      var v = document.createElement('td');
      v.className = 'v';
      tr.appendChild(k);
      tr.appendChild(v);
      refDim.push(v);
    });
    dm.appendChild(tr);
  });

  var t = el('stTabel');
  t.innerHTML = '';
  refStasiun = st.stasiun.map(function (x) {
    var baris = document.createElement('div');
    baris.className = 'stbaris';
    var nama = document.createElement('div');
    nama.className = 'stnama';
    nama.textContent = x.nama;
    var badge = document.createElement('span');
    badge.className = 'badge';
    var timer = document.createElement('span');
    timer.className = 'sttimer';
    baris.appendChild(nama);
    baris.appendChild(badge);
    baris.appendChild(timer);
    t.appendChild(baris);
    return { nama: nama, badge: badge, timer: timer };
  });
}

// ---------------------------------------------------- panel kinematik (kiri)
// Tiap baris: nama, rumusnya, hasilnya. Dibangun SEKALI seperti panel kanan, dan yang
// diganti tiap gambar cuma dua teks.
//
// SEMUA angkanya datang dari fkSteps()/ikSteps() di kin.js - fungsi yang SAMA yang
// dipanggil forwardKinematicV2/inverseKinematicV2, jadi yang dijelaskan panel ini
// benar-benar yang dihitung. Panel yang menghitung sendiri pasti melenceng suatu hari,
// dan penjelasan yang melenceng lebih berbahaya daripada tidak ada penjelasan.
var refFK = {}, refIK = {}, refRT = {}, refFrame = [];

function barisRumus(induk, kunci, ref, nama, rumus) {
  var d = document.createElement('div');
  d.className = 'rumus';
  var n = document.createElement('span');
  n.className = 'nm';
  n.textContent = nama;
  var e = document.createElement('span');
  e.className = 'ex';
  e.textContent = rumus || '';
  var h = document.createElement('span');
  h.className = 'hs';
  h.textContent = '-';
  d.appendChild(n);
  d.appendChild(e);
  d.appendChild(h);
  induk.appendChild(d);
  ref[kunci] = { baris: d, ex: e, hs: h };
}

var FRAME_NAMA = [
  ['world zero', 'floor, middle of the rail'],
  ['axis 0 zero', 'carriage on the rail'],
  ['joint 1 zero', 'shoulder, top of the column'],
  ['joint 2 zero', 'elbow'],
  ['joint 3 zero', 'wrist'],
  ['tool zero (TCP)', 'between the fingertips']
];

function bikinPanelKiri() {
  var ft = el('frameTabel');
  ft.innerHTML = '';
  refFrame = FRAME_NAMA.map(function (n) {
    var d = document.createElement('div');
    d.className = 'rumus';
    var a = document.createElement('span');
    a.className = 'nm';
    a.style.width = '104px';
    a.textContent = n[0];
    var b = document.createElement('span');
    b.className = 'ex';
    b.textContent = n[1];
    var c = document.createElement('span');
    c.className = 'hs';
    c.textContent = '-';
    d.appendChild(a);
    d.appendChild(b);
    d.appendChild(c);
    ft.appendChild(d);
    return c;
  });

  var f = el('fkFlow');
  f.innerHTML = '';
  refFK = {};
  barisRumus(f, 'a1', refFK, 'a1', 'theta1');
  barisRumus(f, 'a2', refFK, 'a2', 'theta1 + theta2');
  barisRumus(f, 'a3', refFK, 'a3', 'theta1 + theta2 + theta3 = theta_EE');
  barisRumus(f, 'wy', refFK, 'Y', 'L2 cos a1 + L3 cos a2 + L4 cos a3');
  barisRumus(f, 'wz', refFK, 'Z', 'L1 + L2 sin a1 + L3 sin a2 + L4 sin a3');
  barisRumus(f, 'tool', refFK, 'tool', 'r (cos, sin)(theta_tool + a3)');
  barisRumus(f, 'tcp', refFK, 'TCP', 'wrist + tool offset');

  var k = el('ikFlow');
  k.innerHTML = '';
  refIK = {};
  barisRumus(k, 'in', refIK, 'in', 'target X Y Z theta_EE');
  barisRumus(k, 'y3', refIK, 'Y3', 'Y - tool_r cos(theta_tool + theta_EE) - L4 cos theta_EE');
  barisRumus(k, 'z3', refIK, 'Z3', 'Z - tool_r sin(...) - L1 - L4 sin theta_EE');
  barisRumus(k, 'r', refIK, 'R', 'sqrt(Y3^2 + Z3^2)   distance to the wrist');
  barisRumus(k, 'beta', refIK, 'beta', 'acos((L2^2 + L3^2 - R^2) / (2 L2 L3))');
  barisRumus(k, 'gamma', refIK, 'gam', 'acos((R^2 + L2^2 - L3^2) / (2 L2 R))');
  barisRumus(k, 'alfa', refIK, 'alfa', 'atan(Z3/Y3) + quadrant fix');
  barisRumus(k, 't1', refIK, 'th1', 'alfa + gam');
  barisRumus(k, 't2', refIK, 'th2', 'beta - 180');
  barisRumus(k, 't3', refIK, 'th3', 'theta_EE - th1 - th2');

  var r = el('rtFlow');
  r.innerHTML = '';
  refRT = {};
  barisRumus(r, 'y', refRT, 'dY', 'FK(IK(target)).Y - target.Y');
  barisRumus(r, 'z', refRT, 'dZ', 'FK(IK(target)).Z - target.Z');
  barisRumus(r, 'th', refRT, 'dEE', 'FK(IK(target)).theta_EE - target.theta_EE');
}

var f3 = function (x) { return (typeof x === 'number' && isFinite(x)) ? x.toFixed(3) : '-'; };
var deg = function (rad) { return f3(rad * KIN_RAD_TO_DEGREE) + '\u00b0'; };

function panelKiriTampil() {
  if (!refFK.a1) return;
  var cfg = cfgKin();
  var titik = chainPoints(st.joint, cfg);
  var g = gripperPoints(st.joint, cfg, st.grip.pos);
  var koor = function (p) { return f3(p.x) + ', ' + f3(p.y) + ', ' + f3(p.z); };
  var tp = [{ x: 0, y: 0, z: 0 }, titik[0], titik[1], titik[2], titik[3], g.tcp];
  for (var i = 0; i < refFrame.length; i++) refFrame[i].textContent = koor(tp[i]);

  // --- forward: dari sudut sumbu yang SEKARANG
  var fk = fkSteps(st.joint, cfg);
  refFK.a1.hs.textContent = deg(fk.a1);
  refFK.a2.hs.textContent = deg(fk.a2);
  refFK.a3.hs.textContent = deg(fk.a3);
  refFK.wy.ex.textContent = f3(fk.cos[0]) + ' + ' + f3(fk.cos[1]) + ' + ' + f3(fk.cos[2]);
  refFK.wy.hs.textContent = f3(fk.wy);
  refFK.wz.ex.textContent = f3(st.dim.L1) + ' + ' + f3(fk.sin[0]) + ' + ' + f3(fk.sin[1])
    + ' + ' + f3(fk.sin[2]);
  refFK.wz.hs.textContent = f3(fk.wz);
  refFK.tool.ex.textContent = 'r=' + f3(fk.toolR) + ' at ' + deg(fk.thTool);
  refFK.tool.hs.textContent = f3(fk.dy) + ', ' + f3(fk.dz);
  refFK.tcp.ex.textContent = 'X from the rail, Y and Z from above';
  refFK.tcp.hs.textContent = f3(fk.worldL[0]) + ', ' + f3(fk.worldL[1]) + ', ' + f3(fk.worldL[2]);

  // --- inverse: dari pose sekarang, atau dari target move point
  var target = st.ikSrc === 1
    ? [+el('tx').value, +el('ty').value, +el('tz').value, +el('tt').value]
    : fk.worldL.slice();
  if (target.some(function (x) { return !isFinite(x); })) target = fk.worldL.slice();
  var ik = ikSteps(target, cfg, st.elbowUp);

  refIK['in'].hs.textContent = f3(target[0]) + ', ' + f3(target[1]) + ', ' + f3(target[2])
    + ', ' + f3(target[3]) + '\u00b0';
  refIK.y3.hs.textContent = f3(ik.y3);
  refIK.z3.hs.textContent = f3(ik.z3);
  refIK.r.ex.textContent = 'reach ' + f3(ik.jangkauMin) + ' .. ' + f3(ik.jangkauMaks);
  refIK.r.hs.textContent = f3(ik.r);
  var pakai = ['beta', 'gamma', 'alfa', 't1', 't2', 't3'];
  var nilai = ik.done
    ? [deg(ik.beta), deg(ik.gamma), deg(ik.alfa), f3(ik.joint[1]) + '\u00b0',
       f3(ik.joint[2]) + '\u00b0', f3(ik.joint[3]) + '\u00b0']
    : ['-', '-', '-', '-', '-', '-'];
  for (var q = 0; q < pakai.length; q++) {
    refIK[pakai[q]].hs.textContent = nilai[q];
    refIK[pakai[q]].baris.className = 'rumus' + (ik.done ? '' : ' gagal');
  }
  refIK.r.baris.className = 'rumus' + (ik.done ? '' : ' gagal');
  // Cabang elbow mengubah DUA baris rumusnya, bukan cuma hasilnya - kalau tulisannya
  // tidak ikut berubah, panel menjelaskan rumus yang tidak sedang dipakai.
  refIK.t1.ex.textContent = st.elbowUp ? 'alfa - gam' : 'alfa + gam';
  refIK.t2.ex.textContent = st.elbowUp ? '180 - beta' : 'beta - 180';

  var v = el('ikVerdict');
  if (!ik.done) {
    v.className = 'kotakBad';
    v.textContent = (ERR_TEKS[ik.errorId] || 'rejected') + ' - no solution for this pose';
  } else if (ik.limitAny) {
    v.className = 'kotakBad';
    v.textContent = 'solved, but outside a soft limit - the PLC refuses this one';
  } else {
    v.className = 'kotakOk';
    v.textContent = 'solved: ' + (st.elbowUp ? 'elbow up' : 'elbow down') + ' branch, within limits';
  }

  // --- round trip
  if (ik.done) {
    var balik = fkSteps(ik.joint, cfg).worldL;
    refRT.y.hs.textContent = (balik[1] - target[1]).toExponential(2);
    refRT.z.hs.textContent = (balik[2] - target[2]).toExponential(2);
    refRT.th.hs.textContent = (balik[3] - target[3]).toExponential(2);
  } else {
    refRT.y.hs.textContent = refRT.z.hs.textContent = refRT.th.hs.textContent = '-';
  }
}

// ================================================== pembanding DH / Jacobian
// Semua angkanya dari kin.js: fkSteps (cara mesin), fkDH (cara buku teks), jacobian,
// ikJacobian. Halaman ini tidak menghitung satu pun rumusnya sendiri - kalau menghitung
// sendiri, yang dibandingkan bukan lagi dua cara, tapi dua salinan.
var refDH = {}, refJac = {};

function bikinPanelBanding() {
  var d = el('dhFlow');
  d.innerHTML = '';
  refDH = {};
  barisRumus(d, 'geo', refDH, 'geo', 'closed form: sum of link vectors');
  barisRumus(d, 'dh', refDH, 'DH', 'product of 6 DH matrices');
  barisRumus(d, 'beda', refDH, 'diff', 'same pose, two methods');

  var j = el('jacFlow');
  j.innerHTML = '';
  refJac = {};
  barisRumus(j, 'det', refJac, 'detJ', 'L2 L3 sin(theta2)   mm^2/rad');
  barisRumus(j, 'dekat', refJac, 'sing', 'how close to a singularity');
  barisRumus(j, 'ikg', refJac, 'IK-g', 'closed form, one step');
  barisRumus(j, 'ikj', refJac, 'IK-J', 'Jacobian, iterative');
  barisRumus(j, 'ikd', refJac, 'diff', 'same angles?');
}

function bandingTampil() {
  if (!refDH.geo) return;
  var cfg = cfgKin();
  var g = fkSteps(st.joint, cfg).worldL;
  var h = fkDH(st.joint, cfg).world;
  var tulis = function (r, v) {
    r.hs.textContent = f3(v[1]) + ', ' + f3(v[2]) + ', ' + f3(v[3]) + '\u00b0';
  };
  refDH.geo.baris.hidden = st.metode === 1;
  refDH.dh.baris.hidden = st.metode === 0;
  refDH.beda.baris.hidden = st.metode !== 2;
  tulis(refDH.geo, g);
  tulis(refDH.dh, h);
  var beda = Math.max(Math.abs(g[1] - h[1]), Math.abs(g[2] - h[2]), Math.abs(g[3] - h[3]));
  refDH.beda.hs.textContent = beda.toExponential(2);
  // Bedanya BUKAN nol, dan itu bukan kesalahan: DH memakai PI/2 utuh untuk sudut
  // offsetnya, sementara rumus mesin memakai 90 x DEGREE_TO_RAD yang dipotong 9 angka.
  // Sudut yang sama, pembulatan yang beda.
  refDH.beda.ex.textContent = beda < 1e-3
    ? 'same answer (floor: truncated constants)' : 'DISAGREE - one of them is wrong';
  refDH.beda.baris.className = 'rumus' + (beda < 1e-3 ? '' : ' gagal');

  // tabel DH
  var t = el('dhTabel');
  if (t && !t.dataset.siap) {
    t.dataset.siap = '1';
    t.innerHTML = '<table class="dht"><thead><tr><th>link</th><th>theta</th><th>d</th>'
      + '<th>a</th><th>alpha</th></tr></thead><tbody></tbody></table>';
  }
  if (t) {
    var tb = t.querySelector('tbody'), tabel = dhTable(st.joint, cfg), html = '';
    for (var i = 0; i < tabel.length; i++) {
      var b = tabel[i];
      html += '<tr><td>' + b.nama + '</td><td>' + (b.theta * KIN_RAD_TO_DEGREE).toFixed(1)
        + '&deg;</td><td>' + b.d.toFixed(1) + '</td><td>' + b.a.toFixed(1)
        + '</td><td>' + (b.alpha * KIN_RAD_TO_DEGREE).toFixed(0) + '&deg;</td></tr>';
    }
    tb.innerHTML = html;
  }

  // Jacobian + seberapa dekat singular
  var jc = jacobian(st.joint, cfg);
  var maksDet = cfg.L2 * cfg.L3;
  var rasio = Math.abs(jc.det) / maksDet;
  refJac.det.hs.textContent = jc.det.toFixed(0);
  refJac.dekat.hs.textContent = (rasio * 100).toFixed(1) + '%';
  refJac.dekat.ex.textContent = rasio < 0.08
    ? 'NEAR SINGULAR - elbow almost straight' : 'clear of singularity';
  refJac.dekat.baris.className = 'rumus' + (rasio < 0.08 ? ' gagal' : '');

  // IK dua cara, pada pose yang sedang dilihat
  var ikg = ikSteps(g, cfg, st.elbowUp);
  var ikj = ikJacobian(g, st.joint, cfg);
  refJac.ikg.hs.textContent = ikg.done
    ? f3(ikg.joint[1]) + ', ' + f3(ikg.joint[2]) + ', ' + f3(ikg.joint[3]) : 'no solution';
  refJac.ikj.hs.textContent = ikj.done
    ? f3(ikj.joint[1]) + ', ' + f3(ikj.joint[2]) + ', ' + f3(ikj.joint[3]) : 'did not converge';
  refJac.ikj.ex.textContent = ikj.iter + ' iterations, closed form needs 0';
  var bedaIk = ikg.done && ikj.done
    ? Math.max(Math.abs(ikg.joint[1] - ikj.joint[1]), Math.abs(ikg.joint[2] - ikj.joint[2]),
               Math.abs(ikg.joint[3] - ikj.joint[3])) : NaN;
  refJac.ikd.hs.textContent = isFinite(bedaIk) ? bedaIk.toExponential(2) + '\u00b0' : '-';
}

// ------------------------------------------------- uji lintasan: lurus atau tidak
function ujiLintasan() {
  var cfg = cfgKin();
  var A = fkSteps(st.joint, cfg).worldL.slice();
  var jarak = +el('ujiJarak').value || 300;
  var arah = +el('ujiArah').value;
  var B = A.slice();
  if (arah === 1) B[1] += jarak;
  else if (arah === 2) B[2] += jarak;
  else { B[1] += jarak * 0.707; B[2] += jarak * 0.707; }

  var ikA = inverseKinematicV2(A, cfg, st.elbowUp);
  var ikB = inverseKinematicV2(B, cfg, st.elbowUp);
  if (!ikA.done || !ikB.done) {
    el('ujiHasil').innerHTML = '<span class="kotakBad">The end point is out of reach. '
      + 'Try a smaller distance, or move the arm first.</span>';
    st.uji = null;
    gambarLintasan();
    return;
  }

  // Gerak sumbu: motion model yang SAMA dengan PLC, bukan interpolasi sudut yang rapi.
  var pj = pathJoint(ikA.joint, ikB.joint, cfg,
    { dt: st.dt, vel: st.vel, acc: st.acc });
  var pl = pathLine(A, B, cfg, 80, st.elbowUp, false);

  var titikA = { x: A[0], y: A[1], z: A[2] }, titikB = { x: B[0], y: B[1], z: B[2] };
  var dj = deviasiLurus(pj.titik, titikA, titikB);
  var dl = deviasiLurus(pl.titik, titikA, titikB);
  st.uji = { pj: pj, pl: pl, dj: dj, dl: dl, A: titikA, B: titikB };

  ujiHasilTampil();
  gambarGrafik();
  gambarLintasan();
}

var NAMA_ABORT_LINE = { 0: '', 1: 'stopped: IK refused a point on the line',
                        2: 'cancelled by another command' };

// Satu tabel untuk dua sumber: yang diramal browser dan yang DIUKUR PLC. Ditulis
// berdampingan supaya bedanya kelihatan - ramalan yang meleset dari mesin itu kabar,
// bukan gangguan.
function ujiHasilTampil() {
  var baris = function (nama, ramal, plc, satuan) {
    return '<tr><td class="k">' + nama + '</td><td class="v">' + ramal
      + '</td><td class="v"><b>' + plc + '</b></td></tr>';
  };
  var f = function (u, kunci) {
    if (!u) return '-';
    return kunci === 'maks' ? u.maks.toFixed(2) + ' mm' : u.waktu.toFixed(2) + ' s';
  };
  var pj = st.uji ? { maks: st.uji.dj.maks, waktu: st.uji.pj.waktu } : null;
  var pl = st.uji ? { maks: st.uji.dl.maks, waktu: 0 } : null;
  var h = '<table><tr><td class="k"></td><td class="v" style="color:var(--label)">predicted</td>'
        + '<td class="v" style="color:var(--label)">PLC</td></tr>'
        + baris('joint: off line', f(pj, 'maks'), f(st.plcUji[0], 'maks'))
        + baris('line: off line', f(pl, 'maks'), f(st.plcUji[1], 'maks'))
        + baris('joint: time', pj ? pj.waktu.toFixed(2) + ' s' : '-', f(st.plcUji[0], 't'))
        + baris('line: time', '-', f(st.plcUji[1], 't'))
        + '</table>';
  var ab = st.plcUji[1] && st.plcUji[1].abort ? NAMA_ABORT_LINE[st.plcUji[1].abort] : '';
  if (ab) h += '<div class="kotakBad" style="margin-top:6px">' + ab + '</div>';
  el('ujiHasil').innerHTML = h;
}

// Menjalankan perpindahan yang SAMA di PLC, sekali per mode. Titik tujuannya dihitung
// dari pose sekarang, jadi dua kali menekan berturut-turut TIDAK membandingkan hal yang
// sama - tombol kedua berangkat dari tempat tombol pertama berhenti. Karena itu titik
// awalnya disimpan, dan tombol kedua pulang dulu ke situ.
var awalUji = null;
function jalankanPlc(mode) {
  if (!st.plc) { el('ujiHasil').innerHTML = '<span class="kotakBad">needs the PLC</span>'; return; }
  var cfg = cfgKin();
  var A = awalUji || fkSteps(st.joint, cfg).worldL.slice();
  var jarak = +el('ujiJarak').value || 150;
  var arah = +el('ujiArah').value;
  var B = A.slice();
  if (arah === 1) B[1] += jarak;
  else if (arah === 2) B[2] += jarak;
  else { B[1] += jarak * 0.707; B[2] += jarak * 0.707; }
  awalUji = A;

  var sekarang = fkSteps(st.joint, cfg).worldL;
  var jauhDariAwal = Math.hypot(sekarang[1] - A[1], sekarang[2] - A[2]);
  st.tungguPlc = mode;
  st.plcUji[mode] = null;
  ujiHasilTampil();

  var kirimMove = function (target, m) {
    return kirim('SIM_MOVE_MODE', m)
      .then(function () { return kirim('SIM_WORLD_CMD', target); })
      .then(function () { return kirim('SIM_MOVE_EXEC', false); })
      .then(function () { return kirim('SIM_MOVE_EXEC', true); })
      .then(function () { return kirim('SIM_MOVE_EXEC', false); });
  };

  if (jauhDariAwal > 1) {
    // Pulang dulu ke titik awal, LURUS, supaya perjalanan pulang itu sendiri tidak
    // ikut terukur sebagai hasil. Baru sesudah itu perpindahan yang diukur dijalankan.
    st.tungguPlc = -1;
    kirimMove(A, 1).then(function () {
      setTimeout(function () { st.tungguPlc = mode; kirimMove(B, mode); }, 1600);
    });
  } else {
    kirimMove(B, mode);
  }
}

// Grafik simpangan. Kanvas 2D biasa - satu pustaka chart untuk dua kurva itu satu
// unduhan CDN lagi yang bisa gagal sendiri.
function gambarGrafik() {
  var cv = el('grafik');
  if (!cv) return;
  var g = cv.getContext('2d');
  var W = cv.width, H = cv.height;
  var gaya = getComputedStyle(document.body);
  var warnaTeks = gaya.getPropertyValue('--label') || '#888';
  var warnaGaris = gaya.getPropertyValue('--garis') || '#ccc';
  g.clearRect(0, 0, W, H);
  if (!st.uji && !st.plcUji[0] && !st.plcUji[1]) {
    g.fillStyle = warnaTeks;
    g.font = '22px system-ui, sans-serif';
    g.textAlign = 'center';
    g.fillText('press Run', W / 2, H / 2);
    return;
  }
  var pad = 34;
  // Skalanya ikut kurva TERTINGGI dari semuanya. Diskalakan per kurva, dua kurva yang
  // bedanya seribu kali lipat tergambar sama tingginya - dan itu kebalikan dari yang
  // mau ditunjukkan.
  var maks = 1;
  if (st.uji) maks = Math.max(maks, st.uji.dj.maks, st.uji.dl.maks);
  if (st.plcUji[0]) maks = Math.max(maks, st.plcUji[0].maks);
  if (st.plcUji[1]) maks = Math.max(maks, st.plcUji[1].maks);
  // Sumbu
  g.strokeStyle = warnaGaris;
  g.lineWidth = 2;
  g.beginPath();
  g.moveTo(pad, 8); g.lineTo(pad, H - pad); g.lineTo(W - 8, H - pad);
  g.stroke();
  g.fillStyle = warnaTeks;
  g.font = '18px system-ui, sans-serif';
  g.textAlign = 'right';
  g.fillText(maks.toFixed(0) + ' mm', pad - 6, 20);
  g.fillText('0', pad - 6, H - pad + 6);
  g.textAlign = 'center';
  g.fillText('start', pad + 18, H - 10);
  g.fillText('end', W - 24, H - 10);

  var kurva = function (nilai, warna, putus) {
    if (!nilai || !nilai.length) return;
    g.strokeStyle = warna;
    g.lineWidth = putus ? 2 : 3.5;
    g.setLineDash(putus ? [7, 5] : []);
    g.beginPath();
    for (var i = 0; i < nilai.length; i++) {
      var x = pad + (W - pad - 8) * (i / Math.max(1, nilai.length - 1));
      var y = (H - pad) - (H - pad - 12) * (nilai[i] / maks);
      if (i === 0) g.moveTo(x, y); else g.lineTo(x, y);
    }
    g.stroke();
    g.setLineDash([]);
  };
  if (st.uji) {
    kurva(st.uji.dj.nilai, '#f59e0b', true);
    kurva(st.uji.dl.nilai, '#22d3ee', true);
  }
  if (st.plcUji[0]) kurva(st.plcUji[0].trace, '#f59e0b', false);
  if (st.plcUji[1]) kurva(st.plcUji[1].trace, '#22d3ee', false);
}

// Lintasan digambar di 3D juga: grafik menjawab BERAPA, gambar menjawab DI MANA.
function gambarLintasan() {
  if (!bagian.pathJ) return;
  var tampil = st.showPath && st.uji;
  bagian.pathJ.visible = !!tampil;
  bagian.pathL.visible = !!tampil;
  if (!tampil) return;
  var ke = function (t) { return t.map(function (p) { return ke3(p); }); };
  bagian.pathJ.geometry.setFromPoints(ke(st.uji.pj.titik));
  bagian.pathL.geometry.setFromPoints(ke(st.uji.pl.titik));
}

// -------------------------------------------------------------- slider jog
// Slider menyetel TARGET, dan targetnya dikirim waktu slider DILEPAS. Satu tulis per
// piksel gerakan mouse membanjiri sesi OPC UA yang sama yang sedang membaca 80 tag,
// dan yang kelihatan justru robot yang tersendat - lawan dari yang sedang disetel.
var refSlider = [];
var seretSlider = -1;

function batasSlider(i) {
  var L = st.dim.limitv;
  if (st.mode === 0) return [L[i * 2], L[i * 2 + 1]];
  if (i === 0) return [L[0], L[1]];
  var jangkau = st.dim.L2 + st.dim.L3 + st.dim.L4 + st.dim.toolY;
  if (i === 3) return [-180, 180];
  return [i === 2 ? 0 : -jangkau, jangkau];
}

function bikinSlider() {
  var w = el('jogSlider');
  w.innerHTML = '';
  refSlider = [];
  for (var i = 0; i < 4; i++) {
    (function (i) {
      var row = document.createElement('div');
      row.className = 'slider';
      var lbl = document.createElement('span');
      lbl.className = 'lbl';
      var inp = document.createElement('input');
      inp.type = 'range';
      inp.step = '0.5';
      var val = document.createElement('span');
      val.className = 'val';
      row.appendChild(lbl);
      row.appendChild(inp);
      row.appendChild(val);
      w.appendChild(row);
      inp.oninput = function () {
        seretSlider = i;
        val.textContent = (+this.value).toFixed(1);
      };
      inp.onchange = function () {
        seretSlider = -1;
        sliderKirim(i, +this.value);
      };
      refSlider.push({ lbl: lbl, inp: inp, val: val });
    })(i);
  }
}

function sliderTampil() {
  for (var i = 0; i < refSlider.length; i++) {
    var r = refSlider[i], b = batasSlider(i);
    r.lbl.textContent = (st.mode === 0 ? NAMA_JOINT[i] : NAMA_WORLD[i]);
    r.inp.min = b[0];
    r.inp.max = b[1];
    if (seretSlider === i) continue;             // jangan lawan tangan yang sedang menyeret
    var nilai = st.mode === 0 ? st.jointPlc[i] : st.world[i];
    r.inp.value = nilai;
    r.val.textContent = f2(nilai);
  }
}

function sliderKirim(i, nilai) {
  if (st.mode === 0) {
    var j = st.jointPlc.slice();
    j[i] = nilai;
    if (!st.plc) { st.cmd = j; return; }
    kirim('SIM_JOG_SET', j)
      .then(function () { return kirim('SIM_JOG_SET_EXEC', false); })
      .then(function () { return kirim('SIM_JOG_SET_EXEC', true); })
      .then(function () { return kirim('SIM_JOG_SET_EXEC', false); });
    return;
  }
  var pose = st.world.slice();
  pose[i] = nilai;
  if (!st.plc) { offlineMinta(pose); return; }
  kirim('SIM_WORLD_CMD', pose)
    .then(function () { return kirim('SIM_MOVE_EXEC', false); })
    .then(function () { return kirim('SIM_MOVE_EXEC', true); })
    .then(function () { return kirim('SIM_MOVE_EXEC', false); });
}

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
  for (var k = 0; k < refLamp.length; k++) {
    refLamp[k].className = st.limit[k] ? 'lamp on' : 'lamp';
  }
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
  if (el('plcJoint')) {
    var bisaUji = st.plc && !st.auto && !st.estop && !st.selAuto;
    el('plcJoint').disabled = !bisaUji;
    el('plcLine').disabled = !bisaUji;
  }
  el('home').className = (!st.homed && !st.estop) ? 'act' : '';
  el('selAuto').checked = st.selAuto;
  el('selAuto').disabled = !st.plc;
  el('selKotak').className = 'sel' + (st.selAuto ? ' auto' : '');
  // Kartu manual dibuka/ditutup mengikuti selector, tapi HANYA waktu selectornya
  // berpindah - bukan tiap gambaran panel. Kalau tiap kali, kartu yang sengaja dibuka
  // orang buat melihat posisi akan menutup sendiri dua per sepuluh detik kemudian.
  if (selTerakhir !== st.selAuto) {
    selTerakhir = st.selAuto;
    el('kartuManual').open = !st.selAuto;
  }
  var eb = el('estopBtn');
  eb.textContent = st.estop ? 'Lepas E-STOP' : 'E-STOP';
  eb.className = st.estop ? 'act' : 'bahaya';
  eb.disabled = !st.plc;

  var cs = el('cState');
  cs.textContent = NAMA_STATE[st.state] || st.state;
  // Warna keadaan: hijau jalan, kuning menunggu tindakan, merah berhenti. Satu kata di
  // ukuran itu terbaca dari jauh - orang yang sedang melihat robotnya tidak sedang
  // membaca tabel.
  cs.className = (st.state === 3) ? 'jalan'
    : (st.state === 5 || st.state === 6) ? 'stop'
    : (st.state === 1 || st.state === 4) ? 'tunggu' : '';
  el('cStep').textContent = st.langkah;
  el('cJob').textContent = (st.jobSrc >= 0 && st.stasiun[st.jobSrc] && st.stasiun[st.jobDst])
    ? st.stasiun[st.jobSrc].nama + ' → ' + st.stasiun[st.jobDst].nama : 'menganggur';
  var sd = st.stasiun[st.tujuan];
  el('cTuju').textContent = sd ? sd.nama : st.tujuan;
  el('cPart').textContent = st.part === 1 ? 'HOLDING' : 'empty';
  el('cCount').textContent = st.siklus;
  // Cycle time: keluar ke keluar. Ditulis "-" sampai ada dua produk keluar, bukan 0 -
  // angka 0 di layar terbaca seperti sel yang sangat cepat, bukan seperti belum diukur.
  el('cCt').textContent = st.ctLast > 0 ? st.ctLast.toFixed(1) + 's' : '-';
  el('cCtAvg').textContent = st.ctN > 0 ? st.ctAvg.toFixed(1) + 's' : '-';
  el('cCtRun').textContent = st.ctRun > 0 ? st.ctRun.toFixed(1) + ' s' : '-';
  // Berapa sampel yang dipakai ikut ditulis: "avg 10" yang ternyata rata-rata 3 sampel
  // itu angka yang berbeda, dan bedanya tidak kelihatan dari angkanya sendiri.
  el('cCtAvg').parentNode.lastChild.textContent = st.ctN >= 10 ? 'avg 10' : 'avg ' + st.ctN;
  var bawa = st.part === 1 && st.jobDst >= 0 && st.stasiun[st.jobDst];
  el('cSebab').textContent = !st.homed
    ? (NAMA_ABORT[st.abortId] || 'stopped') + ' - press Home first'
      + (bawa ? ' (board still held; delivery to ' + st.stasiun[st.jobDst].nama
                + ' resumes on Autorun)' : '')
    : st.stopReq ? 'cycle stop: finishing the job in hand'
    : bawa && !st.auto ? 'board in gripper - Autorun resumes delivery to '
        + st.stasiun[st.jobDst].nama + '. Opening the gripper DROPS it'
    : (st.drop ? st.drop + ' board(s) dropped' : '-');

  // Baris stasiun: keadaan + sisa waktu tiap mesin. Ini yang menjawab "kenapa robotnya
  // diam" - biasanya karena kedua ICC masih menghitung.
  //
  // Barisnya dibangun SEKALI (bikinPanelStatis) dan di sini cuma teksnya yang diganti.
  // Menyusun ulang innerHTML tiap kabar berarti browser membuang lalu membuat lagi
  // puluhan elemen 8x per detik, dan yang tersendat justru animasi 3D-nya.
  for (var s = 0; s < refStasiun.length; s++) {
    var r = refStasiun[s], keadaan = st.stState[s] || 0;
    // "pressing" dibedakan dari "busy": penutup yang sudah rapat itu yang menekan PCB
    // ke probe, dan waktu prosesnya baru jalan sejak saat itu. Dua keadaan yang di
    // layar sama persis bikin "kenapa timernya belum turun" jadi pertanyaan.
    var nekan = keadaan === 1 && (st.stCover[s] || 0) <= 0.5;
    r.badge.textContent = nekan ? 'pressing' : (NAMA_STST[keadaan] || '-');
    r.badge.className = 'badge' + (keadaan === 1 ? ' proses' : keadaan === 2 ? ' siap' : '');
    r.timer.textContent = st.stTimer[s] > 0.05 ? st.stTimer[s].toFixed(0) + 's' : '';
    r.nama.className = 'stnama' + (st.auto && st.tujuan === s ? ' tuju' : '');
  }
  el('cTabrak').textContent = !st.collide ? 'clear'
    : (st.collideSt >= 0 && st.stasiun[st.collideSt] ? 'touching ' + st.stasiun[st.collideSt].nama
       : 'touching the floor');
  el('cTabrak').className = st.collide ? 'v bad' : 'v';

  if (!seretOvr) el('ovrNilai').textContent = Math.round(st.ovr) + '%';
  el('gripPos').textContent = f2(st.grip.pos) + ' / ' + f2(st.grip.stroke) + ' mm';
  var gb = el('gripBtn');
  gb.textContent = st.grip.cmd ? 'Open' : 'Close';
  gb.className = st.grip.cmd ? '' : 'act';

  var d = st.dim;
  var nilai = [d.L1, d.L2, d.L3, d.L4, d.toolY, d.toolZ, st.grip.len, st.grip.stroke];
  for (var q = 0; q < refDim.length; q++) refDim[q].textContent = f2(nilai[q]);

  sliderTampil();
  panelKiriTampil();
  bandingTampil();
}

var NAMA_STATE = ['MANUAL', 'HOMING', 'AUTO READY', 'RUNNING', 'STOPPING AT END OF CYCLE',
                  'HOME REQUIRED', 'EMERGENCY'];
var NAMA_ABORT = { 0: 'stopped', 1: 'emergency pressed', 2: 'selector changed while running',
                   3: 'collision' };
var NAMA_STST = { 0: 'empty', 1: 'busy', 2: 'ready' };

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
    sliderTampil();
    panelTampil();
  };
  el('hold').onchange = function () { st.hold = this.checked; kirim('SIM_JOG_HOLD', st.hold); };
  el('elbow').onchange = function () { st.elbowUp = this.checked; kirim('SIM_ELBOW_UP', st.elbowUp); };
  el('stepSet').onclick = function () {
    st.step = +el('step').value || 10;
    kirim('SIM_JOG_STEP', st.step);
  };
  // Slider dikirim waktu dilepas, bukan tiap piksel: satu tulis OPC UA per gerakan
  // mouse membanjiri sesi yang sama yang dipakai membaca 76 tag, dan yang kelihatan
  // justru gerakan robot yang tersendat - persis lawan dari yang mau disetel.
  var lepasOvr = function () {
    seretOvr = false;
    st.ovr = +el('ovr').value || 100;
    kirim('SIM_SPEED_OVR', st.ovr);
  };
  el('ovr').oninput = function () {
    seretOvr = true;
    el('ovrNilai').textContent = this.value + '%';
  };
  el('ovr').onchange = lepasOvr;
  // Dua panel, dua-duanya bisa disembunyikan: yang dilihat waktu menjelaskan bukan yang
  // dilihat waktu menjalankan, dan 3D-nya butuh ruang di dua-duanya.
  el('hideKiri').onclick = function () {
    var k = el('panelKiri');
    k.classList.toggle('sembunyi');
    this.textContent = k.classList.contains('sembunyi') ? '\u25b6 kinematics' : '\u25c0 kinematics';
    ukur();
  };
  el('hideKanan').onclick = function () {
    var k = el('panelKanan');
    k.classList.toggle('sembunyi');
    this.textContent = k.classList.contains('sembunyi') ? 'controls \u25c0' : 'controls \u25b6';
    ukur();
  };
  el('showFrames').onchange = function () { st.frames = this.checked; };
  el('showTri').onchange = function () { st.tri = this.checked; };
  el('ikSrc').onchange = function () { st.ikSrc = +this.value; panelKiriTampil(); };
  el('metode').onchange = function () { st.metode = +this.value; bandingTampil(); };
  el('ujiRun').onclick = function () { awalUji = null; ujiLintasan(); };
  el('plcJoint').onclick = function () { jalankanPlc(0); };
  el('plcLine').onclick = function () { jalankanPlc(1); };
  el('lineVelSet').onclick = function () {
    st.lineVel = +el('lineVel').value || 250;
    kirim('SIM_LINE_VEL', st.lineVel);
  };
  el('ujiArah').onchange = function () { awalUji = null; };
  el('showPath').onchange = function () { st.showPath = this.checked; gambarLintasan(); };
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
    if (pose.some(function (x) { return !isFinite(x); })) { el('err').textContent = 'fill in all four numbers'; return; }
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
    st.vel = c.jog.sumbu; st.acc = c.jog.akselerasi; st.ovr = c.jog.override_persen;
    st.dt = c.task.periode_ms / 1000;
    el('ovr').value = Math.round(st.ovr);
    st.velW = c.jog.world; st.step = c.jog.langkah;
    st.grip = { pos: c.gripper.bukaan_awal, stroke: c.gripper.stroke, tutup: c.gripper.tutup,
                jari: c.gripper.tebal_jari, len: c.gripper.panjang,
                cmd: false, vel: c.gripper.kecepatan };
    if (c.siklus) {
      st.stasiun = c.siklus.stasiun.map(x => ({ nama: x.nama, tipe: x.tipe, x: x.x, y: x.y,
                                                z: x.z, theta: x.theta, proses: x.proses }));
      st.approach = c.siklus.approach;
      st.mesin = c.siklus.mesin;
      st.coverSudut = c.siklus.mesin.cover.sudut;
      // Offline: penutup digambar TERBUKA. Halaman tidak menjalankan sekuensnya, jadi
      // penutup yang digambar tertutup akan tertutup selamanya - dan itu terbaca
      // seperti mesin yang menggantung, bukan seperti simulator yang mati.
      st.stCover = c.siklus.stasiun.map(function () { return c.siklus.mesin.cover.sudut; });
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
  // Panel paling sering 8x per detik. Mata tidak bisa membaca angka yang berganti 20x
  // per detik, dan tiap gambaran panel menyentuh puluhan elemen di tengah frame.
  if ((perluPanel || !st.plc) && t - panelTerakhir > 120) {
    perluPanel = false;
    panelTerakhir = t;
    panelTampil();
  }
  requestAnimationFrame(putar);
}

bikinJog();
pasangKontrol();
muatConfig().then(function () {
  bikinPanelStatis();
  bikinPanelKiri();
  bikinPanelBanding();
  bikinSlider();
  gambarGrafik();
  panelTampil();
  if (bikinScene()) { ukur(); requestAnimationFrame(putar); }
  window.addEventListener('resize', ukur);
  ping();
  stream();
  setInterval(ping, 5000);
});
