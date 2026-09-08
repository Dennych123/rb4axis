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
  joint: [0, 90, -90, 0],
  world: [0, 0, 0, 0],
  cmd: [0, 90, -90, 0],
  limit: [false, false, false, false, false, false, false, false],
  beat: 0, err: false, errId: 0, elbowUp: false,
  grip: { pos: 80, stroke: 80, len: 90, cmd: false, vel: 120 },
  dim: { L1: 400, L2: 300, L3: 250, L4: 100, toolY: 140, toolZ: 0,
         offset: [0, 0, 0, 0, 0], limitv: [-500, 500, -90, 180, -150, 0, -120, 120] },
  vel: [200, 30, 30, 45], velW: [100, 100, 100, 30], step: 10, mode: 0, hold: false
};

var el = function (id) { return document.getElementById(id); };
var f2 = function (x) { return (typeof x === 'number' && isFinite(x)) ? x.toFixed(2) : '-'; };

function cfgKin() {
  return { L1: st.dim.L1, L2: st.dim.L2, L3: st.dim.L3, L4: st.dim.L4,
           toolY: st.dim.toolY, toolZ: st.dim.toolZ, gripLen: st.grip.len,
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
      if (v[TAG.joint]) { st.joint = Array.from(v[TAG.joint]); st.cmd = st.joint.slice(); }
      if (v[TAG.world]) st.world = Array.from(v[TAG.world]);
      if (v[TAG.limit]) st.limit = Array.from(v[TAG.limit]);
      if (v[TAG.beat] !== undefined) st.beat = v[TAG.beat];
      st.err = !!v[TAG.err]; st.errId = v[TAG.errId] || 0;
      if (v.SIM_ELBOW_UP !== undefined) st.elbowUp = !!v.SIM_ELBOW_UP;
      if (v.SIM_GRIP_POS !== undefined) st.grip.pos = v.SIM_GRIP_POS;
      if (v.SIM_GRIP_STROKE) st.grip.stroke = v.SIM_GRIP_STROKE;
      if (v.SIM_GRIP_LEN) st.grip.len = v.SIM_GRIP_LEN;
      if (v.SIM_GRIP_CMD !== undefined) st.grip.cmd = !!v.SIM_GRIP_CMD;
      if (v.ROBOT_L1_LREAL) {
        st.dim.L1 = v.ROBOT_L1_LREAL; st.dim.L2 = v.ROBOT_L2_LREAL;
        st.dim.L3 = v.ROBOT_L3_LREAL; st.dim.L4 = v.ROBOT_L4_LREAL;
        st.dim.toolY = v.ROBOT_TOOL_Y_LREAL; st.dim.toolZ = v.ROBOT_TOOL_Z_LREAL;
      }
      if (v.SIM_VEL) st.vel = Array.from(v.SIM_VEL);
      if (v.SIM_VEL_W) st.velW = Array.from(v.SIM_VEL_W);
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
  var ik = inverseKinematicV2(pose, cfgKin(), st.elbowUp);
  st.err = !ik.done;
  st.errId = ik.errorId;
  el('err').textContent = ik.done ? '' : ('ditolak: ' + (ERR_TEKS[ik.errorId] || ik.errorId));
  if (ik.done) st.cmd = ik.joint;
}

var tSebelum = 0;
function offlineStep(t) {
  var dt = Math.min((t - tSebelum) / 1000, 0.1);
  tSebelum = t;
  if (st.plc || !dt) return;

  for (var i = 0; i < 4; i++) {
    var langkah = st.vel[i] * dt;
    var d = st.cmd[i] - st.joint[i];
    if (Math.abs(d) <= langkah) st.joint[i] = st.cmd[i];
    else st.joint[i] += (d > 0 ? langkah : -langkah);
  }

  // Gripper: model yang sama dengan di ST - didorong ke target dengan batas kecepatan.
  var target = st.grip.cmd ? 0 : st.grip.stroke;
  var dg = target - st.grip.pos;
  var lg = st.grip.vel * dt;
  st.grip.pos = (Math.abs(dg) <= lg) ? target : st.grip.pos + (dg > 0 ? lg : -lg);

  var fk = forwardKinematicV2(st.joint, cfgKin());
  st.world = fk.world;
  var ik = inverseKinematicV2(fk.worldL, cfgKin(), st.elbowUp);
  st.limit = ik.limits;
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
  c.left = -1600; c.right = 1600; c.top = 1600; c.bottom = -1600; c.near = 100; c.far = 4200;
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
    el('j' + i).textContent = f2(st.joint[i]);
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

function pasangKontrol() {
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
    if (!st.plc) { st.cmd = [0, 90, -90, 0]; return; }
    kirim('SIM_HOME_EXEC', false).then(function () { return kirim('SIM_HOME_EXEC', true); });
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
    st.vel = c.jog.sumbu; st.velW = c.jog.world; st.step = c.jog.langkah;
    st.grip = { pos: c.gripper.bukaan_awal, stroke: c.gripper.stroke, len: c.gripper.panjang,
                cmd: false, vel: c.gripper.kecepatan };
    st.joint = c.home.sumbu.slice(); st.cmd = c.home.sumbu.slice();
    el('step').value = st.step;
    // Kamera diatur dari UKURAN robot, bukan angka tetap: ganti L1..L4 di config
    // jadi dua kali lipat, dan angka tetap bikin lengannya keluar layar.
    var jangkau = c.link.L1 + c.link.L2 + c.link.L3 + c.link.L4 + c.gripper.panjang;
    orbit.jarak = jangkau * 2.1;
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
