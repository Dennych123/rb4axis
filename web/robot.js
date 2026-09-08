// Viz 3D Blurobot: scene three.js + panel kontrol + sambungan ke bridge.
//
// Dua sumber gerakan, dan halaman SELALU memberi tahu yang mana sedang dipakai:
//
//   PLC    - nilai datang dari simulator NX lewat bridge (SSE). Ini yang sebenarnya.
//   OFFLINE- simulator mati, halaman menjalankan kin.js + motion model yang sama
//            di browser. Berguna buat melihat bentuk gerakannya, TAPI itu bukan
//            bukti program PLC-nya benar. Karena itu ditandai kuning di layar,
//            bukan diam-diam menggantikan.
//
// Sumbu: PLC (X, Y, Z) -> three (x, z, y). PLC Z itu ketinggian, three pakai Y-up.
// Rantai planarnya jadi bidang (z, y) di three, berputar pada sumbu x.
'use strict';

var TAG = { joint: 'SIM_JOINT_POS', world: 'SIM_WORLD_POS', limit: 'SIM_LIMIT',
            beat: 'SIM_HEARTBEAT', err: 'SIM_ERROR', errId: 'SIM_ERROR_ID' };

var st = {
  plc: false,              // bridge bilang PLC tersambung
  bridge: false,           // bridge-nya sendiri hidup
  joint: [0, 90, -90, 0],
  world: [0, 0, 0, 0],
  cmd: [0, 90, -90, 0],    // target sumbu (dipakai mode offline)
  limit: [false, false, false, false, false, false, false, false],
  beat: 0, err: false, errId: 0,
  cfg: null,
  dim: { L1: 400, L2: 300, L3: 250, L4: 100, toolY: 50, toolZ: 0,
         offset: [0, 0, 0, 0, 0], limitv: [-500, 500, -90, 180, -150, 0, -120, 120] },
  vel: [200, 30, 30, 45], velW: [100, 100, 100, 30], step: 10, mode: 0, hold: false
};

var el = function (id) { return document.getElementById(id); };
var f2 = function (x) { return (typeof x === 'number' && isFinite(x)) ? x.toFixed(2) : '-'; };

// ------------------------------------------------------------------ bridge
function post(url, body) {
  return fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' },
                      body: JSON.stringify(body) }).then(function (r) { return r.json(); });
}

// Status ditanyakan ke bridge, TIDAK ditebak dari location.protocol: halaman ini
// juga bisa dibuka langsung dari file:// sementara bridge-nya jalan, dan tebakan
// dari protokol bikin alat yang siap dipakai kelihatan mati.
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

// Perintah ke PLC. Waktu offline perintahnya dijalankan di browser dengan rumus
// yang sama - supaya tombol tidak mati dan orang bisa melihat bentuk gerakannya.
function kirim(nama, nilai, indeks) {
  if (!st.plc) return Promise.resolve({ offline: true });
  return post('/api/write', { nama: nama, nilai: nilai, indeks: indeks })
    .then(function (j) { if (j.err) el('err').textContent = j.err; return j; });
}

// ------------------------------------------------------- kinematik offline
function cfgKin() {
  return { L1: st.dim.L1, L2: st.dim.L2, L3: st.dim.L3, L4: st.dim.L4,
           toolY: st.dim.toolY, toolZ: st.dim.toolZ,
           offset: st.dim.offset, limit: st.dim.limitv };
}

function offlineMinta(pose) {
  var c = cfgKin();
  var rc = reachable(pose, c);
  if (!rc.ok) {
    st.err = true; st.errId = (rc.y3 <= 0) ? 2 : 1;
    el('err').textContent = (rc.y3 <= 0)
      ? 'ditolak: Y3 <= 0 (ATAN kehilangan kuadran di algoritma asli)'
      : 'ditolak: di luar jangkauan L2+L3';
    return;
  }
  st.err = false; st.errId = 0; el('err').textContent = '';
  st.cmd = inverseKinematic(pose, c).joint;
}

var tSebelum = 0;
function offlineStep(t) {
  var dt = Math.min((t - tSebelum) / 1000, 0.1);
  tSebelum = t;
  if (st.plc) return;
  for (var i = 0; i < 4; i++) {
    var langkah = st.vel[i] * dt;
    var d = st.cmd[i] - st.joint[i];
    if (Math.abs(d) <= langkah) st.joint[i] = st.cmd[i];
    else st.joint[i] += (d > 0 ? langkah : -langkah);
  }
  var fk = forwardKinematic(st.joint, cfgKin());
  st.world = fk.world;
  var ik = inverseKinematic(st.world, cfgKin());
  st.limit = ik.limits;
}

// ------------------------------------------------------------------- scene
var scene, cam, renderer, sendi = [];
var orbit = { theta: -0.9, phi: 1.15, jarak: 1600, tX: 0, tY: 300 };

function bikinScene() {
  if (typeof THREE === 'undefined') {
    el('view').innerHTML = '<p style="padding:20px">three.js tidak termuat. Kalau mesin ini offline, '
      + 'taruh salinan <code>three.min.js</code> di folder <code>blurobot/web/</code>.</p>';
    return false;
  }
  var cv = el('cv');
  renderer = new THREE.WebGLRenderer({ canvas: cv, antialias: true });
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0f1216);
  cam = new THREE.PerspectiveCamera(50, 1, 10, 12000);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 1.1));
  var dir = new THREE.DirectionalLight(0xffffff, 0.7);
  dir.position.set(600, 900, 700);
  scene.add(dir);

  // Lantai + rel X. Rel digambar sepanjang soft limit sumbu 0, jadi batas kerjanya
  // kelihatan sebagai benda, bukan cuma angka di panel.
  var grid = new THREE.GridHelper(2400, 24, 0x374151, 0x1f2937);
  scene.add(grid);
  var rel = new THREE.Mesh(new THREE.BoxGeometry(1000, 30, 90),
    new THREE.MeshStandardMaterial({ color: 0x475569 }));
  rel.name = 'rel';
  scene.add(rel);

  // Lima ruas, ditaruh DATAR di scene: kereta->bahu (L1), bahu->siku (L2),
  // siku->pergelangan (L3), pergelangan->ujung (L4), ujung->tool.
  //
  // Posisinya datang dari chainPoints() di kin.js, bukan dari rotasi bersarang di
  // sini. Bersarang juga benar, tapi rumus rantainya jadi ada DUA - satu di kin.js
  // yang dites, satu di halaman yang tidak - dan yang kedua bebas melenceng sambil
  // tetap menggambar lengan yang tampak wajar.
  var warna = [0x94a3b8, 0x38bdf8, 0x22c55e, 0xf59e0b, 0xef4444];
  var tebal = [34, 30, 26, 22, 14];
  sendi = [];
  for (var i = 0; i < 5; i++) {
    // Geometri tebal 1 di sumbu z: scale.z langsung jadi panjang dalam mm, jadi
    // ganti dimensi tidak perlu hitung ulang terhadap ukuran kotak bawaan.
    var m = new THREE.Mesh(new THREE.BoxGeometry(tebal[i], tebal[i], 1),
      new THREE.MeshStandardMaterial({ color: warna[i] }));
    scene.add(m);
    sendi.push(m);
  }
  for (var k = 0; k < 5; k++) {
    var bola = new THREE.Mesh(new THREE.SphereGeometry(k === 4 ? 14 : 22, 16, 12),
      new THREE.MeshStandardMaterial({ color: 0xe5e7eb }));
    bola.name = 'sendi' + k;
    scene.add(bola);
  }

  pasangOrbit(cv);
  return true;
}

// PLC (X, Y, Z) -> three (x, y, z). PLC Z itu KETINGGIAN; three pakai Y-up, jadi
// Z PLC jadi y three dan Y PLC jadi z three. Ketukar, lengan tergambar rebah dan
// rel-nya berdiri - gambar yang tetap "masuk akal" sampai dibandingkan angkanya.
function ke3(p) { return new THREE.Vector3(p.x, p.z, p.y); }

// Orbit ditulis sendiri, bukan OrbitControls: OrbitControls bukan bagian dari
// build inti three.js, jadi memakainya berarti satu unduhan CDN lagi yang bisa
// gagal sendiri - untuk lima baris matematika.
function pasangOrbit(cv) {
  var seret = false, lx = 0, ly = 0;
  cv.addEventListener('mousedown', function (e) { seret = true; lx = e.clientX; ly = e.clientY; });
  window.addEventListener('mouseup', function () { seret = false; });
  window.addEventListener('mousemove', function (e) {
    if (!seret) return;
    orbit.theta -= (e.clientX - lx) * 0.008;
    orbit.phi = Math.max(0.15, Math.min(1.5, orbit.phi - (e.clientY - ly) * 0.006));
    lx = e.clientX; ly = e.clientY;
  });
  cv.addEventListener('wheel', function (e) {
    e.preventDefault();
    orbit.jarak = Math.max(400, Math.min(6000, orbit.jarak * (1 + e.deltaY * 0.001)));
  }, { passive: false });
}

function ukur() {
  var v = el('view');
  if (!renderer) return;
  renderer.setSize(v.clientWidth, v.clientHeight, false);
  cam.aspect = v.clientWidth / Math.max(1, v.clientHeight);
  cam.updateProjectionMatrix();
}

function gambar() {
  if (!renderer) return;
  var d = st.dim;
  var rel = scene.getObjectByName('rel');
  var span = Math.max(200, d.limitv[1] - d.limitv[0]);
  rel.scale.x = span / 1000;
  rel.position.set((d.limitv[0] + d.limitv[1]) / 2, -15, 0);

  // Sumbu 0 itu PRISMATIK - kereta digeser, tidak diputar. Sudah terjaga di
  // chainPoints(), jadi di sini tinggal menggambar antar titik.
  var titik = chainPoints(st.joint, cfgKin());
  for (var i = 0; i < 5; i++) {
    ruasKe(sendi[i], ke3(titik[i]), ke3(titik[i + 1]));
    var bola = scene.getObjectByName('sendi' + i);
    if (bola) bola.position.copy(ke3(titik[i]));
  }
  orbit.tY = d.L1;

  cam.position.set(
    orbit.tX + orbit.jarak * Math.cos(orbit.phi) * Math.sin(orbit.theta),
    orbit.tY + orbit.jarak * Math.sin(orbit.phi),
    orbit.jarak * Math.cos(orbit.phi) * Math.cos(orbit.theta));
  cam.lookAt(orbit.tX, orbit.tY, 0);
  renderer.render(scene, cam);
}

// Satu ruas = kotak antara dua titik: ditaruh di tengahnya, dipanjangkan sepanjang
// jaraknya, lalu diputar menghadap titik ujung. lookAt() mengarahkan +z lokal ke
// sasaran, dan geometri ruasnya memang memanjang di +z - itu yang bikin dua baris
// ini cukup, tanpa menyusun quaternion sendiri.
function ruasKe(mesh, a, b) {
  var panjang = a.distanceTo(b);
  mesh.position.copy(a).add(b).multiplyScalar(0.5);
  mesh.scale.z = Math.max(1, panjang);
  mesh.lookAt(b);
  mesh.visible = panjang > 0.5;
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
        // Tombol jog dikirim sebagai TEKAN dan LEPAS terpisah, bukan satu klik:
        // mode tahan-jalan butuh tombolnya benar-benar bertahan ON di PLC.
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
  // Offline: satu langkah per tekan (tahan-jalan tidak ditiru di sini - yang
  // penting bentuk gerakannya, bukan mengulang mekanik tombolnya).
  var arah = (tag === 'SIM_JOG_P') ? 1 : -1;
  var d = arah * st.step;
  if (st.mode === 0) { st.cmd[i] += d; return; }
  var pose = st.world.slice();
  if (st.mode === 1) pose[i] += d;
  else {
    var th = pose[3] * KIN_DEGREE_TO_RAD;
    if (i === 0) pose[0] += d;
    else if (i === 1) { pose[1] += d * Math.cos(th); pose[2] += d * Math.sin(th); }
    else if (i === 2) { pose[1] -= d * Math.sin(th); pose[2] += d * Math.cos(th); }
    else pose[3] += d;
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
  el('beat').textContent = 'heartbeat ' + st.beat + (st.err ? '   -   error ' + st.errId : '');
  var d = st.dim;
  el('dims').innerHTML =
    '<tr><td class="k">L1</td><td class="v">' + f2(d.L1) + '</td>'
    + '<td class="k">L2</td><td class="v">' + f2(d.L2) + '</td></tr>'
    + '<tr><td class="k">L3</td><td class="v">' + f2(d.L3) + '</td>'
    + '<td class="k">L4</td><td class="v">' + f2(d.L4) + '</td></tr>'
    + '<tr><td class="k">tool Y</td><td class="v">' + f2(d.toolY) + '</td>'
    + '<td class="k">tool Z</td><td class="v">' + f2(d.toolZ) + '</td></tr>';
}

function pasangKontrol() {
  el('mode').onchange = function () {
    st.mode = +this.value;
    kirim('SIM_JOG_MODE', st.mode);
    panelTampil();
  };
  el('hold').onchange = function () { st.hold = this.checked; kirim('SIM_JOG_HOLD', st.hold); };
  el('stepSet').onclick = function () {
    st.step = +el('step').value || 10;
    kirim('SIM_JOG_STEP', st.step);
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
    // target lama - dan lengannya bergerak ke tempat yang benar cuma kalau
    // kebetulan targetnya belum berubah.
    kirim('SIM_WORLD_CMD', pose).then(function () {
      return kirim('SIM_MOVE_EXEC', false);
    }).then(function () { return kirim('SIM_MOVE_EXEC', true); });
  };
  el('home').onclick = function () {
    if (!st.plc) { st.cmd = [0, 90, -90, 0]; return; }
    kirim('SIM_HOME_EXEC', false).then(function () { return kirim('SIM_HOME_EXEC', true); });
  };
  el('reset').onclick = function () { kirim('SIM_RESET', true); };
}

function muatConfig() {
  return fetch('/api/config').then(function (r) { return r.json(); }).then(function (c) {
    st.cfg = c;
    st.dim.L1 = c.link.L1; st.dim.L2 = c.link.L2; st.dim.L3 = c.link.L3; st.dim.L4 = c.link.L4;
    st.dim.toolY = c.tool.Y; st.dim.toolZ = c.tool.Z;
    st.dim.offset = c.offset.nilai;
    st.dim.limitv = [c.limit.PD1300_000, c.limit.PD1300_001, c.limit.PD1300_002, c.limit.PD1300_003,
                     c.limit.PD1300_004, c.limit.PD1300_005, c.limit.PD1300_006, c.limit.PD1300_007];
    st.vel = c.jog.sumbu; st.velW = c.jog.world; st.step = c.jog.langkah;
    st.joint = c.home.sumbu.slice(); st.cmd = c.home.sumbu.slice();
    el('step').value = st.step;
  }).catch(function () { /* bridge mati: pakai bawaan di st.dim */ });
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
