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

console.log(fail ? 'GAGAL ' + fail : 'LULUS');
process.exit(fail ? 1 : 0);
