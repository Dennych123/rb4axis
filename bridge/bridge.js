#!/usr/bin/env node
// Jembatan simulator NX <-> halaman viz 3D.
//
//   node blurobot/bridge/bridge.js                       lalu buka http://127.0.0.1:7656
//   node blurobot/bridge/bridge.js --endpoint opc.tcp://127.0.0.1:4840
//   node blurobot/bridge/bridge.js --user <nama> --pass <sandi>    kalau anonymous Prohibit
//
// Satu sesi OPC UA dipegang proses ini dan dipakai bersama semua tab yang terbuka.
// Sesi baru tiap klik berarti menunggu handshake berkali-kali dan simulator melihat
// belasan klien - persis alasan yang sama dengan tools/opcua/server.js.
//
// PERUBAHAN dari rencana awal: halaman disuapi lewat SSE (EventSource), bukan
// WebSocket. Node tidak punya server WebSocket bawaan dan tools/opcua/node_modules
// tidak memuat `ws`; menulis framing WebSocket sendiri cuma buat aliran SATU ARAH
// itu kerja tambahan tanpa imbalan. Perintah dari halaman lewat POST biasa.
//
// Dependensi diambil dari tools/opcua/node_modules - repo utama tetap tanpa
// dependensi, dan tidak ada node_modules kedua yang harus dipasang.
'use strict';
const fs = require('fs');
const http = require('http');
const path = require('path');

const UA = path.join(__dirname, '..', '..', 'tools', 'opcua', 'node_modules');
let OPCUAClient, AttributeIds, MessageSecurityMode, SecurityPolicy, TimestampsToReturn, OPCUACertificateManager;
try {
  ({ OPCUAClient, AttributeIds, MessageSecurityMode, SecurityPolicy, TimestampsToReturn }
    = require(path.join(UA, 'node-opcua-client')));
  // Kelasnya ada di paketnya sendiri, BUKAN diekspor ulang node-opcua-client.
  ({ OPCUACertificateManager } = require(path.join(UA, 'node-opcua-certificate-manager')));
} catch (e) {
  console.error('GAGAL: paket OPC UA tidak ketemu di ' + UA);
  console.error('       pasang dulu:  cd tools/opcua && npm install');
  process.exit(2);
}

const WEB = path.join(__dirname, '..', 'web');
const TAGS = JSON.parse(fs.readFileSync(path.join(__dirname, 'tags.json'), 'utf8'));

const arg = (nama, dflt) => {
  const i = process.argv.indexOf(nama);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1] : dflt;
};
const PORT = +arg('--port', process.env.BLUROBOT_PORT || 7656);
const ENDPOINT = arg('--endpoint', TAGS.endpoint);
const USER = arg('--user', null);
const PASS = arg('--pass', null);

// ------------------------------------------------------------------ keadaan
let klien = null, sesi = null, sub = null;
let plc = { sambung: false, sejak: null, pesan: 'belum pernah menyambung', percobaan: 0 };
const nilai = {};        // nama -> nilai terakhir yang diketahui
const meta = {};         // nama -> {nodeId, dataType, arrayType} hasil baca pertama
const klienSSE = new Set();

const jalur = n => TAGS.prefix + n;
const bolehTulis = new Set(TAGS.tulis.map(t => t.nama));
const dibaca = TAGS.baca.map(t => t.nama);

function siar() {
  const data = 'data: ' + JSON.stringify({ plc: ringkas(), nilai }) + '\n\n';
  for (const res of klienSSE) { try { res.write(data); } catch (e) { klienSSE.delete(res); } }
}
function ringkas() {
  return { sambung: plc.sambung, sejak: plc.sejak, pesan: plc.pesan, endpoint: ENDPOINT };
}

// Perubahan dikirim ter-throttle: simulator memberi kabar tiap 50 ms per tag, dan
// mengirim satu pesan SSE per tag bikin halaman menggambar ulang belasan kali per
// frame tanpa ada yang berubah di layar.
let siarTertunda = false;
function siarNanti() {
  if (siarTertunda) return;
  siarTertunda = true;
  setTimeout(() => { siarTertunda = false; siar(); }, 50);
}

// --------------------------------------------------------------- sambungan
async function putus() {
  try { if (sub) await sub.terminate(); } catch (e) {}
  try { if (sesi) await sesi.close(); } catch (e) {}
  try { if (klien) await klien.disconnect(); } catch (e) {}
  sub = null; sesi = null; klien = null;
}

// Telusuri pohon OPC UA sampai ketemu simpul variabel, lalu petakan jalur -> nodeId.
// Cabang standar OPC UA dilewati dan ada pagu keras: tanpa itu telusuran menghabiskan
// menit di node diagnostik yang tidak ada hubungannya dengan program mesin.
const LEWATI = /^(Types|Views|Server|Aliases|Locations|DataTypes|EventTypes|ObjectTypes|ReferenceTypes|VariableTypes)$/;
async function petaNode() {
  const peta = new Map();
  let dikunjungi = 0;
  async function telusuri(node, jln, dalam) {
    if (dalam > 5 || dikunjungi > 6000) return;
    let hasil;
    try { hasil = await sesi.browse(node); } catch (e) { return; }
    for (const ref of hasil.references || []) {
      if (dikunjungi++ > 6000) return;
      const nama = ref.browseName.name;
      if (LEWATI.test(nama)) continue;
      const j = jln ? jln + '.' + nama : nama;
      if (ref.nodeClass === 2) peta.set(j, ref.nodeId.toString());
      else if (ref.nodeClass === 1) await telusuri(ref.nodeId, j, dalam + 1);
    }
  }
  await telusuri('ObjectsFolder', '', 0);
  return peta;
}

async function sambung() {
  await putus();
  plc.percobaan++;
  // Sertifikat di folder tetap. Dibiarkan implisit, node-opcua berhenti selamanya
  // di "Creating default certificate" - dua kali 150 detik tanpa hasil.
  const cm = new OPCUACertificateManager({
    rootFolder: path.join(__dirname, '..', '..', 'tools', 'opcua', 'pki'),
    automaticallyAcceptUnknownCertificate: true
  });
  await cm.initialize();
  klien = OPCUAClient.create({
    endpointMustExist: false,
    connectionStrategy: { maxRetry: 1 },
    clientCertificateManager: cm,
    // None = yang dicentang di Security Settings simulator. Tanpa None, sertifikat
    // klien harus dipercaya dulu lewat Certificate management, dan penolakannya
    // memberi pesan yang terlihat seperti salah password.
    securityMode: MessageSecurityMode.None,
    securityPolicy: SecurityPolicy.None
  });
  await klien.connect(ENDPOINT);
  pasangEventPutus();
  sesi = USER ? await klien.createSession({ userName: USER, password: PASS })
              : await klien.createSession();

  // NodeId DITELUSURI, bukan dikarang. Indeks namespace ("ns=4") berbeda antar
  // controller dan antar versi Studio; nodeId karangan yang meleset ditolak sebagai
  // BadNodeIdUnknown, dan itu terbaca persis seperti "variabelnya belum ada".
  const peta = await petaNode();

  // Baca sekali dulu: itu yang memberi tahu tipe data dan bentuk (skalar/array)
  // tiap tag. Tipe bisa saja ditebak dari nama tipe IEC, tapi tebakan yang meleset
  // ditolak server sebagai BadTypeMismatch - dan pesannya tidak menyebut tag mana.
  const semua = [...new Set(dibaca.concat([...bolehTulis]))];
  let hilang = [];
  for (const n of semua) {
    const nodeId = peta.get(jalur(n));
    if (!nodeId) { hilang.push(n); continue; }
    try {
      const d = await sesi.read({ nodeId, attributeId: AttributeIds.Value });
      if (!d.statusCode.isGood()) { hilang.push(n); continue; }
      meta[n] = { nodeId, dataType: d.value.dataType, arrayType: d.value.arrayType };
      nilai[n] = d.value.value;
    } catch (e) { hilang.push(n); }
  }
  if (hilang.length) {
    console.log('  tag tidak terbaca (' + hilang.length + '): ' + hilang.join(' '));
    console.log('  biasanya: tabel variabel global belum ditempel, atau program belum ditugaskan ke task');
  }
  if (!Object.keys(meta).length) throw new Error('tidak satu pun tag terbaca di ' + ENDPOINT);

  sub = await sesi.createSubscription2({
    requestedPublishingInterval: 50, requestedLifetimeCount: 1000,
    requestedMaxKeepAliveCount: 20, publishingEnabled: true
  });
  for (const n of dibaca) {
    if (!meta[n]) continue;
    const item = await sub.monitor({ nodeId: meta[n].nodeId, attributeId: AttributeIds.Value },
      { samplingInterval: 50, queueSize: 4, discardOldest: true }, TimestampsToReturn.Neither);
    item.on('changed', d => { nilai[n] = d.value.value; siarNanti(); });
  }

  plc = { sambung: true, sejak: new Date().toISOString(), pesan: Object.keys(meta).length + ' tag', percobaan: 0 };
  console.log('tersambung : ' + ENDPOINT + '  (' + Object.keys(meta).length + ' tag)');
  siar();
}

// Simulator sering dimatikan dan dinyalakan lagi waktu orang menyunting program.
// Halaman TIDAK boleh mati bersamanya: dia pindah ke kinematik JS-nya sendiri dan
// menunggu, jadi bridge cukup mencoba lagi pelan-pelan.
async function jagaSambungan() {
  for (;;) {
    if (!plc.sambung) {
      try { await sambung(); }
      catch (e) {
        plc = { sambung: false, sejak: null, pesan: String(e.message || e), percobaan: plc.percobaan };
        siar();
      }
    }
    await new Promise(r => setTimeout(r, plc.sambung ? 2000 : 3000));
  }
}

// Simulator yang dimatikan tidak selalu menutup soket dengan rapi. Yang menyatakan
// putus adalah event kliennya, bukan tebakan dari objek sesi - sesi yang sudah mati
// tetap terlihat "tersambung" dari luar, dan halaman ikut menampilkan angka basi
// yang tidak berubah lagi tanpa satu pun tanda.
function pasangEventPutus() {
  if (!klien) return;
  for (const ev of ['connection_lost', 'close', 'abort']) {
    klien.on(ev, () => {
      if (!plc.sambung) return;
      plc = { sambung: false, sejak: null, pesan: 'sambungan putus (' + ev + ')', percobaan: 0 };
      console.log('putus      : ' + ev);
      siar();
    });
  }
}

// ------------------------------------------------------------------- tulis
// Elemen array ditulis dengan MENIMPA seluruh array: nilai terakhir yang diketahui
// diambil dari bayangan lokal, satu elemen diganti, lalu dikirim utuh. Tanpa itu
// butuh IndexRange, dan IndexRange yang salah ditolak dengan pesan yang tidak
// menyebut indeksnya.
async function tulis(nama, nilaiBaru, indeks) {
  if (!bolehTulis.has(nama)) throw new Error('tag tidak boleh ditulis: ' + nama);
  if (!plc.sambung || !meta[nama]) throw new Error('PLC tidak tersambung');
  const m = meta[nama];
  let v = nilaiBaru;
  if (indeks !== undefined && indeks !== null) {
    const arr = Array.isArray(nilai[nama]) ? Array.from(nilai[nama]) : [];
    if (indeks < 0 || indeks >= arr.length) throw new Error('indeks di luar array: ' + nama + '[' + indeks + ']');
    arr[indeks] = nilaiBaru;
    v = arr;
  }
  await sesi.write({
    nodeId: m.nodeId,
    attributeId: AttributeIds.Value,
    value: { value: { dataType: m.dataType, arrayType: m.arrayType, value: v } }
  });
  nilai[nama] = v;      // tag tulis-saja tidak dipantau, jadi bayangannya diurus di sini
  siarNanti();
}

// -------------------------------------------------------------------- HTTP
const MIME = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
               '.css': 'text/css; charset=utf-8', '.json': 'application/json; charset=utf-8' };

// Server lokal yang menerima perintah dari halaman web mana pun itu lubang, bukan
// alat. /api/ping dikecualikan karena dia yang dipakai halaman lain buat tahu
// bridge-nya hidup - dan dia tidak mengubah apa pun.
function asalLokal(req) {
  const o = req.headers.origin;
  if (!o) return true;
  return /^https?:\/\/(127\.0\.0\.1|localhost)(:\d+)?$/.test(o);
}

function kirim(res, kode, isi, tipe) {
  res.writeHead(kode, { 'Content-Type': tipe || 'application/json; charset=utf-8' });
  res.end(typeof isi === 'string' ? isi : JSON.stringify(isi));
}

const server = http.createServer((req, res) => {
  const u = new URL(req.url, 'http://127.0.0.1');
  const p = u.pathname;

  if (p === '/api/ping') {
    // Halaman MENANYAKAN status, tidak menebaknya dari protokol - halaman sering
    // dibuka dari file:// sementara servernya jalan, dan tebakan bikin alat yang
    // siap dipakai kelihatan mati.
    res.setHeader('Access-Control-Allow-Origin', '*');
    return kirim(res, 200, { ok: true, plc: ringkas(), tag: Object.keys(meta).length });
  }
  if (!asalLokal(req)) return kirim(res, 403, { err: 'Origin asing ditolak' });

  if (p === '/api/tags') return kirim(res, 200, TAGS);

  // Halaman butuh dimensi robot bahkan waktu PLC mati - kalau tidak, mode offline
  // menggambar lengan sepanjang nol dan itu terbaca seperti halamannya yang rusak.
  // Waktu PLC hidup, yang dipakai tetap nilai dari controller.
  if (p === '/api/config') {
    return kirim(res, 200, fs.readFileSync(path.join(__dirname, '..', 'sim', 'robot.config.json'), 'utf8'),
      'application/json; charset=utf-8');
  }

  if (p === '/api/stream') {
    res.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-cache',
                         Connection: 'keep-alive' });
    res.write('retry: 2000\n\n');
    res.write('data: ' + JSON.stringify({ plc: ringkas(), nilai }) + '\n\n');
    klienSSE.add(res);
    req.on('close', () => klienSSE.delete(res));
    return;
  }

  if (p === '/api/write' && req.method === 'POST') {
    let body = '';
    req.on('data', c => { body += c; if (body.length > 64000) req.destroy(); });
    req.on('end', async () => {
      try {
        const j = JSON.parse(body || '{}');
        await tulis(j.nama, j.nilai, j.indeks);
        kirim(res, 200, { ok: true });
      } catch (e) { kirim(res, 400, { err: String(e.message || e) }); }
    });
    return;
  }

  // Berkas statis dari blurobot/web. Path dari luar TIDAK pernah dipakai apa
  // adanya: yang diambil cuma nama berkasnya, jadi ../ tidak bisa keluar folder.
  const nama = p === '/' ? 'index.html' : path.basename(p);
  const berkas = path.join(WEB, nama);
  if (!fs.existsSync(berkas)) return kirim(res, 404, { err: 'tidak ada: ' + nama });
  kirim(res, 200, fs.readFileSync(berkas, 'utf8'), MIME[path.extname(nama)] || 'text/plain; charset=utf-8');
});

server.on('error', e => {
  if (e.code === 'EADDRINUSE') {
    console.error('GAGAL: port ' + PORT + ' sudah dipakai - jendela bridge yang lama masih hidup.');
    console.error('       tutup yang lama, atau jalankan dengan --port <lain>');
    process.exit(2);
  }
  throw e;
});

server.listen(PORT, '127.0.0.1', () => {
  console.log('bridge     : http://127.0.0.1:' + PORT);
  console.log('endpoint   : ' + ENDPOINT);
  console.log('tag        : ' + TAGS.baca.length + ' dibaca, ' + TAGS.tulis.length + ' boleh ditulis');
  console.log('');
  console.log('Halaman tetap jalan walau simulator belum hidup - dia pindah ke kinematik JS');
  console.log('dan menandainya di layar. Bridge menyambung sendiri begitu simulator ada.');
  jagaSambungan();
});
