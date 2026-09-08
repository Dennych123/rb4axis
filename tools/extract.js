#!/usr/bin/env node
// Ekstrak algoritma kinematik Blurobot dari project mesin (.smc2) - HANYA BACA.
//
//   node blurobot/tools/extract.js [project.smc2] [--out <folder>]
//
// Hasilnya di blurobot/extract/:
//   <FB>.st          badan ST verbatim, TIDAK dirapikan dan TIDAK diformat ulang
//   interfaces.md    pin tiap FB + variabel temp + external, plus yang tidak dipakai
//   variables.tsv    variabel GLOBAL yang benar-benar dirujuk badan ST
//
// Verbatim itu syarat, bukan gaya: sim yang "memperbaiki" rumusnya menjawab robot
// LAIN dari yang jalan di mesin, dan bedanya baru ketahuan waktu lengan bergerak.
// tests/extract.test.js mengadu isi berkas ini ke .smc2 dan menuntut jalan dua kali
// menghasilkan berkas yang sama persis.
//
// Parser .smc2-nya dipinjam dari reader/ - JANGAN tulis parser kedua di sini.
// Yang ditulis sendiri cuma pemecah MEDAN baris SLWD, karena parseVars() milik
// reader membuang IV= (nilai awal), R= (retain) dan Ord= (urutan pin) - tiga medan
// yang justru jadi isi berkas ini.
'use strict';
const fs = require('fs');
const path = require('path');

const RSRC = path.join(__dirname, '..', '..', 'reader', 'src');
const { unzip, inflate } = require(path.join(RSRC, 'zip.js'));
const { text } = require(path.join(RSRC, 'env.js'));
const { xmlParse, xmlChild } = require(path.join(RSRC, 'xml.js'));
const { readStText } = require(path.join(RSRC, 'smc2.js'));

const SMC2_DEFAULT = 'C:/Users/denny/Downloads/Blurobot ECU/Blurobot ECU/BLUEROBOT ECU 28032020.smc2';
const OUT_DEFAULT = path.join(__dirname, '..', 'extract');

// FB yang diangkat. Daftar EKSPLISIT, bukan "semua FB yang ketemu": daftar yang
// diam-diam bertambah bikin tes determinisme gagal karena alasan yang tidak ada
// hubungannya dengan algoritmanya.
const FB_WANTED = ['FORWARD_KINEMATIC', 'INVERSE_KINEMATIC'];

// ---------------------------------------------------------------- baris SLWD
// ++D=LREAL <TAB> N=THETA_RAD1 <TAB> IV=0 <TAB> R=1 <TAB> Const=1 <TAB> G=VAR
// +GN=VAR_INPUT ... menandai grup yang berlaku buat baris di bawahnya. Grup itu
// dibaca dari medan G= tiap baris (selalu ada), +GN= cuma dicatat buat urutan.
function slwdRows(t) {
  const out = [];
  let grup = '';
  for (const line of t.split('\n')) {
    const s = line.trim();
    if (s.startsWith('+GN=')) { grup = (s.slice(4).split('\t')[0] || '').trim(); continue; }
    if (!s.startsWith('++D=')) continue;
    const rec = { _grup: grup };
    for (const part of s.slice(2).split('\t')) {
      const k = part.indexOf('=');
      if (k > 0) rec[part.slice(0, k).trim()] = part.slice(k + 1).trim();
    }
    if (rec.N) out.push(rec);
  }
  return out;
}

// Nama yang benar-benar disebut badan ST. Dipakai buat memisahkan external yang
// dipakai dari external yang cuma nyangkut di deklarasi (FORWARD_KINEMATIC
// mendeklarasi empat _sAXIS_REF yang tidak pernah disentuh rumusnya).
function namesIn(st) {
  const set = new Set();
  for (const m of st.match(/[A-Za-z_][A-Za-z0-9_]*/g) || []) set.add(m);
  return set;
}

function md(v) { return String(v === undefined ? '' : v).replace(/\|/g, '\\|'); }

// Pohon .oem: Entity[FunctionBlock] name=<nama FB>
//               +- Entity[Variables]  id=<berkas SLWD>
//               +- Entity[PouBody]    id=<berkas ST>
// Reader melaporkan badan FB sebagai program "(tanpa program)" - nama FB-nya
// memang cuma ada di sini, jadi pohonnya ditelusuri sendiri.
function findFbs(oemText) {
  const fbs = new Map();
  function walk(node) {
    if (node.attrs && node.attrs.type === 'FunctionBlock') {
      const rec = { name: node.attrs.name || '', varsId: null, bodyId: null };
      const kids = xmlChild(node, 'ChildEntities');
      if (kids) for (const ch of kids.kids) {
        if (ch.attrs.type === 'Variables') rec.varsId = ch.attrs.id;
        else if (ch.attrs.type === 'PouBody') rec.bodyId = ch.attrs.id;
      }
      fbs.set(rec.name, rec);
    }
    const kids = xmlChild(node, 'ChildEntities');
    if (kids) for (const ch of kids.kids) if (ch.tag === 'Entity') walk(ch);
  }
  for (const e of xmlParse(oemText).kids) if (e.tag === 'Entity') walk(e);
  return fbs;
}

function tabelPin(rows, dipakai, judul, kolomOrd) {
  if (!rows.length) return [];
  const b = ['### ' + judul, '',
    kolomOrd ? '| Ord | Nama | Tipe | Const | Dipakai badan ST |'
             : '| Nama | Tipe | Const | Dipakai badan ST |',
    kolomOrd ? '|---|---|---|---|---|' : '|---|---|---|---|'];
  for (const r of rows) {
    const cek = dipakai.has(r.N) ? 'ya' : '**TIDAK**';
    const inti = '`' + md(r.N) + '` | ' + md(r.D) + ' | ' + (r.Const ? 'ya' : '') + ' | ' + cek + ' |';
    b.push(kolomOrd ? '| ' + md(r.Ord) + ' | ' + inti : '| ' + inti);
  }
  b.push('');
  return b;
}

async function main() {
  const argv = process.argv.slice(2);
  let smc2 = null, out = OUT_DEFAULT;
  for (let i = 0; i < argv.length; i++) {
    if (argv[i] === '--out') out = argv[++i];
    else if (!smc2) smc2 = argv[i];
  }
  smc2 = smc2 || SMC2_DEFAULT;
  if (!fs.existsSync(smc2)) { console.error('GAGAL: tidak ketemu ' + smc2); process.exit(2); }

  const files = unzip(fs.readFileSync(smc2));
  const byId = new Map();
  let oemName = null;
  for (const n of files.keys()) {
    if (n.endsWith('.oem')) oemName = n;
    if (n.endsWith('.xml')) byId.set(n.split('/').pop().slice(0, -4), n);
  }
  if (!oemName) { console.error('GAGAL: bukan project Sysmac (.oem tidak ada di ZIP)'); process.exit(2); }
  const entri = async n => text(await inflate(files.get(n)));
  const byid = async id => entri(byId.get(id));

  const fbs = findFbs(await entri(oemName));

  // Tabel global: berkas SLWD yang memuat baris G=VAR_GLOBAL.
  const globals = new Map();
  for (const [, f] of files) {
    const b = await inflate(f);
    if (b.length < 8 || !text(b.subarray(0, 8)).startsWith('[SLWD ')) continue;
    const t = text(b);
    if (t.indexOf('G=VAR_GLOBAL') < 0) continue;
    for (const r of slwdRows(t)) if (r.G === 'VAR_GLOBAL' && !globals.has(r.N)) globals.set(r.N, r);
  }

  fs.mkdirSync(out, { recursive: true });
  const iface = [];
  const dipakaiGlobal = new Set();
  let jumlahBaris = 0;

  for (const nama of FB_WANTED) {
    const fb = fbs.get(nama);
    if (!fb || !fb.bodyId || !fb.varsId) {
      console.error('GAGAL: FB ' + nama + ' tidak lengkap di project ini'); process.exit(2);
    }
    const st = readStText(await byid(fb.bodyId));
    if (!st.trim()) { console.error('GAGAL: badan ST ' + nama + ' kosong'); process.exit(2); }
    // Verbatim: CRLF milik Studio dipertahankan apa adanya.
    fs.writeFileSync(path.join(out, nama + '.st'), st, 'utf8');
    const baris = st.split('\n').length;
    jumlahBaris += baris;

    const rows = slwdRows(await byid(fb.varsId));
    const dipakai = namesIn(st);
    for (const r of rows) if (r.G === 'VAR_EXTERNAL' && dipakai.has(r.N)) dipakaiGlobal.add(r.N);
    const bagian = g => rows.filter(r => r.G === g);

    iface.push('## ' + nama, '',
      'Badan ST: [`' + nama + '.st`](' + nama + '.st) - ' + baris
      + ' baris, akhiran baris CRLF (punya Studio, jangan diubah).', '');
    iface.push(...tabelPin(bagian('VAR_INPUT').concat(bagian('VAR_OUTPUT'))
      .sort((a, b) => (+a.Ord || 0) - (+b.Ord || 0)), dipakai, 'VAR_INPUT / VAR_OUTPUT (pin)', true));
    iface.push(...tabelPin(bagian('VAR'), dipakai, 'VAR (temp di dalam FB)', false));
    iface.push(...tabelPin(bagian('VAR_EXTERNAL'), dipakai,
      'VAR_EXTERNAL (harus ada sebagai variabel global)', false));
  }

  // variables.tsv: TANPA baris judul, dan kolomnya urutan tabel Global Variable
  // Studio - PERSIS seperti GlobalVariables.tsv yang dihasilkan generator repo ini
  // (TSV_HEAD di js/gen_all.js). Urutan kolom sendiri berarti berkas ini kelihatan
  // bisa ditempel padahal kolomnya melenceng, dan Studio menerimanya tanpa keluhan.
  //
  //   Name  Data type  Initial value  AT  Retain  Constant  Network Publish  Comment
  const tsv = [];
  for (const n of [...dipakaiGlobal].sort()) {
    const g = globals.get(n);
    if (!g) { console.error('GAGAL: external ' + n + ' dipakai ST tapi bukan variabel global'); process.exit(2); }
    tsv.push([g.N, g.D || '', g.IV || '', '', g.R === '1' ? 'True' : 'False',
      g.Const === '1' ? 'True' : 'False', 'Do not publish',
      (g.Com || '').replace(/\$t/g, ' ')].join('\t'));
  }
  fs.writeFileSync(path.join(out, 'variables.tsv'), tsv.join('\n') + '\n', 'utf8');

  const head = [
    '# Antarmuka FB kinematik Blurobot',
    '',
    'DIBANGKITKAN oleh `blurobot/tools/extract.js` - jangan disunting tangan.',
    'Sumber: `' + path.basename(smc2) + '`.',
    '',
    'Kolom `variables.tsv` = urutan tabel Global Variable Studio, TANPA baris judul',
    '(judul yang ikut tertempel mendarat sebagai variabel bernama `Name` bertipe `Data type`):',
    '`Name`, `Data type`, `Initial value`, `AT`, `Retain`, `Constant`, `Network Publish`, `Comment`.',
    '',
    'Nilai awal KOSONG berarti project asli memang tidak menyimpannya - dimensi robot',
    'diisi dari HMI waktu mesin dipakai, jadi untuk simulasi angkanya datang dari',
    '`blurobot/sim/robot.config.json`.',
    ''
  ];
  fs.writeFileSync(path.join(out, 'interfaces.md'), head.concat(iface).join('\n'), 'utf8');

  console.log('OK  ' + FB_WANTED.length + ' FB, ' + jumlahBaris + ' baris ST, '
    + tsv.length + ' variabel global -> ' + path.relative(process.cwd(), out));
}

main().catch(e => { console.error('GAGAL: ' + (e && e.message || e)); process.exit(1); });
