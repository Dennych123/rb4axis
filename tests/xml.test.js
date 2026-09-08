// XML import Sysmac (sim/BlurobotSim.xml) - bentuknya, bukan maksudnya.
//
// Dua gerbang, dan keduanya perlu:
//
//   * XSD RESMI milik Studio (lewat scripts/validate_xml.ps1). Menangkap bentuk
//     elemen yang salah, yang di Studio cuma muncul sebagai "(Import failed)" tanpa
//     nomor baris. SKIP kalau Sysmac/pwsh tidak ada di mesin ini - dan SKIP-nya
//     bersuara, karena SKIP yang diam tidak bisa dibedakan dari lulus.
//   * tes di bawah ini. XSD menerima <TypeName>ARRAY[0..3] OF LREAL</TypeName> tanpa
//     keluhan (TypeName itu xsd:string apa saja); yang menolaknya Studio, belakangan.
//
// Yang TIDAK bisa dijawab di sini: apakah Studio benar-benar mau meng-import POU
// ber-badan ST lewat XML. Itu butuh satu putaran ke Studio, dan sampai itu dilakukan
// jalur tempel manual di SETUP.md tetap jalur yang terbukti.
'use strict';
const fs = require('fs');
const path = require('path');
const { spawnSync } = require('child_process');

const ROOT = path.join(__dirname, '..');
const SIM = path.join(ROOT, 'sim');
const XML_PATH = path.join(SIM, 'BlurobotSim.xml');

let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

// --------------------------------------------------- masih sesuai isi sim/
const r = spawnSync(process.execPath, [path.join(ROOT, 'tools', 'gen_xml.js'), '--check'],
  { encoding: 'utf8' });
chk('BlurobotSim.xml sesuai sim/*.st + *.tsv', r.status === 0,
    ((r.stdout || '') + (r.stderr || '')).trim());

const xml = fs.readFileSync(XML_PATH, 'utf8');

// ------------------------------------------------------------- bentuk POU
chk('dua FB + satu program ada',
    /<FunctionBlock name="FORWARD_KINEMATIC_V2">/.test(xml)
    && /<FunctionBlock name="INVERSE_KINEMATIC_V2">/.test(xml)
    && /<Program name="PRG_SIM_ROBOT">/.test(xml));
chk('FB V1 TIDAK ikut ter-import', !/name="(FORWARD|INVERSE)_KINEMATIC"/.test(xml),
    'yang di extract/ itu catatan, bukan bagian project sim');
chk('badan ST dipakai bentuk Omron (BodyContent xsi:type="ST")',
    (xml.match(/<BodyContent xsi:type="ST">/g) || []).length === 3);

// ARRAY harus jadi ArrayTypeSpec. Ditulis sebagai TypeName, XSD tetap lolos dan
// Studio yang menolak belakangan - persis kelas kegagalan yang mahal.
chk('tidak ada ARRAY yang nyangkut di TypeName', !/<TypeName>[^<]*ARRAY/i.test(xml),
    'ARRAY harus <InstantlyDefinedType xsi:type="ArrayTypeSpec">');
chk('ArrayTypeSpec dipakai', (xml.match(/xsi:type="ArrayTypeSpec"/g) || []).length >= 10);

// orderWithinParamSet: SATU pencacah yang jalan terus dari InputVars ke OutputVars,
// mulai 0, tanpa lompat. Angka itu yang menentukan urutan pin kotaknya di ladder.
for (const nama of ['FORWARD_KINEMATIC_V2', 'INVERSE_KINEMATIC_V2']) {
  const i = xml.indexOf('<FunctionBlock name="' + nama + '">');
  const blok = xml.slice(i, xml.indexOf('</Parameters>', i));
  const ord = (blok.match(/orderWithinParamSet="(\d+)"/g) || [])
    .map(s => +s.replace(/\D/g, ''));
  chk(nama + ': orderWithinParamSet 0..' + (ord.length - 1) + ' berurutan',
      ord.length >= 4 && ord.every((v, k) => v === k), ord.join(','));
}

// -------------------------------------------------- teks ST utuh, bukan mirip
// Escape yang meleset menghasilkan XML yang tetap sah tapi ST-nya berubah - dan
// perbandingan "kelihatan sama" tidak menangkap satu &lt; yang hilang.
function unesc(s) {
  return s.replace(/&lt;/g, '<').replace(/&gt;/g, '>')
          .replace(/&quot;/g, '"').replace(/&amp;/g, '&');
}
const isiST = [...xml.matchAll(/<ST>([\s\S]*?)<\/ST>/g)].map(m => unesc(m[1]));
chk('tiga badan ST terbaca balik', isiST.length === 3);
for (const f of ['FORWARD_KINEMATIC_V2.st', 'INVERSE_KINEMATIC_V2.st', 'PRG_SIM_ROBOT.st']) {
  const asli = fs.readFileSync(path.join(SIM, f), 'utf8').replace(/\r\n/g, '\n').replace(/\s+$/, '');
  chk(f + ': isinya identik sesudah di-unescape', isiST.some(s => s === asli),
      'kalau merah, ada escape yang meleset - XML-nya tetap sah, ST-nya yang berubah');
}
chk('perbandingan di ST ikut ter-escape', /&lt;=|&gt;/.test(xml),
    'ST penuh < dan >; kalau tidak ada yang ter-escape berarti ada yang lolos mentah');

// Akhiran baris LF, dan itu disengaja: pembaca XML menormalkan CRLF jadi LF sebelum
// teksnya sampai ke Studio (XML 1.0 2.11). Aturan "ST wajib CRLF" itu milik jalur
// .smc2 yang menulis ke dalam ZIP tanpa lewat parser XML. Dua jalur, dua aturan.
chk('tanpa CR di dalam berkas', xml.indexOf('\r') < 0,
    'XML menormalkan akhiran baris - memaksa CRLF di sini tidak ada gunanya');

// ------------------------------------------------------------ variabel global
const glob = fs.readFileSync(path.join(SIM, 'GlobalVariables.tsv'), 'utf8')
  .split('\n').filter(Boolean).map(l => l.split('\t'));
chk('semua variabel global ikut', glob.every(c => xml.includes('<Variable name="' + c[0] + '"')),
    glob.filter(c => !xml.includes('<Variable name="' + c[0] + '"')).map(c => c[0]).join(' '));

// Constant dan Retain itu atribut KONTAINER, bukan atribut variabel - GlobalVars
// boleh muncul berkali-kali, dan itu yang dilakukan Sample.xml Omron.
chk('konstanta ada di kontainer constant="true"', (() => {
  const i = xml.indexOf('<GlobalVars constant="true"');
  if (i < 0) return false;
  const blok = xml.slice(i, xml.indexOf('</GlobalVars>', i));
  return ['PI', 'DEGREE_TO_RAD', 'RAD_TO_DEGREE'].every(n => blok.includes('name="' + n + '"'));
})(), 'kalau constant ditulis sebagai atribut Variable, XSD menolak');
chk('nilai awal konstanta ikut', /<InitialValue><SimpleValue value="3.141592654" \/>/.test(xml));

// Network Publish: variabel yang tidak di-publish bisa TIDAK MUNCUL sama sekali di
// server OPC UA simulator, dan gejalanya "tag tidak terbaca" - sama persis dengan
// gejala program yang belum ditugaskan ke task. Sudah kejadian: 47 tag, nol terbaca.
// Yang diperiksa di sini TAG-nya, bukan daftar variabelnya: yang tidak dicari bridge
// boleh saja tidak di-publish.
// Dicari HANYA di dalam <Instances> - tabel variabel globalnya. Nama yang sama juga
// muncul di ExternalVars milik program, dan yang itu memang tidak punya (dan tidak
// boleh punya) atribut publish. Mencari di seluruh berkas menemukan yang salah
// duluan, dan tesnya melapor "nol ter-publish" padahal semuanya ter-publish.
const tags = JSON.parse(fs.readFileSync(path.join(ROOT, 'bridge', 'tags.json'), 'utf8'));
const namaTag = [...new Set(tags.baca.concat(tags.tulis).map(t => t.nama))];
const instances = xml.slice(xml.indexOf('<Instances>'));
const takPublish = namaTag.filter(n => {
  const i = instances.indexOf('<Variable name="' + n + '">');
  if (i < 0) return true;
  return !instances.slice(i, instances.indexOf('</Variable>', i)).includes('networkPublish="PublishOnly"');
});
chk('tiap tag yang dicari bridge di-publish (' + namaTag.length + ' tag)', takPublish.length === 0,
    takPublish.join(' ') + ' - tanpa Publish Only, tag itu tidak ada di pohon OPC UA');
chk('nilai networkPublish sesuai enum XSD',
    !/networkPublish="(?!PublishOnly|Input|Output|DoNotPublish)/.test(xml),
    'ejaan label Studio ("Publish Only") BUKAN nilai XML ("PublishOnly")');

// ExternalVars per program: simbol global yang dipakai PRG_SIM_ROBOT harus dideklarasi
// ULANG di programnya. Yang lupa lolos XSD, lolos import, lalu muncul sebagai
// variabel merah di Studio - tidak ada yang memberi tahu.
const iProg = xml.indexOf('<Program name="PRG_SIM_ROBOT">');
const extBlok = xml.slice(iProg, xml.indexOf('</ExternalVars>', iProg));
const kode = fs.readFileSync(path.join(SIM, 'PRG_SIM_ROBOT.st'), 'utf8')
  .split('\n').map(l => l.replace(/\/\/.*$/, '')).join('\n');
const kurang = glob.map(c => c[0])
  .filter(n => new RegExp('\\b' + n + '\\b').test(kode) && !extBlok.includes('name="' + n + '"'));
chk('ExternalVars program memuat tiap global yang dipakainya', kurang.length === 0,
    kurang.join(' '));

// ------------------------------------------------------------ XSD resmi Studio
const XSD = 'C:\\Program Files\\OMRON\\Sysmac Studio\\Sample\\IEC 61131-10 XML\\Controller';
const adaXsd = fs.existsSync(path.join(XSD, 'IEC61131_10_Ed1_0_Spc1_0.xsd'));
// Validatornya milik repo ALAT (sysmac-generator), bukan repo ini. rb4axis bisa
// di-klon sendirian, dan waktu itu gerbang XSD memang tidak bisa dijalankan -
// bukan gagal.
const VALIDATOR = path.join(ROOT, '..', 'scripts', 'validate_xml.ps1');
const pwsh = spawnSync('pwsh', ['-NoProfile', '-Command', '$PSVersionTable.PSVersion.Major'],
  { encoding: 'utf8' });
if (!fs.existsSync(VALIDATOR)) {
  console.log('  SKIP  validate_xml.ps1 tidak ada - repo alat sysmac-generator tidak di sebelah repo ini');
  console.log('        ' + VALIDATOR);
} else if (!adaXsd) {
  console.log('  SKIP  XSD Sysmac tidak ada di mesin ini (' + XSD + ')');
  console.log('        XSD-nya milik Studio, memang tidak boleh disalin ke repo');
} else if (pwsh.status !== 0) {
  console.log('  SKIP  pwsh tidak ada di mesin ini - XSD tidak bisa dijalankan');
} else {
  const v = spawnSync('pwsh', ['-NoProfile', '-File', VALIDATOR, XML_PATH], { encoding: 'utf8' });
  const out = ((v.stdout || '') + (v.stderr || '')).trim();
  chk('lolos XSD resmi Sysmac', v.status === 0, out.split('\n').slice(-3).join(' | '));
}

console.log(fail ? 'GAGAL ' + fail : 'LULUS');
process.exit(fail ? 1 : 0);
