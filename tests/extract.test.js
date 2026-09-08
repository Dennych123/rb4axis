// `blurobot/tools/extract.js` - ST dari project mesin ke berkas yang bisa di-commit.
//
// Dua sifat yang menentukan alat ini berguna atau tidak:
//
//   1. jalan dua kali menghasilkan berkas yang SAMA PERSIS. Kalau tidak, tiap ekstrak
//      jadi diff palsu dan riwayatnya berhenti dibaca.
//   2. isi `extract/*.st` masih SAMA dengan yang di `.smc2` sekarang. Berkas ekstrak
//      itu yang ditempel ke Studio; kalau project asli berubah dan ekstraknya tidak,
//      sim menjawab robot yang sudah tidak ada - tanpa satu pun tanda.
//
// Berkas `.smc2`-nya di luar repo (project mesin pelanggan), jadi suite ini SKIP kalau
// tidak ketemu - dan SKIP-nya bersuara, karena SKIP yang diam tidak bisa dibedakan
// dari lulus.
'use strict';
const fs = require('fs');
const os = require('os');
const path = require('path');
const { spawnSync } = require('child_process');

const ROOT = path.join(__dirname, '..');
const SCRIPT = path.join(ROOT, 'tools', 'extract.js');
const SMC2 = 'C:/Users/denny/Downloads/Blurobot ECU/Blurobot ECU/BLUEROBOT ECU 28032020.smc2';
const KELUAR = path.join(ROOT, 'extract');

let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

// Repo ini bisa di-klon sendirian (rb4axis), tanpa repo alat di atasnya. Kalau
// begitu, extract/ yang ter-commit tetap dipakai - yang tidak bisa cuma
// MEMBANGKITKANNYA ulang, dan itu bukan kegagalan.
const READER = path.join(ROOT, '..', 'reader', 'src', 'zip.js');
if (!fs.existsSync(READER)) {
  console.log('  SKIP  pembaca .smc2 (repo alat sysmac-generator) tidak ada di sebelah repo ini');
  console.log('        ' + READER);
  console.log('        extract/ yang ter-commit tetap sah - yang tidak bisa cuma regenerasinya');
  process.exit(0);
}

if (!fs.existsSync(SMC2)) {
  console.log('  SKIP  project mesin tidak ada di mesin ini:');
  console.log('        ' + SMC2);
  console.log('        (berkas project pelanggan, memang tidak ikut repo)');
  process.exit(0);
}

const jalan = dir => spawnSync(process.execPath, [SCRIPT, SMC2, '--out', dir], { encoding: 'utf8' });
const isi = dir => fs.readdirSync(dir).sort()
  .map(f => [f, fs.readFileSync(path.join(dir, f))]);

const a = fs.mkdtempSync(path.join(os.tmpdir(), 'bluro-a-'));
const b = fs.mkdtempSync(path.join(os.tmpdir(), 'bluro-b-'));
const ra = jalan(a), rb = jalan(b);

chk('jalan tanpa galat', ra.status === 0 && rb.status === 0, (ra.stderr || '').trim());

const fa = isi(a), fb = isi(b);
chk('berkas yang sama dihasilkan', fa.map(x => x[0]).join() === fb.map(x => x[0]).join(),
    fa.map(x => x[0]).join(' '));
chk('jalan dua kali -> byte identik',
    fa.every(([n, buf], i) => n === fb[i][0] && buf.equals(fb[i][1])),
    'kalau ini merah, ada urutan yang bergantung urutan entri ZIP');

// ST verbatim: CRLF milik Studio, tidak dirapikan, dan rumus intinya utuh.
const fkPath = path.join(a, 'FORWARD_KINEMATIC.st');
const ikPath = path.join(a, 'INVERSE_KINEMATIC.st');
chk('dua berkas ST ada', fs.existsSync(fkPath) && fs.existsSync(ikPath));

const fk = fs.readFileSync(fkPath, 'utf8'), ik = fs.readFileSync(ikPath, 'utf8');
chk('akhiran baris CRLF dipertahankan', fk.includes('\r\n') && ik.includes('\r\n'),
    'ST yang di-LF-kan bikin section .smc2 muncul dengan rung KOSONG - lihat CLAUDE.md');
chk('FK memuat rantai planar L2/L3/L4',
    /ROBOT_L2_LREAL\*COS/.test(fk) && /ROBOT_L4_LREAL\*COS/.test(fk));
chk('IK memuat hukum kosinus (BETA, GAMMA) dan ALFA',
    /BETA:=ACOS/.test(ik) && /GAMMA:=ACOS/.test(ik) && /ALFA:=ATAN/.test(ik));
chk('CACAT ikut terangkut apa adanya: ALFA pakai ATAN, bukan ATAN2',
    /ALFA:=ATAN\(Z3\/Y3\)/.test(ik), 'kalau ini merah, projectnya sudah dibetulkan - update ANALYSIS.md');
chk('CACAT ikut terangkut apa adanya: DONE:=TRUE di cabang ELSE',
    /ELSE\r?\n\s*DONE:=TRUE;/.test(ik));
chk('FK tidak menyentuh EXECUTE maupun DONE', !/\bEXECUTE\b/.test(fk) && !/\bDONE\b/.test(fk));

// Isi extract/ yang ter-commit harus sama dengan hasil ekstrak sekarang. Ini yang
// menangkap "project diganti, ekstraknya lupa dijalankan lagi".
//
// Yang dibandingkan cuma berkas yang DIBANGKITKAN. ANALYSIS.md tinggal di folder
// yang sama tapi ditulis tangan - dibandingkan juga, tesnya merah tiap kali ada yang
// menambah catatan, dan merah yang salah alasan bikin orang berhenti membacanya.
if (fs.existsSync(KELUAR)) {
  const kurang = [], beda = [];
  for (const [n, buf] of fa) {
    const p = path.join(KELUAR, n);
    if (!fs.existsSync(p)) kurang.push(n);
    else if (!fs.readFileSync(p).equals(buf)) beda.push(n);
  }
  chk('extract/ yang ter-commit masih segar', !kurang.length && !beda.length,
      (kurang.length ? 'belum ada: ' + kurang.join(' ') + '  ' : '')
      + (beda.length ? 'sudah basi: ' + beda.join(' ') + '  ' : '')
      + 'jalanin: node blurobot/tools/extract.js');
} else {
  console.log('  SKIP  blurobot/extract/ belum pernah dibangkitkan');
}

// variables.tsv: tanpa baris judul (Studio menempelkan judulnya jadi variabel
// bernama "Name" bertipe "Data type"), dan kolomnya urutan tabel Studio:
//   Name  Data type  Initial value  AT  Retain  Constant  Network Publish  Comment
// Urutan kolom sendiri bikin berkasnya KELIHATAN bisa ditempel padahal melenceng,
// dan Studio menerima tempelan yang melenceng tanpa satu pun keluhan.
const tsv = fs.readFileSync(path.join(a, 'variables.tsv'), 'utf8').split('\n').filter(Boolean);
chk('variables.tsv tanpa baris judul', !/^Nama\t/i.test(tsv[0]), tsv[0]);
chk('tiap baris 8 kolom = urutan tabel Studio', tsv.every(l => l.split('\t').length === 8));
chk('konstanta project ikut terbawa', tsv.some(l => l.startsWith('PI\tLREAL\t3.141592654\t')));
chk('kolom Constant terisi buat PI/DEGREE_TO_RAD/RAD_TO_DEGREE',
    ['PI', 'DEGREE_TO_RAD', 'RAD_TO_DEGREE'].every(n =>
      tsv.some(l => l.split('\t')[0] === n && l.split('\t')[5] === 'True')),
    'Studio menolak menulis ke variabel Constant - itu yang menjaga konstanta tidak ikut ditimpa init sim');
chk('dimensi robot memang KOSONG nilai awalnya',
    tsv.some(l => /^ROBOT_L2_LREAL\tLREAL\t\t/.test(l)),
    'diisi dari HMI di mesin asli - untuk sim datang dari robot.config.json');

fs.rmSync(a, { recursive: true, force: true });
fs.rmSync(b, { recursive: true, force: true });

console.log(fail ? 'GAGAL ' + fail : 'LULUS');
process.exit(fail ? 1 : 0);
