// Program simulasi (blurobot/sim/) - yang bisa diperiksa TANPA membuka Studio.
//
// Yang paling mahal di alur ini bukan rumus yang salah, tapi satu nama yang salah
// ketik: Studio baru mengeluh setelah project dibuat, FB ditempel, tabel variabel
// ditempel, dan Build dijalankan - satu putaran penuh untuk satu huruf. Jadi tes
// ini mengadu SETIAP nama yang disebut PRG_SIM_ROBOT.st ke dua tabel variabel yang
// ikut ditempel, plus memastikan blok init masih sama dengan robot.config.json.
'use strict';
const fs = require('fs');
const path = require('path');
const { spawnSync } = require('child_process');

const SIM = path.join(__dirname, '..', 'sim');
const ST = fs.readFileSync(path.join(SIM, 'PRG_SIM_ROBOT.st'), 'utf8');
const cfg = JSON.parse(fs.readFileSync(path.join(SIM, 'robot.config.json'), 'utf8'));

let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

// ------------------------------------------------- config masih jadi sumbernya
const r = spawnSync(process.execPath, [path.join(__dirname, '..', 'tools', 'gen_sim.js'), '--check'],
  { encoding: 'utf8' });
chk('sim/ sesuai robot.config.json', r.status === 0,
    ((r.stdout || '') + (r.stderr || '')).trim());

chk('SIM_DT = periode task config', ST.includes('SIM_DT := ' + (cfg.task.periode_ms / 1000) + ';'),
    'kalau ST dan config beda, kecepatan di layar bukan yang diminta');
chk('langkah jog = 10 seperti mesin', cfg.jog.langkah === 10,
    'JOG_WORLD_STEP di ladder asli tetap 10 karena cabang penumbuhnya digerbang DONE yang selalu TRUE');

// ------------------------------------------------------- tidak ada MC_ sama sekali
chk('tanpa instruksi MC_*', !/\bMC_[A-Za-z]/.test(ST),
    'project sim sengaja tanpa axis: tidak ada setting motion yang bisa salah');

// --------------------------------------------------- semua nama harus dideklarasi
// Kata kunci ST, konstanta bahasa, dan fungsi standar yang dipakai berkas ini.
// Daftar sengaja PENDEK dan eksplisit: daftar panjang yang menampung apa saja
// bikin tes ini berhenti menangkap salah ketik, yang justru satu-satunya tugasnya.
const KATA = new Set([
  'IF', 'THEN', 'ELSE', 'ELSIF', 'END_IF', 'FOR', 'TO', 'DO', 'END_FOR',
  'CASE', 'OF', 'END_CASE', 'AND', 'OR', 'NOT', 'TRUE', 'FALSE',
  'ABS', 'SQRT', 'SIN', 'COS', 'ATAN', 'ACOS', 'MOD',
  'REAL_TO_LREAL', 'LREAL_TO_REAL', 'UDINT_TO_INT', 'INT_TO_LREAL', 'LREAL_TO_INT'
]);

// Nama pin FB (dipakai sebagai IK2.DONE, FK2.ROBOT_POS_WORLD_OUTPUT, ...) diambil
// dari tabel variabel FB yang dibangkitkan, BUKAN didaftar tangan di sini. Daftar
// tangan di tes itu tempat drift berikutnya: pin FB berubah, tesnya tetap hijau.
for (const f of fs.readdirSync(SIM).filter(n => n.endsWith('.vars.tsv'))) {
  for (const line of fs.readFileSync(path.join(SIM, f), 'utf8').split('\n')) {
    const n = line.split('\t')[0];
    if (n) KATA.add(n);
  }
}

const kolom0 = f => fs.readFileSync(path.join(SIM, f), 'utf8').split('\n')
  .filter(Boolean).map(l => l.split('\t')[0]);
const glob = kolom0('GlobalVariables.tsv');
const lokal = kolom0('ProgramVariables.tsv');
const dideklarasi = new Set(glob.concat(lokal));

// Komentar dibuang dulu - komentar di berkas ini menyebut nama variabel yang
// memang tidak selalu ada (nama di project ASLI, misalnya JOG_WORLD_STEP).
const kode = ST.split('\n').map(l => l.replace(/\/\/.*$/, '')).join('\n');
const tidakAda = [...new Set(kode.match(/[A-Za-z_][A-Za-z0-9_]*/g) || [])]
  .filter(n => !KATA.has(n) && !dideklarasi.has(n));
chk('tiap nama di ST ada di salah satu tabel variabel', tidakAda.length === 0,
    tidakAda.length ? 'belum dideklarasi: ' + tidakAda.join(' ') : '');

// Kebalikannya: variabel yang dideklarasi tapi tidak pernah dipakai. Bukan galat -
// tapi baris yang ditempel ke Studio tanpa ada yang memakainya biasanya sisa dari
// rancangan yang berubah, dan Studio TIDAK akan memberitahu.
//
// External milik FB DIKECUALIKAN: yang menuntutnya FB, bukan program ini.
// RAD_TO_DEGREE contohnya - cuma dipakai di dalam INVERSE_KINEMATIC, dan kalau
// baris itu dibuang dari tabel global, yang gagal justru FB-nya.
const EXT_FB = new Set(fs.readFileSync(path.join(__dirname, '..', 'extract', 'variables.tsv'), 'utf8')
  .split('\n').filter(Boolean).map(l => l.split('\t')[0]));
const tidakDipakai = glob.concat(lokal)
  .filter(n => !EXT_FB.has(n) && !new RegExp('\\b' + n + '\\b').test(kode));
chk('tidak ada variabel yang dideklarasi tapi nganggur', tidakDipakai.length === 0,
    tidakDipakai.join(' '));

// ------------------------------------------------------------ bentuk tabelnya
const g = fs.readFileSync(path.join(SIM, 'GlobalVariables.tsv'), 'utf8').split('\n').filter(Boolean);
chk('GlobalVariables.tsv 8 kolom, tanpa judul',
    g.every(l => l.split('\t').length === 8) && !/^Name\t/i.test(g[0]));
chk('external kedua FB ikut di tabel global',
    ['PI', 'DEGREE_TO_RAD', 'RAD_TO_DEGREE', 'ROBOT_L2_LREAL', 'PD1300_000']
      .every(n => glob.includes(n)),
    'FB menuntutnya sebagai VAR_EXTERNAL - kurang satu, Build gagal di FB, bukan di program');

// FB instance harus bertipe FB-nya sendiri; salah tipe di sini = (DefinitionError)
// yang pesannya tidak menyebut sebabnya.
const p = fs.readFileSync(path.join(SIM, 'ProgramVariables.tsv'), 'utf8').split('\n').filter(Boolean);
chk('FK2/IK2 bertipe FB V2',
    p.some(l => l.startsWith('FK2\tFORWARD_KINEMATIC_V2\t'))
    && p.some(l => l.startsWith('IK2\tINVERSE_KINEMATIC_V2\t')));

// ----------------------------------------------------- FB V2 yang dipakai sim
const V2 = ['FORWARD_KINEMATIC_V2', 'INVERSE_KINEMATIC_V2'];
for (const nama of V2) {
  const stFile = path.join(SIM, nama + '.st');
  const varFile = path.join(SIM, nama + '.vars.tsv');
  chk(nama + ': ST + tabel variabelnya ada', fs.existsSync(stFile) && fs.existsSync(varFile));
  if (!fs.existsSync(stFile)) continue;
  const body = fs.readFileSync(stFile, 'utf8').split('\n').map(l => l.replace(/\/\/.*$/, '')).join('\n');

  // ATAN2 TIDAK ada di daftar 353 instruksi W560 (docs/SYSMAC_INSTRUCTIONS.md),
  // jadi belum terbukti ter-import. Kuadran dibetulkan lewat ATAN + koreksi.
  chk(nama + ': tanpa ATAN2 (bukan instruksi yang terbukti)', !/\bATAN2\b/.test(body),
      'kalau perlu, buktikan dulu lewat probe import - bukan dengan menganggapnya ada');
  chk(nama + ': tidak menulis balik ke variabel global tool',
      !/ROBOT_TOOL_[YZ]_LREAL\s*:=/.test(body),
      'V1 menimpa globalnya sendiri waktu toolY=0, dan efeknya permanen');
  chk(nama + ': DONE benar-benar ditulis', /\bDONE\s*:=/.test(body));
  chk(nama + ': EXECUTE benar-benar dibaca', /IF\s+EXECUTE\s+THEN/.test(body));
}

// ------------------------------------- array milik instance FB tidak boleh diindeks
// DITOLAK STUDIO, sudah kejadian di Build (bukan di import):
//   "Cannot use an element of array or a member of structure for the reference of
//    function block instance variables."
// `IK2.ROBOT_POS_OUTPUT[i]` kena; `IK2.DONE` tidak - yang dilarang MENGINDEKS anggota
// milik instance. Jalan keluarnya menyalin arraynya UTUH ke variabel lokal dulu.
//
// Nama instance-nya dibaca dari tabel variabel program, bukan didaftar tangan: daftar
// tangan berhenti menangkap begitu ada instance FB baru.
const instFb = p.map(l => l.split('\t')).filter(c => /_V2$/.test(c[1])).map(c => c[0]);
chk('instance FB terbaca dari tabel program (' + instFb.join(' ') + ')', instFb.length >= 2);
const indeksInstance = instFb.filter(n => new RegExp('\\b' + n + '\\.\\w+\\s*\\[').test(kode));
chk('tidak ada array milik instance FB yang diindeks langsung', indeksInstance.length === 0,
    indeksInstance.length ? indeksInstance.join(' ') + ' - salin arraynya utuh dulu ke variabel lokal' : '');
chk('salinan array keluaran FB ada di variabel lokal',
    /IK_OUT\s*:=\s*IK2\.ROBOT_POS_OUTPUT;/.test(kode)
    && /FK_WORLD\s*:=\s*FK2\.ROBOT_POS_WORLD_OUTPUT;/.test(kode));

// Nama POU tidak boleh diawali `P_`: itu awalan variabel sistem Sysmac (P_On,
// P_First_Run). Program bernama P_SIM_ROBOT DINAMAI ULANG SENDIRI oleh Studio jadi
// PR_SIM_ROBOT waktu import - tanpa pesan, dan sesudah itu penugasan task serta tiap
// dokumen menunjuk POU yang tidak ada.
const namaPou = ['PRG_SIM_ROBOT'].concat(V2);
chk('nama POU tidak diawali P_ / angka / garis bawah',
    namaPou.every(n => !/^(P_|_|\d)/.test(n)), namaPou.join(' '));

// V1 tetap ada sebagai catatan, tapi TIDAK di-instance: instance yang tidak dipakai
// bikin orang mengira dua-duanya ikut menentukan gerakan.
chk('FB V1 tidak di-instance di program sim',
    !p.some(l => /\t(FORWARD|INVERSE)_KINEMATIC\t/.test(l)));
chk('kuadran IK dibetulkan (ada cabang Y3 < 0)',
    /ELSIF\s+Y3\s*<\s*0\.0\s+THEN/.test(fs.readFileSync(path.join(SIM, 'INVERSE_KINEMATIC_V2.st'), 'utf8')),
    'tanpa cabang itu, separuh ruang kerja meleset 180 derajat');
chk('jangkauan diperiksa SEBELUM ACOS dipanggil', (() => {
  // Komentar dibuang dulu: kepala berkas menyebut ACOS jauh di atas kodenya, dan
  // membandingkan posisi di teks mentah bikin tes ini menjawab pertanyaan lain.
  const b = fs.readFileSync(path.join(SIM, 'INVERSE_KINEMATIC_V2.st'), 'utf8')
    .split('\n').map(l => l.replace(/\/\/.*$/, '')).join('\n');
  return b.indexOf('ERROR_ID := 1') < b.indexOf('ACOS');
})(), 'memeriksa sesudah ACOS tidak menolong - yang meledak ACOS-nya');

// --------------------------------------------------------------- gripper
chk('tag gripper ada di tabel global',
    ['SIM_GRIP_CMD', 'SIM_GRIP_POS', 'SIM_GRIP_STROKE', 'SIM_GRIP_LEN'].every(n => glob.includes(n)));
chk('panjang gripper dijumlahkan ke tool TEPAT SEKALI',
    (kode.match(/ROBOT_TOOL_Y_LREAL\s*:=/g) || []).length === 1
    && kode.includes('ROBOT_TOOL_Y_LREAL := ' + (cfg.tool.Y + cfg.gripper.panjang)),
    'dijumlahkan dua kali, lengannya panjang dua kali gripper - dan itu tidak kelihatan salah di layar');
chk('gripper punya motion model sendiri', /SIM_GRIP_POS\s*:=\s*SIM_GRIP_POS\s*[+-]/.test(kode));

// Konstanta project TIDAK boleh ikut ditulis init sim: di tabel mereka Constant,
// dan Studio menolak assignment ke variabel Constant waktu Build.
chk('init sim tidak menulis ke PI/DEGREE_TO_RAD/RAD_TO_DEGREE',
    !/\b(PI|DEGREE_TO_RAD|RAD_TO_DEGREE)\s*:=/.test(kode));

// ------------------------------------------------ selector, emergency, home
// Yang menghentikan robot bukan POSISI selector tapi PERUBAHANNYA: kalau dipakai
// posisinya, robot tidak akan pernah bisa dijalankan dari selector yang memang sudah
// di AUTO sejak awal - dan gejalanya "tombol Autorun tidak berfungsi".
chk('selector dinilai dari perubahannya, bukan posisinya',
    /SEL_UBAH\s*:=\s*SIM_SEL_AUTO\s*<>\s*LAST_SEL/.test(kode));
chk('emergency dan selector-saat-jalan sama-sama membatalkan',
    /IF\s+SIM_ESTOP\s+THEN[\s\S]{0,200}?ELSIF\s+SEL_UBAH\s+AND\s+SIM_AUTO\s+THEN/.test(kode));
chk('berhenti total memadamkan SIM_HOMED', /IF ABORT THEN[\s\S]{0,400}?SIM_HOMED := FALSE;/.test(kode),
    'tanpa itu robot melanjutkan dari pose yang tidak pernah direncanakan sekuenser');
chk('berhenti total menyamakan perintah dengan posisi',
    /IF ABORT THEN[\s\S]{0,400}?SIM_JOINT_CMD\[i\] := SIM_JOINT_POS\[i\];/.test(kode),
    'perintah lama yang dibiarkan bikin lengan lanjut jalan sesudah emergency dilepas');
chk('Autorun digerbang selector + sudah home + tidak emergency',
    /EDGE_RUN AND SIM_SEL_AUTO AND SIM_HOMED AND NOT SIM_ESTOP/.test(kode),
    'syarat yang ditegakkan di halaman tidak ikut waktu tombolnya ditekan dari tempat lain');
chk('tabrakan membatalkan cuma di TEPI-nya',
    /SIM_COLLIDE AND NOT LAST_COLLIDE/.test(kode),
    'menahan selama masih menempel bikin lengan terkunci di dalam benda yang ditabraknya');

// Cycle stop BUKAN emergency: pekerjaan diselesaikan dulu, dan SIM_HOMED tidak
// dipadamkan - itu yang bikin Autorun bisa langsung dipakai lagi sesudahnya.
chk('cycle stop dilayani di AKHIR pekerjaan, bukan seketika', (() => {
  const iStop = kode.indexOf('EDGE_CSTOP');
  const iLayani = kode.lastIndexOf('IF SIM_STOP_REQ THEN');
  return iStop > 0 && iLayani > iStop && /31:[\s\S]{0,400}?IF SIM_STOP_REQ THEN/.test(kode);
})());
chk('cycle stop tidak memadamkan SIM_HOMED',
    !/SIM_STOP_REQ THEN[\s\S]{0,200}?SIM_HOMED := FALSE/.test(kode));

// SIM_MOVE_DONE dihitung motion model di AKHIR scan, jadi waktu Home baru saja
// diperintahkan isinya masih jawaban scan sebelumnya - TRUE, karena robot memang
// sedang berhenti. Tanpa dipadamkan, home dinyatakan SELESAI pada scan yang sama:
// SIM_HOMED menyala tanpa lengannya bergerak, dan syarat yang mau ditegakkan hilang
// tanpa satu pun tanda. Sekuenser tidak kena karena perintah dan penungguannya ada
// di langkah - dan scan - yang berbeda.
chk('home tidak bisa dinyatakan selesai pada scan yang sama',
    /HOMING := TRUE;[\s\S]{0,700}?SIM_MOVE_DONE := FALSE;[\s\S]{0,80}?END_IF;[\s\S]{0,40}?IF HOMING AND SIM_MOVE_DONE THEN/.test(kode));
// Produk yang sedang dijepit TETAP dipegang lewat emergency, cycle stop, selector,
// dan Home. Gripper yang membuka sendiri waktu pulih berarti barang jatuh ke lantai
// tiap kali orang menekan emergency - di mesin aslinya itu barang sungguhan.
chk('home TIDAK melepas produk yang sedang dijepit',
    !/IF HOMING AND SIM_MOVE_DONE THEN[\s\S]{0,300}?SIM_PART_STATE := 0;/.test(kode));
chk('berhenti total tidak menghapus ingatan pekerjaan',
    !/IF ABORT THEN[\s\S]{0,500}?SIM_JOB_DST := -1;/.test(kode),
    'ingatan yang dihapus = produk di gripper tidak akan pernah sampai ke tujuannya');
chk('Autorun MELANJUTKAN pengantaran kalau masih memegang produk',
    /IF \(SIM_PART_STATE = 1\) AND \(SIM_JOB_DST >= 0\) THEN[\s\S]{0,700}?SIM_CYCLE_STEP := 18;[\s\S]{0,60}?ELSE[\s\S]{0,40}?SIM_CYCLE_STEP := 0;/.test(kode),
    'mulai dari nol = robot berangkat mengambil produk kedua sambil tangannya masih penuh');

// Gripper dibuka MANUAL sambil memegang: produknya jatuh, dan pekerjaannya ikut
// hilang. Ingatan yang ditinggal bikin Autorun berikutnya mengantar produk yang tidak
// ada - stasiun tercatat berisi, dan salahnya baru ketahuan waktu produk hantu itu
// diambil belasan menit kemudian.
chk('produk jatuh kalau gripper dibuka manual sambil memegang',
    /IF \(SIM_PART_STATE = 1\) AND SIM_GRIP_OPEN AND NOT SIM_AUTO THEN[\s\S]{0,200}?SIM_PART_STATE := 0;/.test(kode));
chk('produk jatuh ikut menghapus ingatan pekerjaan',
    /SIM_GRIP_OPEN AND NOT SIM_AUTO THEN[\s\S]{0,300}?SIM_JOB_SRC := -1;[\s\S]{0,60}?SIM_JOB_DST := -1;/.test(kode));
chk('produk yang jatuh DIHITUNG',
    /SIM_DROP_COUNT := SIM_DROP_COUNT \+ 1;/.test(kode) && glob.includes('SIM_DROP_COUNT'),
    'produk yang hilang tanpa angka bikin jumlah masuk dan keluar tidak bisa diadu');
chk('yang jatuh cuma di MANUAL, bukan waktu sekuenser meletakkan',
    /SIM_GRIP_OPEN AND NOT SIM_AUTO/.test(kode));

// Home cuma melipat LENGAN. Rel tidak ikut pulang: memaksanya ke satu titik berarti
// tiap pemulihan menyeret lengan melewati mesin yang tidak ada urusannya, dan langkah
// pertama tiap pekerjaan toh menggeser rel sendiri.
chk('home tidak menyentuh sumbu 0 (rel bebas di mana saja)',
    /IF EDGE_HOME[\s\S]{0,700}?FOR i := 1 TO 3 DO[\s\S]{0,120}?SIM_JOINT_CMD\[i\] := SIM_HOME\[i\];/.test(kode)
    && !/IF EDGE_HOME[\s\S]{0,700}?FOR i := 0 TO 3 DO[\s\S]{0,120}?SIM_JOINT_CMD\[i\] := SIM_HOME\[i\];/.test(kode));
chk('pose home menggantung tegak seperti robot aslinya',
    cfg.home.sumbu[1] + cfg.home.sumbu[2] + cfg.home.sumbu[3] === -90,
    'theta_EE home = ' + (cfg.home.sumbu[1] + cfg.home.sumbu[2] + cfg.home.sumbu[3])
    + '; kalau bukan -90 gripper mendatar dan tiap turun ke stasiun harus memutar dulu');

// --------------------------------------------------- rel cuma dilewati terlipat
// Lengan yang bergeser sambil menjulur ke bawah menyapu tiap mesin yang dilewatinya.
// Yang menjaganya urutan langkah: lipat DULU (sumbu 1-3 ke pose home), tunggu selesai,
// baru sumbu 0 digerakkan.
chk('rel digerakkan SESUDAH lengan dilipat ke pose jalan', (() => {
  const lipat = kode.indexOf('SIM_JOINT_CMD[1] := SIM_HOME[1];');
  const rel = kode.indexOf('SIM_JOINT_CMD[0] := SIM_ST_X[ST_IDX];');
  const tunggu = kode.indexOf('SIM_MOVE_DONE', lipat);
  return lipat > 0 && rel > lipat && tunggu > lipat && tunggu < rel;
})(), 'kalau rel duluan - atau tanpa menunggu lipatannya selesai - lengan menyapu mesin, dan di layar itu mulus');
chk('sumbu 0 tidak ikut disetel waktu melipat',
    !/SIM_JOINT_CMD\[1\] := SIM_HOME\[1\];[\s\S]{0,80}?SIM_JOINT_CMD\[0\] :=/.test(kode),
    'melipat sambil menggeser rel = lengan menjulur waktu bergerak, persis yang mau dihindari');

// ------------------------------------------------------- buffer ICC dulu
// URUTAN tiga pencarian ini yang menentukan perilaku selnya, dan urutannya bukan
// selera:
//   ICC kosong dulu   -> mesin tes 17 detik tidak menganggur menunggu robot
//   DW selesai kedua  -> selalu ada DW kosong buat produk ICC berikutnya
//   ICC selesai -> DW ketiga
// Dibalik (ICC->DW didahulukan dari DW->WIP OUT), dua DW penuh + dua ICC selesai
// bikin sel MACET, dan macetnya cuma terlihat sebagai robot yang diam.
chk('penjadwal: isi ICC -> kosongkan DW -> pindah ICC ke DW', (() => {
  const isiIcc = kode.indexOf('(SIM_ST_TIPE[i] = 1) AND (SIM_ST_STATE[i] = 0)');
  const dwKeluar = kode.indexOf('(SIM_ST_TIPE[i] = 2) AND (SIM_ST_STATE[i] = 2)');
  const iccKeDw = kode.indexOf('(SIM_ST_TIPE[i] = 1) AND (SIM_ST_STATE[i] = 2)');
  return isiIcc > 0 && dwKeluar > isiIcc && iccKeDw > dwKeluar;
})());
chk('tiap mesin punya penghitung waktunya sendiri',
    /FOR i := 0 TO SIM_ST_N - 1 DO[\s\S]{0,600}?SIM_ST_TIMER\[i\] := SIM_ST_TIMER\[i\] - SIM_DT;/.test(kode),
    'satu penghitung bersama = satu produk di seluruh sel, dan buffer jadi tidak ada artinya');
chk('WIP IN selalu berisi, WIP OUT selalu kosong',
    /SIM_ST_TIPE\[i\] = 0 THEN[\s\S]{0,120}?SIM_ST_STATE\[i\] := 2;/.test(kode)
    && /SIM_ST_TIPE\[i\] = 3 THEN[\s\S]{0,120}?SIM_ST_STATE\[i\] := 0;/.test(kode));
chk('cycle time dari config: ICC 17 s, DW 15 s',
    cfg.siklus.stasiun.filter(s => s.tipe === 1).every(s => s.proses === 17)
    && cfg.siklus.stasiun.filter(s => s.tipe === 2).every(s => s.proses === 15));

// ----------------------------------------------------------------- tabrakan
chk('pose yang diminta diperiksa SEBELUM IK dijalankan', (() => {
  return kode.indexOf('CAND_HIT := TRUE') < kode.indexOf('IF NEED_IK THEN');
})(), 'memeriksa sesudah IK berarti sumbu sudah diberi perintah menembus mesin');
chk('perintah yang menabrak dibuang, bukan cuma ditandai',
    /IF CAND_HIT THEN[\s\S]{0,200}?NEED_IK := FALSE;/.test(kode));
chk('kotak tabrakan dari tag yang sama dengan yang digambar',
    /CK_MX := SIM_ST_W \/ 2\.0 \+ SIM_ST_MARGIN/.test(kode) && /CK_MY := SIM_ST_D/.test(kode),
    'kotak kedua buat penjaga = gripper berhenti di udara atau menembus kotak yang digambar');
chk('jari ikut dihitung, bukan cuma TCP', /SIM_GRIP_POS \/ 2\.0 \+ SIM_GRIP_JARI/.test(kode),
    'yang menabrak duluan jari yang menjulur ke samping, bukan titik TCP-nya');
chk('permukaan stasiun bukan tabrakan', /CK_Z\[k\] < SIM_ST_Z\[i\] - 2\.0/.test(kode),
    'tanpa selisih itu, tiap penempatan produk memicu alarm tabrakan');
chk('lantai ikut dijaga', /IF CK_Z\[k\] < 0\.0 THEN/.test(kode));

// --------------------------------------------------------- gerakan halus
// Berangkat dan berhenti pada kecepatan penuh dalam satu scan terlihat patah-patah
// begitu angkanya lewat jaringan - dan yang ditanyakan jadi soal jaringan, padahal
// sebabnya profil gerakan.
chk('motion model pakai profil trapesium (akselerasi + jarak rem)',
    /VB := SQRT\(2\.0 \* SIM_ACC\[i\] \* ABS\(D\)\)/.test(kode)
    && /DV := SIM_ACC\[i\] \* SIM_DT/.test(kode));
chk('kecepatan sumbu disimpan antar scan', /SIM_JOINT_VEL\[i\] :=/.test(kode));
chk('halaman menghaluskan GAMBAR, bukan angkanya', (() => {
  const r = fs.readFileSync(path.join(__dirname, '..', 'web', 'robot.js'), 'utf8');
  return /function haluskan\(dt\)/.test(r) && /el\('j' \+ i\)\.textContent = f2\(st\.jointPlc\[i\]\)/.test(r);
})(), 'panel yang ikut dihaluskan menghapus satu-satunya tempat membandingkan layar dengan simulator');

// ------------------------------------------------------ pose world: REAL dan LREAL
// FB mengeluarkan dua bentuk pose yang sama. Yang REAL ada supaya bentuknya identik
// dengan FB di mesin; yang LREAL yang dipakai MENGHITUNG. REAL cuma ~7 angka berarti,
// dan jog world membangun pose berikutnya DARI pose sekarang - pembulatan di jalur
// umpan balik menumpuk tiap tekan.
chk('pose world diterbitkan dalam dua bentuk',
    glob.includes('SIM_WORLD_POS') && glob.includes('SIM_WORLD_POS_L'));
chk('yang LREAL diambil dari pin WORLD_LREAL milik FB, bukan dikonversi balik',
    /FK_WORLD_L := FK2\.WORLD_LREAL;/.test(kode)
    && !/SIM_WORLD_POS_L\[i\] := REAL_TO_LREAL/.test(kode),
    'mengonversi balik dari REAL tidak mengembalikan angka yang sudah hilang');
chk('jog world membangun pose dari yang LREAL',
    /POSE_REQ\[i\] := SIM_WORLD_POS_L\[i\];/.test(kode)
    && !/POSE_REQ\[i\] := REAL_TO_LREAL\(SIM_WORLD_POS\[i\]\)/.test(kode));
chk('penjaga tabrakan menilai pose LREAL', /CK_X\[2\] := SIM_WORLD_POS_L\[0\];/.test(kode));
chk('halaman memakai LREAL kalau ada, REAL kalau tidak', (() => {
  const r = fs.readFileSync(path.join(__dirname, '..', 'web', 'robot.js'), 'utf8');
  return /if \(v\.SIM_WORLD_POS_L\)[\s\S]{0,120}?else if \(v\[TAG\.world\]\)/.test(r);
})(), 'jatuh balik ke REAL bikin halaman tetap hidup di project lama yang belum punya tag itu');

// ------------------------------------------------- gerak lurus (Cartesian) di PLC
// Dua cara sampai ke titik yang sama, dua-duanya JALAN di PLC. Yang diuji di sini
// bukan bahwa kodenya ada, tapi aturan yang bikin perbandingannya berarti.
chk('dua mode perpindahan', glob.includes('SIM_MOVE_MODE') && glob.includes('SIM_LINE_ACTIVE'));
chk('gerak lurus menghitung IK TIAP SCAN, bukan sekali',
    /ELSIF SIM_LINE_ACTIVE THEN[\s\S]{0,1800}?POSE_REQ\[0\] := LN_A\[0\] \+ LN_UX \* SIM_LINE_S;[\s\S]{0,400}?NEED_IK := TRUE;/.test(kode),
    'itu bedanya dengan gerak sumbu: satu IK vs satu IK per scan');
chk('sudut end effector ikut diinterpolasi, bukan dilompati di akhir',
    /POSE_REQ\[3\] := LN_A\[3\] \+ \(LN_B\[3\] - LN_A\[3\]\)/.test(kode),
    'tool yang berputar mendadak di titik terakhir itu justru gerakan yang paling gampang menabrak');

// IK menolak di tengah garis = BERHENTI. Titik di garis yang terus maju sementara
// lengannya tertinggal berarti "gerak lurus" tidak lurus lagi, dan tidak ada yang tahu.
chk('gerak lurus berhenti kalau IK menolak satu titik',
    /IF SIM_LINE_ACTIVE THEN[\s\S]{0,160}?SIM_LINE_ACTIVE := FALSE;[\s\S]{0,60}?SIM_LINE_ABORT := 1;/.test(kode));
chk('gerak lurus dibatalkan perintah lain (jog, home, emergency, auto)',
    /IF SIM_LINE_ACTIVE AND \(\(AX >= 0\) OR EDGE_HOME OR SIM_ESTOP OR SIM_AUTO\) THEN[\s\S]{0,120}?SIM_LINE_ABORT := 2;/.test(kode));
chk('kecepatan garis ikut override kecepatan', /VT := SIM_LINE_VEL \* OVR;/.test(kode));

// Siklus otomatis TETAP gerak sumbu. Kalau mode ikut mengubah siklus, angka cycle time
// yang sudah dikumpulkan berubah artinya tanpa ada yang mengubah sekuensnya.
chk('mode perpindahan TIDAK menyentuh siklus otomatis', (() => {
  // Yang dipotong SEKUENSERNYA (CASE), bukan "dari IF SIM_AUTO pertama": kata itu
  // muncul juga di pembatal gerak lurus dan di penghitung cycle time, dan potongan yang
  // salah bikin tes ini menjawab pertanyaan yang lain.
  const auto = kode.slice(kode.indexOf('CASE SIM_CYCLE_STEP OF'),
                          kode.indexOf('ELSIF (AX >= 0)'));
  return !/SIM_MOVE_MODE|SIM_LINE_ACTIVE/.test(auto);
})(), 'siklus ikut berubah mode = angka cycle time yang sudah dikumpulkan berubah artinya');

// ---- pengukurannya, dan ini bagian yang bikin angkanya bisa dipercaya
chk('simpangan diukur dari pose SEKARANG terhadap garis awal-tujuan',
    /DX := SIM_WORLD_POS_L\[0\] - LN_A\[0\];/.test(kode)
    && /PROY := DX \* LN_UX \+ DY \* LN_UY \+ DZ \* LN_UZ;/.test(kode),
    'diukur dari pose yang DIPERINTAH, angkanya selalu nol - yang diukur perintahnya sendiri');
chk('pengukuran berlaku untuk KEDUA mode', (() => {
  // UKUR dinyalakan di blok EDGE_MOVE, sebelum percabangan mode - jadi gerak sumbu dan
  // gerak lurus diukur alat yang sama. Diukur alat yang beda, angkanya tidak bisa diadu.
  const i = kode.indexOf('UKUR := TRUE;');
  const j = kode.indexOf('IF (SIM_MOVE_MODE = 1)');
  return i > 0 && j > i;
})());
chk('pengukuran berhenti sesudah sumbu benar-benar diam',
    /IF SIM_MOVE_DONE AND NOT SIM_LINE_ACTIVE THEN[\s\S]{0,60}?UKUR := FALSE;/.test(kode),
    'berhenti waktu garisnya padam saja = ekor gerakan tidak ikut terukur');
chk('kurva simpangan direkam PLC, 50 titik',
    glob.includes('SIM_DEV_TRACE')
    && /SIM_DEV_TRACE\[TR_IDX\] := SIM_DEV_NOW;/.test(kode));
chk('kurva diindeks menurut KEMAJUAN, bukan waktu',
    /TR_IDX := LREAL_TO_INT\(PROY \/ SIM_LINE_LEN \* 49\.0\)/.test(kode),
    'diindeks waktu, dua mode yang lamanya beda tidak bisa ditumpuk di sumbu yang sama');
chk('kurva dikosongkan waktu perpindahan MULAI',
    /UKUR := TRUE;[\s\S]{0,400}?FOR i := 0 TO 49 DO[\s\S]{0,80}?SIM_DEV_TRACE\[i\] := 0\.0;/.test(kode),
    'dikosongkan waktu selesai, kurva lama tertinggal di layar selama gerakan berikutnya');
chk('indeks kurva dijepit 0..49', /IF TR_IDX < 0 THEN[\s\S]{0,120}?IF TR_IDX > 49 THEN/.test(kode));

// ---------------------------------------------------------- cycle time
// Diukur KELUAR ke KELUAR: itu yang menentukan berapa produk per jam. Diukur di tempat
// lain (mulai ambil, mulai antar) angkanya lebih kecil dan lebih enak dilihat, tapi
// menjawab pertanyaan yang lain.
chk('cycle time diambil waktu produk keluar di WIP OUT',
    /IF SIM_ST_TIPE\[ST_IDX\] = 3 THEN[\s\S]{0,900}?SIM_CT_LAST := SIM_CT_RUN;/.test(kode));
chk('produk PERTAMA tidak dihitung (tidak punya produk sebelumnya)',
    /IF CT_ADA THEN[\s\S]{0,900}?CT_ADA := TRUE;/.test(kode),
    'menghitungnya berarti waktu mengisi sel kosong ikut jadi cycle time');
chk('penghitung jalan HANYA selama siklus jalan',
    /IF SIM_AUTO THEN[\s\S]{0,120}?SIM_CT_RUN := SIM_CT_RUN \+ SIM_DT;/.test(kode),
    'ikut menghitung waktu berhenti = satu jeda menelan rata-rata sepuluh produk');
chk('rata-rata dibagi JUMLAH SAMPEL, bukan selalu 10',
    /CT_SUM \/ INT_TO_LREAL\(SIM_CT_N\)/.test(kode)
    && /FOR i := 0 TO SIM_CT_N - 1 DO/.test(kode),
    'dibagi 10 sejak awal bikin menit-menit pertama terbaca jauh lebih baik dari kenyataan');
chk('buffer melingkar sepuluh slot',
    /IF CT_IDX > 9 THEN[\s\S]{0,60}?CT_IDX := 0;/.test(kode)
    && p.some(l => l.startsWith('CT_BUF\tARRAY[0..9] OF LREAL')));
chk('buffer dikosongkan waktu init - variabel program tidak ikut blok init',
    /FOR i := 0 TO 9 DO[\s\S]{0,80}?CT_BUF\[i\] := 0\.0;/.test(kode)
    && /CT_ADA := FALSE;/.test(kode));
chk('tag cycle time dipublikasikan',
    ['SIM_CT_RUN', 'SIM_CT_LAST', 'SIM_CT_AVG10', 'SIM_CT_N'].every(n => glob.includes(n)));

// ------------------------------------------------------------ penutup mesin
// Penutup berengsel yang menekan PCB ke probe base. Tiga aturannya, dan ketiganya
// gagal tanpa keluhan kalau dilanggar.
chk('penutup menutup HANYA selama memproses',
    /IF \(SIM_ST_STATE\[i\] = 1\) AND \(SIM_ST_TIPE\[i\] <> 0\) AND \(SIM_ST_TIPE\[i\] <> 3\)[\s\S]{0,120}?COVER_TARGET := 0\.0;/.test(kode),
    'penutup yang menutup di luar proses menutup jalan masuk gripper');

// Interlock. Tanpa ini penutup mulai menutup begitu produk diletakkan, sementara
// gripper masih naik lewat ruang yang sama - daun penutup mengayun MENIMPA lengan, dan
// di layar dua benda saling menembus tanpa satu pun yang mengeluh.
chk('penutup tidak menutup selama ada bagian robot di ruangnya',
    /AND NOT SIM_ST_ZONA\[i\] THEN[\s\S]{0,60}?COVER_TARGET := 0\.0;/.test(kode));
chk('zona dihitung dari titik lengan yang SEKARANG, bukan dari langkah sekuenser',
    /IF \(k >= 2\) AND ZONA_HIT THEN[\s\S]{0,120}?SIM_ST_ZONA\[i\] := TRUE;/.test(kode),
    'menebak dari nomor langkah berarti jog dan gerakan tangan tidak ikut terlindungi');
chk('zona dibersihkan tiap scan sebelum dihitung ulang',
    /FOR i := 0 TO SIM_ST_N - 1 DO[\s\S]{0,80}?SIM_ST_ZONA\[i\] := FALSE;[\s\S]{0,40}?END_FOR;/.test(kode),
    'bit yang tidak pernah dipadamkan mengunci penutup terbuka selamanya');

// Penutup jadi BADAN TABRAKAN, tapi cuma waktu belum terbuka penuh: terbuka penuh
// daunnya berdiri di belakang engsel, dan menghitungnya tetap menutup jalan berarti
// robot tidak akan pernah bisa turun ke stasiun mana pun.
chk('penutup ikut dihitung penjaga tabrakan',
    /ZONA_HIT AND \(SIM_ST_COVER\[i\] < SIM_COVER_SUDUT - 0\.5\)/.test(kode));
chk('ruang sapuan penutup setinggi daunnya',
    /CK_Z\[k\] < SIM_ST_Z\[i\] \+ SIM_COVER_JANGKAU/.test(kode)
    && glob.includes('SIM_COVER_JANGKAU'));
chk('waktu proses cuma jalan waktu penutupnya RAPAT',
    /IF SIM_ST_COVER\[i\] <= 0\.5 THEN[\s\S]{0,200}?SIM_ST_TIMER\[i\] := SIM_ST_TIMER\[i\] - SIM_DT;/.test(kode),
    'menghitung sebelum rapat = mesin mengaku menguji papan yang belum tersentuh probe');
chk('robot turun HANYA sesudah penutup terbuka penuh',
    (kode.match(/SIM_MOVE_DONE AND \(SIM_ST_COVER\[ST_IDX\] >= SIM_COVER_SUDUT - 0\.5\)/g) || []).length === 2,
    'dua tempat: waktu mengambil dan waktu meletakkan');
chk('WIP tidak punya penutup', /SIM_ST_TIPE\[i\] <> 0\) AND \(SIM_ST_TIPE\[i\] <> 3/.test(kode));
chk('penutup ada di tabel global + dipublikasikan',
    glob.includes('SIM_ST_COVER') && glob.includes('SIM_COVER_SUDUT'));

// ------------------------------------------------------------ slider jog
// Slider menyetel sudut SUMBU langsung, tidak lewat IK: memaksanya lewat IK berarti
// pose yang tidak punya solusi jadi tidak bisa dituju padahal sumbunya sanggup.
chk('slider jog menyetel target sumbu lewat tepi SIM_JOG_SET_EXEC',
    /EDGE_JSET AND NOT SIM_ESTOP AND NOT SIM_SEL_AUTO THEN[\s\S]{0,300}?SIM_JOINT_CMD\[i\] := SIM_JOG_SET\[i\];/.test(kode));
chk('slider jog mati waktu AUTO dan waktu emergency', /EDGE_JSET AND NOT SIM_ESTOP AND NOT SIM_SEL_AUTO/.test(kode));

// --------------------------------------------------------- override kecepatan
// Dijepit DI PLC. Nilainya boleh ditulis dari mana saja lewat OPC UA, dan 500 % atau
// angka negatif yang lolos bikin sumbu melompati targetnya tiap scan - di layar itu
// terlihat seperti lengan yang berkedip, bukan seperti setelan yang salah.
chk('override dijepit 1..100 di PLC, bukan di halaman',
    /IF SIM_SPEED_OVR > 100\.0 THEN SIM_SPEED_OVR := 100\.0; END_IF;/.test(kode)
    && /IF SIM_SPEED_OVR < 1\.0 THEN SIM_SPEED_OVR := 1\.0; END_IF;/.test(kode));
chk('override dipakai motion model', /VT := SIM_VEL\[i\] \* OVR;/.test(kode));
chk('override ikut ke jog', /SIM_VEL\[i\] \* OVR \* SIM_DT/.test(kode),
    'jog yang tidak ikut override bikin satu tombol tetap kencang waktu semua diperlambat');
// Override yang ikut mengubah akselerasi memendekkan DAN memanjangkan jarak pengereman
// sekaligus - pelan-pelan berhenti jadi lebih aman, padahal itu satu-satunya alasan
// orang menurunkannya.
chk('akselerasi TIDAK ikut diskalakan', !/SIM_ACC\[i\] \* OVR/.test(kode));
chk('gripper TIDAK ikut diskalakan', !/SIM_GRIP_VEL \* OVR/.test(kode),
    'jarinya pneumatik di mesin asli - kecepatannya tidak bisa disetel controller');
chk('override ada di tabel global dan nilai awalnya dari config',
    glob.includes('SIM_SPEED_OVR')
    && kode.includes('SIM_SPEED_OVR := ' + cfg.jog.override_persen.toFixed(1)));

// ------------------------------------------------------------------ gripper
chk('gripper menutup ke lebar produk, bukan ke nol',
    /IF SIM_GRIP_CMD THEN GRIP_TARGET := SIM_GRIP_TUTUP;/.test(kode),
    'jari yang bertemu di 0 menembus barang yang sedang dipegangnya');
chk('halaman menulis TOMBOL, bukan SIM_AUTO langsung', (() => {
  const r = fs.readFileSync(path.join(__dirname, '..', 'web', 'robot.js'), 'utf8');
  return /pulsa\('SIM_AUTORUN'\)/.test(r) && !/kirim\('SIM_AUTO',/.test(r);
})(), 'halaman yang menyalakan SIM_AUTO menegakkan syaratnya di browser - tempat yang tidak dijalankan simulator');

console.log(fail ? 'GAGAL ' + fail : 'LULUS');
process.exit(fail ? 1 : 0);
