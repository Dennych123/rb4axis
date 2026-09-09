# Blurobot — kinematik dari project mesin → simulator NX → viz 3D

Mengambil **algoritma kinematik** dari project robot 4 sumbu
`BLUEROBOT ECU 28032020.smc2` (NJ501, 1219 rung), menjalankannya di simulator
Sysmac Studio, dan menggambarnya 3D di browser lewat OPC UA.

Yang diangkat cuma kinematik + jog + move point + gripper. Interpreter program teach
(`P100`–`P102`) dan sekuens station (`RB_MOVING`, `WIP_MOVING`) **tidak** — itu
keputusan, bukan kelupaan. Latarnya di
[`extract/ANALYSIS.md`](extract/ANALYSIS.md).

**Dua versi algoritma, berdampingan.** `extract/*.st` verbatim dari mesin, cacatnya
utuh, itu catatannya. `sim/*_V2.st` yang sudah dibetulkan, itu yang dijalankan
simulator. Tesnya berpasangan: satu menuntut perilaku yang salah, satu menuntut yang
benar, dan satu lagi membuktikan keduanya memang beda di pose yang sama.

| | V1 (mesin, `extract/`) | V2 (sim, `sim/`) |
|---|---|---|
| `DONE` | selalu TRUE di semua cabang | TRUE hanya kalau sah dan di dalam soft limit |
| kuadran `ALFA` | `ATAN` polos, `Y3 < 0` meleset 180° | `ATAN` + koreksi kuadran |
| `ACOS` | tanpa penjaga, bisa error runtime | jangkauan diperiksa dulu, ditolak lewat `ERROR_ID` |
| `EXECUTE`/`DONE` di FK | tidak pernah disentuh | dibaca dan ditulis |
| elbow | selalu elbow-down | `ELBOW_UP` bisa dipilih |
| tool | menimpa globalnya waktu `toolY = 0` | dibaca ke lokal |

`ATAN2` tidak dipakai di V2 — tidak ada di daftar 353 instruksi W560, jadi belum
terbukti ter-import. Kuadrannya dibetulkan lewat `ATAN` + koreksi eksplisit, dan port
JS-nya melakukan hal yang sama supaya hasil PLC dan hasil JS tetap bisa diadu.

**Sel kerja + siklus pick and place.** Rel 3 m, enam stasiun: WIP IN, dua ICC test
(17 detik), dua DW data writer (15 detik), WIP OUT. Bedanya tinggi ICC dan DW cuma
35 mm - mesin nyata berdiri di satu lantai, dan beda ratusan mm memaksa lengan
mengambil pose yang tidak pernah terjadi di sel aslinya.

**Robot mengisi buffer ICC dulu sampai dua-duanya penuh**, baru mengurus yang lain.
Urutan pencarian pekerjaannya - dan urutan itu yang menentukan perilaku selnya:

1. ICC yang **kosong** diisi dari WIP IN. Mesin tes 17 detik tidak boleh menganggur
   menunggu robot selesai mengurus DW.
2. DW yang **selesai** dikosongkan ke WIP OUT. Didahulukan dari nomor 3 supaya selalu
   ada DW kosong buat produk ICC berikutnya - kalau tidak, dua DW penuh + dua ICC
   selesai bikin sel macet, dan macetnya tidak kelihatan sebagai kesalahan, cuma
   sebagai robot yang berhenti.
3. ICC yang **selesai** dipindah ke DW yang kosong.

Tiap mesin menghitung waktunya sendiri, berbarengan - itu inti buffer-nya. Satu
penghitung bersama berarti satu produk di seluruh sel, dan buffer jadi tidak ada
artinya.

Sekuensnya jalan **di PLC** (`PRG_SIM_ROBOT.st`), bukan di halaman. Halaman cuma
menggambar, dan pose stasiun yang digambarnya dibaca dari tag PLC (`SIM_ST_*`) -
bukan dari config. Kalau keduanya sempat berbeda, yang salah kelihatan sebagai
gripper yang turun di sebelah mesin, bukan tersembunyi di balik dua angka yang
masing-masing benar. Selama siklus jalan, jog dimatikan: dua sumber perintah untuk
satu lengan berebut tiap scan.

Tiap pekerjaan bentuknya sama: **lipat ke pose jalan -> geser rel -> turun tegak ->
gripper -> naik -> lipat -> geser rel -> turun -> gripper -> naik -> lipat.** Rel cuma
dilewati dalam pose jalan (= pose home, TCP di 700 mm): lengan yang bergeser sambil
menjulur ke bawah menyapu tiap mesin yang dilewatinya, dan di layar itu mulus sampai
ada yang memperhatikan.

**Panel sel: selector AUTO/MANUAL, E-STOP, Autorun, Cycle stop.** Selector diubah SAAT
robot jalan, atau E-STOP ditekan, atau lengan menabrak → berhenti di tempat dan
**wajib Home dulu** sebelum Autorun mau jalan lagi. Itu bukan kerewelan: posisi sumbu
sesudah berhenti mendadak tidak diketahui sekuenser, dan melanjutkan dari situ berarti
langkah pertama dijalankan dari pose yang tidak pernah direncanakan. Cycle stop beda -
pekerjaan yang sedang dipegang diselesaikan dulu (produk tidak ditinggal di udara),
lengan kembali ke pose jalan, dan karena berhentinya terkendali Autorun bisa langsung
dipakai lagi.

**Produk yang sedang dijepit TETAP dipegang** lewat E-STOP, cycle stop, pergantian
selector, dan Home. Robot pulang membawa produknya, ingatan pekerjaannya
(`SIM_JOB_SRC`/`SIM_JOB_DST`) tidak dihapus, dan Autorun berikutnya **melanjutkan
pengantaran yang tertunda** - bukan memilih pekerjaan baru. Gripper yang membuka
sendiri waktu pulih berarti barang jatuh ke lantai tiap kali orang menekan emergency,
dan mulai dari nol berarti robot berangkat mengambil produk kedua sambil tangannya
masih penuh.

**Satu-satunya cara produk keluar dari gripper selain diletakkan sekuenser: dibuka
manual.** Di MANUAL, membuka gripper sambil memegang = produk **jatuh**, dan
pekerjaannya ikut terhapus. Ingatan yang ditinggal bikin Autorun berikutnya mengantar
produk yang tidak ada - stasiun tercatat berisi, dan salahnya baru ketahuan belasan
menit kemudian waktu robot mengambil produk hantu itu. Yang jatuh **dihitung**
(`SIM_DROP_COUNT`): produk yang hilang tanpa angka bikin jumlah masuk dan jumlah keluar
tidak akan pernah bisa diadu.

Syaratnya ditegakkan **di PLC**, bukan di halaman: halaman cuma mengirim tepi tombol
(`SIM_AUTORUN`, `SIM_CYCLE_STOP`, `SIM_HOME_EXEC`) dan tidak pernah menulis `SIM_AUTO`
sendiri. Syarat yang ditegakkan di browser tidak ikut waktu tombol yang sama ditekan
dari tempat lain.

**Tabrakan diperiksa PLC, dan perintahnya ditolak sebelum sumbu bergerak.** Badan mesin
diperlakukan sebagai kotak (ukuran yang SAMA dengan yang digambar halaman), plus lantai.
Yang diperiksa TCP dan pangkal gripper, dan jangkauan jari ikut dihitung - yang menabrak
duluan biasanya jari yang menjulur ke samping, bukan titik TCP-nya. Pose yang menabrak
ditolak (`SIM_ERROR_ID` 5); kalau lengan sudah terlanjur menyentuh, gerakan dihentikan
di TEPI sentuhan - bukan ditahan terus, karena menahan berarti terkunci di dalam benda
yang ditabraknya tanpa arah keluar.

**Gripper dua jari yang menyumpit dari ATAS.** Pose stasiun `theta_EE = -90`, jadi tool
tegak menghadap bawah, dan jarinya membuka **sepanjang sumbu X (arah rel)** - menjepit
sisi kiri-kanan produk. Jari yang membuka di bidang lengan menjepit sisi depan-belakang,
dan dari kamera mana pun itu tetap terlihat "menjepit", cuma bukan sisi yang benar.
Menutupnya berhenti di **lebar produk**, bukan di nol: jari yang bertemu di nol menembus
barang yang sedang dipegangnya. Panjang gripper masuk TCP - `gen_sim.js` menulis
`ROBOT_TOOL_Y_LREAL = tool.Y + gripper.panjang`, satu kali, satu tempat.

**Tiap mesin punya penutup berengsel yang MENEKAN PCB ke probe base.** Penutup menutup
hanya selama memproses, dan **waktu prosesnya baru jalan setelah penutupnya rapat** -
menghitung sebelum rapat berarti mesin mengaku menguji papan yang belum tersentuh probe,
dan hasilnya tetap keluar "selesai". Robot **tidak turun** ke stasiun yang penutupnya
belum terbuka penuh: penutup itu badan yang bergerak di ruang yang sama dengan gripper,
dan penjaga tabrakan tidak mengenalnya - yang menjaga di sini URUTAN, bukan geometri.

**Dua panel, dua-duanya bisa disembunyikan.** Kanan menjalankan sel; kiri menjelaskan
kinematiknya sambil menunjukkan angkanya bergerak: koordinat nol tiap kerangka (world,
sumbu 0, sendi 1-3, tool/TCP), aliran FK per suku, aliran IK langkah demi langkah
(Y3, Z3, R, beta, gamma, alfa, theta1..3) berikut vonis jangkauan/soft limit, dan
round-trip FK(IK(target)) yang memperlihatkan lantai ~5e-6 derajat itu.

Panel penjelas **tidak menghitung apa pun sendiri**. Angkanya datang dari `fkSteps()` dan
`ikSteps()` di `kin.js` - fungsi yang DIPANGGIL `forwardKinematicV2`/`inverseKinematicV2`,
jadi yang dijelaskan memang yang dihitung. Panel yang menghitung sendiri adalah cara
paling halus untuk berbohong: gambarnya benar, angkanya benar, penjelasannya salah - dan
yang membacanya justru orang yang belum tahu mana yang benar. `tests/viz.test.js` mengadu
keduanya bit per bit, dan menolak `Math.acos/atan/asin` muncul di halaman sama sekali.

**Jog punya slider** per sumbu (mode joint) atau per koordinat world, dengan nilainya di
sebelahnya. Dikirim waktu slider DILEPAS: satu tulis per piksel gerakan mouse membanjiri
sesi OPC UA yang sama yang sedang membaca 80 tag, dan yang kelihatan justru robot yang
tersendat - lawan dari yang sedang disetel.

**Override kecepatan (`SIM_SPEED_OVR`, 1..100 %)** menskalakan semua gerakan sumbu -
siklus otomatis maupun jog. Dua hal yang TIDAK ikut, dan keduanya sengaja: **akselerasi**
(override yang ikut mengubahnya bikin jarak pengereman berubah, jadi pelan-pelan
berhenti lebih aman - padahal itu satu-satunya alasan orang menurunkannya) dan
**gripper** (jarinya pneumatik di mesin aslinya, kecepatannya tidak disetel controller).
Dijepitnya di PLC, bukan di halaman: nilainya boleh ditulis dari mana saja lewat OPC UA,
dan 500 % yang lolos bikin sumbu melompati targetnya tiap scan.

**Gerakannya dihaluskan di dua tempat, dan keduanya perlu.** Di PLC: profil trapesium
per sumbu (akselerasi + jarak rem), jadi sumbu tidak lagi berangkat dan berhenti pada
kecepatan penuh dalam satu scan. Di halaman: di antara dua kabar (~50 ms, sementara layar
menggambar tiap ~16 ms) posisi **diramal dari kecepatan sumbu yang dipublikasikan PLC**,
bukan sekadar dikejar ke posisi terakhir - yang mengejar selalu tertinggal, dan makin
cepat sumbunya makin jauh tertinggal. Ramalannya dibatasi 120 ms: kalau kabarnya berhenti
datang, lengan yang diramal terus akan terbang menjauh, dan itu terlihat seperti robot
yang kabur alih-alih sambungan yang putus.

Yang dihaluskan cuma gambarnya - **panel tetap menampilkan angka PLC apa adanya**, jadi
masih ada tempat untuk membandingkan layar dengan simulator. Panelnya sendiri digambar
paling sering 8x per detik dan mengganti teks di elemen yang sudah ada, bukan menyusun
ulang `innerHTML` tiap kabar: yang tersendat karena itu justru animasi 3D-nya, bukan
panelnya.

Project mesinnya **hanya dibaca**. Tidak ada satu pun berkas di
`C:\Users\denny\Downloads\Blurobot ECU\` yang ditulis alat di folder ini.

## Perintah

Semua path di dokumen ini relatif terhadap AKAR REPO INI (`rb4axis`). Kalau folder ini
kebetulan tinggal di dalam repo alat `sysmac-generator`, tambahkan awalan `blurobot/`
— di situ `node tests/run.js` menjalankan suite repo alat, bukan suite ini.

```bash
node tools/extract.js     # .smc2 -> extract/ (ST verbatim + tabel variabel)
node tools/gen_sim.js     # robot.config.json -> blok init ST, tabel variabel, tags.json
node tools/gen_xml.js     # sim/*.st + *.tsv -> sim/BlurobotSim.xml (import Sysmac)
node tests/run.js         # 5 suite, tanpa Studio dan tanpa PLC
node bridge/bridge.js     # OPC UA <-> halaman, http://127.0.0.1:7656
```

Bridge butuh paket OPC UA - dipasang sekali, dan itu satu-satunya dependensi di
seluruh repo ini:

```bash
cd bridge && npm install
node bridge.js --list SIM_                       # periksa simulator dari terminal
node bridge.js --write SIM_JOG_MODE=0            # tekan tombol dari luar
node bridge.js --watch SIM_JOINT_POS             # pantau perubahan
```

Langkah di Studio yang tidak bisa diotomatiskan ada di [`sim/SETUP.md`](sim/SETUP.md).

## Alurnya

```
BLUEROBOT ECU.smc2  --extract.js-->  extract/*.st          (verbatim, tidak dirapikan)
                                     extract/variables.tsv
robot.config.json   --gen_sim.js-->  sim/PRG_SIM_ROBOT.st    (blok init saja)
                                     sim/*.tsv             (tempel ke Studio)
                                     bridge/tags.json
sim/*_V2.st         (tulis tangan)   FB yang dibetulkan, dipakai simulator
sim/*.st + *.tsv    --gen_xml.js-->  sim/BlurobotSim.xml   (satu berkas, di-import Studio)
        Studio (manual)  ->  project NX102  ->  simulator + OPC UA 127.0.0.1:4840
        bridge.js        ->  SSE + POST     ->  web/  (three.js)
```

## Yang menahan supaya ini tidak diam-diam salah

| | |
|---|---|
| ST **verbatim** | rumusnya tidak pernah diketik ulang. `extract.test.js` mengadu isi `extract/*.st` ke `.smc2` dan menuntut jalan dua kali menghasilkan berkas identik |
| cacat asli **ditiru** | `DONE` selalu TRUE, `ATAN` kehilangan kuadran, `ACOS` tanpa penjaga, FK yang tidak menyentuh `EXECUTE`/`DONE`. Ada tesnya, dan tes itu MENUNTUT perilaku yang salah — sim yang lebih benar dari mesinnya menjawab pertanyaan yang salah |
| penjaga di **satu tempat** | jangkauan diperiksa di dalam `INVERSE_KINEMATIC_V2`, dan program cuma membaca `DONE`/`ERROR_ID`-nya. Dulu penjaganya di pemanggil - dua penjaga untuk satu hal pasti berbeda pendapat suatu hari, dan yang di luar tidak pernah tahu rumus di dalam |
| perbaikan **berdampingan** | `extract/` tidak ikut dibetulkan. Tiap cacat punya DUA tes: satu menuntut perilaku V1, satu menuntut V2, plus satu yang membuktikan keduanya beda di pose yang sama |
| satu sumber angka | dimensi cuma ada di `robot.config.json`; literal ST dibangkitkan darinya, dan `gen_sim.js --check` menolak kalau sudah basi |
| satu sumber daftar tag | `tags.json`, `GlobalVariables.tsv` dan tabel Studio lahir dari daftar yang sama. Daftar kedua yang ditulis tangan pasti drift, dan driftnya diam: halaman menulis ke tag yang tidak ada, bridge menjawab OK, tidak ada yang bergerak |
| gambar ikut FK | viz menggambar dari `chainPoints()` di `kin.js`, bukan menghitung rantai sendiri. `viz.test.js` mengadu titik ujungnya ke keluaran FK — gambar yang tampak wajar tapi menceritakan robot lain itu kegagalan yang tidak ada yang mengeluh |
| konstanta dari project | `PI` = 3.141592654, bukan `Math.PI`. Beda di digit ke-10, tapi tanpa itu perbandingan hasil PLC vs JS jadi berisik dan berhenti dipakai |

## Isi folder

| | |
|---|---|
| `tools/extract.js` | `.smc2` → `extract/`. Parser `.smc2`-nya dipinjam dari `reader/` — jangan tulis parser kedua |
| `tools/gen_sim.js` | `robot.config.json` → blok init ST + `sim/*.tsv` + `bridge/tags.json`. `--check` buat CI |
| `tools/gen_xml.js` | `sim/` → `BlurobotSim.xml`: dua FB + program + tabel global dalam satu berkas import. Bentuknya ditiru dari `Sample.xml` Omron, dan diadu ke XSD resmi Studio |
| `extract/*.st` | badan ST dua FB, **verbatim** (CRLF milik Studio dipertahankan) |
| `extract/interfaces.md` | pin, temp, external tiap FB + kolom "dipakai badan ST" |
| `extract/variables.tsv` | variabel global yang dirujuk ST, kolom urutan tabel Studio, tanpa baris judul |
| `extract/ANALYSIS.md` | struktur robot, peta pemanggil, empat cacat asli. **Ditulis tangan** |
| `sim/robot.config.json` | dimensi, batas, kecepatan, periode task. Satu-satunya tempat angka |
| `sim/*_V2.st` + `*.vars.tsv` | FB kinematik yang sudah dibetulkan + tabel variabelnya |
| `sim/PRG_SIM_ROBOT.st` | program sim: jog, move point, motion model, gripper, FK tiap scan |
| `sim/SETUP.md` | langkah Studio sampai OPC UA hidup, plus tabel gejala→sebab |
| `bridge/bridge.js` | satu sesi OPC UA, SSE ke halaman, POST buat menulis, plus mode CLI (`--list`/`--write`/`--watch`) yang memakai sesi yang sama |
| `bridge/package.json` | satu-satunya dependensi di repo ini; sisanya jalan tanpa `npm install` |
| `web/kin.js` | port JS FK/IK + `chainPoints()`. Dipakai tes DAN mode offline halaman |
| `web/index.html`, `web/robot.js` | viz 3D + panel jog/move |
| `tests/*.test.js` | 5 suite: extract, kin (V1 + V2), sim, viz (rantai + gripper), xml (bentuk + XSD resmi) |

## Batasnya — supaya tidak dikira lebih dari yang ada

* **Mode offline bukan bukti.** Kalau simulator mati, halaman menghitung sendiri
  dengan `kin.js`. Itu berguna buat melihat bentuk gerakan, tapi yang diuji cuma
  port JS-nya — bukan program yang jalan di PLC. Halaman menandainya kuning.
* **Dimensi masih placeholder.** `L1..L4` dan tool tidak ada di project mesin
  (diisi dari HMI Pro-face). Angka di `robot.config.json` cuma supaya ada yang
  digambar.
* **Motion model bukan dinamika.** Profil trapesium per sumbu (kecepatan +
  akselerasi), tanpa massa, tanpa inersia, tanpa jerk. Yang dinilai kinematiknya.
* **Tabrakan cuma TCP + pangkal gripper vs kotak mesin dan lantai.** Siku dan ruas
  lengan TIDAK diperiksa, begitu juga PCB yang sedang dipegang, dan **penutup
  berengsel juga tidak** - yang menjaga gripper tidak menabrak penutup itu urutan
  langkah (turun hanya setelah penutup terbuka penuh), bukan geometri. Pose yang
  menabrak dengan sikunya sendiri lolos: ini penjaga terhadap perintah yang salah,
  bukan mesin fisika.
* **PCB-nya penanda, bukan benda fisik.** Tidak ada gaya jepit: dia ikut gripper
  karena sekuenser bilang begitu, bukan karena dijepit. Jatuh, selip, atau terjepit
  miring tidak ada di model ini.
* **Mesin ICC dan DW tidak mensimulasikan apa pun.** Yang ada cuma penundaan
  (`proses` detik di config) - tidak ada hasil tes, tidak ada data yang ditulis, dan
  tidak ada yang bisa GAGAL tes.
* **Siklusnya butuh PLC.** Waktu offline tombol Autorun dimatikan, bukan dijalankan di
  halaman: sekuens kedua di JS berarti dua sumber kebenaran, dan yang di layar bakal
  terlihat benar justru waktu yang di PLC salah.
* **Round-trip tidak bisa lebih rapat dari ~5e-6 derajat.** `DEGREE_TO_RAD` dan
  `RAD_TO_DEGREE` di project dipotong 9 angka, jadi perkaliannya `1 - 2.98e-8`.
  Itu lantainya, berapa pun benarnya rumusnya — dan konstanta itu yang dipakai PLC.
* **Arti tombol jog TOOL ditetapkan sim.** Penyiap pose kandidat di project asli
  (`ROBOT_POS_JOG_INPUT`) tidak ditulis satu rung pun — kemungkinan besar dari HMI
  Pro-face, yang tidak bisa dibaca alat di repo ini.
* **Import XML-nya belum dibuktikan di Studio.** `BlurobotSim.xml` lolos XSD resmi
  Sysmac, tapi XSD cuma memeriksa BENTUK — apakah Studio mau meng-import POU
  ber-badan ST lewat XML baru terjawab setelah dicoba. Jalur tempel manual di
  `sim/SETUP.md` tetap jalur yang terbukti.
* **Penugasan task tidak bisa lewat XML.** XSD IEC 61131-10 tidak punya elemennya
  sama sekali; program yang tidak ditugaskan ke task tidak dieksekusi, dan Studio
  tidak mengeluh.
* **`.prx` (HMI Pro-face) tidak dibaca sama sekali.** Tidak ada pembacanya di repo,
  dan untuk algoritma memang tidak perlu.

## Suite terpisah dari `node tests/run.js`

Empat gerbang XML repo utama tidak ada urusannya dengan kinematik, dan suite ini
harus boleh SKIP waktu project mesinnya tidak ada di mesin ini (berkas pelanggan,
memang tidak ikut repo). SKIP-nya selalu bersuara — SKIP yang diam tidak bisa
dibedakan dari lulus.
