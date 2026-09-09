# Catatan kerja untuk Claude — repo `rb4axis` (blurobot)

Alur end-to-end, isi folder, dan batas-batasnya ada di [README.md](README.md).
Berkas ini isinya hal yang **tidak kelihatan dari kode tapi merusak diam-diam**
kalau dilanggar. Baca dulu sebelum mengubah apa pun.

Folder ini punya repo git SENDIRI (`rb4axis`). Kalau kebetulan tinggal di dalam
repo alat `sysmac-generator`, semua path di sini tetap relatif ke AKAR FOLDER INI —
tambahkan awalan `blurobot/` waktu memanggil dari akar repo alat. Salah di situ
bikin `node tests/run.js` menjalankan suite repo alat, bukan suite ini.

## Perintah

```bash
node tools/extract.js            # .smc2 mesin -> extract/   (HANYA BACA project mesin)
node tools/gen_sim.js            # robot.config.json -> blok init ST + *.tsv + tags.json
node tools/gen_sim.js --check    # exit 1 kalau berkas yang ter-commit sudah basi
node tools/gen_xml.js            # sim/ -> sim/BlurobotSim.xml (satu berkas import Studio)
node tools/gen_xml.js --check
node tests/run.js                # 5 suite, tanpa Studio dan tanpa PLC
cd bridge && npm install         # satu-satunya dependensi di repo ini
node bridge/bridge.js            # OPC UA <-> halaman, http://127.0.0.1:7656
node bridge/bridge.js --list SIM_ | --write SIM_JOG_MODE=0 | --watch SIM_JOINT_POS | --tree
```

Bentuk XML-nya diadu ke XSD resmi Sysmac lewat repo alat:
`pwsh scripts/validate_xml.ps1 blurobot/sim/BlurobotSim.xml`.

## Yang TIDAK boleh diubah

**`extract/` jangan pernah dibetulkan.** Itu badan ST verbatim dari project mesin,
CRLF milik Studio dipertahankan. Begitu dirapikan atau dibetulkan, tidak ada lagi
yang bisa diadu ke mesin — dan seluruh alasan repo ini ada ikut hilang. Perbaikan
tempatnya di `sim/*_V2.st`, BERDAMPINGAN.

**Blok di antara `// <<< DIBANGKITKAN` dan `// >>> DIBANGKITKAN` di
`sim/PRG_SIM_ROBOT.st` ditulis `gen_sim.js`.** Sunting `sim/robot.config.json`
lalu generate ulang. Yang disunting tangan hilang di generate berikutnya, atau
lebih buruk: `--check` merah di klon segar tanpa ada yang benar-benar berubah.

**Angka cuma ada di `sim/robot.config.json`.** Panjang link, tool, gripper, batas
soft limit, kecepatan, periode task, pose stasiun. Angka yang sama di dua tempat
pasti drift, dan drift-nya diam: gripper turun di sebelah mesin, bukan di atasnya.

**Panjang gripper dijumlahkan ke `ROBOT_TOOL_Y_LREAL` TEPAT SEKALI**, di `gen_sim.js`.
Dijumlahkan lagi di viz atau di rung, lengannya panjang dua kali gripper — dan di
layar itu cuma tampak seperti lengan yang sedikit lebih panjang.

**Parser `.smc2` dipinjam dari `reader/` repo alat, jangan ditulis ulang.** Parser
kedua diam-diam drift, dan drift di sisi baca menghasilkan ekstraksi yang tampak
wajar tapi bukan rumus yang jalan di mesin. `tools/extract.js` keluar dengan pesan
jelas (exit 3) kalau reader-nya tidak ada — itu bukan bug.

**`.gitattributes` = `* -text`, jangan diaktifkan konversinya.** Hampir semua berkas
di sini DIBANGKITKAN dan diadu byte per byte. Git yang menukar LF↔CRLF bikin semua
gerbang merah di klon segar tanpa ada yang benar-benar berubah.

**CRLF vs LF: dua jalur, dua aturan.**

| jalur | akhiran baris | kenapa |
|---|---|---|
| `.smc2` (tulis ke dalam ZIP) | **CRLF wajib** | tidak lewat parser XML; LF = section muncul di Studio dengan rung KOSONG |
| `<ST>` di dalam XML import | **LF** | pembaca XML menormalkan CRLF→LF sebelum teksnya sampai ke Studio (XML 1.0 §2.11) |

## Aturan Studio yang baru terbukti waktu Build, bukan waktu import

Dua-duanya sudah kena sekali di project ini:

| | |
|---|---|
| array milik instance FB **tidak boleh diindeks** | `IK2.ROBOT_POS_OUTPUT[i]` ditolak: *"Cannot use an element of array or a member of structure for the reference of function block instance variables"*. Anggota skalar (`IK2.DONE`) tidak kena. Salin arraynya UTUH dulu: `IK_OUT := IK2.ROBOT_POS_OUTPUT;` |
| nama POU **tidak boleh diawali `P_`** | itu awalan variabel sistem (`P_On`). Studio menamai ulang sendiri jadi `PR_...` **tanpa satu pun pesan**, dan sesudah itu penugasan task dan tiap rujukan menunjuk POU yang tidak ada. Karena itu namanya `PRG_SIM_ROBOT` |

Dua lagi di jalur XML:

* **ARRAY tidak boleh jadi `<TypeName>`.** `<TypeName>ARRAY[0..3] OF LREAL</TypeName>`
  LOLOS XSD (TypeName itu `xsd:string` apa saja) dan baru ditolak Studio. Yang benar
  `<InstantlyDefinedType xsi:type="ArrayTypeSpec">`.
* **Penugasan task TIDAK bisa lewat XML** — XSD IEC 61131-10 tidak punya elemennya.
  Itu tetap langkah tangan di Studio, dan program yang tidak ditugaskan **tidak
  dieksekusi tanpa keluhan**. Ini sebab nomor satu "tag terbaca tapi tidak ada yang
  bergerak".

`ATAN2` **tidak dipakai** — tidak ada di daftar 353 instruksi W560, jadi belum
terbukti ter-import. Kuadran dibetulkan lewat `ATAN` + koreksi eksplisit, dan port
JS-nya melakukan hal yang sama supaya hasil PLC dan hasil JS tetap bisa diadu.

## OPC UA — yang masing-masing sempat memakan satu putaran

| | |
|---|---|
| **jalankan simulatornya DULU** | menu *Simulation → Use the OPC UA Server for the simulator* abu-abu selama simulator belum jalan. Run (F5) dulu |
| **Security policy centang `None` + anonymous Permit** | yang ditolak karena sertifikat memberi pesan yang **terlihat seperti salah password** |
| **NX102 maupun NX1P2 sama-sama bisa** | menu abu-abu itu soal simulasi belum jalan, bukan model controller |
| **`networkPublish` jangan dianggap otomatis** | project sim yang dibuat dari nol tidak terbaca satu tag pun sampai `networkPublish="PublishOnly"` ditulis ke XML. `gen_xml.js` yang menulisnya, dan ada tesnya |
| **`OPCUACertificateManager` wajib `rootFolder` eksplisit** | dibiarkan implisit, `node-opcua` menggantung selamanya di *"Creating default certificate"*. Kelasnya di paket `node-opcua-certificate-manager`, **bukan** re-export `node-opcua-client` |
| **browse WAJIB pakai `browseNext`** | tanpa continuation point, node-nya hilang diam-diam dan yang tercatat jadi "tag tidak ada" padahal ada. Ini yang sempat kusalahkan ke Network Publish — dan itu salah |
| **`bridge/pki/` di-ignore** | isinya private key. Pernah ikut ter-commit sekali di repo alat |

**Array LREAL lewat JSON jadi `{"0":..,"1":..}`, bukan `[..]`.** `node-opcua`
mengembalikan typed array (`Float64Array`); `JSON.stringify` menulisnya sebagai
objek, `Array.from` di halaman mengembalikan `[]`, dan geometri jadi NaN **tanpa
satu pun error di konsol** — lengannya cuma hilang. Dinormalkan di sumber
(`polos()` di `bridge.js`) DAN ditoleransi di halaman (`keArray()` di `kin.js`).
Jangan hapus salah satunya: yang di bridge membetulkan datanya, yang di halaman
menjaga halaman tidak mati kalau datanya datang dari tempat lain.

## Sel kerja — yang gagal tanpa mengeluh

**`SIM_MOVE_DONE` dihitung motion model di AKHIR scan.** Perintah dan penungguan di
scan yang SAMA berarti yang dibaca jawaban scan sebelumnya — dan sesudah robot
berhenti jawabannya TRUE. Sudah kena sekali di Home: `SIM_HOMED` menyala tanpa
lengannya bergerak satu milimeter, jadi syarat "wajib home dulu" hilang tanpa satu
pun tanda. Karena itu blok Home memadamkan `SIM_MOVE_DONE` sendiri sebelum
memeriksanya. Sekuenser tidak kena karena perintah dan penungguannya ada di langkah —
dan scan — yang berbeda; kalau menambah langkah baru, pertahankan pola itu.

**Home cuma melipat LENGAN — sumbu 0 tidak ikut.** Home yang menyeret rel ke satu titik
bikin tiap pemulihan melewatkan lengan di depan mesin yang tidak ada urusannya, padahal
langkah pertama tiap pekerjaan memang menggeser rel sendiri.

**Rel cuma boleh dilewati dalam pose jalan** (= pose home). Lengan yang bergeser
sambil menjulur ke bawah menyapu tiap mesin yang dilewatinya, dan di layar itu mulus.
Urutannya: lipat → TUNGGU selesai → geser sumbu 0. Menggabungnya jadi satu langkah
menghemat satu detik dan menghapus seluruh gunanya.

**Urutan tiga pencarian pekerjaan itu logika sel, bukan gaya penulisan.** Isi ICC
kosong → kosongkan DW yang selesai → pindah ICC yang selesai ke DW kosong. Nomor 3
didahulukan dari nomor 2 = dua DW penuh + dua ICC selesai bikin sel MACET, dan
macetnya cuma terlihat sebagai robot yang diam.

**Tiap mesin punya penghitung waktunya sendiri.** Satu `SIM_WAIT` bersama (bentuk yang
lama) memaksa satu produk di seluruh sel — buffer ICC jadi tidak ada artinya sementara
kodenya tetap kelihatan benar.

**Produk yang dijepit bertahan lewat SEMUA berhenti** — emergency, cycle stop,
selector, Home. Yang melepasnya cuma perintah gripper dari operator. Dan ingatannya
(`SIM_JOB_SRC`/`SIM_JOB_DST`) ikut bertahan: Autorun sesudah pemulihan MELANJUTKAN
pengantaran (langkah 18), bukan memilih pekerjaan baru — kalau tidak, robot berangkat
mengambil produk kedua sambil tangannya masih penuh.

**Produk jatuh WAJIB menghapus ingatan pekerjaannya.** Menghapus `SIM_PART_STATE` saja
bikin Autorun berikutnya mengantar produk yang tidak ada: robot pergi ke tujuan dengan
tangan kosong, membuka gripper, dan stasiun itu tercatat berisi. Baru ketahuan belasan
menit kemudian, waktu produk hantu itu diambil.

**Kejadian sesaat dipublikasikan sebagai PENCACAH, bukan pulsa.** Bridge mengambil
sampel tiap 50 ms; pulsa satu scan (4 ms) lewat tanpa pernah terlihat. `SIM_DROP_COUNT`
naik, halaman membandingkannya dengan yang terakhir dilihat — dan pesan PERTAMA cuma
menyelaraskan angkanya, kalau tidak membuka halaman sesudah ada produk jatuh
menampilkan animasi jatuh yang tidak sedang terjadi.

**Penjaga tabrakan ditulis SEKALI, dipakai dua arah.** Satu loop memeriksa empat
titik: dua dari pose yang DIMINTA (ditolak sebelum bergerak) dan dua dari pose
SEKARANG (deteksi sentuhan). Rumus yang sama ditulis dua kali pasti berbeda pendapat
suatu hari. Tiga hal yang menempel padanya:

- **Batas vertikalnya sedikit DI BAWAH permukaan stasiun.** Tepat di permukaan berarti
  tiap penempatan produk memicu alarm tabrakan sendiri.
- **Sentuhan membatalkan cuma di TEPI-nya.** Menahan selama masih menempel bikin
  perintah ditimpa posisi tiap scan — lengan terkunci di dalam benda yang ditabraknya,
  tanpa arah keluar.
- **Kotaknya kotak yang SAMA dengan yang digambar** (`SIM_ST_W`/`SIM_ST_D`). Kotak
  kedua untuk penjaga = gripper berhenti di udara atau menembus kotak yang kelihatan,
  dan dua-duanya terbaca sebagai bug yang lain.

**Gripper menutup ke LEBAR PRODUK, bukan ke nol.** `SIM_GRIP_CLOSED` diukur terhadap
`SIM_GRIP_TUTUP`. Menutup ke nol berarti jari bertemu menembus barang yang sedang
dipegangnya — dan di layar itu cuma terlihat seperti jepitan yang rapat.

**Jari membuka sepanjang sumbu X.** Itu satu-satunya arah yang tidak ikut berputar
bersama lengan (rantai 1–3 planar di Y-Z), dan itu sisi yang dijepit mesin aslinya.

**Syarat jalan ditegakkan di PLC, bukan di halaman.** Halaman mengirim TEPI tombol dan
tidak pernah menulis `SIM_AUTO`. Syarat yang ditegakkan di browser tidak ikut waktu
tombol yang sama ditekan dari tempat lain.

## Viz — yang bikin gambar berbohong sambil tetap tampak wajar

**Gambar HARUS dari `chainPoints()` di `kin.js`,** bukan rantai yang dihitung ulang
di `robot.js`. `viz.test.js` mengadu titik ujung gambar ke keluaran FK. Robotnya
**1 prismatik + 3 revolute planar**, bukan 4R: sumbu 0 tidak pernah masuk fungsi
trigonometri mana pun dan langsung jadi X; sumbu 1–3 rantai planar di bidang Y–Z.
Salah di sini menggambar robot yang lain sambil tetap tampak masuk akal.

**`ruasKe()` membagi dengan `mesh.geometry.parameters.depth`, bukan angka tetap.**
Pernah kena: `kotak(58, 1, 46)` menaruh tebal 1 mm di sumbu Y sementara penskalaan
memakai `scale.z` — lengannya 46× terlalu panjang. Dibagi dengan depth geometrinya
sendiri, salah sumbu tidak bisa terjadi lagi.

**`ke3(p)` memetakan PLC (x,y,z) → three (x,z,y).** Semua penempatan objek lewat
situ; yang menulis koordinat mentah langsung ke three menaruhnya di sumbu lain.

**Pose stasiun digambar dari tag PLC (`SIM_ST_*`), bukan dari config.** Kalau
keduanya sempat berbeda, salahnya kelihatan sebagai gripper yang turun di sebelah
mesin — bukan tersembunyi di balik dua angka yang masing-masing benar.

Dua hal kecil yang gampang balik salah: shadow camera harus melingkupi seluruh sel
(±2200; mesin di ±1400 — yang ±1600 memotong bayangan mesin ujung), dan label
stasiun digambar ulang **hanya waktu teksnya berubah** — tiap frame berarti
`CanvasTexture` baru terus.

**Penghalusan gambar meramal dari KECEPATAN PLC, bukan mengejar posisi.** Mengejar
posisi selalu tertinggal, dan makin cepat sumbunya makin jauh tertinggal — itu yang
terbaca sebagai "laggy". Ramalannya wajib dibatasi (120 ms): tanpa batas, kabar yang
berhenti datang bikin lengan terbang menjauh, dan itu terlihat seperti robot yang kabur
alih-alih sambungan yang putus.

**Panel penjelas WAJIB membaca `fkSteps()`/`ikSteps()`, bukan menghitung sendiri.**
Kedua fungsi itu yang dipanggil `forwardKinematicV2`/`inverseKinematicV2`, jadi satu
sumber untuk PLC, tes, gambar, dan penjelasan. Panel yang menghitung sendiri adalah cara
paling halus untuk berbohong - gambar benar, angka benar, penjelasan salah - dan yang
membacanya justru orang yang belum bisa menilai. `viz.test.js` mengadu keduanya bit per
bit DAN menolak `Math.acos/atan/asin` muncul di `robot.js` sama sekali.

**Perintah yang DITOLAK tidak menggerakkan apa pun — dan `SIM_MOVE_DONE` tetap TRUE.**
Tiap penantian sesudah permintaan IK karena itu menuntut `NOT SIM_ERROR` juga. Tanpa itu
sekuenser melangkah maju seolah lengannya sudah sampai, dan waypoint yang dilewati justru
approach — yang justru ada untuk menjaganya tidak menyapu mesin. Gejalanya: lengan turun
langsung dari pose jalan ke permukaan, mulus, tanpa satu pun keluhan.

**Penutup mesin: tiga aturan yang gagal tanpa keluhan.** (1) menutup HANYA selama
memproses — di luar itu dia menutup jalan masuk gripper; (2) waktu proses baru jalan
setelah penutup RAPAT — kalau tidak, mesin mengaku menguji papan yang belum tersentuh
probe dan tetap melaporkan selesai; (3) robot turun HANYA setelah penutup terbuka penuh —
penutup tidak ada di penjaga tabrakan, jadi yang menjaga di sini urutan langkah.
Di 3D penutup diputar pada ENGSEL (pivot di garis engsel, kotak jadi anaknya): kotak yang
diputar di tengahnya menembus meja tiap kali membuka.

**Penutup ikut jadi badan tabrakan, tapi HANYA waktu belum terbuka penuh.** Terbuka penuh
daunnya berdiri di belakang engsel, di luar jalan masuk; menghitungnya tetap menutup jalan
berarti robot tidak akan pernah bisa turun ke stasiun mana pun. Pasangannya interlock
`SIM_ST_ZONA`: penutup tidak menutup selama ada titik lengan di ruang sapuannya, dan
membuka balik kalau ada yang masuk lagi. Zonanya dihitung dari titik lengan yang SEKARANG
di dalam loop penjaga tabrakan — bukan ditebak dari nomor langkah, karena jog dan gerakan
tangan tidak punya nomor langkah.

**Gerbang penutup ditunggu SEBELUM pose approach diminta (langkah 8 dan 21), bukan
sesudahnya (11/23).** Kalau sesudah: pose approach diminta selagi penutup masih menutup,
penjaga menolaknya dengan benar, sumbu tidak bergerak, `SIM_MOVE_DONE` tetap TRUE — dan
langkah berikutnya jalan. Lengan turun langsung dari pose jalan ke permukaan.

**Slider mengirim waktu DILEPAS (`onchange`), bukan tiap piksel (`oninput`).** Satu tulis
OPC UA per gerakan mouse membanjiri sesi yang sama yang membaca puluhan tag, dan yang
terlihat justru robot tersendat — lawan dari yang sedang disetel.

**Panel jangan digambar dari tiap pesan SSE.** ~20 pesan per detik × puluhan elemen
`innerHTML` yang disusun ulang = layout ulang di tengah frame, dan yang tersendat justru
animasi 3D-nya. Bentuk panel dibangun SEKALI (`bikinPanelStatis`), `panelTampil()` cuma
mengganti teks, dan pemanggilannya di-throttle dari `putar()`.

**Penghalusan gambar TIDAK boleh menyentuh angka panel.** Bridge mengirim tiap ~50 ms,
layar menggambar tiap ~16 ms, jadi yang digambar dikejar ke nilai PLC terakhir
(`haluskan()`); panel tetap menampilkan `st.jointPlc` apa adanya. Panel yang ikut
dihaluskan menghapus satu-satunya tempat membandingkan layar dengan simulator.

**Siklus otomatis jalan di PLC, jangan dipindah ke halaman.** Sekuens kedua di JS =
dua sumber kebenaran, dan yang di layar bakal terlihat benar justru waktu yang di
PLC salah. Waktu offline tombol Start dimatikan, bukan disimulasikan.

## Tes

`node tests/run.js` — 5 suite, sengaja terpisah dari suite repo alat: empat gerbang
XML di sana tidak ada urusannya dengan kinematik, dan suite ini harus boleh SKIP
waktu project mesin `.smc2` tidak ada di mesin ini (berkas pelanggan, memang tidak
ikut repo).

**SKIP wajib bersuara.** SKIP yang diam tidak bisa dibedakan dari lulus.

**Tiap cacat V1 punya DUA tes** — satu menuntut perilaku V1 yang salah, satu
menuntut V2 yang benar — plus satu yang membuktikan keduanya beda di pose yang
sama. Tes yang "membetulkan" V1 sebenarnya menjawab pertanyaan yang lain.

**Round-trip kinematik tidak bisa lebih rapat dari ~5e-6 derajat.** `DEGREE_TO_RAD`
dan `RAD_TO_DEGREE` di project dipotong 9 angka, jadi perkaliannya `1 − 2.98e-8`.
Itu lantainya, berapa pun benarnya rumusnya — tes yang menuntut lebih rapat
sebenarnya menuntut konstanta yang lain. Konstanta diambil persis dari project
(`PI` = 3.141592654), BUKAN `Math.PI`.

**Kalau tes gagal, periksa dulu apakah TESNYA yang salah.** Empat "kegagalan" V2 di
sesi pertama semuanya ekspektasi tes yang keliru, bukan cacat produk. Buktikan dulu
mana yang salah sebelum memperbaiki.

## Urutan setelah mengubah sesuatu

```
robot.config.json diubah  ->  node tools/gen_sim.js && node tools/gen_xml.js && node tests/run.js
sim/*_V2.st diubah        ->  node tools/gen_xml.js && node tests/run.js
web/kin.js diubah         ->  node tests/run.js          (viz.test.js mengadu gambar ke FK)
XML diubah                ->  pwsh scripts/validate_xml.ps1 blurobot/sim/BlurobotSim.xml
```

Di mesin orangnya, setelah XML berubah: import ulang ke Studio → **Build** →
tugaskan `PRG_SIM_ROBOT` ke task primer → Transfer to simulator → hidupkan OPC UA →
restart bridge → halaman Ctrl+F5. Yang terlewat di langkah "tugaskan ke task"
kelihatan persis seperti bridge yang rusak.
