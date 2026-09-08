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

**Gripper dua jari** di ujung tool. Panjangnya masuk TCP: `gen_sim.js` menulis
`ROBOT_TOOL_Y_LREAL = tool.Y + gripper.panjang` — satu kali, satu tempat.

Project mesinnya **hanya dibaca**. Tidak ada satu pun berkas di
`C:\Users\denny\Downloads\Blurobot ECU\` yang ditulis alat di folder ini.

## Perintah

```bash
node blurobot/tools/extract.js     # .smc2 -> extract/ (ST verbatim + tabel variabel)
node blurobot/tools/gen_sim.js     # robot.config.json -> blok init ST, tabel variabel, tags.json
node blurobot/tools/gen_xml.js     # sim/*.st + *.tsv -> sim/BlurobotSim.xml (import Sysmac)
node blurobot/tests/run.js         # 5 suite, tanpa Studio dan tanpa PLC
node blurobot/bridge/bridge.js     # OPC UA <-> halaman, http://127.0.0.1:7656
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
| `bridge/bridge.js` | satu sesi OPC UA, SSE ke halaman, POST buat menulis |
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
* **Motion model bukan dinamika.** Sumbu dan jari gripper didorong ke target dengan
  batas kecepatan, tanpa profil trapesium, tanpa massa. Yang dinilai kinematiknya.
* **Gripper tidak memegang apa-apa.** Jarinya membuka dan menutup, tapi tidak ada
  benda kerja, tidak ada tabrakan, tidak ada gaya jepit.
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
