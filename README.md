# Blurobot — kinematik dari project mesin → simulator NX → viz 3D

Mengambil **algoritma kinematik** dari project robot 4 sumbu
`BLUEROBOT ECU 28032020.smc2` (NJ501, 1219 rung), menjalankannya di simulator
Sysmac Studio, dan menggambarnya 3D di browser lewat OPC UA.

Yang diangkat cuma kinematik + jog + move point. Interpreter program teach
(`P100`–`P102`) dan sekuens station (`RB_MOVING`, `WIP_MOVING`) **tidak** — itu
keputusan, bukan kelupaan. Latarnya di
[`extract/ANALYSIS.md`](extract/ANALYSIS.md).

Project mesinnya **hanya dibaca**. Tidak ada satu pun berkas di
`C:\Users\denny\Downloads\Blurobot ECU\` yang ditulis alat di folder ini.

## Perintah

```bash
node blurobot/tools/extract.js     # .smc2 -> extract/ (ST verbatim + tabel variabel)
node blurobot/tools/gen_sim.js     # robot.config.json -> blok init ST, 2 TSV, tags.json
node blurobot/tests/run.js         # 4 suite, tanpa Studio dan tanpa PLC
node blurobot/bridge/bridge.js     # OPC UA <-> halaman, http://127.0.0.1:7656
```

Langkah di Studio yang tidak bisa diotomatiskan ada di [`sim/SETUP.md`](sim/SETUP.md).

## Alurnya

```
BLUEROBOT ECU.smc2  --extract.js-->  extract/*.st          (verbatim, tidak dirapikan)
                                     extract/variables.tsv
robot.config.json   --gen_sim.js-->  sim/P_SIM_ROBOT.st    (blok init saja)
                                     sim/*.tsv             (tempel ke Studio)
                                     bridge/tags.json
        Studio (manual)  ->  project NX102  ->  simulator + OPC UA 127.0.0.1:4840
        bridge.js        ->  SSE + POST     ->  web/  (three.js)
```

## Yang menahan supaya ini tidak diam-diam salah

| | |
|---|---|
| ST **verbatim** | rumusnya tidak pernah diketik ulang. `extract.test.js` mengadu isi `extract/*.st` ke `.smc2` dan menuntut jalan dua kali menghasilkan berkas identik |
| cacat asli **ditiru** | `DONE` selalu TRUE, `ATAN` kehilangan kuadran, `ACOS` tanpa penjaga, FK yang tidak menyentuh `EXECUTE`/`DONE`. Ada tesnya, dan tes itu MENUNTUT perilaku yang salah — sim yang lebih benar dari mesinnya menjawab pertanyaan yang salah |
| penjaga di **pemanggil** | penjaga jangkauan ada di `P_SIM_ROBOT.st` dan `reachable()`, bukan di dalam FB. FB tetap identik dengan yang jalan di mesin |
| satu sumber angka | dimensi cuma ada di `robot.config.json`; literal ST dibangkitkan darinya, dan `gen_sim.js --check` menolak kalau sudah basi |
| satu sumber daftar tag | `tags.json`, `GlobalVariables.tsv` dan tabel Studio lahir dari daftar yang sama. Daftar kedua yang ditulis tangan pasti drift, dan driftnya diam: halaman menulis ke tag yang tidak ada, bridge menjawab OK, tidak ada yang bergerak |
| gambar ikut FK | viz menggambar dari `chainPoints()` di `kin.js`, bukan menghitung rantai sendiri. `viz.test.js` mengadu titik ujungnya ke keluaran FK — gambar yang tampak wajar tapi menceritakan robot lain itu kegagalan yang tidak ada yang mengeluh |
| konstanta dari project | `PI` = 3.141592654, bukan `Math.PI`. Beda di digit ke-10, tapi tanpa itu perbandingan hasil PLC vs JS jadi berisik dan berhenti dipakai |

## Isi folder

| | |
|---|---|
| `tools/extract.js` | `.smc2` → `extract/`. Parser `.smc2`-nya dipinjam dari `reader/` — jangan tulis parser kedua |
| `tools/gen_sim.js` | `robot.config.json` → blok init ST + `sim/*.tsv` + `bridge/tags.json`. `--check` buat CI |
| `extract/*.st` | badan ST dua FB, **verbatim** (CRLF milik Studio dipertahankan) |
| `extract/interfaces.md` | pin, temp, external tiap FB + kolom "dipakai badan ST" |
| `extract/variables.tsv` | variabel global yang dirujuk ST, kolom urutan tabel Studio, tanpa baris judul |
| `extract/ANALYSIS.md` | struktur robot, peta pemanggil, empat cacat asli. **Ditulis tangan** |
| `sim/robot.config.json` | dimensi, batas, kecepatan, periode task. Satu-satunya tempat angka |
| `sim/P_SIM_ROBOT.st` | program sim: jog, move point, penjaga, motion model, FK tiap scan |
| `sim/SETUP.md` | langkah Studio sampai OPC UA hidup, plus tabel gejala→sebab |
| `bridge/bridge.js` | satu sesi OPC UA, SSE ke halaman, POST buat menulis |
| `web/kin.js` | port JS FK/IK + `chainPoints()`. Dipakai tes DAN mode offline halaman |
| `web/index.html`, `web/robot.js` | viz 3D + panel jog/move |
| `tests/*.test.js` | 4 suite: extract, kin, sim, viz |

## Batasnya — supaya tidak dikira lebih dari yang ada

* **Mode offline bukan bukti.** Kalau simulator mati, halaman menghitung sendiri
  dengan `kin.js`. Itu berguna buat melihat bentuk gerakan, tapi yang diuji cuma
  port JS-nya — bukan program yang jalan di PLC. Halaman menandainya kuning.
* **Dimensi masih placeholder.** `L1..L4` dan tool tidak ada di project mesin
  (diisi dari HMI Pro-face). Angka di `robot.config.json` cuma supaya ada yang
  digambar.
* **Motion model bukan dinamika.** Sumbu didorong ke target dengan batas kecepatan,
  tanpa profil trapesium, tanpa massa. Yang dinilai kinematiknya.
* **Arti tombol jog TOOL ditetapkan sim.** Penyiap pose kandidat di project asli
  (`ROBOT_POS_JOG_INPUT`) tidak ditulis satu rung pun — kemungkinan besar dari HMI
  Pro-face, yang tidak bisa dibaca alat di repo ini.
* **`.prx` (HMI Pro-face) tidak dibaca sama sekali.** Tidak ada pembacanya di repo,
  dan untuk algoritma memang tidak perlu.

## Suite terpisah dari `node tests/run.js`

Empat gerbang XML repo utama tidak ada urusannya dengan kinematik, dan suite ini
harus boleh SKIP waktu project mesinnya tidak ada di mesin ini (berkas pelanggan,
memang tidak ikut repo). SKIP-nya selalu bersuara — SKIP yang diam tidak bisa
dibedakan dari lulus.
