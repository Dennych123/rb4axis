# Bikin project simulasi di Sysmac Studio

Langkah yang memang harus dikerjakan orang di Studio — tidak ada jalur otomatisnya,
dan menebak-nebak di sini mahal karena tiap kesalahan baru kelihatan setelah Build.

Sebelum mulai, pastikan berkasnya segar:

```bash
node tools/extract.js     # extract/*.st + variables.tsv dari project mesin
node tools/gen_sim.js     # blok init ST + tabel variabel + tags.json dari config
node tools/gen_xml.js     # semuanya jadi SATU berkas import: sim/BlurobotSim.xml
node tests/run.js         # 5 suite - jangan ke Studio kalau ini merah
```

Ada DUA jalur memasukkan ini ke Studio. Pilih satu:

| | |
|---|---|
| **A. Import XML** (langkah 2A) | satu berkas, satu klik. Bentuknya sudah lolos XSD resmi Studio, tapi apakah Studio mau meng-import POU ber-badan ST lewat XML **belum pernah dibuktikan di mesin ini** |
| **B. Tempel manual** (langkah 2B–4) | lebih panjang dan lebih rawan salah kolom, tapi tiap langkahnya jalur yang memang dipakai sehari-hari |

Coba A dulu. Kalau Studio bilang `(Import failed)` atau `(DefinitionError)`, jangan
menebak-nebak: catat pesannya, lalu kerjakan B — dan pesan itu yang jadi bahan
memperbaiki `gen_xml.js`.

## 1. Project baru

**New Project → Controller NX102** (mana saja dari keluarga NX102), versi Studio yang
terpasang. Kosong: tanpa EtherCAT, tanpa axis, tanpa axes group.

Project sim sengaja **tanpa `MC_*`** — posisi sumbu diintegrasi di ST. Jadi tidak ada
setting motion yang bisa salah, dan tidak ada langkah "assign axis" yang gampang
terlewat.

Controller-nya beda dari project mesin (NJ501). Yang diangkat cuma ST murni, jadi
tidak ada yang bergantung ke model controller — dan NX102 dipilih karena OPC UA
server untuk simulator memang ada di situ.

## 2A. Import XML (satu berkas)

```bash
node tools/gen_xml.js
pwsh ../scripts/validate_xml.ps1 sim/BlurobotSim.xml      # validator milik repo ALAT
```

Validator itu bagian dari repo alat `sysmac-generator`, bukan repo ini - path-nya
tergantung di mana repo alat itu kamu taruh. Kalau tidak punya, langkah ini boleh
dilewati: `node tests/run.js` sudah memeriksa bentuk XML-nya sendiri, dan gerbang XSD
di dalamnya SKIP dengan alasan kalau validatornya tidak ketemu.

Kalau ada, validator harus bilang `Semua lolos XSD` sebelum berkasnya dibawa ke Studio. Studio
sendiri cuma bilang `(Import failed)` tanpa nomor baris; validator menyebut elemen dan
barisnya.

Di Studio: project NX102 baru (langkah 1) → **Multiview Explorer → klik kanan
Programming → Import** (atau menu **File → Import**) → pilih `sim/BlurobotSim.xml`.

Yang ikut di berkas itu: dua FB V2 lengkap dengan pin dan variabelnya, program
`PRG_SIM_ROBOT` berikut variabel lokal dan `ExternalVars`-nya, dan seluruh tabel
variabel global (termasuk `Constant` dan nilai awal `PI`/`DEGREE_TO_RAD`/
`RAD_TO_DEGREE`).

Yang **TIDAK** ikut, dan tetap harus dikerjakan tangan:

* **Penugasan task** — XSD IEC 61131-10 tidak punya elemen untuk itu sama sekali.
  Lanjut ke langkah 4 butir 4 dan 5.
* Setelan controller, periode task, dan setelan OPC UA server.

Sesudah import berhasil, **lompat ke langkah 4 butir 4** (penugasan task).

Dua hal yang sudah terbukti di Studio lewat jalur ini, dan sudah dibetulkan di berkas
yang dibangkitkan sekarang — catat kalau nanti menulis POU baru:

* **Array milik instance FB tidak boleh diindeks.** `IK2.ROBOT_POS_OUTPUT[i]` ditolak
  waktu Build; `IK2.DONE` tidak. Arraynya disalin utuh dulu ke variabel lokal.
* **Nama POU tidak boleh diawali `P_`.** Studio menamai ulang sendiri jadi `PR_...`
  tanpa memberi tahu, dan sesudah itu penugasan task menunjuk nama yang tidak ada.

## 2B. Dua function block (jalur tempel)

Yang ditempel ke Studio adalah **versi V2** — yang sudah dibetulkan:
[`FORWARD_KINEMATIC_V2.st`](FORWARD_KINEMATIC_V2.st) dan
[`INVERSE_KINEMATIC_V2.st`](INVERSE_KINEMATIC_V2.st).

Yang verbatim dari mesin ada di [`../extract/`](../extract/) dan **tidak ditempel** —
dia catatan tentang apa yang jalan di mesin, bukan bagian dari sim. Daftar bedanya
ada di kepala kedua berkas V2 dan di [`../extract/ANALYSIS.md`](../extract/ANALYSIS.md).

Untuk masing-masing:

1. **Programming → POUs → Function Blocks → klik kanan → Add → ST**.
   Namanya harus **PERSIS** `FORWARD_KINEMATIC_V2` dan `INVERSE_KINEMATIC_V2` —
   nama itu yang dipakai `ProgramVariables.tsv` sebagai tipe instance.
2. Tabel variabelnya tempel dari `<nama>.vars.tsv`. Kolomnya
   `Name`, `Data type`, `Initial value`, `Grup` — grup di kolom keempat itu penunjuk
   ke mana barisnya masuk (`VAR_INPUT`, `VAR_OUTPUT`, `VAR`, `VAR_EXTERNAL`); di
   Studio tiap grup punya bloknya sendiri, jadi tempel per blok, bukan sekaligus.
3. Badan ST-nya copy-paste dari `<nama>.st`.

**Tidak ada `_sAXIS_REF` di FB V2**, jadi tidak ada yang perlu dihapus seperti dulu:
V1 mendeklarasi empat `BLUE_ROBOT_AXIS` yang badan ST-nya tidak pernah sentuh, dan
di project tanpa axis itu bikin Build gagal.

## 3. Variabel global (jalur tempel)

**Programming → Data → Global Variables**, klik sel pertama, lalu tempel isi
[`GlobalVariables.tsv`](GlobalVariables.tsv) apa adanya.

Berkas itu **tanpa baris judul**, disengaja: judul yang ikut tertempel mendarat
sebagai variabel bernama `Name` bertipe `Data type`. Kolomnya urutan tabel Studio:
`Name`, `Data type`, `Initial value`, `AT`, `Retain`, `Constant`, `Network Publish`,
`Comment`.

Nilai awalnya sengaja kosong — yang mengisi `ROBOT_L*`, `PD1300_*` dan `SIM_*` adalah
blok init di `PRG_SIM_ROBOT.st`, dari `robot.config.json`. Satu sumber angka, bukan dua.

Kecuali `PI`, `DEGREE_TO_RAD`, `RAD_TO_DEGREE`: itu `Constant` dengan nilai awal dari
project mesin, dan memang tidak boleh ditulis program mana pun.

**Network Publish tidak perlu disetel.** Variabel global ter-publish otomatis ke OPC UA
server simulator; path-nya `GlobalVars.<nama>`.

## 4. Program + penugasan task

Kalau lewat jalur A, butir 1–3 sudah selesai; langsung ke butir 4.

1. **Programming → POUs → Programs → Add → ST**, namanya `PRG_SIM_ROBOT`.
2. Tabel variabel programnya: tempel [`ProgramVariables.tsv`](ProgramVariables.tsv).
   Kolomnya `Name`, `Data type`, `Initial value`, `Retain`, `Constant`, `Comment`
   (tabel program tidak punya kolom Network Publish). Kalau susunan kolom di versi
   Studio-mu berbeda, ketik 19 barisnya manual — jangan tempel yang kolomnya melenceng,
   Studio menerimanya tanpa keluhan dan yang salah baru ketahuan waktu Build.
3. Badan programnya: copy-paste [`PRG_SIM_ROBOT.st`](PRG_SIM_ROBOT.st).
4. **Task Settings → PrimaryTask → Program Assignment → tambahkan `PRG_SIM_ROBOT`.**
   Program yang tidak ditugaskan ke task **tidak dieksekusi, dan Studio tidak
   mengeluh** — gejalanya: semua tag ada di OPC UA, semuanya diam, `SIM_HEARTBEAT`
   tidak pernah naik.
5. Periode task primer harus sama dengan `task.periode_ms` di `robot.config.json`
   (default **4 ms**). Beda, dan kecepatan gerak di layar bukan kecepatan yang diminta.

Build (F8) harus bersih sebelum lanjut.

## 5. Simulator + OPC UA

Urutannya penting — menu OPC UA abu-abu selama simulator belum jalan:

1. **Simulation → Run (F5)**. Tunggu simulatornya benar-benar RUN.
2. **Simulation → Use the OPC UA Server for the simulator**.
3. Di setelan OPC UA server: **Security policy centang `None`** dan **Anonymous =
   Permit**. Tanpa `None`, sambungan wajib Sign + sertifikat klien yang dipercaya, dan
   penolakan karena sertifikat memberi pesan yang **terlihat seperti salah password**.
4. **Transfer to simulator**.

Endpointnya `opc.tcp://127.0.0.1:4840`.

## 6. Bukti bahwa jalan

Paketnya dipasang sekali (ini SATU-SATUNYA bagian repo ini yang punya dependensi):

```bash
cd bridge && npm install && cd ../..
node bridge/bridge.js --list SIM_
```

Yang dicari bukan "daftarnya keluar", tapi tiga hal:

| | |
|---|---|
| `GlobalVars.SIM_HEARTBEAT` naik | program benar-benar dieksekusi task, bukan cuma ada di project |
| `GlobalVars.SIM_JOINT_POS` = pose home (`0, 90, -90, 0`) | blok init jalan, config sampai ke controller |
| `GlobalVars.SIM_GRIP_POS` = 80 lalu 0 waktu `SIM_GRIP_CMD` ditulis TRUE | gripper hidup |
| `GlobalVars.ROBOT_TOOL_Y_LREAL` = 140 (50 tool + 90 gripper) | TCP di ujung jari, dijumlahkan sekali |
| `GlobalVars.SIM_WORLD_POS` masuk akal | FK dipanggil dan hasilnya keluar |

Lalu tekan tombol dari luar:

```bash
node bridge/bridge.js --write SIM_JOG_MODE=0
node bridge/bridge.js --write "SIM_JOG_P[1]=true"
node bridge/bridge.js --watch SIM_JOINT_POS SIM_WORLD_POS
```

Perintah itu memakai **sesi dan peta tag yang sama** dengan halaman viz - bukan klien
kedua. Alat terpisah buat "cek cepat" selalu berakhir jadi jalur yang perilakunya
berbeda, dan yang berbeda diam-diam itu yang paling mahal.

`tools/opcua/browse.js` yang disebut di catatan lama itu milik repo ALAT
(sysmac-generator). Di klon rb4axis yang berdiri sendiri berkas itu memang tidak ada.

Sesudah itu baru jalankan bridge + halaman viz — lihat [`../README.md`](../README.md).

## Kalau salah

| gejala | sebabnya hampir selalu |
|---|---|
| tag ada, semua diam, `SIM_HEARTBEAT` tetap 0 | program belum ditugaskan ke task |
| Build gagal menyebut `BLUE_ROBOT_AXIS1` | yang ditempel FB V1 dari `extract/`, bukan V2 dari `sim/` |
| `(Import failed)` tanpa nomor baris | validasi dulu ke XSD (validator repo alat) — dia menyebut elemen dan barisnya, Studio tidak |
| `(DefinitionError)` sesudah import XML | susunan pin FB tidak cocok; catat nama POU-nya, itu bahan buat memperbaiki `gen_xml.js` |
| `Cannot use an element of array or a member of structure for the reference of function block instance variables` | ada `FK2.ARRAY[i]` — array milik instance FB tidak boleh diindeks. Salin arraynya UTUH dulu ke variabel lokal. Anggota skalar (`IK2.DONE`) tidak kena |
| nama program di daftar error bukan yang kamu import | Studio menamai ulang POU yang awalannya `P_` (awalan itu milik variabel sistem: `P_On`, `P_First_Run`) — **tanpa satu pun pesan**. Karena itu programnya `PRG_SIM_ROBOT`, bukan `P_SIM_ROBOT` |
| gripper tidak bergerak | `SIM_GRIP_VEL` 0, atau `SIM_GRIP_STROKE` 0 — dua-duanya diisi blok init |
| TCP meleset sepanjang gripper | panjang gripper dijumlahkan dua kali; di ST harus muncul TEPAT SEKALI, di `ROBOT_TOOL_Y_LREAL` |
| Build gagal "cannot assign to constant" | `PI`/`DEGREE_TO_RAD`/`RAD_TO_DEGREE` ikut ditempel tanpa kolom Constant |
| menu OPC UA abu-abu | simulator belum Run |
| klien OPC UA ditolak, pesannya seperti salah password | Security policy `None` belum dicentang |
| lengan bergerak, kecepatannya aneh | periode task ≠ `task.periode_ms` |
| variabel bernama `Name` bertipe `Data type` muncul di tabel | baris judul ikut tertempel |
