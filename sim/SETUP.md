# Bikin project simulasi di Sysmac Studio

Langkah yang memang harus dikerjakan orang di Studio — tidak ada jalur otomatisnya,
dan menebak-nebak di sini mahal karena tiap kesalahan baru kelihatan setelah Build.

Sebelum mulai, pastikan berkasnya segar:

```bash
node blurobot/tools/extract.js     # extract/*.st + variables.tsv dari project mesin
node blurobot/tools/gen_sim.js     # blok init ST + dua tabel variabel dari config
node blurobot/tests/run.js         # 3 suite - jangan ke Studio kalau ini merah
```

## 1. Project baru

**New Project → Controller NX102** (mana saja dari keluarga NX102), versi Studio yang
terpasang. Kosong: tanpa EtherCAT, tanpa axis, tanpa axes group.

Project sim sengaja **tanpa `MC_*`** — posisi sumbu diintegrasi di ST. Jadi tidak ada
setting motion yang bisa salah, dan tidak ada langkah "assign axis" yang gampang
terlewat.

Controller-nya beda dari project mesin (NJ501). Yang diangkat cuma ST murni, jadi
tidak ada yang bergantung ke model controller — dan NX102 dipilih karena OPC UA
server untuk simulator memang ada di situ.

## 2. Dua function block

Untuk `FORWARD_KINEMATIC` dan `INVERSE_KINEMATIC`, masing-masing:

1. **Programming → POUs → Function Blocks → klik kanan → Add → ST**.
   Namanya harus **PERSIS** `FORWARD_KINEMATIC` dan `INVERSE_KINEMATIC` — nama itu
   yang dipakai `ProgramVariables.tsv` sebagai tipe instance.
2. Tabel variabel FB diisi dari [`../extract/interfaces.md`](../extract/interfaces.md).
   Urutan pin (`Ord`) menentukan bentuk kotaknya; untuk pemanggilan dari ST urutan
   tidak berpengaruh, tapi **nama** berpengaruh mutlak.
   * `VAR_INPUT`: `EXECUTE : BOOL`, `ROBOT_POS_INPUT : ARRAY[0..3] OF LREAL`
   * `VAR_OUTPUT` FK: `DONE : BOOL`, `ROBOT_POS_JOINT_OUTPUT : ARRAY[0..3] OF REAL`,
     `ROBOT_POS_WORLD_OUTPUT : ARRAY[0..3] OF REAL`
   * `VAR_OUTPUT` IK: `DONE : BOOL`, `ROBOT_POS_OUTPUT : ARRAY[0..3] OF LREAL`
   * `VAR` (temp) dan `VAR_EXTERNAL`: lihat tabelnya di `interfaces.md`.
     Yang bertanda **TIDAK** di kolom "Dipakai badan ST" tetap didaftar — itu bentuk
     aslinya, dan `_sAXIS_REF` di FK memang tidak dipakai rumusnya.
3. Badan ST-nya **copy-paste dari `../extract/FORWARD_KINEMATIC.st` dan
   `../extract/INVERSE_KINEMATIC.st`**. Jangan diketik ulang, jangan dirapikan.
   Kalau ada yang ingin membetulkan `ATAN` jadi `ATAN2`: baca dulu
   [`../extract/ANALYSIS.md`](../extract/ANALYSIS.md) — sim yang lebih benar dari
   mesinnya menjawab pertanyaan yang salah.

`_sAXIS_REF` butuh axis terdaftar. Karena project sim tidak punya axis, **hapus empat
baris `BLUE_ROBOT_AXIS1..4` dari `VAR_EXTERNAL` FK** — badan ST-nya tidak menyentuh
mereka sama sekali (`interfaces.md` menandainya **TIDAK**), jadi ini satu-satunya
tempat di seluruh alur ini yang boleh menyimpang dari project asli. Kalau tidak
dihapus, Build gagal dengan keluhan variabel axis yang tidak ada.

## 3. Variabel global

**Programming → Data → Global Variables**, klik sel pertama, lalu tempel isi
[`GlobalVariables.tsv`](GlobalVariables.tsv) apa adanya.

Berkas itu **tanpa baris judul**, disengaja: judul yang ikut tertempel mendarat
sebagai variabel bernama `Name` bertipe `Data type`. Kolomnya urutan tabel Studio:
`Name`, `Data type`, `Initial value`, `AT`, `Retain`, `Constant`, `Network Publish`,
`Comment`.

Nilai awalnya sengaja kosong — yang mengisi `ROBOT_L*`, `PD1300_*` dan `SIM_*` adalah
blok init di `P_SIM_ROBOT.st`, dari `robot.config.json`. Satu sumber angka, bukan dua.

Kecuali `PI`, `DEGREE_TO_RAD`, `RAD_TO_DEGREE`: itu `Constant` dengan nilai awal dari
project mesin, dan memang tidak boleh ditulis program mana pun.

**Network Publish tidak perlu disetel.** Variabel global ter-publish otomatis ke OPC UA
server simulator; path-nya `GlobalVars.<nama>`.

## 4. Program

1. **Programming → POUs → Programs → Add → ST**, namanya `P_SIM_ROBOT`.
2. Tabel variabel programnya: tempel [`ProgramVariables.tsv`](ProgramVariables.tsv).
   Kolomnya `Name`, `Data type`, `Initial value`, `Retain`, `Constant`, `Comment`
   (tabel program tidak punya kolom Network Publish). Kalau susunan kolom di versi
   Studio-mu berbeda, ketik 24 barisnya manual — jangan tempel yang kolomnya melenceng,
   Studio menerimanya tanpa keluhan dan yang salah baru ketahuan waktu Build.
3. Badan programnya: copy-paste [`P_SIM_ROBOT.st`](P_SIM_ROBOT.st).
4. **Task Settings → PrimaryTask → Program Assignment → tambahkan `P_SIM_ROBOT`.**
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

```bash
node tools/opcua/browse.js --anon --filter SIM_
```

Yang dicari bukan "daftarnya keluar", tapi tiga hal:

| | |
|---|---|
| `GlobalVars.SIM_HEARTBEAT` naik | program benar-benar dieksekusi task, bukan cuma ada di project |
| `GlobalVars.SIM_JOINT_POS` = pose home (`0, 90, -90, 0`) | blok init jalan, config sampai ke controller |
| `GlobalVars.SIM_WORLD_POS` masuk akal | FK dipanggil dan hasilnya keluar |

Lalu tekan tombol dari luar:

```bash
node tools/opcua/browse.js --anon --write SIM_JOG_MODE=0
node tools/opcua/browse.js --anon --write "SIM_JOG_P[1]=true"
node tools/opcua/browse.js --anon --watch SIM_JOINT_POS SIM_WORLD_POS
```

Sesudah itu baru jalankan bridge + halaman viz — lihat [`../README.md`](../README.md).

## Kalau salah

| gejala | sebabnya hampir selalu |
|---|---|
| tag ada, semua diam, `SIM_HEARTBEAT` tetap 0 | program belum ditugaskan ke task |
| Build gagal menyebut `BLUE_ROBOT_AXIS1` | empat baris `_sAXIS_REF` di FK belum dihapus (langkah 2) |
| Build gagal "cannot assign to constant" | `PI`/`DEGREE_TO_RAD`/`RAD_TO_DEGREE` ikut ditempel tanpa kolom Constant |
| menu OPC UA abu-abu | simulator belum Run |
| klien OPC UA ditolak, pesannya seperti salah password | Security policy `None` belum dicentang |
| lengan bergerak, kecepatannya aneh | periode task ≠ `task.periode_ms` |
| variabel bernama `Name` bertipe `Data type` muncul di tabel | baris judul ikut tertempel |
