# Blurobot ECU — apa yang ada di project mesin, dan apa yang diangkat

Ditulis tangan (bukan hasil `extract.js`). Isinya hal-hal yang TIDAK kelihatan dari
berkas hasil ekstrak, dan yang bakal salah ditebak kalau tidak dicatat.

Sumber: `C:\Users\denny\Downloads\Blurobot ECU\Blurobot ECU\BLUEROBOT ECU 28032020.smc2`
(Sysmac Studio, controller **NJ501**, dibuat 28 Maret 2020, author `Rewat Bunchan`).
Berkas itu **hanya dibaca**; tidak pernah ditulis oleh apa pun di folder ini.

## Isi project asli

12 program, 58 section, 1219 rung, 6345 variabel, 4 sumbu `BLUE_ROBOT_AXIS1..4`
+ 1 axes group.

| program | rung | peran |
|---|---|---|
| `P100_PROG_MOVE` | 102 | interpreter program teach: `OperandLabel`, `OperandGoto`, `OperandMove`, `OperandIO` |
| `P101_EDIT_PROG` | 55 | sunting langkah program dari teach pendant |
| `P102_EDIT_POS` | 14 | sunting posisi titik |
| `P103_SETTING`, `P107_RB_SETUP` | 2 | setelan |
| `P105_RB_MOTION` | 135 | **inti robot**: Main, JogJoint, JogWorld, JogTool, OriginSet, HomeReturn, SoftLimit, TorqueLimit, Fault |
| `P106_RB_IO` | 38 | IO robot |
| `P000_HMI`, `P001_MAIN` | 528 | HMI, master/auto, fault, timer, counter |
| `RB_MOVING` | 320 | sekuens station (WIP 1-5, DW1/DW2, ICC tester 1/2) |
| `WIP_MOVING` | 25 | sekuens WIP |

HMI-nya **Pro-face GP-Pro EX** (`.prx`), bukan NB-Designer. Repo ini tidak punya
pembacanya, dan untuk algoritma memang tidak dibutuhkan.

**Yang diangkat ke sim cuma dua FB kinematik + pemakaian jog/move point.**
Interpreter program teach (`ROBOT_PROG01 : ARRAY[0..19,0..199,0..9] OF UINT`) dan
seluruh sekuens station TIDAK diangkat — keputusan sadar, bukan kelupaan.

## Bentuk robotnya: 1 prismatik + 3 revolute, BUKAN 4R

Terbaca dari rumusnya sendiri, bukan dari nama variabel:

```
FK[0] := ACTUAL_POS_DEGREE[0];                          <- lewat apa adanya
FK[1] := L2*COS(t1) + L3*COS(t1+t2) + L4*COS(t1+t2+t3);
FK[2] := L1 + L2*SIN(t1) + L3*SIN(t1+t2) + L4*SIN(t1+t2+t3);
FK[3] := t1_deg + t2_deg + t3_deg;
```

Sumbu 0 tidak pernah masuk fungsi trigonometri mana pun dan langsung jadi X ⇒ itu
**sumbu linear** (rel). Sumbu 1–3 rantai planar di bidang **Y–Z**, dengan `L1`
sebagai tinggi bahu. `FK[3]` sudut end-effector = jumlah tiga sudut (rantai planar).

Tool digeser **polar**, bukan kartesian: `R_TOOL = √(toolY² + toolZ²)`,
`THETA_TOOL = ATAN(toolZ/toolY)`, lalu diputar bersama `THETA_EE`.
`ROBOT_TOOL_X` **tidak dipakai rumus mana pun** — dia dideklarasi, tapi tidak pernah
dibaca. Jangan menghabiskan waktu mencari di mana X tool berpengaruh: tidak ada.

Satuan: sumbu 0 mm, sumbu 1–3 derajat (konversi lewat `DEGREE_TO_RAD` di dalam FB).

## Siapa memanggil apa

| pemanggil | FB | jumlah instance |
|---|---|---|
| `P105_RB_MOTION/JogWorld` | `INVERSE_KINEMATIC` | 9 |
| `P105_RB_MOTION/JogTool` | `INVERSE_KINEMATIC` | 9 |
| `P100_PROG_MOVE/OperandMove` | `INVERSE_KINEMATIC` | 5 (`..._PROFILE1..4` + 1) |
| `P105_RB_MOTION/HomeReturn` | `INVERSE_KINEMATIC` | 1 |
| `P102_EDIT_POS/MovePoint` | `INVERSE_KINEMATIC` | 1 |
| `P105_RB_MOTION/Main` | `FORWARD_KINEMATIC` | 1 |

Sembilan instance IK per mode jog itu **satu instance per arah/sumbu**, bukan sembilan
rumus berbeda. Di `PRG_SIM_ROBOT.st` semuanya jadi SATU panggilan berparameter. Yang
berubah cuma jumlah instance; rumusnya tidak disentuh sama sekali.

Gerak sumbunya di project asli lewat `MC_*` (`MC_Power` ×4, `MC_MoveJog` ×5,
`MC_Home` ×5, `MC_MoveLinear`, `MC_MoveAbsolute`, `MC_GroupEnable/Disable/Stop/Reset`,
`MC_Reset` ×4). **Sim TIDAK memakai `MC_*`** — posisi sumbu diintegrasi di ST. Itu
yang membuat project sim tidak butuh setting axis, axes group, maupun EtherCAT, dan
tidak ada satu pun titik gagal yang datang dari konfigurasi motion.

## Empat cacat di algoritma asli

Berkas di folder ini **tetap verbatim** — keempat cacat di bawah masih ada di sini,
dan punya tes yang MENUNTUT perilaku aslinya (`tests/kin.test.js`). Itu yang
membuat folder ini bisa diadu ke project mesin kapan pun.

Yang DIPAKAI simulasi adalah versi perbaikannya, `sim/FORWARD_KINEMATIC_V2.st`
dan `INVERSE_KINEMATIC_V2.st` — berdampingan, bukan menggantikan. Tesnya berpasangan:
satu menuntut perilaku salah (V1), satu menuntut yang benar (V2), plus satu yang
membuktikan keduanya memang berbeda di pose yang sama. Kalau nanti mesin aslinya mau
ikut dibetulkan, selisih itu yang jadi buktinya.

| cacat | V1 (mesin) | V2 (sim) |
|---|---|---|
| `DONE` | selalu TRUE | TRUE hanya kalau solusinya sah dan di dalam soft limit |
| kuadran `ALFA` | `ATAN` polos, `Y3 < 0` meleset 180° | `ATAN` + koreksi kuadran eksplisit |
| `ACOS` | tanpa penjaga, argumen bisa > 1 | jangkauan diperiksa DULU, ditolak lewat `ERROR_ID` |
| `EXECUTE`/`DONE` di FK | tidak pernah disentuh | dibaca dan ditulis |
| elbow | selalu elbow-down | `ELBOW_UP` bisa dipilih; FALSE = cabang mesin |
| tool | menimpa variabel global waktu `toolY = 0` | dibaca ke lokal, global tidak disentuh |

`ATAN2` **tidak** dipakai di V2: instruksi itu tidak ada di daftar 353 instruksi W560,
jadi belum terbukti ter-import. Kuadrannya dibetulkan dengan `ATAN` + koreksi, dan port
JS-nya melakukan hal yang sama supaya hasil PLC dan hasil JS tetap bisa diadu.

### 1. `INVERSE_KINEMATIC.DONE` selalu TRUE

```
IF   ROBOT_POS_INPUT[0] < (PD1300_000 + 1) OR ... THEN DONE:=TRUE;
ELSIF ...                                              DONE:=TRUE;
ELSE                                                   DONE:=TRUE;
END_IF;
```

Seluruh rantai pemeriksa soft limit `PD1300_000..007` menulis `TRUE` di **semua**
cabang, `ELSE` termasuk. Jadi batasnya dihitung tapi hasilnya tidak pernah sampai ke
siapa pun: pemanggil yang menunggu `DONE` selalu dapat "sukses". Di sim, batas yang
kena dilaporkan lewat `SIM_LIMIT[0..7]` — jalur BARU, bukan `DONE`.

### 2. `ALFA := ATAN(Z3/Y3)` kehilangan kuadran

`ATAN`, bukan `ATAN2`. Pose dengan `Y3 < 0` (lengan menjangkau ke sisi negatif Y)
menghasilkan sudut yang meleset **tepat 180°** — sudah diukur di tes, bukan perkiraan.
Penjaga bagi-nol cuma ada untuk `ROBOT_TOOL_Y_LREAL`, tidak untuk `Y3` sendiri.

Di mesin nyata ini tidak pernah kelihatan karena jangkauan kerjanya memang di
`Y > 0`. Di V2 kuadrannya dibetulkan, jadi pose ke belakang punya solusi yang benar —
dengan satu akibat yang dicatat di kepala `INVERSE_KINEMATIC_V2.st`: `THETA1` keluar
sebagai +200° dan bukan −160°, dan soft limit membandingkan angka itu apa adanya.

### 3. `ACOS` tanpa penjaga jangkauan

`BETA` dan `GAMMA` menerima argumen > 1 kalau titiknya di luar `L2+L3`. Di PLC itu
**error runtime**, bukan angka salah. Di V2 jangkauan (`R ≤ L2+L3`, `R ≥ |L2−L3|`,
`R ≠ 0`) diperiksa **sebelum** `ACOS` dipanggil dan pose yang gagal ditolak lewat
`ERROR_ID` — memeriksa sesudahnya tidak menolong, karena yang meledak `ACOS`-nya.

### 4. `FORWARD_KINEMATIC` tidak menyentuh `EXECUTE` maupun `DONE`

Kedua pin itu ada di antarmuka, tidak ada satu pun di badan ST. Artinya FK menghitung
**tiap kali dipanggil** (`EXECUTE` tidak berpengaruh) dan `DONE`-nya tidak pernah
menjadi TRUE. Rung yang menunggu `DONE` FK bakal menunggu selamanya.

Efek samping lain yang gampang terlewat: penjaga bagi-nol di kedua FB **menulis balik
ke variabel global** (`ROBOT_TOOL_Y_LREAL := 0.000000000000001`). Sekali `toolY`
diisi 0, nilainya berubah permanen di controller, bukan cuma di dalam FB.

## Yang project TIDAK simpan

`ROBOT_L1..L4` dan `ROBOT_TOOL_X/Y/Z` **tidak punya nilai awal** dan tidak pernah
ditulis satu rung pun — sudah dicek lewat semua pin blok fungsi di 1219 rung.
Semuanya `REAL` retain yang diisi dari HMI Pro-face waktu mesin dipasang, lalu
di-`REAL_TO_LREAL` ke pasangan `*_LREAL`-nya.

Gripper juga tidak ada di project mesin sama sekali — itu tambahan sim. Panjangnya
dijumlahkan ke `ROBOT_TOOL_Y_LREAL` **satu kali**, waktu `gen_sim.js` menulis blok
init, jadi TCP kinematiknya di ujung jari. Menjumlahkannya lagi di viz atau di rung
bikin lengan panjang dua kali gripper, dan di layar itu cuma tampak seperti lengan
yang sedikit lebih panjang.

Konsekuensinya untuk sim: angkanya **placeholder** di
[`../sim/robot.config.json`](../sim/robot.config.json) sampai ada yang mengukur robot
aslinya. Dimensinya ikut ter-publish ke OPC UA, jadi viz 3D menggambar apa yang PLC
pegang — ganti config, gambarnya ikut, tanpa menyunting halaman.

`PD1300_000..007` (soft limit) juga retain tanpa nilai awal: pasangan (min, max) per
sumbu, dan ST membandingkan dengan `min+1` / `max-1` — satu satuan di tiap ujung
memang sengaja tidak terpakai.
