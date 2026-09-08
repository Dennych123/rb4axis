# Antarmuka FB kinematik Blurobot

DIBANGKITKAN oleh `blurobot/tools/extract.js` - jangan disunting tangan.
Sumber: `BLUEROBOT ECU 28032020.smc2`.

Kolom `variables.tsv` = urutan tabel Global Variable Studio, TANPA baris judul
(judul yang ikut tertempel mendarat sebagai variabel bernama `Name` bertipe `Data type`):
`Name`, `Data type`, `Initial value`, `AT`, `Retain`, `Constant`, `Network Publish`, `Comment`.

Nilai awal KOSONG berarti project asli memang tidak menyimpannya - dimensi robot
diisi dari HMI waktu mesin dipakai, jadi untuk simulasi angkanya datang dari
`blurobot/sim/robot.config.json`.

## FORWARD_KINEMATIC

Badan ST: [`FORWARD_KINEMATIC.st`](FORWARD_KINEMATIC.st) - 35 baris, akhiran baris CRLF (punya Studio, jangan diubah).

### VAR_INPUT / VAR_OUTPUT (pin)

| Ord | Nama | Tipe | Const | Dipakai badan ST |
|---|---|---|---|---|
| 0 | `EXECUTE` | BOOL |  | **TIDAK** |
| 1 | `ROBOT_POS_INPUT` | ARRAY[0..3] OF LREAL |  | ya |
| 2 | `DONE` | BOOL |  | **TIDAK** |
| 3 | `ROBOT_POS_JOINT_OUTPUT` | ARRAY[0..3] OF REAL |  | ya |
| 4 | `ROBOT_POS_WORLD_OUTPUT` | ARRAY[0..3] OF REAL |  | ya |

### VAR (temp di dalam FB)

| Nama | Tipe | Const | Dipakai badan ST |
|---|---|---|---|
| `THETA_DEGREE1` | LREAL |  | **TIDAK** |
| `THETA_EE_RAD` | LREAL |  | ya |
| `THETA_DEGREE2` | LREAL |  | **TIDAK** |
| `THETA_DEGREE3` | LREAL |  | **TIDAK** |
| `THETA_RAD1` | LREAL |  | **TIDAK** |
| `THETA_RAD2` | LREAL |  | **TIDAK** |
| `THETA_RAD3` | LREAL |  | **TIDAK** |
| `ACTUAL_POS_DEGREE` | ARRAY[0..3] OF LREAL |  | ya |
| `ACTUAL_POS_RAD` | ARRAY[0..3] OF LREAL |  | ya |
| `ACTUAL_POS_FORWARD_KINEMATIC` | ARRAY[0..3] OF LREAL |  | ya |
| `R_TOOL` | LREAL |  | ya |
| `THETA_TOOL` | LREAL |  | ya |

### VAR_EXTERNAL (harus ada sebagai variabel global)

| Nama | Tipe | Const | Dipakai badan ST |
|---|---|---|---|
| `ROBOT_L4_LREAL` | LREAL | ya | ya |
| `DEGREE_TO_RAD` | LREAL | ya | ya |
| `ROBOT_L2_LREAL` | LREAL | ya | ya |
| `ROBOT_L3_LREAL` | LREAL | ya | ya |
| `PI` | LREAL | ya | **TIDAK** |
| `RAD_TO_DEGREE` | LREAL | ya | **TIDAK** |
| `ROBOT_ROBOT_ORG_OFFSET_LREAL` | ARRAY[0..4] OF LREAL |  | **TIDAK** |
| `BLUE_ROBOT_AXIS1` | _sAXIS_REF | ya | **TIDAK** |
| `BLUE_ROBOT_AXIS2` | _sAXIS_REF | ya | **TIDAK** |
| `BLUE_ROBOT_AXIS3` | _sAXIS_REF | ya | **TIDAK** |
| `BLUE_ROBOT_AXIS4` | _sAXIS_REF | ya | **TIDAK** |
| `ROBOT_L1_LREAL` | LREAL |  | ya |
| `ROBOT_TOOL_Y_LREAL` | LREAL |  | ya |
| `ROBOT_TOOL_Z_LREAL` | LREAL |  | ya |

## INVERSE_KINEMATIC

Badan ST: [`INVERSE_KINEMATIC.st`](INVERSE_KINEMATIC.st) - 41 baris, akhiran baris CRLF (punya Studio, jangan diubah).

### VAR_INPUT / VAR_OUTPUT (pin)

| Ord | Nama | Tipe | Const | Dipakai badan ST |
|---|---|---|---|---|
| 0 | `EXECUTE` | BOOL |  | **TIDAK** |
| 1 | `ROBOT_POS_INPUT` | ARRAY[0..3] OF LREAL |  | ya |
| 2 | `DONE` | BOOL |  | ya |
| 3 | `ROBOT_POS_OUTPUT` | ARRAY[0..3] OF LREAL |  | ya |

### VAR (temp di dalam FB)

| Nama | Tipe | Const | Dipakai badan ST |
|---|---|---|---|
| `Y3` | LREAL |  | ya |
| `Z3` | LREAL |  | ya |
| `THETA_TOOL` | LREAL |  | ya |
| `R_TOOL` | LREAL |  | ya |
| `Y_EE` | LREAL |  | ya |
| `Z_EE` | LREAL |  | ya |
| `R` | LREAL |  | ya |
| `ALFA` | LREAL |  | ya |
| `BETA` | LREAL |  | ya |
| `GAMMA` | LREAL |  | ya |
| `THETA_DEGREE1` | LREAL |  | ya |
| `THETA_DEGREE2` | LREAL |  | ya |
| `THETA_DEGREE3` | LREAL |  | ya |
| `THETA_RAD1` | LREAL |  | ya |
| `THETA_RAD2` | LREAL |  | ya |
| `THETA_RAD3` | LREAL |  | ya |

### VAR_EXTERNAL (harus ada sebagai variabel global)

| Nama | Tipe | Const | Dipakai badan ST |
|---|---|---|---|
| `ROBOT_L4_LREAL` | LREAL | ya | ya |
| `DEGREE_TO_RAD` | LREAL | ya | ya |
| `ROBOT_L2_LREAL` | LREAL | ya | ya |
| `ROBOT_L3_LREAL` | LREAL | ya | ya |
| `PI` | LREAL | ya | ya |
| `RAD_TO_DEGREE` | LREAL | ya | ya |
| `ROBOT_ROBOT_ORG_OFFSET_LREAL` | ARRAY[0..4] OF LREAL |  | ya |
| `ROBOT_L1_LREAL` | LREAL |  | ya |
| `ROBOT_POSITION` | ARRAY[0..9999,0..4] OF REAL |  | **TIDAK** |
| `ROBOT_TOOL_Y_LREAL` | LREAL |  | ya |
| `ROBOT_TOOL_Z_LREAL` | LREAL |  | ya |
| `PD1300_000` | REAL |  | ya |
| `PD1300_001` | REAL |  | ya |
| `PD1300_002` | REAL |  | ya |
| `PD1300_004` | REAL |  | ya |
| `PD1300_006` | REAL |  | ya |
| `PD1300_005` | REAL |  | ya |
| `PD1300_007` | REAL |  | ya |
| `PD1300_003` | REAL |  | ya |
