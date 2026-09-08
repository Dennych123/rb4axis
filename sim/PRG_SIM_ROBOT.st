// PRG_SIM_ROBOT - program simulasi robot Blurobot untuk simulator Sysmac Studio.
//
// NAMANYA JANGAN diawali `P_`. Awalan itu milik variabel sistem Sysmac (P_On,
// P_First_Run, ...), dan program bernama `P_SIM_ROBOT` DINAMAI ULANG SENDIRI oleh
// Studio waktu di-import - jadi `PR_SIM_ROBOT`, tanpa satu pun pesan. Sesudah itu
// penugasan task, dokumen, dan tiap rujukan ke nama lama menunjuk POU yang tidak ada.
//
// Ditempel ke project NX102 KOSONG bersama FORWARD_KINEMATIC_V2 dan
// INVERSE_KINEMATIC_V2. Langkah lengkapnya di SETUP.md. Yang perlu diingat sebelum
// menyunting berkas ini:
//
//   * Rumus kinematiknya TIDAK ada di sini - semuanya di dua FB itu. Program ini
//     cuma PEMANGGIL: tombol, target, motion model, gripper.
//   * FB yang dipakai versi V2 (sudah dibetulkan). Yang verbatim dari mesin ada di
//     ../extract/ dan sengaja TIDAK di-instance - dia catatan, bukan bagian sim.
//     Daftar bedanya di ../extract/ANALYSIS.md dan di kepala kedua berkas V2.
//   * Penjaga jangkauan ADA DI DALAM IK V2, bukan di sini. Program cuma membaca
//     DONE dan ERROR_ID-nya. Dua penjaga untuk satu hal pasti berbeda pendapat.
//   * TIDAK ada MC_* sama sekali. Posisi sumbu diintegrasi sendiri, jadi project
//     sim tidak butuh setting axis, axes group, maupun EtherCAT.
//   * TCP di UJUNG JARI gripper: panjang gripper sudah dijumlahkan ke
//     ROBOT_TOOL_Y_LREAL waktu blok init dibangkitkan. Jangan menjumlahkannya lagi.
//
// Satuan: sumbu 0 mm (rel linear), sumbu 1-3 derajat (rantai planar di bidang Y-Z).

// ---------------------------------------------------------------- inisialisasi
// Nilai dari blurobot/sim/robot.config.json. Sengaja hanya scan pertama: sesudah
// itu semuanya boleh ditulis dari luar lewat OPC UA (halaman viz mengubah dimensi
// tanpa build ulang). SIM_RESET mengembalikannya ke nilai config.
IF SIM_RESET THEN
	SIM_INIT_DONE := FALSE;
	SIM_RESET := FALSE;
END_IF;

IF NOT SIM_INIT_DONE THEN
	// <<< DIBANGKITKAN dari robot.config.json oleh tools/gen_sim.js - jangan sunting tangan
	ROBOT_L1_LREAL := 400.0;
	ROBOT_L2_LREAL := 300.0;
	ROBOT_L3_LREAL := 250.0;
	ROBOT_L4_LREAL := 100.0;

	// tool.Y (50.0) + gripper.panjang (90.0) - TCP di ujung jari
	ROBOT_TOOL_Y_LREAL := 140.0;
	ROBOT_TOOL_Z_LREAL := 0.0;

	SIM_GRIP_LEN := 90.0;
	SIM_GRIP_STROKE := 80.0;
	SIM_GRIP_VEL := 120.0;
	SIM_GRIP_POS := 80.0;

	ROBOT_ROBOT_ORG_OFFSET_LREAL[0] := 0.0;
	ROBOT_ROBOT_ORG_OFFSET_LREAL[1] := 0.0;
	ROBOT_ROBOT_ORG_OFFSET_LREAL[2] := 0.0;
	ROBOT_ROBOT_ORG_OFFSET_LREAL[3] := 0.0;
	ROBOT_ROBOT_ORG_OFFSET_LREAL[4] := 0.0;

	PD1300_000 := -500.0;
	PD1300_001 := 500.0;
	PD1300_002 := -90.0;
	PD1300_003 := 180.0;
	PD1300_004 := -150.0;
	PD1300_005 := 0.0;
	PD1300_006 := -120.0;
	PD1300_007 := 120.0;

	SIM_VEL[0] := 200.0;
	SIM_VEL[1] := 30.0;
	SIM_VEL[2] := 30.0;
	SIM_VEL[3] := 45.0;

	SIM_VEL_W[0] := 100.0;
	SIM_VEL_W[1] := 100.0;
	SIM_VEL_W[2] := 100.0;
	SIM_VEL_W[3] := 30.0;

	SIM_HOME[0] := 0.0;
	SIM_HOME[1] := 90.0;
	SIM_HOME[2] := -90.0;
	SIM_HOME[3] := 0.0;

	SIM_DT := 0.004;
	SIM_JOG_STEP := 10.0;
	// >>> DIBANGKITKAN

	FOR i := 0 TO 3 DO
		SIM_JOINT_POS[i] := SIM_HOME[i];
		SIM_JOINT_CMD[i] := SIM_HOME[i];
	END_FOR;

	SIM_ERROR := FALSE;
	SIM_ERROR_ID := 0;
	SIM_INIT_DONE := TRUE;
END_IF;

// ------------------------------------------------------------------ tepi tombol
// Jog di mesin asli itu LANGKAH per tekan, bukan tahan-jalan: cabang pencari
// langkah di ladder (JOG_WORLD_STEP + 1) digerbang DONE milik IK, dan DONE itu
// selalu TRUE, jadi langkahnya tidak pernah tumbuh dan tetap 10. Perilaku itu yang
// ditiru di sini. SIM_JOG_HOLD menyalakan mode tahan-jalan - itu TAMBAHAN untuk
// viz, bukan sifat mesin.
FOR i := 0 TO 3 DO
	EDGE_P[i] := SIM_JOG_P[i] AND NOT LAST_P[i];
	EDGE_N[i] := SIM_JOG_N[i] AND NOT LAST_N[i];
	LAST_P[i] := SIM_JOG_P[i];
	LAST_N[i] := SIM_JOG_N[i];
END_FOR;
EDGE_MOVE := SIM_MOVE_EXEC AND NOT LAST_MOVE;
LAST_MOVE := SIM_MOVE_EXEC;
EDGE_HOME := SIM_HOME_EXEC AND NOT LAST_HOME;
LAST_HOME := SIM_HOME_EXEC;

// Sumbu mana yang ditekan, dan sebesar apa langkahnya. Satu sumbu sekali jalan:
// dua tombol sekaligus di mesin asli juga saling mengunci lewat latch START_*.
AX := -1;
DELTA := 0.0;
FOR i := 0 TO 3 DO
	IF SIM_JOG_P[i] AND (EDGE_P[i] OR SIM_JOG_HOLD) THEN
		AX := i;
		DELTA := SIM_JOG_STEP;
		IF SIM_JOG_HOLD THEN
			// Mode tahan: kecepatan x dt. DUA daftar, karena satuannya memang beda -
			// SIM_VEL derajat/s pada SUMBU, SIM_VEL_W mm/s pada KOORDINAT world.
			// Satu daftar buat dua-duanya bikin jog world kecepatannya ikut sumbu yang
			// kebetulan senomor, dan itu angka yang tidak ada artinya.
			IF SIM_JOG_MODE = 0 THEN DELTA := SIM_VEL[i] * SIM_DT;
			ELSE DELTA := SIM_VEL_W[i] * SIM_DT; END_IF;
		END_IF;
	ELSIF SIM_JOG_N[i] AND (EDGE_N[i] OR SIM_JOG_HOLD) THEN
		AX := i;
		DELTA := -SIM_JOG_STEP;
		IF SIM_JOG_HOLD THEN
			IF SIM_JOG_MODE = 0 THEN DELTA := -SIM_VEL[i] * SIM_DT;
			ELSE DELTA := -SIM_VEL_W[i] * SIM_DT; END_IF;
		END_IF;
	END_IF;
END_FOR;

// ------------------------------------------------------------ pose yang diminta
// POSE_REQ diisi kalau ada permintaan berbasis WORLD (jog world/tool atau move
// point). NEED_IK menandai bahwa IK perlu dijalankan scan ini.
NEED_IK := FALSE;

IF (AX >= 0) AND (SIM_JOG_MODE = 0) THEN
	// Jog JOINT: tidak lewat IK sama sekali.
	SIM_JOINT_CMD[AX] := SIM_JOINT_CMD[AX] + DELTA;

ELSIF AX >= 0 THEN
	FOR i := 0 TO 3 DO
		POSE_REQ[i] := REAL_TO_LREAL(SIM_WORLD_POS[i]);
	END_FOR;
	IF SIM_JOG_MODE = 1 THEN
		// Jog WORLD: satu sumbu world digeser apa adanya (X, Y, Z, theta_EE).
		POSE_REQ[AX] := POSE_REQ[AX] + DELTA;
	ELSE
		// Jog TOOL: sumbu 1 maju searah tool, sumbu 2 tegak lurus terhadapnya.
		// Penyiap pose kandidat di project asli TIDAK ada di ladder mana pun
		// (kemungkinan ditulis HMI Pro-face), jadi arti tiap tombol tool di sini
		// DITETAPKAN oleh sim - dan ini catatannya, bukan hasil ekstrak.
		TH := POSE_REQ[3] * DEGREE_TO_RAD;
		CASE AX OF
			0: POSE_REQ[0] := POSE_REQ[0] + DELTA;
			1: POSE_REQ[1] := POSE_REQ[1] + DELTA * COS(TH);
			   POSE_REQ[2] := POSE_REQ[2] + DELTA * SIN(TH);
			2: POSE_REQ[1] := POSE_REQ[1] - DELTA * SIN(TH);
			   POSE_REQ[2] := POSE_REQ[2] + DELTA * COS(TH);
			3: POSE_REQ[3] := POSE_REQ[3] + DELTA;
		END_CASE;
	END_IF;
	NEED_IK := TRUE;

ELSIF EDGE_MOVE THEN
	FOR i := 0 TO 3 DO
		POSE_REQ[i] := SIM_WORLD_CMD[i];
	END_FOR;
	NEED_IK := TRUE;

ELSIF EDGE_HOME THEN
	FOR i := 0 TO 3 DO
		SIM_JOINT_CMD[i] := SIM_HOME[i];
	END_FOR;
END_IF;

// ------------------------------------------------------ inverse kinematic
// Penjaga jangkauan ADA DI DALAM INVERSE_KINEMATIC_V2, bukan di sini lagi. Dulu
// program ini menghitung sendiri Y3/Z3/R buat memutuskan boleh atau tidak - dua
// penjaga untuk satu hal, dan yang di luar tidak pernah tahu rumus di dalam. Yang
// dipakai sekarang jawaban FB-nya sendiri: DONE, ERROR_ID.
IF NEED_IK THEN
	IK2(EXECUTE := TRUE, ROBOT_POS_INPUT := POSE_REQ, ELBOW_UP := SIM_ELBOW_UP);
	// Array keluaran FB disalin UTUH dulu ke variabel lokal, baru diindeks.
	// Studio MENOLAK IK2.ROBOT_POS_OUTPUT[i]: "Cannot use an element of array or a
	// member of structure for the reference of function block instance variables."
	// Anggota skalar (IK2.DONE, IK2.ERROR_ID) tidak kena aturan itu - yang dilarang
	// mengindeks anggota milik instance.
	IK_OUT := IK2.ROBOT_POS_OUTPUT;
	IF IK2.DONE THEN
		FOR i := 0 TO 3 DO
			SIM_JOINT_CMD[i] := IK_OUT[i];
		END_FOR;
		SIM_ERROR := FALSE;
		SIM_ERROR_ID := 0;
	ELSE
		// Perintahnya DIBUANG, posisi sekarang dipertahankan. Memakai keluaran FB
		// yang DONE-nya FALSE berarti lengan melompat ke pose yang robotnya tidak
		// pernah bisa capai - dan di layar itu tampak seperti gerakan biasa.
		SIM_ERROR := TRUE;
		SIM_ERROR_ID := IK2.ERROR_ID;
	END_IF;
END_IF;

// ------------------------------------------------------------------ gripper
// Dua jari, bukaan = jarak antar jari. Modelnya sama dengan sumbu: didorong ke
// target dengan batas kecepatan. Panjang gripper TIDAK diurus di sini - dia sudah
// ikut di ROBOT_TOOL_Y_LREAL, jadi TCP kinematiknya memang di ujung jari.
IF SIM_GRIP_CMD THEN GRIP_TARGET := 0.0; ELSE GRIP_TARGET := SIM_GRIP_STROKE; END_IF;
D := GRIP_TARGET - SIM_GRIP_POS;
STEP_M := SIM_GRIP_VEL * SIM_DT;
IF ABS(D) <= STEP_M THEN
	SIM_GRIP_POS := GRIP_TARGET;
ELSIF D > 0.0 THEN
	SIM_GRIP_POS := SIM_GRIP_POS + STEP_M;
ELSE
	SIM_GRIP_POS := SIM_GRIP_POS - STEP_M;
END_IF;
SIM_GRIP_CLOSED := SIM_GRIP_POS <= 0.0;
SIM_GRIP_OPEN := SIM_GRIP_POS >= SIM_GRIP_STROKE;

// ------------------------------------------------------------ motion model
// Pengganti MC_*: tiap sumbu didorong ke target dengan batas kecepatan sendiri.
// Tanpa profil trapesium - yang dinilai viz ini kinematiknya, bukan dinamikanya.
SIM_BUSY := FALSE;
FOR i := 0 TO 3 DO
	STEP_M := SIM_VEL[i] * SIM_DT;
	D := SIM_JOINT_CMD[i] - SIM_JOINT_POS[i];
	IF ABS(D) <= STEP_M THEN
		SIM_JOINT_POS[i] := SIM_JOINT_CMD[i];
	ELSIF D > 0.0 THEN
		SIM_JOINT_POS[i] := SIM_JOINT_POS[i] + STEP_M;
		SIM_BUSY := TRUE;
	ELSE
		SIM_JOINT_POS[i] := SIM_JOINT_POS[i] - STEP_M;
		SIM_BUSY := TRUE;
	END_IF;
END_FOR;
SIM_MOVE_DONE := NOT SIM_BUSY;

// ------------------------------------------------------ forward kinematic + batas
// FK dipanggil TIAP SCAN dengan EXECUTE TRUE. Di V2 pin itu benar-benar dibaca
// (di V1 tidak), jadi menaruh FALSE di sini berarti pose world berhenti diperbarui -
// lengan tetap bergerak, angkanya membeku.
FK2(EXECUTE := TRUE, ROBOT_POS_INPUT := SIM_JOINT_POS);
FK_WORLD := FK2.ROBOT_POS_WORLD_OUTPUT;
FK_JOINT := FK2.ROBOT_POS_JOINT_OUTPUT;
FOR i := 0 TO 3 DO
	SIM_WORLD_POS[i] := FK_WORLD[i];
	SIM_JOINT_OUT[i] := FK_JOINT[i];
END_FOR;

// Batas dinilai atas posisi SEKARANG, bukan atas pose yang diminta - yang diminta
// sudah dinilai IK V2 sendiri lewat LIMIT_HIT. Dua-duanya perlu: yang satu menolak
// perintah, yang satu memberi tahu di mana lengannya berdiri sekarang.
// Perbandingannya sama persis dengan ST asli (min+1 dan max-1).
SIM_LIMIT[0] := SIM_JOINT_POS[0] < (REAL_TO_LREAL(PD1300_000) + 1.0);
SIM_LIMIT[1] := SIM_JOINT_POS[0] > (REAL_TO_LREAL(PD1300_001) - 1.0);
SIM_LIMIT[2] := SIM_JOINT_POS[1] < (REAL_TO_LREAL(PD1300_002) + 1.0);
SIM_LIMIT[3] := SIM_JOINT_POS[1] > (REAL_TO_LREAL(PD1300_003) - 1.0);
SIM_LIMIT[4] := SIM_JOINT_POS[2] < (REAL_TO_LREAL(PD1300_004) + 1.0);
SIM_LIMIT[5] := SIM_JOINT_POS[2] > (REAL_TO_LREAL(PD1300_005) - 1.0);
SIM_LIMIT[6] := SIM_JOINT_POS[3] < (REAL_TO_LREAL(PD1300_006) + 1.0);
SIM_LIMIT[7] := SIM_JOINT_POS[3] > (REAL_TO_LREAL(PD1300_007) - 1.0);

SIM_HEARTBEAT := SIM_HEARTBEAT + 1;
