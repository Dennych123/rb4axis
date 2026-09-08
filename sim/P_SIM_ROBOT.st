// P_SIM_ROBOT - program simulasi robot Blurobot untuk simulator Sysmac Studio.
//
// Ditempel ke project NX102 KOSONG bersama dua FB dari blurobot/extract/. Langkah
// lengkapnya di SETUP.md. Yang perlu diingat sebelum menyunting berkas ini:
//
//   * Rumus kinematiknya TIDAK ada di sini. Semuanya di FORWARD_KINEMATIC dan
//     INVERSE_KINEMATIC, verbatim dari project mesin. Program ini cuma PEMANGGIL.
//   * Penjaga jangkauan (REACH_OK) memang ada di pemanggil, bukan di dalam FB.
//     FB-nya sengaja dibiarkan identik dengan yang jalan di mesin, dan tanpa
//     penjaga ini ACOS di dalam IK dapat argumen > 1 -> error runtime di PLC.
//   * Soft limit dihitung ULANG di sini. Di FB aslinya hasilnya dibuang: rantai
//     IF/ELSIF/ELSE menulis DONE:=TRUE di semua cabang. Lihat extract/ANALYSIS.md.
//   * TIDAK ada MC_* sama sekali. Posisi sumbu diintegrasi sendiri, jadi project
//     sim tidak butuh setting axis, axes group, maupun EtherCAT.
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
	ROBOT_TOOL_Y_LREAL := 50.0;
	ROBOT_TOOL_Z_LREAL := 0.0;

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

// --------------------------------------------------- penjaga jangkauan + IK
IF NEED_IK THEN
	// Hitungan tool yang sama dengan di dalam FB, tapi TANPA menulis balik ke
	// ROBOT_TOOL_Y_LREAL. FB aslinya memang menimpa variabel globalnya sendiri
	// waktu toolY = 0; ditiru di sini, efeknya jadi permanen tanpa ada yang minta.
	IF ROBOT_TOOL_Y_LREAL = 0.0 THEN
		R_TOOL := ABS(ROBOT_TOOL_Z_LREAL);
		TH_TOOL := PI / 2.0;
	ELSE
		R_TOOL := SQRT(ROBOT_TOOL_Y_LREAL * ROBOT_TOOL_Y_LREAL + ROBOT_TOOL_Z_LREAL * ROBOT_TOOL_Z_LREAL);
		TH_TOOL := ATAN(ROBOT_TOOL_Z_LREAL / ROBOT_TOOL_Y_LREAL);
	END_IF;

	TH := POSE_REQ[3] * DEGREE_TO_RAD;
	Y3 := (POSE_REQ[1] - R_TOOL * COS(TH_TOOL + TH)) - ROBOT_L4_LREAL * COS(TH);
	Z3 := ((POSE_REQ[2] - R_TOOL * SIN(TH_TOOL + TH)) - ROBOT_L1_LREAL) - ROBOT_L4_LREAL * SIN(TH);
	RR := SQRT(Y3 * Y3 + Z3 * Z3);

	// Tiga syarat, tiga sebab yang beda:
	//   Y3 = 0   -> ATAN(Z3/Y3) bagi nol
	//   Y3 < 0   -> ATAN kehilangan kuadran, hasilnya meleset 180 derajat
	//   RR       -> ACOS dapat argumen > 1 kalau titiknya di luar jangkauan
	REACH_OK := (Y3 > 0.0)
		AND (RR <= (ROBOT_L2_LREAL + ROBOT_L3_LREAL))
		AND (RR >= ABS(ROBOT_L2_LREAL - ROBOT_L3_LREAL))
		AND (RR > 0.0);

	IF REACH_OK THEN
		IK1(EXECUTE := TRUE, ROBOT_POS_INPUT := POSE_REQ);
		FOR i := 0 TO 3 DO
			SIM_JOINT_CMD[i] := IK1.ROBOT_POS_OUTPUT[i];
		END_FOR;
		SIM_ERROR := FALSE;
		SIM_ERROR_ID := 0;
	ELSE
		// Perintahnya DIBUANG, posisi sekarang dipertahankan. Menjalankan IK di
		// sini tetap menghasilkan angka - angka yang salah - dan lengan di layar
		// melompat ke pose yang robotnya tidak pernah bisa capai.
		SIM_ERROR := TRUE;
		IF Y3 <= 0.0 THEN SIM_ERROR_ID := 2; ELSE SIM_ERROR_ID := 1; END_IF;
	END_IF;
END_IF;

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
// FK dipanggil TIAP SCAN. EXECUTE-nya memang tidak berpengaruh: badan FB tidak
// pernah membacanya, dan DONE-nya tidak pernah ditulis. Lihat extract/ANALYSIS.md.
FK1(EXECUTE := TRUE, ROBOT_POS_INPUT := SIM_JOINT_POS);
FOR i := 0 TO 3 DO
	SIM_WORLD_POS[i] := FK1.ROBOT_POS_WORLD_OUTPUT[i];
	SIM_JOINT_OUT[i] := FK1.ROBOT_POS_JOINT_OUTPUT[i];
END_FOR;

// Pemeriksaan yang di FB aslinya dihitung lalu dibuang. Perbandingannya dibuat
// PERSIS seperti di ST asli, min+1 dan max-1 termasuk - satu satuan di tiap ujung
// memang sengaja tidak terpakai di sana.
SIM_LIMIT[0] := SIM_JOINT_POS[0] < (REAL_TO_LREAL(PD1300_000) + 1.0);
SIM_LIMIT[1] := SIM_JOINT_POS[0] > (REAL_TO_LREAL(PD1300_001) - 1.0);
SIM_LIMIT[2] := SIM_JOINT_POS[1] < (REAL_TO_LREAL(PD1300_002) + 1.0);
SIM_LIMIT[3] := SIM_JOINT_POS[1] > (REAL_TO_LREAL(PD1300_003) - 1.0);
SIM_LIMIT[4] := SIM_JOINT_POS[2] < (REAL_TO_LREAL(PD1300_004) + 1.0);
SIM_LIMIT[5] := SIM_JOINT_POS[2] > (REAL_TO_LREAL(PD1300_005) - 1.0);
SIM_LIMIT[6] := SIM_JOINT_POS[3] < (REAL_TO_LREAL(PD1300_006) + 1.0);
SIM_LIMIT[7] := SIM_JOINT_POS[3] > (REAL_TO_LREAL(PD1300_007) - 1.0);

SIM_HEARTBEAT := SIM_HEARTBEAT + 1;
