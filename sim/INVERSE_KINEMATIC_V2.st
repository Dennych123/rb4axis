// INVERSE_KINEMATIC_V2 - versi yang DIPERBAIKI.
//
// Pasangannya INVERSE_KINEMATIC di ../extract/, yang disalin verbatim dari project
// mesin dan SENGAJA dibiarkan cacat. Berkas ini yang dipakai simulasi.
//
// Apa yang dibetulkan (semuanya ada tesnya di ../tests/kin.test.js):
//
//   1. DONE berarti sesuatu. Di V1 rantai IF/ELSIF/ELSE menulis DONE:=TRUE di
//      SEMUA cabang, termasuk ELSE - jadi pemeriksaan soft limit dihitung lalu
//      dibuang, dan pemanggil yang menunggu DONE selalu dapat "sukses". Efek
//      nyatanya di mesin: DONE menempati slot ENO di ladder, jadi seluruh cabang
//      pencari langkah jog tidak pernah jalan.
//   2. Kuadran ALFA benar. V1 memakai ATAN(Z3/Y3) polos, jadi pose dengan Y3 < 0
//      meleset TEPAT 180 derajat - separuh ruang kerja menghasilkan sudut yang
//      salah tanpa satu pun tanda.
//   3. ACOS dijaga. V1 memberi ACOS argumen > 1 kalau titiknya di luar jangkauan;
//      di PLC itu error runtime, bukan angka salah. Di sini jangkauan diperiksa
//      DULU dan FB menolak dengan ERROR_ID, bukan menghitung lalu meledak.
//   4. Bagi nol dijaga. Y3 = 0 dan R = 0 sekarang cabangnya sendiri.
//   5. Batas yang KENA dilaporkan per batas (LIMIT_HIT[0..7]), bukan cuma
//      "ada masalah" - nomor batasnya yang menyebut sumbu mana dan sisi mana.
//   6. Elbow bisa dipilih. V1 selalu elbow-down (THETA_RAD2 = BETA - PI); pose
//      yang cuma bisa dicapai elbow-up tidak pernah punya solusi di sana.
//      ELBOW_UP = FALSE menghasilkan cabang yang SAMA PERSIS dengan mesin.
//
// ATAN2 tidak dipakai - tidak ada di daftar instruksi W560, jadi belum terbukti
// ter-import. Kuadran dibetulkan lewat ATAN + koreksi eksplisit, dan port JS-nya
// melakukan hal yang sama supaya hasilnya bisa diadu.
//
// AKIBAT yang harus diketahui sebelum menyetel soft limit: koreksi ini menghasilkan
// ALFA di jangkauan (-PI/2, 3PI/2], bukan (-PI, PI] seperti ATAN2. Pose yang
// menjangkau ke BELAKANG karena itu keluar sebagai THETA1 sekitar +200 derajat,
// bukan -160. Posenya sama; yang dibandingkan ke PD1300_002/003 angkanya, jadi yang
// kena batas atas, bukan batas bawah. Kalau -160 yang diinginkan, normalisasinya
// ditambahkan SEKALI di sini dan batasnya ikut ditinjau - bukan ditambal di
// pemanggil, karena di situ dia jadi aturan kedua yang tidak kelihatan dari FB.
//
// ERROR_ID: 0 tidak ada, 1 di luar jangkauan (R > L2+L3), 2 terlalu dekat
//           (R < |L2-L3|), 3 singular (R = 0), 4 soft limit ditembus.

DONE := FALSE;
ERROR := FALSE;
ERROR_ID := 0;

IF EXECUTE THEN

	// ---------------------------------------------------------------- tool polar
	TY := ROBOT_TOOL_Y_LREAL;
	TZ := ROBOT_TOOL_Z_LREAL;
	R_TOOL := SQRT(TY * TY + TZ * TZ);
	IF R_TOOL = 0.0 THEN
		TH_TOOL := 0.0;
	ELSIF TY > 0.0 THEN
		TH_TOOL := ATAN(TZ / TY);
	ELSIF TY < 0.0 THEN
		TH_TOOL := ATAN(TZ / TY) + PI;
	ELSIF TZ >= 0.0 THEN
		TH_TOOL := PI / 2.0;
	ELSE
		TH_TOOL := -PI / 2.0;
	END_IF;

	// --------------------------------------------- pose -> titik pergelangan
	TH_EE := ROBOT_POS_INPUT[3] * DEGREE_TO_RAD;
	Y_EE := ROBOT_POS_INPUT[1] - R_TOOL * COS(TH_TOOL + TH_EE);
	Z_EE := ROBOT_POS_INPUT[2] - R_TOOL * SIN(TH_TOOL + TH_EE);
	Y3 := Y_EE - ROBOT_L4_LREAL * COS(TH_EE);
	Z3 := (Z_EE - ROBOT_L1_LREAL) - ROBOT_L4_LREAL * SIN(TH_EE);
	R := SQRT(Y3 * Y3 + Z3 * Z3);

	// --------------------------------------------------- jangkauan DULU, baru ACOS
	// Urutannya yang penting: memeriksa sesudah ACOS tidak menolong sama sekali,
	// karena yang meledak justru ACOS-nya.
	IF R > (ROBOT_L2_LREAL + ROBOT_L3_LREAL) THEN
		ERROR := TRUE;
		ERROR_ID := 1;
	ELSIF R < ABS(ROBOT_L2_LREAL - ROBOT_L3_LREAL) THEN
		ERROR := TRUE;
		ERROR_ID := 2;
	ELSIF R = 0.0 THEN
		ERROR := TRUE;
		ERROR_ID := 3;
	ELSE
		BETA := ACOS((ROBOT_L2_LREAL * ROBOT_L2_LREAL + ROBOT_L3_LREAL * ROBOT_L3_LREAL - R * R)
			/ (2.0 * ROBOT_L2_LREAL * ROBOT_L3_LREAL));
		GAMMA := ACOS((R * R + ROBOT_L2_LREAL * ROBOT_L2_LREAL - ROBOT_L3_LREAL * ROBOT_L3_LREAL)
			/ (2.0 * ROBOT_L2_LREAL * R));

		// Kuadran ALFA - ini inti perbaikan nomor 2.
		IF Y3 > 0.0 THEN
			ALFA := ATAN(Z3 / Y3);
		ELSIF Y3 < 0.0 THEN
			ALFA := ATAN(Z3 / Y3) + PI;
		ELSIF Z3 >= 0.0 THEN
			ALFA := PI / 2.0;
		ELSE
			ALFA := -PI / 2.0;
		END_IF;

		IF ELBOW_UP THEN
			THETA_RAD1 := ALFA - GAMMA;
			THETA_RAD2 := PI - BETA;
		ELSE
			// Cabang yang dipakai mesin.
			THETA_RAD1 := ALFA + GAMMA;
			THETA_RAD2 := BETA - PI;
		END_IF;
		THETA_RAD3 := TH_EE - THETA_RAD1 - THETA_RAD2;

		THETA_DEGREE1 := THETA_RAD1 * RAD_TO_DEGREE;
		THETA_DEGREE2 := THETA_RAD2 * RAD_TO_DEGREE;
		THETA_DEGREE3 := THETA_RAD3 * RAD_TO_DEGREE;

		ROBOT_POS_OUTPUT[0] := ROBOT_POS_INPUT[0] - ROBOT_ROBOT_ORG_OFFSET_LREAL[0];
		ROBOT_POS_OUTPUT[1] := THETA_DEGREE1 - ROBOT_ROBOT_ORG_OFFSET_LREAL[1];
		ROBOT_POS_OUTPUT[2] := THETA_DEGREE2 - ROBOT_ROBOT_ORG_OFFSET_LREAL[2];
		ROBOT_POS_OUTPUT[3] := THETA_DEGREE3 - ROBOT_ROBOT_ORG_OFFSET_LREAL[3];

		// ------------------------------------------------------------ soft limit
		// Perbandingannya sama persis dengan V1 (min+1 dan max-1, satu satuan di
		// tiap ujung memang sengaja tidak terpakai) - yang berubah cuma HASILNYA
		// sekarang dipakai.
		LIMIT_HIT[0] := ROBOT_POS_OUTPUT[0] < (REAL_TO_LREAL(PD1300_000) + 1.0);
		LIMIT_HIT[1] := ROBOT_POS_OUTPUT[0] > (REAL_TO_LREAL(PD1300_001) - 1.0);
		LIMIT_HIT[2] := THETA_DEGREE1 < (REAL_TO_LREAL(PD1300_002) + 1.0);
		LIMIT_HIT[3] := THETA_DEGREE1 > (REAL_TO_LREAL(PD1300_003) - 1.0);
		LIMIT_HIT[4] := THETA_DEGREE2 < (REAL_TO_LREAL(PD1300_004) + 1.0);
		LIMIT_HIT[5] := THETA_DEGREE2 > (REAL_TO_LREAL(PD1300_005) - 1.0);
		LIMIT_HIT[6] := THETA_DEGREE3 < (REAL_TO_LREAL(PD1300_006) + 1.0);
		LIMIT_HIT[7] := THETA_DEGREE3 > (REAL_TO_LREAL(PD1300_007) - 1.0);

		LIMIT_ANY := FALSE;
		FOR I := 0 TO 7 DO
			IF LIMIT_HIT[I] THEN LIMIT_ANY := TRUE; END_IF;
		END_FOR;

		IF LIMIT_ANY THEN
			// Sudutnya tetap dikeluarkan supaya pemanggil bisa menunjukkan
			// "seharusnya ke sini, tapi ditolak batas" - yang ditahan cuma DONE.
			ERROR := TRUE;
			ERROR_ID := 4;
		ELSE
			DONE := TRUE;
		END_IF;
	END_IF;
END_IF;
