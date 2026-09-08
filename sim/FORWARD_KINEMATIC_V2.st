// FORWARD_KINEMATIC_V2 - versi yang DIPERBAIKI.
//
// Pasangannya FORWARD_KINEMATIC di ../extract/, yang disalin verbatim dari project
// mesin dan SENGAJA dibiarkan cacat. Berkas ini yang dipakai simulasi; yang di
// extract/ tetap ada sebagai catatan apa yang benar-benar jalan di mesin.
//
// Apa yang dibetulkan di sini (semuanya ada tesnya di ../tests/kin.test.js):
//
//   1. EXECUTE dipakai. Di V1 pin itu ada di antarmuka tapi tidak pernah dibaca
//      badan ST-nya - FB menghitung tiap kali dipanggil, mau diminta atau tidak.
//   2. DONE ditulis. Di V1 tidak pernah, jadi rung yang menunggu DONE-nya FK
//      menunggu selamanya.
//   3. Tool TIDAK menulis balik ke variabel global. V1 menimpa
//      ROBOT_TOOL_Y_LREAL dengan 0.000000000000001 waktu nilainya 0, dan
//      perubahan itu bertahan sesudah FB selesai - efek samping permanen yang
//      tidak diminta siapa pun.
//   4. Sudut tool dihitung dengan kuadran yang benar. ATAN saja tidak bisa
//      membedakan tool (Y=+50, Z=0) dari (Y=-50, Z=0).
//   5. Keluaran LREAL ditambahkan. V1 cuma mengeluarkan REAL, dan pembulatan
//      32-bit itu yang bikin round-trip FK->IK tidak pernah menutup rapat.
//
// ATAN2 TIDAK dipakai: instruksi itu tidak ada di daftar 353 instruksi W560
// (docs/SYSMAC_INSTRUCTIONS.md), jadi belum terbukti ter-import. Kuadrannya
// dibetulkan dengan ATAN + koreksi eksplisit, dan port JS-nya melakukan hal yang
// sama persis - bukan Math.atan2 - supaya hasil PLC dan hasil JS bisa dibandingkan.

DONE := FALSE;

IF EXECUTE THEN

	// ---------------------------------------------------------------- tool polar
	// Dibaca ke variabel lokal. Global-nya tidak disentuh.
	TY := ROBOT_TOOL_Y_LREAL;
	TZ := ROBOT_TOOL_Z_LREAL;
	R_TOOL := SQRT(TY * TY + TZ * TZ);

	IF R_TOOL = 0.0 THEN
		// Tanpa tool, arahnya tidak berarti apa-apa. V1 memaksa Y jadi 1e-15 di
		// sini, yang bikin THETA_TOOL melompat ke ~90 derajat - sudut palsu untuk
		// tool yang panjangnya nol.
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

	// ------------------------------------------------------------ rantai planar
	// Sumbu 0 LINEAR (rel X) - lewat apa adanya, tidak masuk trigonometri.
	// Sumbu 1-3 rantai planar di bidang Y-Z, sudutnya menumpuk.
	A1 := ROBOT_POS_INPUT[1] * DEGREE_TO_RAD;
	A2 := A1 + ROBOT_POS_INPUT[2] * DEGREE_TO_RAD;
	A3 := A2 + ROBOT_POS_INPUT[3] * DEGREE_TO_RAD;

	WY := ROBOT_L2_LREAL * COS(A1) + ROBOT_L3_LREAL * COS(A2) + ROBOT_L4_LREAL * COS(A3);
	WZ := ROBOT_L1_LREAL + ROBOT_L2_LREAL * SIN(A1) + ROBOT_L3_LREAL * SIN(A2) + ROBOT_L4_LREAL * SIN(A3);

	// Tool digeser polar, berputar bersama arah end-effector.
	WORLD_LREAL[0] := ROBOT_POS_INPUT[0];
	WORLD_LREAL[1] := WY + R_TOOL * COS(TH_TOOL + A3);
	WORLD_LREAL[2] := WZ + R_TOOL * SIN(TH_TOOL + A3);
	WORLD_LREAL[3] := ROBOT_POS_INPUT[1] + ROBOT_POS_INPUT[2] + ROBOT_POS_INPUT[3];

	FOR I := 0 TO 3 DO
		ROBOT_POS_WORLD_OUTPUT[I] := LREAL_TO_REAL(WORLD_LREAL[I]);
		ROBOT_POS_JOINT_OUTPUT[I] := LREAL_TO_REAL(ROBOT_POS_INPUT[I]);
	END_FOR;

	DONE := TRUE;
END_IF;
