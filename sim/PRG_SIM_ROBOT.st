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
//     cuma PEMANGGIL: tombol, target, motion model, gripper, sekuens sel.
//   * FB yang dipakai versi V2 (sudah dibetulkan). Yang verbatim dari mesin ada di
//     ../extract/ dan sengaja TIDAK di-instance - dia catatan, bukan bagian sim.
//   * Penjaga jangkauan ADA DI DALAM IK V2, bukan di sini. Program cuma membaca
//     DONE dan ERROR_ID-nya. Dua penjaga untuk satu hal pasti berbeda pendapat.
//   * TIDAK ada MC_* sama sekali. Posisi sumbu diintegrasi sendiri, jadi project
//     sim tidak butuh setting axis, axes group, maupun EtherCAT.
//   * TCP di UJUNG JARI gripper: panjang gripper sudah dijumlahkan ke
//     ROBOT_TOOL_Y_LREAL waktu blok init dibangkitkan. Jangan menjumlahkannya lagi.
//
// Satuan: sumbu 0 mm (rel linear), sumbu 1-3 derajat (rantai planar di bidang Y-Z).
//
// ---------------------------------------------------------------- mode & keamanan
// Empat hal menentukan robot boleh bergerak sendiri atau tidak, dan ketiganya
// meniru panel mesin sungguhan:
//
//   SIM_SEL_AUTO   selector AUTO/MANUAL. Diubah SAAT robot jalan = berhenti total.
//   SIM_ESTOP      emergency. Sama: berhenti total.
//   SIM_HOMED      sesudah berhenti total, robot WAJIB di-home dulu. Ini bukan
//                  kerewelan: posisi sumbu setelah berhenti mendadak tidak diketahui
//                  sekuenser, dan melanjutkan dari situ berarti langkah pertama
//                  dijalankan dari pose yang tidak pernah direncanakan.
//   SIM_AUTORUN    tombol start. Cuma jalan kalau selector AUTO, sudah home, tidak
//                  emergency.
//
// SIM_CYCLE_STOP beda dari ketiganya: dia BUKAN berhenti darurat. Robot menyelesaikan
// pekerjaan yang sedang dipegang (produk tidak ditinggal di udara), balik ke pose
// jalan, baru berhenti - dan karena berhentinya terkendali, SIM_HOMED tetap menyala
// dan tombol Autorun bisa langsung dipakai lagi.

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
	SIM_GRIP_STROKE := 180.0;
	// jari berhenti di lebar produk, bukan di 0 - kalau 0, jari menembus PCB
	SIM_GRIP_TUTUP := 120.0;
	SIM_GRIP_JARI := 14.0;
	SIM_GRIP_VEL := 160.0;
	SIM_GRIP_POS := 180.0;

	ROBOT_ROBOT_ORG_OFFSET_LREAL[0] := 0.0;
	ROBOT_ROBOT_ORG_OFFSET_LREAL[1] := 0.0;
	ROBOT_ROBOT_ORG_OFFSET_LREAL[2] := 0.0;
	ROBOT_ROBOT_ORG_OFFSET_LREAL[3] := 0.0;
	ROBOT_ROBOT_ORG_OFFSET_LREAL[4] := 0.0;

	PD1300_000 := -1500.0;
	PD1300_001 := 1500.0;
	PD1300_002 := -90.0;
	PD1300_003 := 180.0;
	PD1300_004 := -150.0;
	PD1300_005 := 0.0;
	PD1300_006 := -120.0;
	PD1300_007 := 120.0;

	SIM_VEL[0] := 900.0;
	SIM_VEL[1] := 90.0;
	SIM_VEL[2] := 90.0;
	SIM_VEL[3] := 120.0;

	SIM_ACC[0] := 1800.0;
	SIM_ACC[1] := 240.0;
	SIM_ACC[2] := 240.0;
	SIM_ACC[3] := 320.0;
	SIM_SPEED_OVR := 100.0;

	SIM_VEL_W[0] := 100.0;
	SIM_VEL_W[1] := 100.0;
	SIM_VEL_W[2] := 100.0;
	SIM_VEL_W[3] := 30.0;

	SIM_HOME[0] := 0.0;
	SIM_HOME[1] := 90.0;
	SIM_HOME[2] := -90.0;
	SIM_HOME[3] := -90.0;

	SIM_DT := 0.004;
	SIM_JOG_STEP := 10.0;

	SIM_APPROACH := 140.0;
	SIM_ST_N := 6;
	SIM_ST_W := 300.0;
	SIM_ST_D := 320.0;
	SIM_ST_MARGIN := 12.0;
	SIM_COVER_SUDUT := 80.0;
	SIM_COVER_VEL := 110.0;
	// WIP IN
	SIM_ST_X[0] := -1400.0;   SIM_ST_Y[0] := 430.0;
	SIM_ST_Z[0] := 250.0;   SIM_ST_T[0] := -90.0;
	SIM_ST_TIPE[0] := 0;   SIM_ST_PROSES[0] := 0.0;
	SIM_ST_STATE[0] := 0;   SIM_ST_TIMER[0] := 0.0;
	SIM_ST_COVER[0] := 80.0;
	// ICC 1
	SIM_ST_X[1] := -700.0;   SIM_ST_Y[1] := 430.0;
	SIM_ST_Z[1] := 285.0;   SIM_ST_T[1] := -90.0;
	SIM_ST_TIPE[1] := 1;   SIM_ST_PROSES[1] := 17.0;
	SIM_ST_STATE[1] := 0;   SIM_ST_TIMER[1] := 0.0;
	SIM_ST_COVER[1] := 80.0;
	// ICC 2
	SIM_ST_X[2] := -300.0;   SIM_ST_Y[2] := 430.0;
	SIM_ST_Z[2] := 285.0;   SIM_ST_T[2] := -90.0;
	SIM_ST_TIPE[2] := 1;   SIM_ST_PROSES[2] := 17.0;
	SIM_ST_STATE[2] := 0;   SIM_ST_TIMER[2] := 0.0;
	SIM_ST_COVER[2] := 80.0;
	// DW 1
	SIM_ST_X[3] := 300.0;   SIM_ST_Y[3] := 430.0;
	SIM_ST_Z[3] := 250.0;   SIM_ST_T[3] := -90.0;
	SIM_ST_TIPE[3] := 2;   SIM_ST_PROSES[3] := 15.0;
	SIM_ST_STATE[3] := 0;   SIM_ST_TIMER[3] := 0.0;
	SIM_ST_COVER[3] := 80.0;
	// DW 2
	SIM_ST_X[4] := 700.0;   SIM_ST_Y[4] := 430.0;
	SIM_ST_Z[4] := 250.0;   SIM_ST_T[4] := -90.0;
	SIM_ST_TIPE[4] := 2;   SIM_ST_PROSES[4] := 15.0;
	SIM_ST_STATE[4] := 0;   SIM_ST_TIMER[4] := 0.0;
	SIM_ST_COVER[4] := 80.0;
	// WIP OUT
	SIM_ST_X[5] := 1400.0;   SIM_ST_Y[5] := 430.0;
	SIM_ST_Z[5] := 250.0;   SIM_ST_T[5] := -90.0;
	SIM_ST_TIPE[5] := 3;   SIM_ST_PROSES[5] := 0.0;
	SIM_ST_STATE[5] := 0;   SIM_ST_TIMER[5] := 0.0;
	SIM_ST_COVER[5] := 80.0;

	SIM_SEL_AUTO := FALSE;
	SIM_ESTOP := FALSE;
	SIM_AUTO := FALSE;
	SIM_STOP_REQ := FALSE;
	SIM_ABORT_ID := 0;
	SIM_COLLIDE := FALSE;
	SIM_COLLIDE_ST := -1;
	SIM_CYCLE_STEP := 0;
	SIM_PART_STATE := 0;
	SIM_JOB_SRC := -1;
	SIM_JOB_DST := -1;
	SIM_DROP_COUNT := 0;
	SIM_GRIP_CMD := FALSE;
	// >>> DIBANGKITKAN

	FOR i := 0 TO 3 DO
		SIM_JOINT_POS[i] := SIM_HOME[i];
		SIM_JOINT_CMD[i] := SIM_HOME[i];
		SIM_JOINT_VEL[i] := 0.0;
	END_FOR;

	// Init menaruh sumbu TEPAT di pose home, jadi robot memang sedang home.
	SIM_HOMED := TRUE;
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
EDGE_JSET := SIM_JOG_SET_EXEC AND NOT LAST_JSET;
LAST_JSET := SIM_JOG_SET_EXEC;
EDGE_RUN := SIM_AUTORUN AND NOT LAST_RUN;
LAST_RUN := SIM_AUTORUN;
EDGE_CSTOP := SIM_CYCLE_STOP AND NOT LAST_CSTOP;
LAST_CSTOP := SIM_CYCLE_STOP;
// Selector: yang menghentikan robot bukan posisi selectornya, tapi PERUBAHANNYA.
// Memakai posisi ("kalau MANUAL maka berhenti") berarti robot tidak akan pernah bisa
// dijalankan dari selector AUTO yang memang sudah di situ sejak awal.
SEL_UBAH := SIM_SEL_AUTO <> LAST_SEL;
LAST_SEL := SIM_SEL_AUTO;

// -------------------------------------------------------- override kecepatan
// Satu angka persen yang menskalakan SEMUA gerakan sumbu - siklus otomatis maupun
// jog. Dijepit DI SINI, di PLC, bukan di halaman: nilainya boleh ditulis dari mana
// saja lewat OPC UA, dan 500 % atau angka negatif yang lolos bikin sumbu melompati
// targetnya tiap scan - gerakan yang di layar terlihat seperti lengan yang berkedip,
// bukan seperti setelan yang salah.
//
// Yang diskalakan KECEPATAN saja, bukan akselerasi. Override yang ikut mengubah
// akselerasi bikin jarak pengereman berubah dua kali lipat arahnya, dan pelan-pelan
// jadi TIDAK lebih aman - padahal itu satu-satunya alasan orang menurunkannya.
//
// Gripper TIDAK ikut: jarinya pneumatik di mesin aslinya, kecepatannya tidak bisa
// disetel dari controller. Override yang ikut memperlambat gripper bikin waktu siklus
// di layar tidak sama dengan yang di mesin.
IF SIM_SPEED_OVR > 100.0 THEN SIM_SPEED_OVR := 100.0; END_IF;
IF SIM_SPEED_OVR < 1.0 THEN SIM_SPEED_OVR := 1.0; END_IF;
OVR := SIM_SPEED_OVR / 100.0;

// -------------------------------------------------------- berhenti total (abort)
// Emergency, selector diubah saat jalan, atau tabrakan. Ketiganya berakhir sama:
// perintah disamakan dengan posisi sekarang (sumbu berhenti di tempat), siklus
// dibatalkan, dan SIM_HOMED dipadamkan - robot tidak boleh jalan lagi sebelum home.
ABORT := FALSE;
IF SIM_ESTOP THEN
	ABORT := TRUE;
	SIM_ABORT_ID := 1;
ELSIF SEL_UBAH AND SIM_AUTO THEN
	ABORT := TRUE;
	SIM_ABORT_ID := 2;
ELSIF SIM_COLLIDE AND NOT LAST_COLLIDE THEN
	// Cuma di TEPI-nya. Menahan selama masih menempel bikin robot terkunci di dalam
	// benda yang ditabraknya: tiap scan perintahnya ditimpa posisi sekarang, jadi
	// tidak ada arah yang bisa dipakai keluar.
	ABORT := TRUE;
	SIM_ABORT_ID := 3;
END_IF;
LAST_COLLIDE := SIM_COLLIDE;

IF ABORT THEN
	SIM_AUTO := FALSE;
	SIM_STOP_REQ := FALSE;
	SIM_CYCLE_STEP := 0;
	SIM_HOMED := FALSE;
	HOMING := FALSE;
	FOR i := 0 TO 3 DO
		SIM_JOINT_CMD[i] := SIM_JOINT_POS[i];
		SIM_JOINT_VEL[i] := 0.0;
	END_FOR;
END_IF;

// ------------------------------------------------------------------- home return
// Home dipakai DUA hal sekaligus: pose istirahat, dan pose JALAN. Lengan cuma boleh
// menyusuri rel dalam pose ini - lihat sekuenser di bawah.
IF EDGE_HOME AND NOT SIM_ESTOP AND NOT SIM_AUTO THEN
	// Sumbu 1-3 SAJA. Rel tidak disentuh: home boleh di mana saja di sepanjang rel.
	// Memaksa rel pulang ke satu titik bikin tiap pemulihan menyeret lengan melewati
	// mesin yang tidak ada urusannya - lebih jauh, lebih lama, dan lebih banyak yang
	// bisa tersenggol daripada melipat di tempat. Yang dibutuhkan sekuenser cuma
	// BENTUK lengannya, karena langkah pertama tiap pekerjaan memang menggeser rel.
	FOR i := 1 TO 3 DO
		SIM_JOINT_CMD[i] := SIM_HOME[i];
	END_FOR;
	HOMING := TRUE;
	SIM_ABORT_ID := 0;
	// WAJIB. SIM_MOVE_DONE dihitung motion model di AKHIR scan, jadi di sini isinya
	// masih jawaban scan sebelumnya - dan sesudah berhenti total jawabannya TRUE.
	// Tanpa baris ini, pemeriksaan tepat di bawah melihat "sudah sampai" pada scan
	// yang sama waktu perintahnya baru saja diberikan: SIM_HOMED menyala tanpa
	// lengannya bergerak satu milimeter pun, dan itu justru syarat yang mau ditegakkan.
	// Sekuenser tidak kena masalah ini karena perintah dan penungguannya di langkah
	// (dan scan) yang berbeda.
	SIM_MOVE_DONE := FALSE;
END_IF;
IF HOMING AND SIM_MOVE_DONE THEN
	HOMING := FALSE;
	SIM_HOMED := TRUE;
END_IF;
// Home TIDAK melepas produk yang sedang dijepit, dan TIDAK menghapus pekerjaannya
// (SIM_JOB_SRC/SIM_JOB_DST). Gripper yang membuka sendiri waktu pulih berarti produk
// jatuh ke lantai tiap kali orang menekan emergency - dan di mesin aslinya yang jatuh
// itu barang sungguhan. Jadi robot pulang MEMBAWA produknya, dan Autorun berikutnya
// melanjutkan pengantaran yang tertunda, bukan memulai dari nol.
// Yang melepas cuma perintah gripper dari operator - lihat blok "produk jatuh".

// -------------------------------------------------------- start / cycle stop
IF EDGE_RUN AND SIM_SEL_AUTO AND SIM_HOMED AND NOT SIM_ESTOP AND NOT HOMING THEN
	SIM_AUTO := TRUE;
	SIM_STOP_REQ := FALSE;
	SIM_ABORT_ID := 0;
	IF (SIM_PART_STATE = 1) AND (SIM_JOB_DST >= 0) THEN
		// Masih memegang produk dari pekerjaan yang tadi dihentikan: LANJUTKAN
		// pengantarannya, jangan pilih pekerjaan baru. Mulai dari nol berarti robot
		// berangkat mengambil produk kedua sambil tangannya masih penuh, dan produk
		// pertama tidak akan pernah sampai ke tujuannya.
		//
		// Langkah 18 = lipat -> geser rel ke tujuan -> turun -> lepas -> naik -> lipat:
		// persis separuh-antar dari pekerjaan yang sama. Stasiun asalnya sudah
		// dikosongkan waktu produknya diambil, jadi tidak ada yang perlu dikembalikan.
		SIM_TARGET_ST := SIM_JOB_DST;
		SIM_CYCLE_STEP := 18;
	ELSE
		SIM_CYCLE_STEP := 0;
	END_IF;
END_IF;
IF EDGE_CSTOP AND SIM_AUTO THEN
	SIM_STOP_REQ := TRUE;
END_IF;

// Keadaan yang dilaporkan ke layar. Satu angka, dibaca halaman - bukan halaman yang
// menyimpulkan sendiri dari empat bit, karena kesimpulannya bisa beda dari yang
// dipakai PLC memutuskan.
IF SIM_ESTOP THEN SIM_STATE := 6;
ELSIF HOMING THEN SIM_STATE := 1;
ELSIF NOT SIM_HOMED THEN SIM_STATE := 5;
ELSIF SIM_AUTO AND SIM_STOP_REQ THEN SIM_STATE := 4;
ELSIF SIM_AUTO THEN SIM_STATE := 3;
ELSIF SIM_SEL_AUTO THEN SIM_STATE := 2;
ELSE SIM_STATE := 0;
END_IF;

// ------------------------------------------------------------- mesin tiap stasiun
// Tiap mesin menghitung waktunya SENDIRI, berbarengan - itu inti buffer: ICC 1 dan
// ICC 2 memang harus bisa memproses bersamaan sementara robot pergi mengerjakan
// yang lain. Satu penghitung waktu bersama (SIM_WAIT yang lama) memaksa satu produk
// di seluruh sel, dan buffer jadi tidak ada artinya.
//
// WIP IN dianggap selalu berisi (stok masuk tidak disimulasikan), WIP OUT selalu
// kosong (produk keluar dari sel).
WIP_IN := 0;
WIP_OUT := 0;
FOR i := 0 TO SIM_ST_N - 1 DO
	IF SIM_ST_TIPE[i] = 0 THEN
		WIP_IN := i;
		SIM_ST_STATE[i] := 2;
	ELSIF SIM_ST_TIPE[i] = 3 THEN
		WIP_OUT := i;
		SIM_ST_STATE[i] := 0;
	ELSIF SIM_ST_STATE[i] = 1 THEN
		// Waktu proses cuma jalan waktu penutupnya SUDAH RAPAT. Penutup itu yang
		// menekan PCB ke probe base; menghitung sebelum rapat berarti mesin mengaku
		// menguji papan yang belum tersentuh probe - dan hasilnya tetap keluar
		// "selesai", karena tidak ada yang memeriksa kontaknya.
		IF SIM_ST_COVER[i] <= 0.5 THEN
			SIM_ST_TIMER[i] := SIM_ST_TIMER[i] - SIM_DT;
			IF SIM_ST_TIMER[i] <= 0.0 THEN
				SIM_ST_TIMER[i] := 0.0;
				SIM_ST_STATE[i] := 2;
			END_IF;
		END_IF;
	END_IF;

	// Penutup berengsel: menutup HANYA selama memproses, selain itu terbuka supaya
	// gripper punya jalan masuk. WIP IN/OUT tidak punya penutup - rak terbuka.
	IF (SIM_ST_STATE[i] = 1) AND (SIM_ST_TIPE[i] <> 0) AND (SIM_ST_TIPE[i] <> 3) THEN
		COVER_TARGET := 0.0;
	ELSE
		COVER_TARGET := SIM_COVER_SUDUT;
	END_IF;
	D := COVER_TARGET - SIM_ST_COVER[i];
	STEP_M := SIM_COVER_VEL * SIM_DT;
	IF ABS(D) <= STEP_M THEN
		SIM_ST_COVER[i] := COVER_TARGET;
	ELSIF D > 0.0 THEN
		SIM_ST_COVER[i] := SIM_ST_COVER[i] + STEP_M;
	ELSE
		SIM_ST_COVER[i] := SIM_ST_COVER[i] - STEP_M;
	END_IF;
END_FOR;

// ------------------------------------------------------------ pose yang diminta
// POSE_REQ diisi kalau ada permintaan berbasis WORLD (jog world/tool, move point,
// atau sekuenser siklus). NEED_IK menandai bahwa IK perlu dijalankan scan ini.
NEED_IK := FALSE;

// Sumbu mana yang ditekan, dan sebesar apa langkahnya. Satu sumbu sekali jalan:
// dua tombol sekaligus di mesin asli juga saling mengunci lewat latch START_*.
// Jog cuma hidup di MANUAL: selector di AUTO sambil tombol jog bisa ditekan berarti
// dua sumber perintah untuk satu lengan, dan yang menang berubah tiap scan.
AX := -1;
DELTA := 0.0;
IF NOT SIM_AUTO AND NOT SIM_SEL_AUTO AND NOT SIM_ESTOP THEN
	FOR i := 0 TO 3 DO
		IF SIM_JOG_P[i] AND (EDGE_P[i] OR SIM_JOG_HOLD) THEN
			AX := i;
			DELTA := SIM_JOG_STEP;
			IF SIM_JOG_HOLD THEN
				// Mode tahan: kecepatan x dt. DUA daftar, karena satuannya memang beda -
				// SIM_VEL derajat/s pada SUMBU, SIM_VEL_W mm/s pada KOORDINAT world.
				IF SIM_JOG_MODE = 0 THEN DELTA := SIM_VEL[i] * OVR * SIM_DT;
				ELSE DELTA := SIM_VEL_W[i] * OVR * SIM_DT; END_IF;
			END_IF;
		ELSIF SIM_JOG_N[i] AND (EDGE_N[i] OR SIM_JOG_HOLD) THEN
			AX := i;
			DELTA := -SIM_JOG_STEP;
			IF SIM_JOG_HOLD THEN
				IF SIM_JOG_MODE = 0 THEN DELTA := -SIM_VEL[i] * OVR * SIM_DT;
				ELSE DELTA := -SIM_VEL_W[i] * OVR * SIM_DT; END_IF;
			END_IF;
		END_IF;
	END_FOR;
END_IF;

// Sekuenser dan jog TIDAK PERNAH jalan berbarengan.
IF SIM_AUTO THEN

	// ============================================================ siklus otomatis
	// Aturan sel, dan urutannya ITU yang menentukan perilakunya:
	//
	//   1. ICC yang KOSONG diisi dulu - dua-duanya - sebelum apa pun yang lain.
	//      Itu arti "buffer": mesin tes yang lama (17 s) tidak boleh menganggur
	//      menunggu robot selesai mengurus DW.
	//   2. DW yang SELESAI dikosongkan ke WIP OUT. Didahulukan dari nomor 3 supaya
	//      selalu ada DW kosong buat produk ICC berikutnya - kalau tidak, dua DW
	//      penuh + dua ICC selesai = sel macet, dan macetnya tidak kelihatan sebagai
	//      kesalahan, cuma sebagai robot yang berhenti.
	//   3. ICC yang SELESAI dipindah ke DW yang kosong.
	//
	// Tiap pekerjaan bentuknya sama: lipat ke pose jalan -> geser rel -> turun ->
	// gripper -> naik -> lipat -> geser rel -> turun -> gripper -> naik -> lipat.
	//
	// Nomor langkah dipublikasikan (SIM_CYCLE_STEP) supaya waktu sekuensnya berhenti,
	// yang kelihatan langkah mana - bukan cuma "robotnya diam".
	CASE SIM_CYCLE_STEP OF

		0:	// pilih pekerjaan
			SIM_JOB_SRC := -1;
			SIM_JOB_DST := -1;
			FOR i := 0 TO SIM_ST_N - 1 DO
				IF (SIM_JOB_DST < 0) AND (SIM_ST_TIPE[i] = 1) AND (SIM_ST_STATE[i] = 0) THEN
					SIM_JOB_SRC := WIP_IN;
					SIM_JOB_DST := i;
				END_IF;
			END_FOR;
			IF SIM_JOB_DST < 0 THEN
				FOR i := 0 TO SIM_ST_N - 1 DO
					IF (SIM_JOB_DST < 0) AND (SIM_ST_TIPE[i] = 2) AND (SIM_ST_STATE[i] = 2) THEN
						SIM_JOB_SRC := i;
						SIM_JOB_DST := WIP_OUT;
					END_IF;
				END_FOR;
			END_IF;
			IF SIM_JOB_DST < 0 THEN
				FOR i := 0 TO SIM_ST_N - 1 DO
					IF (SIM_JOB_DST < 0) AND (SIM_ST_TIPE[i] = 1) AND (SIM_ST_STATE[i] = 2) THEN
						FOR j := 0 TO SIM_ST_N - 1 DO
							IF (SIM_JOB_DST < 0) AND (SIM_ST_TIPE[j] = 2) AND (SIM_ST_STATE[j] = 0) THEN
								SIM_JOB_SRC := i;
								SIM_JOB_DST := j;
							END_IF;
						END_FOR;
					END_IF;
				END_FOR;
			END_IF;
			IF SIM_JOB_DST >= 0 THEN
				ST_IDX := SIM_JOB_SRC;
				SIM_TARGET_ST := ST_IDX;
				SIM_CYCLE_STEP := 5;
			ELSIF SIM_STOP_REQ THEN
				// Tidak ada pekerjaan DAN diminta berhenti: berhenti sekarang, robot
				// memang sedang tidak memegang apa-apa.
				SIM_AUTO := FALSE;
				SIM_STOP_REQ := FALSE;
			END_IF;

		5:	// lipat ke pose jalan - sumbu 1-3 saja, rel tidak disentuh
			SIM_JOINT_CMD[1] := SIM_HOME[1];
			SIM_JOINT_CMD[2] := SIM_HOME[2];
			SIM_JOINT_CMD[3] := SIM_HOME[3];
			SIM_CYCLE_STEP := 6;

		6:	IF SIM_MOVE_DONE THEN SIM_CYCLE_STEP := 7; END_IF;

		7:	// geser rel. Lengan HARUS sudah terlipat: menyusuri rel sambil menjulur ke
			// bawah berarti menyapu tiap mesin yang dilewati, dan di layar itu mulus.
			SIM_JOINT_CMD[0] := SIM_ST_X[ST_IDX];
			SIM_CYCLE_STEP := 8;

		8:	IF SIM_MOVE_DONE THEN SIM_CYCLE_STEP := 10; END_IF;

		10:	// ke atas stasiun
			POSE_REQ[0] := SIM_ST_X[ST_IDX];
			POSE_REQ[1] := SIM_ST_Y[ST_IDX];
			POSE_REQ[2] := SIM_ST_Z[ST_IDX] + SIM_APPROACH;
			POSE_REQ[3] := SIM_ST_T[ST_IDX];
			NEED_IK := TRUE;
			SIM_CYCLE_STEP := 11;

		11:	// Menunggu DUA hal: lengan sampai di atas stasiun, DAN penutupnya terbuka
			// penuh. Turun ke penutup yang masih bergerak berarti gripper dan penutup
			// berbagi ruang yang sama - di layar itu cuma dua benda yang saling
			// menembus, tanpa satu pun yang mengeluh.
			IF SIM_MOVE_DONE AND (SIM_ST_COVER[ST_IDX] >= SIM_COVER_SUDUT - 0.5) THEN
				SIM_CYCLE_STEP := 12;
			END_IF;

		12:	// turun tegak ke permukaan stasiun - gripper menyumpit dari ATAS
			POSE_REQ[0] := SIM_ST_X[ST_IDX];
			POSE_REQ[1] := SIM_ST_Y[ST_IDX];
			POSE_REQ[2] := SIM_ST_Z[ST_IDX];
			POSE_REQ[3] := SIM_ST_T[ST_IDX];
			NEED_IK := TRUE;
			SIM_CYCLE_STEP := 13;

		13:	IF SIM_MOVE_DONE THEN SIM_CYCLE_STEP := 14; END_IF;

		14:	SIM_GRIP_CMD := TRUE;
			SIM_CYCLE_STEP := 15;

		15:	IF SIM_GRIP_CLOSED THEN
				SIM_PART_STATE := 1;
				// Stok WIP IN tidak habis; stasiun mesin ditinggal kosong.
				IF SIM_ST_TIPE[ST_IDX] <> 0 THEN
					SIM_ST_STATE[ST_IDX] := 0;
				END_IF;
				SIM_CYCLE_STEP := 16;
			END_IF;

		16:	POSE_REQ[0] := SIM_ST_X[ST_IDX];
			POSE_REQ[1] := SIM_ST_Y[ST_IDX];
			POSE_REQ[2] := SIM_ST_Z[ST_IDX] + SIM_APPROACH;
			POSE_REQ[3] := SIM_ST_T[ST_IDX];
			NEED_IK := TRUE;
			SIM_CYCLE_STEP := 17;

		17:	IF SIM_MOVE_DONE THEN SIM_CYCLE_STEP := 18; END_IF;

		18:	SIM_JOINT_CMD[1] := SIM_HOME[1];
			SIM_JOINT_CMD[2] := SIM_HOME[2];
			SIM_JOINT_CMD[3] := SIM_HOME[3];
			SIM_CYCLE_STEP := 19;

		19:	IF SIM_MOVE_DONE THEN
				ST_IDX := SIM_JOB_DST;
				SIM_TARGET_ST := ST_IDX;
				SIM_CYCLE_STEP := 20;
			END_IF;

		20:	SIM_JOINT_CMD[0] := SIM_ST_X[ST_IDX];
			SIM_CYCLE_STEP := 21;

		21:	IF SIM_MOVE_DONE THEN SIM_CYCLE_STEP := 22; END_IF;

		22:	POSE_REQ[0] := SIM_ST_X[ST_IDX];
			POSE_REQ[1] := SIM_ST_Y[ST_IDX];
			POSE_REQ[2] := SIM_ST_Z[ST_IDX] + SIM_APPROACH;
			POSE_REQ[3] := SIM_ST_T[ST_IDX];
			NEED_IK := TRUE;
			SIM_CYCLE_STEP := 23;

		23:	IF SIM_MOVE_DONE AND (SIM_ST_COVER[ST_IDX] >= SIM_COVER_SUDUT - 0.5) THEN
				SIM_CYCLE_STEP := 24;
			END_IF;

		24:	POSE_REQ[0] := SIM_ST_X[ST_IDX];
			POSE_REQ[1] := SIM_ST_Y[ST_IDX];
			POSE_REQ[2] := SIM_ST_Z[ST_IDX];
			POSE_REQ[3] := SIM_ST_T[ST_IDX];
			NEED_IK := TRUE;
			SIM_CYCLE_STEP := 25;

		25:	IF SIM_MOVE_DONE THEN SIM_CYCLE_STEP := 26; END_IF;

		26:	SIM_GRIP_CMD := FALSE;
			SIM_CYCLE_STEP := 27;

		27:	IF SIM_GRIP_OPEN THEN
				SIM_PART_STATE := 0;
				IF SIM_ST_TIPE[ST_IDX] = 3 THEN
					// WIP OUT: produk keluar dari sel.
					SIM_CYCLE_COUNT := SIM_CYCLE_COUNT + 1;
				ELSE
					SIM_ST_STATE[ST_IDX] := 1;
					SIM_ST_TIMER[ST_IDX] := SIM_ST_PROSES[ST_IDX];
				END_IF;
				SIM_CYCLE_STEP := 28;
			END_IF;

		28:	POSE_REQ[0] := SIM_ST_X[ST_IDX];
			POSE_REQ[1] := SIM_ST_Y[ST_IDX];
			POSE_REQ[2] := SIM_ST_Z[ST_IDX] + SIM_APPROACH;
			POSE_REQ[3] := SIM_ST_T[ST_IDX];
			NEED_IK := TRUE;
			SIM_CYCLE_STEP := 29;

		29:	IF SIM_MOVE_DONE THEN SIM_CYCLE_STEP := 30; END_IF;

		30:	SIM_JOINT_CMD[1] := SIM_HOME[1];
			SIM_JOINT_CMD[2] := SIM_HOME[2];
			SIM_JOINT_CMD[3] := SIM_HOME[3];
			SIM_CYCLE_STEP := 31;

		31:	IF SIM_MOVE_DONE THEN
				// Di SINI cycle stop dilayani: produk sudah diletakkan, lengan sudah
				// terlipat. Berhentinya terkendali, jadi SIM_HOMED tidak dipadamkan dan
				// tombol Autorun bisa langsung dipakai lagi.
				IF SIM_STOP_REQ THEN
					SIM_AUTO := FALSE;
					SIM_STOP_REQ := FALSE;
				END_IF;
				SIM_CYCLE_STEP := 0;
			END_IF;
	ELSE
		SIM_CYCLE_STEP := 0;
	END_CASE;

ELSIF (AX >= 0) AND (SIM_JOG_MODE = 0) THEN
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

ELSIF EDGE_JSET AND NOT SIM_ESTOP AND NOT SIM_SEL_AUTO THEN
	// Slider jog: satu target sumbu sekaligus, dikirim waktu slidernya DILEPAS.
	// Lewat sumbu langsung, bukan IK - slider sumbu memang menyetel sudut sumbu, dan
	// memaksanya lewat IK berarti pose yang tidak punya solusi jadi tidak bisa dituju
	// padahal sumbunya sendiri sanggup ke situ.
	FOR i := 0 TO 3 DO
		SIM_JOINT_CMD[i] := SIM_JOG_SET[i];
	END_FOR;

ELSIF EDGE_MOVE AND NOT SIM_ESTOP AND NOT SIM_SEL_AUTO THEN
	FOR i := 0 TO 3 DO
		POSE_REQ[i] := SIM_WORLD_CMD[i];
	END_FOR;
	NEED_IK := TRUE;
END_IF;

// ----------------------------------------------------------------- tabrakan
// Badan mesin diperlakukan sebagai kotak: |dx| < lebar/2, |dy| < dalam/2, dan
// ketinggian di BAWAH permukaan stasiun. Ukurannya dari tag yang sama dengan yang
// digambar halaman - satu kotak, bukan satu buat gambar dan satu buat penjaga.
//
// Diperiksa DUA hal sekaligus dalam satu loop, dan itu sengaja: rumus yang sama
// ditulis dua kali pasti berbeda pendapat suatu hari.
//
//   titik 0-1  pose yang DIMINTA (TCP + pangkal gripper). Ditolak sebelum bergerak -
//              ini yang bikin jog dan move point tidak bisa menembus mesin.
//   titik 2-3  pose SEKARANG. Ini deteksi SENTUHAN: kalau lengan sudah terlanjur
//              masuk (misalnya jog JOINT yang tidak lewat pose world sama sekali),
//              gerakan dihentikan dan siklus dibatalkan.
//
// Pose sekarang dibaca dari SIM_WORLD_POS, hasil FK scan SEBELUMNYA - telat satu
// scan (4 ms). Menghitungnya lagi di sini butuh instance FK kedua yang cuma dipakai
// untuk itu.
//
// Lebar jari ikut dihitung: yang menabrak mesin duluan bukan TCP-nya, tapi jari yang
// menjulur ke samping sejauh setengah bukaan.
CK_MX := SIM_ST_W / 2.0 + SIM_ST_MARGIN + SIM_GRIP_POS / 2.0 + SIM_GRIP_JARI;
CK_MY := SIM_ST_D / 2.0 + SIM_ST_MARGIN;

TH := POSE_REQ[3] * DEGREE_TO_RAD;
CK_X[0] := POSE_REQ[0];
CK_Y[0] := POSE_REQ[1];
CK_Z[0] := POSE_REQ[2];
CK_X[1] := POSE_REQ[0];
CK_Y[1] := POSE_REQ[1] - SIM_GRIP_LEN * COS(TH);
CK_Z[1] := POSE_REQ[2] - SIM_GRIP_LEN * SIN(TH);

TH := REAL_TO_LREAL(SIM_WORLD_POS[3]) * DEGREE_TO_RAD;
CK_X[2] := REAL_TO_LREAL(SIM_WORLD_POS[0]);
CK_Y[2] := REAL_TO_LREAL(SIM_WORLD_POS[1]);
CK_Z[2] := REAL_TO_LREAL(SIM_WORLD_POS[2]);
CK_X[3] := CK_X[2];
CK_Y[3] := CK_Y[2] - SIM_GRIP_LEN * COS(TH);
CK_Z[3] := CK_Z[2] - SIM_GRIP_LEN * SIN(TH);

CAND_HIT := FALSE;
NOW_HIT := FALSE;
NOW_ST := -1;
FOR k := 0 TO 3 DO
	IF (k >= 2) OR NEED_IK THEN
		FOR i := 0 TO SIM_ST_N - 1 DO
			// Permukaan stasiun BUKAN tabrakan: produk memang diletakkan di situ.
			// Yang dihitung menembus badan mesin, jadi batasnya sedikit DI BAWAH
			// permukaan - tanpa itu, tiap penempatan produk memicu alarm tabrakan.
			IF (ABS(CK_X[k] - SIM_ST_X[i]) < CK_MX) AND (ABS(CK_Y[k] - SIM_ST_Y[i]) < CK_MY)
			   AND (CK_Z[k] < SIM_ST_Z[i] - 2.0) THEN
				IF k >= 2 THEN
					NOW_HIT := TRUE;
					NOW_ST := i;
				ELSE
					CAND_HIT := TRUE;
				END_IF;
			END_IF;
		END_FOR;
		// Lantai. Tanpa ini, jog ke bawah menembus lantai tanpa satu pun keluhan -
		// dan yang di layar cuma lengan yang separuhnya hilang di balik grid.
		IF CK_Z[k] < 0.0 THEN
			IF k >= 2 THEN
				NOW_HIT := TRUE;
				NOW_ST := -1;
			ELSE
				CAND_HIT := TRUE;
			END_IF;
		END_IF;
	END_IF;
END_FOR;
SIM_COLLIDE := NOW_HIT;
SIM_COLLIDE_ST := NOW_ST;

IF CAND_HIT THEN
	// Perintahnya dibuang sebelum sumbu bergerak. ERROR_ID 5 = tabrakan, dan nomor
	// itu sengaja di luar 1-4 milik IK: yang menolak di sini bukan kinematiknya.
	NEED_IK := FALSE;
	SIM_ERROR := TRUE;
	SIM_ERROR_ID := 5;
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
// Dua jari yang membuka SEPANJANG SUMBU X (arah rel) dan menjepit sisi kiri-kanan
// produk. Menutup berhenti di SIM_GRIP_TUTUP - lebar produk - bukan di nol: jari
// yang bertemu di nol berarti menembus barang yang sedang dipegangnya.
IF SIM_GRIP_CMD THEN GRIP_TARGET := SIM_GRIP_TUTUP; ELSE GRIP_TARGET := SIM_GRIP_STROKE; END_IF;
D := GRIP_TARGET - SIM_GRIP_POS;
STEP_M := SIM_GRIP_VEL * SIM_DT;
IF ABS(D) <= STEP_M THEN
	SIM_GRIP_POS := GRIP_TARGET;
ELSIF D > 0.0 THEN
	SIM_GRIP_POS := SIM_GRIP_POS + STEP_M;
ELSE
	SIM_GRIP_POS := SIM_GRIP_POS - STEP_M;
END_IF;
SIM_GRIP_CLOSED := SIM_GRIP_POS <= (SIM_GRIP_TUTUP + 0.5);
SIM_GRIP_OPEN := SIM_GRIP_POS >= (SIM_GRIP_STROKE - 0.5);

// ------------------------------------------------------------- produk jatuh
// Gripper dibuka MANUAL sambil memegang produk: produknya jatuh, dan pekerjaannya
// ikut hilang bersamanya. Ini bukan pengecualian yang dibuat-buat - itu satu-satunya
// cara produk keluar dari gripper selain diletakkan di stasiun oleh sekuenser.
//
// Ingatannya (SIM_JOB_SRC/DST) DIHAPUS di sini, bukan cuma SIM_PART_STATE: pekerjaan
// yang ditinggal bikin Autorun berikutnya mengira masih ada produk yang harus diantar,
// robot pergi ke tujuan dengan tangan kosong dan membuka gripper di sana - stasiun
// tercatat berisi produk yang tidak ada. Salahnya baru ketahuan belasan menit
// kemudian, waktu robot mengambil produk hantu itu.
//
// Dihitung, bukan didiamkan: produk yang hilang dari sel harus ada angkanya, kalau
// tidak jumlah masuk dan jumlah keluar tidak akan pernah bisa diadu.
IF (SIM_PART_STATE = 1) AND SIM_GRIP_OPEN AND NOT SIM_AUTO THEN
	SIM_PART_STATE := 0;
	SIM_JOB_SRC := -1;
	SIM_JOB_DST := -1;
	SIM_DROP_COUNT := SIM_DROP_COUNT + 1;
END_IF;

// ------------------------------------------------------------ motion model
// Pengganti MC_*: profil trapesium per sumbu - dipercepat sampai SIM_VEL, lalu
// direm tepat waktu supaya berhenti PERSIS di target.
//
// Versi lama melompat langsung ke kecepatan penuh dan berhenti mendadak. Itu bukan
// cuma soal enak dilihat: gerakan yang berangkat dan berhenti dalam satu scan
// terlihat patah-patah begitu angkanya lewat jaringan (bridge mengirim tiap 50 ms,
// jadi tiap patahan jatuh di frame yang berbeda), dan yang ditanyakan orang jadi
// "kenapa gerakannya tersendat" - pertanyaan tentang jaringan, padahal sebabnya di
// sini.
//
// VB = kecepatan tertinggi yang MASIH bisa direm sebelum sampai target
// (v = sqrt(2*a*s)). Tanpa itu sumbu melewati target lalu balik - dan di layar itu
// terlihat seperti lengan yang bergetar di ujung gerakan.
SIM_BUSY := FALSE;
FOR i := 0 TO 3 DO
	D := SIM_JOINT_CMD[i] - SIM_JOINT_POS[i];
	VB := SQRT(2.0 * SIM_ACC[i] * ABS(D));
	VT := SIM_VEL[i] * OVR;
	IF VB < VT THEN VT := VB; END_IF;
	IF D < 0.0 THEN VT := -VT; END_IF;

	DV := SIM_ACC[i] * SIM_DT;
	IF ABS(VT - SIM_JOINT_VEL[i]) <= DV THEN
		SIM_JOINT_VEL[i] := VT;
	ELSIF VT > SIM_JOINT_VEL[i] THEN
		SIM_JOINT_VEL[i] := SIM_JOINT_VEL[i] + DV;
	ELSE
		SIM_JOINT_VEL[i] := SIM_JOINT_VEL[i] - DV;
	END_IF;

	STEP_M := SIM_JOINT_VEL[i] * SIM_DT;
	IF ABS(D) <= ABS(STEP_M) THEN
		SIM_JOINT_POS[i] := SIM_JOINT_CMD[i];
		SIM_JOINT_VEL[i] := 0.0;
	ELSE
		SIM_JOINT_POS[i] := SIM_JOINT_POS[i] + STEP_M;
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
