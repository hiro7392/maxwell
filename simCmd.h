#pragma once
////////////////////////////////////////////////////////
// シミュレーションリスタート
////////////////////////////////////////////////////////
static void restart()
{
	auto sim = EntityManager::get();
	auto finger = sim->getFinger();
	//変数初期化
	sim->step = 0;		//ステップ数初期化
	finger->state_contact = 0;				// 状態変数初期化
	finger->dist = 0.0;
	for (int jnt = 0; jnt < ARM_JNT; jnt++) {
		finger->ref_jnt_pos[jnt] = 0.0;
		finger->jnt_pos[jnt] = 0.0;
		finger->jnt_vel[jnt] = 0.0;
		finger->jnt_force[jnt] = 0.0;
		finger->past_jnt_pos[jnt] = 0.0;
	}
	//	sim->jnt_pos[ARM_M1] = 4 * PI / 4.0; sim->jnt_pos[ARM_M2] = PI / 4.0;
	for (int crd = 0; crd < DIM3; crd++) {
		finger->ref_eff_pos[crd] = 0.0;
		finger->eff_pos[crd] = 0.0;
		finger->eff_force[crd] = 0.0;
	}

	auto finger2 = sim->getFinger2();
	//変数初期化
	sim->step = 0;		//ステップ数初期化
	finger2->state_contact = 0;				// 状態変数初期化
	finger2->dist = 0.0;
	for (int jnt = 0; jnt < ARM_JNT; jnt++) {
		finger2->ref_jnt_pos[jnt] = 0.0;
		finger2->jnt_pos[jnt] = 0.0;
		finger2->jnt_vel[jnt] = 0.0;
		finger2->jnt_force[jnt] = 0.0;
		finger2->past_jnt_pos[jnt] = 0.0;
	}
	//	sim->jnt_pos[ARM_M1] = 4 * PI / 4.0; sim->jnt_pos[ARM_M2] = PI / 4.0;
	for (int crd = 0; crd < DIM3; crd++) {
		finger2->ref_eff_pos[crd] = 0.0;
		finger2->eff_pos[crd] = 0.0;
		finger2->eff_force[crd] = 0.0;
	}
	// ODE
	sim->destroyRobot();	// ロボット破壊
	sim->destroyObject();	// 対象破壊
	dJointGroupDestroy(sim->contactgroup);     // ジョイントグループの破壊
	sim->contactgroup = dJointGroupCreate(0);  // ジョイントグループの生成
#if 0
	createRobot(sim);                      // ロボットの生成
#else
	sim->setup();

	sim->createRobot();
#endif
	//	createObject(sim);                      // 衝突対象の生成
	sim->createObject();
}

////////////////////////////////////////////////////////
// コマンド実行
////////////////////////////////////////////////////////
int exeCmd(int argc, char* argv[])
{
	auto sim = EntityManager::get();
	int   command;
	char  buf[BUFSIZE];
	static int hist = 0;   // コマンド保存
	static int incount = 0;
	int	flag_quit = 0;

	while (1) {
#if 0
		// コマンド表示 & 取得
		getChar(buf, "command > ");
		command = atoi(buf);      // アスキーコードに変換
		sscanf(buf, "%c", &command);
		// コマンド処理
		switch (command) {
			//			case 'd': drawData(); break;
		case 'h': showHelp(); break;
		case 'q': flag_quit = 1;	// 終了判定
		default:;
		}
		if (flag_quit)	break;
#endif

		// シミュレーションループ
#if FLAG_DRAW_SIM
		DrawStuff DS;
		//dsSimulationLoop(argc, argv, DISPLAY_WIDTH, DISPLAY_HEIGHT, DS.getFn());
		dsSimulationLoop(argc, argv, 1280, 960, DS.getFn());
#else
		while (1) {
			simLoop(0);
			if (_this->step == DATA_CNT_NUM)	break;				// 終了設定
		}
#endif
		// ファイル保存
		sprintf(sim->getFinger()->data_file_name, FILENAME_DATA, incount);		// ファイル名を連番に設定
		sprintf(sim->getFinger()->filename_info, FILENAME_INFO, incount);		// ファイル名を連番に設定
		sim->getFinger()->saveData();
		sim->getFinger()->saveInfo();
		sim->getFinger()->saveGraph();

		//// ファイル保存
		//sprintf(sim->getFinger2()->data_file_name, FILENAME_DATA, incount);		// ファイル名を連番に設定
		//sprintf(sim->getFinger2()->filename_info, FILENAME_INFO, incount);		// ファイル名を連番に設定
		//sim->getFinger2()->saveData();
		//sim->getFinger2()->saveInfo();
		//sim->getFinger2()->saveGraph();

		// シミュレーションリスタート
		restart();
		// インクリメント
		incount++;

		// デバッグ用（1回で終了）
		break;
	}
	// ファイル保存
//	if(incount != 0)	saveGraph2(incount);
#if	FLAG_SAVE_VIDEO
	final_video();
#endif
	return 0;
}
