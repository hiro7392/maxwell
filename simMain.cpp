
#include"finger.hpp"
#include"filter.h"
#include"setJoint.h"
#include"simCmd.h"
////////////////////////////////////////////////////////
// 描画設定
// main関数
////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
	filterTest();
	auto sim = EntityManager::init();
#if FLAG_DRAW_SIM
//	setDrawStuff();		//ドロースタッフ
#endif

	// 環境設定
	sim->setEnv();
	

	// 物体生成
	sim->ground = dCreatePlane(sim->getSpace(), 0, 0, 1, 0);		// 地面の設定
	sim->setup();

	// パラメータ設定 指1
	auto Finger1 = EntityManager::get()->getFinger();
	for (int jnt = 0; jnt < ARM_JNT; jnt++)Finger1->init_jnt_pos[jnt] = init_jnt_pos[jnt];

	for (int crd = 0; crd < DIM3; crd++) {
		Finger1->init_obj_pos[crd] = Finger1->init_obj_pos[crd];
		for (int axis = 0; axis < DIM3; axis++)	Finger1->init_obj_att[axis][crd] = Finger1->init_obj_att[axis][crd];
	}

	//パラメータ設定 指2
	auto Finger2 = EntityManager::get()->getFinger2();
	for (int jnt = 0; jnt < ARM_JNT; jnt++)	Finger2->init_jnt_pos[jnt] = init_jnt_posF2[jnt];
	for (int crd = 0; crd < DIM3; crd++) {
		Finger2->init_obj_pos[crd] = Finger2->init_obj_pos[crd];
		for (int axis = 0; axis < DIM3; axis++)	Finger2->init_obj_att[axis][crd] = init_obj_att[axis][crd];
	}
	sim->createRobot();
	sim->createObject();
	//  乱数種設定
	//	dRandSetSeed(0);
	// コマンド開始
	exeCmd(argc, argv);		// 内部でdsSimulationLoop()を実行

#if	GLAPHIC_OPENGL
	// グラフィック表示
	glutInitWindowPosition(200, 100);
	glutInitWindowSize(600, 600);
	glutInit(&argc, (char**)argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow((char*)argv[0]);
	glutDisplayFunc(display);
	glutReshapeFunc(resize);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
	init();
	glutMainLoop();
#endif
	return 0;
}



////////////////////////////////////////////////////////
// シミュレーションループ
////////////////////////////////////////////////////////
void DrawStuff::simLoop(int pause)
{
	auto _this = EntityManager::get()->getFinger();
#if finger2_use
	//二本目の指用　kawahara
	auto _this2 = EntityManager::get()->getFinger2();
	
#endif
#if print_debug
	_this->printInfo();
	_this2->printInfo();
#endif
	auto sim = EntityManager::get();

#if usePlateToGrasp
	// 把持対象のプレートの描画
	dsSetColorAlpha(0.3, 0.3, 0.5, 1);
	pos2 = dBodyGetPosition(plateToGrasp.body);
	R2 = dBodyGetRotation(plateToGrasp.body);
	dReal sides[3] = { plateX,plateY,plateZ };
	dsDrawBox(pos2, R2, sides);  // plateの描画
#endif
	if (!pause) {
		auto sensor = _this->getParts()[3];
		auto sensor2 = _this2->getParts()[3];

		// 状態取得
		for (int jnt = 0; jnt < ARM_JNT; jnt++) {
			_this->jnt_pos[jnt] = dJointGetHingeAngle(_this->r_joint[jnt]) + _this->init_jnt_pos[jnt];	// 関節位置（x軸が基準角0）
			_this->jnt_vel[jnt] = dJointGetHingeAngleRate(_this->r_joint[jnt]);	// 関節速度
#if finger2_use
			//追加 kawahara
			_this2->jnt_pos[jnt] = dJointGetHingeAngle(_this2->r_joint[jnt]) + _this2->init_jnt_pos[jnt];	// 関節位置（x軸が基準角0）
			_this2->jnt_vel[jnt] = dJointGetHingeAngleRate(_this2->r_joint[jnt]);	// 関節速度
#endif
		}
		//　旋回関節角の角度と角速度を取得
		_this->senkai_base_jnt = dJointGetHingeAngle(_this->senkai_link_joint);
		_this->senkai_base_vel = dJointGetHingeAngleRate(_this->senkai_link_joint);

		_this2->senkai_base_jnt = dJointGetHingeAngle(_this2->senkai_link_joint);
		_this2->senkai_base_vel = dJointGetHingeAngleRate(_this2->senkai_link_joint);
		printf("fingerID =%d senkai_angle=%lf senkai speed =%lf\n", _this->fingerID, radToAng(_this->senkai_base_jnt),radToAng(_this->senkai_base_vel));
		printf("fingerID =%d senkai_angle=%lf senkai speed =%lf\n", _this2->fingerID, radToAng(_this2->senkai_base_jnt), radToAng(_this2->senkai_base_vel));

		dBodyGetRelPointPos(sensor->getBody(), 0.0, 0.0, sensor->getl() / 2.0, _this->eff_pos);			// 手先位置
		dBodyGetRelPointVel(sensor->getBody(), 0.0, 0.0, sensor->getl() / 2.0, _this->eff_vel);			// 手先速度
#if finger2_use
		dBodyGetRelPointPos(sensor2->getBody(), 0.0, 0.0, sensor2->getl() / 2.0, _this2->eff_pos);			// 手先位置
		dBodyGetRelPointVel(sensor2->getBody(), 0.0, 0.0, sensor2->getl() / 2.0, _this2->eff_vel);			// 手先速度
#endif

		//関節にかかるトルクを取得する
		//_this->p_force = dJointGetFeedback(_this->f2_joint);	//参考: もともとのmaxwell制御
		_this->p_force = dJointGetFeedback(_this->sensor2FingerTop);
#if finger2_use
		_this2->p_force = dJointGetFeedback(_this2->sensor2FingerTop);
#endif
		//	旋回関節にかかっているトルクを取得する
		_this->senkai_p_force = dJointGetFeedback(_this->senkai_base_joint);
		_this2->senkai_p_force = dJointGetFeedback(_this2->senkai_base_joint);
		printf("fingerID =%d senkai_torque=(%lf,%lf,%lf) \n", _this->fingerID,_this->senkai_p_force->t1[0], _this->senkai_p_force->t1[1],_this->senkai_p_force->t1[2]);
		printf("fingerID =%d senkai_torque=(%lf,%lf,%lf) \n", _this2->fingerID, _this2->senkai_p_force->t1[0], _this2->senkai_p_force->t1[1], _this2->senkai_p_force->t1[2]);

		for (int crd = 0; crd < DIM3; crd++) {
			_this->eff_force[crd] = -_this->p_force->f1[crd];					// 対象がセンサに及ぼしている力=センサが関節に及ぼしている力
#if finger2_use
			_this2->eff_force[crd] = -_this2->p_force->f1[crd];
#endif
		}
#if addSensorNoise
		for (int crd = 0; crd < DIM3; crd++) {
			//_this->eff_force[crd] += sdlab_normal(NOISE_AVES[crd], NOISE_DISTRIBUTES[crd]);			
			//_this2->eff_force[crd] += sdlab_normal(NOISE_AVES[crd], NOISE_DISTRIBUTES[crd]);		//qガウス分布のノイズを付加
			_this->eff_force[crd] += sdlab_normal(0,6);			//平均0,標準偏差0.5のガウス分布のノイズを付加
			_this2->eff_force[crd] += sdlab_normal(0,6);		//平均0,標準偏差0.5のガウス分布のノイズを付加

		}
#endif		
		auto entity = EntityManager::get();
		// 距離計算	//
		_this->calcDist();
#if finger2_use
		_this2->calcDist();
#endif

		// 外力設定
#if SIM_ADD_EXT_FORCE
		//_this->addExtForce();
#if finger2_use
		//_this2->addExtForce();
#endif
#endif
		// ODE摩擦手動設定(粘性摩擦)
		_this->setJntFric();
#if finger2_use
		_this2->setJntFric();
#endif
		// 力計算
		_this->control();
#if finger2_use
		_this2->control();
#endif

#if usePlateToGrasp
		//plateを二次元平面に拘束する=>z==0に毎回戻す
		//plateの角速度
		const dReal* rot = dBodyGetAngularVel(plateToGrasp.body);
		const dReal* quat_ptr;
		dReal quat[4], quat_len;

		quat_ptr = dBodyGetQuaternion(plateToGrasp.body);
		quat[0] = quat_ptr[0];
		quat[1] = 0.0;
		quat[2] = 0.0;
		quat[3] = 0.0;// quat_ptr[3];	//外力による回転を無視したいので0にする
		
		//現在の位置
		const dReal* nowPos = dBodyGetPosition(plateToGrasp.body);
		
		static double beforeXpos; 
		if(sim->step==0)beforeXpos=nowPos[0];
		//速度,角度
		dBodySetQuaternion(plateToGrasp.body, quat);
		dBodySetAngularVel(plateToGrasp.body, 0, 0, rot[2]);
		dBodySetPosition(plateToGrasp.body, beforeXpos, nowPos[1], capZ);
		beforeXpos = nowPos[0];

		//x座標を記録
		//plateの端に外力を加える

		if(ADD_EXT_FORCE && sim->step > 1000 && sim->step <= 1500) {
			static int duty = 30000;	//外力の向きの周期
			static int forceVal = 5;
			int forceDir = -1;// ((sim->step % duty) <= duty / 2) ? 1 : -1;
			int forceDirX = 0;// (sim->step % duty <= duty / 4) ? -1 : 1;
			double distFromCenter = 0.0;
			// distFromCenter = 0.2; duty=100,forceVal=30,1点のみについて力を加える
			//外力ベクトル(x,y,z),加える位置(x,y,z)
			dBodyAddForceAtPos(plateToGrasp.body, forceVal/2.0 * forceDirX, forceVal * forceDir, 0.0, nowPos[0] - distFromCenter, nowPos[1], nowPos[2]);
			dVector3 ext_f{ 0, forceVal *(forceDir), 0.0 };
			//drawArrowOriginal(dVector3{ nowPos[0] - distFromCenter, nowPos[1], nowPos[2]+0.5 }, dVector3{nowPos[0]-distFromCenter-ext_f[0]*0.3, nowPos[1] - ext_f[1] * 0.3, nowPos[2]+0.5 - ext_f[2] * 0.3 }, ext_f);
		}
#endif
		// 過去データとして代入
		for (int jnt = 0; jnt < ARM_JNT; jnt++)	_this->past_jnt_pos[jnt] = _this->jnt_pos[jnt];
		matCopy(&_this->var_prev2.r, &_this->var_prev.r); matCopy(&_this->var_prev2.dr, &_this->var_prev.dr);
		matCopy(&_this->var_prev.r, &_this->var.r); matCopy(&_this->var_prev.dr, &_this->var.dr);
#if finger2_use
		// 二本目の指　kawahara
		for (int jnt = 0; jnt < ARM_JNT; jnt++)	_this2->past_jnt_pos[jnt] = _this2->jnt_pos[jnt];
		matCopy(&_this2->var_prev2.r, &_this2->var_prev.r); matCopy(&_this2->var_prev2.dr, &_this2->var_prev.dr);
		matCopy(&_this2->var_prev.r, &_this2->var.r); matCopy(&_this2->var_prev.dr, &_this2->var.dr);
#endif
		// 現在値を保存領域へコピー
		//_this->state_contact = 0;
#if finger2_use
		//_this2->state_contact = 0;
#endif	
		// シミュレーションを１ステップ進行
		entity->update();
		sim->step++;
#if FLAG_DRAW_SIM
		// 終了設定
		if (sim->step == DATA_CNT_NUM+5)	dsStop();	//csv出力したら終わり
#endif
	}
#if FLAG_DRAW_SIM
	//	drawRobot(); // ロボットの描画
	_this->draw();
#if finger2_use
	_this2->draw();
#endif
#if usePlateToGrasp
	_this2->plate.draw();
#endif
	
	//力覚センサの出力用
	if (sim->step < DATA_CNT_NUM) {
		_this->setNums(sim->step);
		_this2->setNums(sim->step);
	}else if(sim->step == DATA_CNT_NUM) {
		//	外力を出力
		_this->outputForce();
		_this2->outputForce();
		//	関節角を出力
		_this->outputJntAngle(DATA_CNT_NUM);
		_this2->outputJntAngle(DATA_CNT_NUM);
	}
	
	std::cout << "step:" << sim->step << std::endl;

#if SIM_OBJ_IMPACT
	//	drawObject(); // 衝突対象の描画
	_this->getObj()->draw();
#elif SIM_ADD_EXT_FORCE
	//drawExtForce();		// 外力の描画(指1と指2)
	//drawExtForcePlate(); // 外力の描画(把持物体)
#endif

#endif
#if FLAG_DRAW_SIM & FLAG_SAVE_IMAGE
					// 画像保存
	if (!pause) {
		if (_this->step % SAVE_IMG_RATE == 0)	saveImage(640, 480);	// SAVE_IMG_RATE毎に画像保存
	}
#endif
#if	FLAG_SAVE_VIDEO
	if (sim->step % SAVE_VIDEO_RATE == 0)	save_video();
#endif
	//衝突している物体の集合を空にする
	//dJointGroupEmpty(contactgroup);
}

////////////////////////////////////////////////////////
// ODE初期設定
////////////////////////////////////////////////////////
void DrawStuff::start() {
#if 0
	xyz[0] = -0.2;	xyz[1] = 2.5;	xyz[2] = 0.5;
	hpr[0] = -90.0;	hpr[1] = 0.0;	hpr[2] = 0.0;	// +yからの視点(右が-x,上が+z)
#elif 0
	xyz[0] = 2.5;	xyz[1] = 0.2;	xyz[2] = 0.5;
	hpr[0] = -180.0;	hpr[1] = 0.0;	hpr[2] = 0.0;	// +xからの視点(右が+y,上が+z)
#elif 1
	xyz[0] = -2.0;	xyz[1] =-0.5;	xyz[2] = 3.0;
	//xyz[0] = -2.0;	xyz[1] = 1.5;	xyz[2] = 3.0;
	hpr[0] = 180.0;	hpr[1] = -120.0;	hpr[2] = 180;	// +zからの視点(右が+x,上が+y)
#endif
	dsSetViewpoint(xyz, hpr);               // 視点，視線の設定
	dsSetSphereQuality(3);					// 球の品質設定
#if	FLAG_SAVE_VIDEO
	init_video();
#endif
	//capX = -1.2, capY = -0.50, capZ = 0.3;
}

////////////////////////////////////////////////////////
// キーボードコマンドの処理関数
////////////////////////////////////////////////////////
void DrawStuff::command(int cmd) {
	auto _this = EntityManager::get();
	switch (cmd) {
	case 'x': xyz[0] += 0.1; dsSetViewpoint(xyz, hpr);	break;		// x方向
	case 'X': xyz[0] -= 0.1; dsSetViewpoint(xyz, hpr);	break;		// -x方向
	case 'y': xyz[1] += 0.1; dsSetViewpoint(xyz, hpr);	break;		// y方向
	case 'Y': xyz[1] -= 0.1; dsSetViewpoint(xyz, hpr);	break;		// -y方向
	case 'z': xyz[2] += 0.1; dsSetViewpoint(xyz, hpr);	break;		// z方向
	case 'Z': xyz[2] -= 0.1; dsSetViewpoint(xyz, hpr);	break;		// -z方向
//			case 'u':	dBodyAddForce(_this->getObj()->getBody(), 500.0, 0.0, 0.0);	break;
	case 'u':	_this->setAddForceObj(500.0, 0.0, 0.0);	break;
	case 'r':	restart();	break;
	case 'q': {
		//各変数を出力する
		_this->getFinger()->outputJntAngle(_this->step);
		_this->getFinger2()->outputJntAngle(_this->step);
		
		dsStop();	break;
	}
	default: std::cout << "key missed" << std::endl; break;
	}
}
