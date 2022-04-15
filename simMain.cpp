
#include"finger.hpp"
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
		dsSimulationLoop(argc, argv,1280, 960, DS.getFn());
#else
		while (1) {
			simLoop(0);
			if (_this->step == DATA_CNT_NUM)	break;				// 終了設定
		}
#endif
		// ファイル保存
		//sprintf(sim->getFinger()->data_file_name, FILENAME_DATA, incount);		// ファイル名を連番に設定
		//sprintf(sim->getFinger()->filename_info, FILENAME_INFO, incount);		// ファイル名を連番に設定
		//sim->getFinger()->saveData();
		//sim->getFinger()->saveInfo();
		//sim->getFinger()->saveGraph();

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


////////////////////////////////////////////////////////
// 実体定義
////////////////////////////////////////////////////////
cParts::cParts(dReal m) : m(m) { 
	this->body = dBodyCreate(EntityManager::get()->getWorld()); dMassSetZero(&mass);
	//this->geom = dCreateBox(EntityManager::get()->getSpace(), 1.0, 1.0, 1.0);
}


cParts::cParts(dReal m, Vec3 init_pos) : cParts(m) {
	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
}

cPartsBox::cPartsBox(dReal m, Vec3 l) : cParts(m), sides{ l.x, l.y, l.z } {
	dMassSetBoxTotal(&this->mass, this->m, this->sides[CRD_X], this->sides[CRD_Y], this->sides[CRD_Z]);
	dBodySetMass(this->body, &mass);
	this->geom = dCreateBox(EntityManager::get()->getSpace(), this->sides[CRD_X], this->sides[CRD_Y], this->sides[CRD_Z]);
	dGeomSetBody(this->geom, this->body);
}

cPartsBox::cPartsBox(dReal m, Vec3 init_pos, Vec3 l) : cPartsBox(m, l) {
	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
	//this->geom = dCreateBox(EntityManager::get()->getSpace(), init_pos.x, init_pos.y, init_pos.z);
}

cPartsCylinder::cPartsCylinder(dReal m, dReal l, dReal r) : cParts(m), l(l), r(r) {

	dMassSetCylinderTotal(&this->mass, this->m, DIR_LONG_AXIS_Z, this->r, this->l);
	dBodySetMass(this->body, &mass);
	//ジオメトリの生成
	this->geom = dCreateCylinder(EntityManager::get()->getSpace(), this->r, this->l);
	dGeomSetBody(this->geom, this->body);
}

cPartsCapsule::cPartsCapsule(dReal m, dReal l, dReal r) : cParts(m), l(l), r(r) {

	//	ここからカプセル関連
	dMassSetZero(&mass);
	
	//	Cylinderの部分とほぼ同じ
	//this->body = dBodyCreate(EntityManager::get()->getWorld());
	dMassSetCapsuleTotal(&this->mass, this->m, DIR_LONG_AXIS_Z, this->r, this->l);
	dBodySetMass(this->body, &mass);

	//	ジオメトリの生成
	this->geom = dCreateCapsule(EntityManager::get()->getSpace(), this->r, this->l);
	dGeomCapsuleSetParams(this->geom, this->r, this->l);
	//ジオメトリとボディの対応付け
	dGeomSetBody(this->geom, this->body);

}

cPartsCylinder::cPartsCylinder(dReal m, Vec3 init_pos, dReal l, dReal r) : cPartsCylinder(m, l, r) {		// デフォルトコンストラクタ
	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
}

//各関節を結合
//HingeJoint:=稼働する関節
void cFinger::setJoint() {

	auto sim = EntityManager::get();

	// 固定ジョイント
	f_joint = dJointCreateFixed(sim->getWorld(), 0);
	dJointAttach(f_joint, finger[0]->getBody(), 0);
	dJointSetFixed(f_joint);

	// ヒンジジョイント1
	r_joint[ARM_M1] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M1], finger[1]->getBody(), finger[0]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M1], x0, y0, 0.4 / 2);
	dJointSetHingeAxis(r_joint[ARM_M1], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);

	// ヒンジジョイント2
	r_joint[ARM_M2] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M2], finger[2]->getBody(), finger[1]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M2], x0 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]), y0 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]), 0.4 / 2);
	dJointSetHingeAxis(r_joint[ARM_M2], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M2], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M2], dParamHiStop, M_PI);

	// 固定ジョイント
	f2_joint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(f2_joint, finger[3]->getBody(), finger[2]->getBody());
	dJointSetFixed(f2_joint);

	// 固定ジョイント	センサと指先の円柱	
	sensor2FingerTop = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(sensor2FingerTop, fingerTopCapsule.getBody(), finger[3]->getBody());
	dJointSetFixed(sensor2FingerTop);

	// センサ設定（力とトルクの取得に必要）
	dJointSetFeedback(sensor2FingerTop, &force);

#if useForceContactPoint
	//指先カプセルとセンサを接続	//固定ジョイント
	FingerTop2ForcePoint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(FingerTop2ForcePoint,forceContactPoint.getBody(), fingerTopCapsule.getBody());
	dJointSetFixed(FingerTop2ForcePoint);

	dJointSetFeedback(FingerTop2ForcePoint, &fingerTop2ForcePoint_joint);
#endif
	// センサ設定（力とトルクのに必要）
	//dJointSetFeedback(f2_joint, &force);

	
}
//把持物体の初期位置
dReal capX = -1.2, capY = -0.60, capZ = 0.3;
//把持物体の大きさ
const dReal plateX = 1.5, plateY = 0.50, plateZ = 0.4;


//二本目の指の初期位置設定
void cFinger::setJoint2() {

	//実際に把持する用のプレートの生成
#if usePlateToGrasp
	plateToGrasp.body= dBodyCreate(EntityManager::get()->getWorld());
	dMassSetBox(&mass, DENSITY, plateX, plateY, plateZ);

	//Bodyで位置と質量の設定
	dMass massPlate;
	dReal newMass=0.5;
	dMassSetZero(&massPlate);
	dMassSetBoxTotal(&massPlate, newMass, plateX,plateY, plateZ);
	dBodySetPosition(plateToGrasp.body, capX, capY, capZ);	//位置 //x=-1.1
	//ジオメトリの生成
	auto geomBodyPlate = dCreateBox(EntityManager::get()->getSpace(), plateX, plateY, plateZ);
	//	動力学Bodyと衝突計算ジオメトリの対応
	dGeomSetBody(geomBodyPlate, plateToGrasp.body);
#endif
	auto sim = EntityManager::get();

	

	// 固定ジョイント
	f_joint = dJointCreateFixed(sim->getWorld(), 0);
	dJointAttach(f_joint, finger[0]->getBody(), 0);
	dJointSetFixed(f_joint);

	// ヒンジジョイント1
	r_joint[ARM_M1] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M1], finger[1]->getBody(), finger[0]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M1], x1, y1, 0.4 / 2);
	dJointSetHingeAxis(r_joint[ARM_M1], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);

	// ヒンジジョイント2
	r_joint[ARM_M2] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M2], finger[2]->getBody(), finger[1]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M2], x1 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]), y1 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]), 0.4 / 2);
	dJointSetHingeAxis(r_joint[ARM_M2], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);

	// 固定ジョイント
	f2_joint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(f2_joint, finger[3]->getBody(), finger[2]->getBody());
	dJointSetFixed(f2_joint);

	// 固定ジョイント	センサと指先の円柱	
	sensor2FingerTop = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(sensor2FingerTop, fingerTopCapsule.getBody(), finger[3]->getBody());
	dJointSetFixed(sensor2FingerTop);

	// センサ設定（力とトルクの取得に必要）
	dJointSetFeedback(sensor2FingerTop, &force);

#if useForceContactPoint
	//指先カプセルとセンサを接続	//固定ジョイント
	FingerTop2ForcePoint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(FingerTop2ForcePoint, forceContactPoint.getBody(), fingerTopCapsule.getBody());
	dJointSetFixed(FingerTop2ForcePoint);

	dJointSetFeedback(FingerTop2ForcePoint, &fingerTop2ForcePoint_joint);
#endif

	// センサ設定（力とトルクの取得に必要）
	//dJointSetFeedback(f2_joint, &force);

}
////////////////////////////////////////////////////////
// 描画設定
// main関数
////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
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
	// 乱数種設定
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
	dsSetColorAlpha(1, 1, 1, 1);
	pos2 = dBodyGetPosition(plateToGrasp.body);
	R2 = dBodyGetRotation(plateToGrasp.body);
	dReal sides[3] = { plateX,plateY,plateZ };
	dsDrawBox(pos2, R2, sides);  // plateの描画
#endif
	//dSpaceCollide(EntityManager::get()->getSpace(), 0, &nearCallback);
	if (!pause) {
		auto sensor = _this->getParts()[3];
		auto sensor2 = _this2->getParts()[3];
		//auto obj = EntityManager::get()->getObj();

		// 初期設定
//#if SIM_OBJ_IMPACT
//		if (_this->step == 0)	dBodySetLinearVel(obj.body, _this->init_obj_att[AXIS_X][CRD_X] * SIM_OBJ_INIT_ABS_VEL, _this->init_obj_att[AXIS_X][CRD_Y] * SIM_OBJ_INIT_ABS_VEL, _this->init_obj_att[AXIS_X][CRD_Z] * SIM_OBJ_INIT_ABS_VEL);		// 対象速度
//		if (_this->step == 0)	dBodySetAngularVel(obj.body, _this->init_obj_att[AXIS_Z][CRD_X] * SIM_OBJ_INIT_ABS_VEL / OBJ_RADIUS, _this->init_obj_att[AXIS_Z][CRD_Y] * SIM_OBJ_INIT_ABS_VEL / OBJ_RADIUS, _this->init_obj_att[AXIS_Z][CRD_Z] * SIM_OBJ_INIT_ABS_VEL / OBJ_RADIUS);		// 対象角速度
//#elif SIM_ADD_EXT_FORCE
//		if (sim->step == 0) {
//			//objが邪魔なのでいったんコメントアウト
//			//dBodyDisable(obj->getBody());		// 対象無効化			
//			//dBodyDisable(obj2->getBody());		// 対象無効化
//		}
//#endif
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
		dBodyGetRelPointPos(sensor->getBody(), 0.0, 0.0, sensor->getl() / 2.0, _this->eff_pos);			// 手先位置
		dBodyGetRelPointVel(sensor->getBody(), 0.0, 0.0, sensor->getl() / 2.0, _this->eff_vel);			// 手先速度
#if finger2_use
		dBodyGetRelPointPos(sensor2->getBody(), 0.0, 0.0, sensor2->getl() / 2.0, _this2->eff_pos);			// 手先位置
		dBodyGetRelPointVel(sensor2->getBody(), 0.0, 0.0, sensor2->getl() / 2.0, _this2->eff_vel);			// 手先速度
#endif
		//関節のフィードバックを反映
		//_this->p_force = dJointGetFeedback(_this->f2_joint);	//参考: もともとのmaxwell制御
		//関節にかかるトルクを取得する
		_this->p_force = dJointGetFeedback(_this->sensor2FingerTop);
#if finger2_use
		_this2->p_force = dJointGetFeedback(_this2->sensor2FingerTop);

#endif
		for (int crd = 0; crd < DIM3; crd++) {
			_this->eff_force[crd] = -_this->p_force->f1[crd];					// 対象がセンサに及ぼしている力=センサが関節に及ぼしている力
			
			//この部分はなくても動く
			//_this->obj_pos[crd] = (dBodyGetPosition(obj->getBody()))[crd];		// 対象位置
			//_this->obj_vel[crd] = (dBodyGetLinearVel(obj->getBody()))[crd];		// 対象速度
			
#if finger2_use
			_this2->eff_force[crd] = -_this2->p_force->f1[crd];
			//_this2->obj_pos[crd] = (dBodyGetPosition(obj2->getBody()))[crd];		// 対象位置
			//_this2->obj_vel[crd] = (dBodyGetLinearVel(obj2->getBody()))[crd];		// 対象速度
#endif
		}
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
		//正規化
		/*quat_len = sqrt(quat[0] * quat[0] + quat[3] * quat[3]);
		quat[0] /= quat_len;
		quat[3] /= quat_len;*/

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
		if (sim->step > 10 && sim->step<=500) {

			static int duty = 1500;	//外力の向きの周期
			static int forceVal = 5;
			int forceDir = (sim->step % duty <= duty/2) ? -1 : 1;
			//double  forceDir = sin(sim->step*((double)2.0*PI/duty));
			int forceDirX = 0;// (sim->step % duty <= duty / 4) ? -1 : 1;
			//int forceDir = -1;
			double distFromCenter = 0.0;
			// distFromCenter = 0.2; duty=100,forceVal=30,1点のみについて力を加える
			//外力ベクトル(x,y,z),加える位置(x,y,z)
			dBodyAddForceAtPos(plateToGrasp.body, forceVal/2.0 * forceDirX, forceVal * forceDir, 0.0, nowPos[0] - distFromCenter, nowPos[1], nowPos[2]);
			//dBodyAddForceAtPos(plateToGrasp.body, forceVal / 2.0 * forceDirX, forceVal* forceDir, 0.0, nowPos[0] + distFromCenter, nowPos[1], nowPos[2]);
			//printf("forceDir = %lf\n", forceDir);
			dVector3 ext_f{ 0, forceVal *(forceDir), 0.0 };
			drawArrowOriginal(dVector3{ nowPos[0] - distFromCenter, nowPos[1], nowPos[2]+0.5 }, dVector3{nowPos[0]-distFromCenter-ext_f[0]*0.3, nowPos[1] - ext_f[1] * 0.3, nowPos[2]+0.5 - ext_f[2] * 0.3 }, ext_f);
		}
#endif

		// 過去データとして代入
		for (int jnt = 0; jnt < ARM_JNT; jnt++)	_this->past_jnt_pos[jnt] = _this->jnt_pos[jnt];
		matCopy(&_this->var_prev2.r, &_this->var_prev.r); matCopy(&_this->var_prev2.dr, &_this->var_prev.dr);
		matCopy(&_this->var_prev.r, &_this->var.r); matCopy(&_this->var_prev.dr, &_this->var.dr);

		// 二本目の指　kawahara
#if finger2_use
		for (int jnt = 0; jnt < ARM_JNT; jnt++)	_this2->past_jnt_pos[jnt] = _this2->jnt_pos[jnt];
		matCopy(&_this2->var_prev2.r, &_this2->var_prev.r); matCopy(&_this2->var_prev2.dr, &_this2->var_prev.dr);
		matCopy(&_this2->var_prev.r, &_this2->var.r); matCopy(&_this2->var_prev.dr, &_this2->var.dr);
#endif
		// 現在値を保存領域へコピー
		//copyData(_this.get());
		//copyData(_this2.get());
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
		//if (sim->step == 10000)	dsStop();
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
	//初期位置を出力(制約条件付きでは平衡点として扱う)
	std::cout << "F1 initial position r= " << std::endl;
	matPrint(&_this->var_init.r);

	std::cout << "F2 initial position r= "<< std::endl;
	matPrint(&_this2->var_init.r);
	
	

	//力覚センサの出力用
	if (sim->step < DATA_CNT_NUM) {
		_this->setNums(sim->step);
		_this2->setNums(sim->step);
	}else if(sim->step == DATA_CNT_NUM) {
		//外力を出力
		_this->outputForce();
		_this2->outputForce();

		//関節角を出力
		_this->outputJntAngle();
		_this2->outputJntAngle();
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
	if (_this->step % SAVE_VIDEO_RATE == 0)	save_video();
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
	xyz[0] = -1.2;	xyz[1] =-0.5;	xyz[2] = 3.5;
	hpr[0] = 0.0;	hpr[1] = -90.0;	hpr[2] = 180;	// +zからの視点(右が+x,上が+y)
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
	case 'q':	dsStop();	break;
	default: std::cout << "key missed" << std::endl; break;
	}
}
