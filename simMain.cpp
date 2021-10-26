//#pragma comment(lib,"C:/ode-0.13/lib/DebugDoubleDLL/ode_doubled.lib")
//#pragma comment(lib,"C:/ode-0.13/lib/DebugDoubleDLL/drawstuffd.lib")

#pragma comment(lib,"C:\\ode-0.16.1\\lib\\DebugDoubleDLL\\ode_doubled.lib")
#pragma comment(lib,"C:\\ode-0.16.1\\lib\\DebugDoubleDLL\\drawstuffd.lib")

#include "texturepath.h"
#include "simMain.h"
#include "simMainF2.h"

#include "simMode.h"
#include "setEnv.h"
#include "makeRobot.h"
#include "command.h"
#include "matBase.h"
#include "setRobot.h"
#include "ctrlRobot.h"

#if	GLAPHIC_OPENGL 
#include <GL/glut.h>	// stdlib.hより後に読み込む必要あり
#include "graphic.h"
#endif
#if	FLAG_SAVE_VIDEO
#include "video.h"
#endif

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#define finger2_use 1
#define print_debug 0

// クラス静的メンバの初期化
dsFunctions DrawStuff::fn;
float DrawStuff::xyz[3];
float DrawStuff::hpr[3];

// 初期設定変数
//double	init_jnt_pos[ARM_JNT] = {3*PI/4.0, PI/2.0};	// ロボット初期姿勢
double	init_jnt_pos[ARM_JNT] = { 4 * PI / 4.0, PI / 4.0 };	// ロボット初期姿勢

#if SIM_OBJ_CASE1
double	init_obj_pos[DIM3] = {-0.8-2*0.75/sqrt(2.0), -0.035, OBJ_RADIUS};	// 対象初期位置
double	init_obj_att[DIM3][DIM3] = { {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0} };	// 回転軸がy方向
#elif SIM_OBJ_CASE2
double	init_obj_pos[DIM3] = {-2*0.75/sqrt(2.0)-0.035, -0.8, OBJ_RADIUS};	// 対象初期位置
double	init_obj_att[DIM3][DIM3] = { {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}, {-1.0, 0.0, 0.0} };	// 回転軸がx方向
#elif SIM_OBJ_CASE3
double	init_obj_pos[DIM3] = {-0.8/sqrt(2.0)-2*0.75/sqrt(2.0), -0.8/sqrt(2.0), OBJ_RADIUS};	// 対象初期位置
double	init_obj_att[DIM3][DIM3] = { {sqrt(2.0)/2, sqrt(2.0)/2, 0.0}, {0.0, 0.0, -1.0}, {-sqrt(2.0)/2, sqrt(2.0)/2, 0.0} };	// 回転軸がx方向
#endif

#define practice 0			//ODE OpenGLの練習用 kawahara
////////////////////////////////////////////////////////
// 関節制御
// 戻り値：関節一般化力（直動関節では力，回転関節ではトルク）
////////////////////////////////////////////////////////
void cFinger::control() {
	auto entity = EntityManager::get();

	Matrix	tau;
	// 初期化

	if (entity->step == 0) {
		matInit(&tau, 2, 1);
		// 初期化
		ctrlInitErr();		// パラメータ誤差を追加
		// 初期値保存
		for (int crd = 0; crd < DIM2; crd++) {
			var_init.q.el[crd][0] = jnt_pos[crd]; var_init.dq.el[crd][0] = jnt_vel[crd] = 0.0;
			var_init.r.el[crd][0] = eff_pos[crd]; var_init.dr.el[crd][0] = eff_vel[crd] = 0.0;
			var_init.F.el[crd][0] = eff_force[crd] = 0.0;
		}
	}

	// 手先変数代入
	for (int crd = 0; crd < DIM2; crd++) {
		var.r.el[crd][0] = eff_pos[crd]; var.dr.el[crd][0] = eff_vel[crd];
		var.F.el[crd][0] = eff_force[crd];
	}
	// 関節変数代入
	for (int jnt = 0; jnt < ARM_JNT; jnt++) {
		var.q.el[jnt][0] = jnt_pos[jnt]; var.dq.el[jnt][0] = jnt_vel[jnt];
	}

	// パラメータセット
	armDynPara();

	// インピーダンス設定
	// 制御指令計算
#if SIM_CTRL_MODE_MAXWELL & SIM_ADD_EXT_FORCE
	double	impM[] = { 2.0, 2.0 }, impC[] = { 4.0, 4.0 }, impK[] = { 40.0, 40.0 }, impK0[] = { 10.0, 10.0 };

	matSetValDiag(&imp.M, impM); matSetValDiag(&imp.C, impC); matSetValDiag(&imp.K, impK); matSetValDiag(&imp.K0, impK0);	// ゲイン設定
	ctrlPreProcessing();


#if 1
	if(fingerID==1)ctrlMaxwell(&tau);
	else {
		ctrlMaxwell2(&tau);
	}
	

//	ctrlMaxwellWithoutInertiaShaping(_this, &tau);
#elif 0
//	ctrlMaxwellConv(_this, &tau);
	ctrlMaxwellConvInnerLoop(_this, &tau);
//	ctrlMaxwellConvRK(_this, &tau);
//	ctrlMaxwellConvRK2(_this, &tau);
#else
	ctrlMaxwellInnerLoop(_this, &tau);
#endif
//	ctrlMaxwellE(_this, &tau);
//	ctrlMaxwellImplicit(_this, &tau);
//	ctrlMaxwellInnerLoopImplicit(_this, &tau);			// 制御指令計算
//	ctrlMaxwellInnerLoopJntSpace(_this, &tau);		// デバッグ中
//	ctrlSLS(_this, &tau);
//	ctrlMaxwellVar2(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Md, &Cd, &Kd);
//	ctrlMaxwellVar2(_this, &tau, &_this->dyn.Mq, &_this->dyn.h, &_this->kine.J, &_this->kine.dJ, &_this->var.q, &_this->var.dq, &re, &dre, &_this->var.F, &_this->imp.M, &_this->imp.C, &_this->imp.K);
//	ctrlMaxwellVar(_this, &tau);
#elif SIM_CTRL_MODE_MAXWELL & SIM_OBJ_IMPACT
//		_this->imp.m_d[CRD_X] = 0.5;	_this->imp.m_d[CRD_Y] = 0.5;	// 慣性
//		_this->imp.c_d[CRD_X] = 1.0;	_this->imp.c_d[CRD_Y] = 1.0;	// 粘性
//		_this->imp.k_d[CRD_X] = 10.0;	_this->imp.k_d[CRD_Y] = 10.0;	// 弾性
	_this->imp.M.el[CRD_X][CRD_X] = 0.5;	_this->imp.M.el[CRD_Y][CRD_Y] = 0.5;	// 慣性
	_this->imp.C.el[CRD_X][CRD_X] = 0.8;	_this->imp.C.el[CRD_Y][CRD_Y] = 0.8;	// 粘性
	_this->imp.K.el[CRD_X][CRD_X] = 5.2;	_this->imp.K.el[CRD_Y][CRD_Y] = 5.2;	// 弾性
	_this->imp.K0.el[CRD_X][CRD_X] = 10.0;	_this->imp.K0.el[CRD_Y][CRD_Y] = 10.0;	// 弾性(SLS並列バネ)
#elif SIM_CTRL_MODE_HYBRID & SIM_ADD_EXT_FORCE
	_this->imp.m_d[CRD_X] = 2.0;	_this->imp.m_d[CRD_Y] = 1.0;	// 慣性
	_this->imp.c_d[CRD_X] = 4.0;	_this->imp.c_d[CRD_Y] = 0.5;	// 粘性
	_this->imp.k_d[CRD_X] = 100.0;	_this->imp.k_d[CRD_Y] = 50.0;	// 弾性
	// MaxwellとVoigtの方向切替
	if (_this->step == IMP_SWITCH_STEP) {
		// 変数初期化
		for (crd = 0; crd<2; crd++)	Fint.el[crd][0] = 0.0;
		// 目標インピーダンス（手先）
		_this->imp.M.el[CRD_X][CRD_X] = 1.0;	_this->imp.M.el[CRD_Y][CRD_Y] = 2.0;	// 慣性
		_this->imp.C.el[CRD_X][CRD_X] = 0.5;	_this->imp.C.el[CRD_Y][CRD_Y] = 4.0;	// 粘性
		_this->imp.K.el[CRD_X][CRD_X] = 50.0;	_this->imp.K.el[CRD_Y][CRD_Y] = 100.0;	// 弾性
		for (crd = 0; crd<2; crd++) {
			_this->var_init.r.el[crd][0] = _this->eff_pos[crd];
}
	}
	ctrlHybrid(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Fint, &Md, &Cd, &Kd);
#elif 0
	_this->imp.m_d[CRD_X] = 200.0;	_this->imp.m_d[CRD_Y] = 2.0;	// 慣性
	_this->imp.c_d[CRD_X] = 4.0;	_this->imp.c_d[CRD_Y] = 4.0;	// 粘性
	_this->imp.k_d[CRD_X] = 10.0;	_this->imp.k_d[CRD_Y] = 100.0;	// 弾性
#elif 1
	_this->imp.m_d[CRD_X] = 10.0;	_this->imp.m_d[CRD_Y] = 10.0;	// 慣性
	_this->imp.c_d[CRD_X] = 100.0;	_this->imp.c_d[CRD_Y] = 1.0;	// 粘性
	_this->imp.k_d[CRD_X] = 10.0;	_this->imp.k_d[CRD_Y] = 10.0;	// 弾性
#elif SIM_CTRL_MODE_VOIGT
	ctrlVoigt(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Md, &Cd, &Kd);
#endif

	// 返り値に代入
	jnt_force[ARM_M1] = tau.el[ARM_M1][0];
	jnt_force[ARM_M2] = tau.el[ARM_M2][0];

//	if(_this->step == IMP_SWITCH_STEP){	_this->jnt_force[ARM_M1] = 0.0;	_this->jnt_force[ARM_M2] = 0.0;}
	// 駆動力制限
//	for(jnt=0;jnt<ARM_JNT;jnt++)	if(_this->jnt_force[jnt] > 100 || _this->jnt_force[jnt] < -100)	_this->jnt_force[jnt] = 0.0;
	// 駆動力入力をODEに設定

	for(int jnt=0;jnt<ARM_JNT;jnt++)	dJointAddHingeTorque(r_joint[jnt],jnt_force[jnt]);		// トルクは上書きではなくインクリメントされることに注意
}

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
	for(int jnt=0;jnt<ARM_JNT;jnt++){
		finger->ref_jnt_pos[jnt] = 0.0;
		finger->jnt_pos[jnt] = 0.0;
		finger->jnt_vel[jnt] = 0.0;
		finger->jnt_force[jnt] = 0.0;
		finger->past_jnt_pos[jnt] = 0.0;
	}
//	sim->jnt_pos[ARM_M1] = 4 * PI / 4.0; sim->jnt_pos[ARM_M2] = PI / 4.0;
	for(int crd=0;crd<DIM3;crd++){
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
int exeCmd(int argc, char *argv[])
{
	auto sim = EntityManager::get();
	int   command;
	char  buf[BUFSIZE];
	static int hist = 0;   // コマンド保存
	static int incount = 0;
	int	flag_quit = 0;

	while(1){
#if 0
		// コマンド表示 & 取得
		getChar(buf,"command > ");
		command = atoi(buf);      // アスキーコードに変換
		sscanf(buf, "%c", &command);
		// コマンド処理
		switch(command){
//			case 'd': drawData(); break;
			case 'h': showHelp(); break;
			case 'q': flag_quit = 1;	// 終了判定
			default: ;
		}
		if(flag_quit)	break;
#endif

		// シミュレーションループ
#if FLAG_DRAW_SIM
		DrawStuff DS;
		dsSimulationLoop(argc, argv, DISPLAY_WIDTH, DISPLAY_HEIGHT, DS.getFn());
#else
		while(1){
			simLoop(0);
			if(_this->step == DATA_CNT_NUM)	break;				// 終了設定
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

#if practice
static dWorldID world;
dBodyID ball;
const dReal   radius = 0.2;
const dReal   mass = 100.0;

static void simLoop(int pause)
{
	const dReal* pos, * R;

	dWorldStep(world, 0.05);

	dsSetColor(1.0, 0.0, 0.0);
	pos = dBodyGetPosition(ball);
	R = dBodyGetRotation(ball);
	dsDrawSphere(pos, R, radius);
}

void start()
{
	static float xyz[3] = { 0.0,-3.0,1.0 };
	static float hpr[3] = { 90.0,0.0,0.0 };
	dsSetViewpoint(xyz, hpr);
}

int main(int argc, char** argv)
{
	dReal x0 = 0.0, y0 = 0.0, z0 = 1.0;
	dMass m1;

	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = NULL;
	fn.stop = NULL;
	fn.path_to_textures = "C:\\ode-0.16.1\\drawstuff\\textures";

	dInitODE();
	world = dWorldCreate();
	dWorldSetGravity(world, 0, 0, -0.001);

	ball = dBodyCreate(world);
	dMassSetZero(&m1);
	dMassSetSphereTotal(&m1, mass, radius);
	dBodySetMass(ball, &m1);
	dBodySetPosition(ball, x0, y0, z0);

	dsSimulationLoop(argc, argv, 352, 288, &fn);

	dWorldDestroy(world);
	dCloseODE();

	return 0;
}

#else


////////////////////////////////////////////////////////
// main関数
////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	auto sim = EntityManager::init();
#if FLAG_DRAW_SIM
	// 描画設定
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
	for (int jnt = 0; jnt < ARM_JNT; jnt++)	Finger2->init_jnt_pos[jnt] = init_jnt_pos[jnt];
	for (int crd = 0; crd < DIM3; crd++) {
		Finger2->init_obj_pos[crd] = init_obj_pos[crd];
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

#endif


////////////////////////////////////////////////////////
// 実体定義
////////////////////////////////////////////////////////
cParts::cParts(dReal m): m(m) { this->body = dBodyCreate(EntityManager::get()->getWorld()); dMassSetZero(&mass); }


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
}

cPartsCylinder::cPartsCylinder(dReal m, dReal l, dReal r) : cParts(m), l(l), r(r) {
	dMassSetCylinderTotal(&this->mass, this->m, DIR_LONG_AXIS_Z, this->r, this->l);
	dBodySetMass(this->body, &mass);
	this->geom = dCreateCylinder(EntityManager::get()->getSpace(), this->r, this->l);
	dGeomSetBody(this->geom, this->body);
}

cPartsCylinder::cPartsCylinder(dReal m, Vec3 init_pos, dReal l, dReal r) : cPartsCylinder(m, l, r) {		// デフォルトコンストラクタ
	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
}

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
	// センサ設定（力とトルクのに必要）
	dJointSetFeedback(f2_joint, &force);
}
//二本目の指の初期位置設定
void cFinger::setJoint2() {
	auto sim = EntityManager::get();

	//把持する板を設置(位置は可変)
	/*graspObj = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(graspObj, plate.getBody(), 0);
	dJointSetHingeAnchor(graspObj,-1.2,-0.5,0.0);*/


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
	dJointSetHingeParam(r_joint[ARM_M2], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M2], dParamHiStop, M_PI);
	
	// 固定ジョイント
	f2_joint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(f2_joint, finger[3]->getBody(), finger[2]->getBody());
	dJointSetFixed(f2_joint);
	// センサ設定（力とトルクの取得に必要）
	dJointSetFeedback(f2_joint, &force);
}


////////////////////////////////////////////////////////
// シミュレーションループ
////////////////////////////////////////////////////////
void DrawStuff::simLoop(int pause)
{
	
	auto _this = EntityManager::get()->getFinger();
	//_this->printInfo();

#if finger2_use
	//二本目の指用　kawahara
	auto _this2 = EntityManager::get()->getFinger2();//kawahara二本目の指
	auto sensor2 = _this2->getParts()[3];
	auto obj2 = EntityManager::get()->getObj2();
#endif
#if print_debug
	_this->printInfo();
	_this2->printInfo();
#endif
	auto sim = EntityManager::get();

	if (!pause) {
		auto sensor =_this->getParts()[3];
		auto obj = EntityManager::get()->getObj();
		
		// 初期設定
#if SIM_OBJ_IMPACT
		if (_this->step == 0)	dBodySetLinearVel(obj.body, _this->init_obj_att[AXIS_X][CRD_X] * SIM_OBJ_INIT_ABS_VEL, _this->init_obj_att[AXIS_X][CRD_Y] * SIM_OBJ_INIT_ABS_VEL, _this->init_obj_att[AXIS_X][CRD_Z] * SIM_OBJ_INIT_ABS_VEL);		// 対象速度
		if (_this->step == 0)	dBodySetAngularVel(obj.body, _this->init_obj_att[AXIS_Z][CRD_X] * SIM_OBJ_INIT_ABS_VEL / OBJ_RADIUS, _this->init_obj_att[AXIS_Z][CRD_Y] * SIM_OBJ_INIT_ABS_VEL / OBJ_RADIUS, _this->init_obj_att[AXIS_Z][CRD_Z] * SIM_OBJ_INIT_ABS_VEL / OBJ_RADIUS);		// 対象角速度
#elif SIM_ADD_EXT_FORCE
		if (sim->step == 0) {
			dBodyDisable(obj->getBody());		// 対象無効化			
			dBodyDisable(obj2->getBody());		// 対象無効化
		}
#endif
		// 状態取得
		for (int jnt = 0; jnt<ARM_JNT; jnt++) {
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
		dBodyGetRelPointPos(sensor2->getBody(), 0.0,0.0, sensor2->getl() / 2.0, _this2->eff_pos);			// 手先位置
		dBodyGetRelPointVel(sensor2->getBody(), 0.0,0.0, sensor2->getl() / 2.0, _this2->eff_vel);			// 手先速度
#endif
		//関節のフィードバックを反映
		_this->p_force = dJointGetFeedback(_this->f2_joint);
		//追加
#if finger2_use
		_this2->p_force = dJointGetFeedback(_this2->f2_joint);
#endif

		for (int crd = 0; crd<DIM3; crd++) {
			_this->eff_force[crd] = -_this->p_force->f1[crd];				// 対象がセンサに及ぼしている力=センサが関節に及ぼしている力
			_this->obj_pos[crd] = (dBodyGetPosition(obj->getBody()))[crd];		// 対象位置
			_this->obj_vel[crd] = (dBodyGetLinearVel(obj->getBody()))[crd];		// 対象速度
#if finger2_use
			_this2->eff_force[crd] = -_this2->p_force->f1[crd];
			_this2->obj_pos[crd] = (dBodyGetPosition(obj2->getBody()))[crd];		// 対象位置
			_this2->obj_vel[crd] = (dBodyGetLinearVel(obj2->getBody()))[crd];		// 対象速度
#endif
		}
		auto entity = EntityManager::get();
		// 距離計算
		_this->calcDist();
#if finger2_use
		//_this2->calcDist();
#endif

		// 外力設定
#if SIM_ADD_EXT_FORCE
		_this->addExtForce();
	#if finger2_use
		_this2->addExtForce();
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

		// 過去データとして代入
		for (int jnt = 0; jnt<ARM_JNT; jnt++)	_this->past_jnt_pos[jnt] = _this->jnt_pos[jnt];
		matCopy(&_this->var_prev2.r, &_this->var_prev.r); matCopy(&_this->var_prev2.dr, &_this->var_prev.dr);
		matCopy(&_this->var_prev.r, &_this->var.r); matCopy(&_this->var_prev.dr, &_this->var.dr);

		// 二本目の指　kawahara
#if finger2_use
		for (int jnt = 0; jnt < ARM_JNT; jnt++)	_this2->past_jnt_pos[jnt] = _this2->jnt_pos[jnt];
		matCopy(&_this2->var_prev2.r, &_this2->var_prev.r); matCopy(&_this2->var_prev2.dr, &_this2->var_prev.dr);
		matCopy(&_this2->var_prev.r, &_this2->var.r); matCopy(&_this2->var_prev.dr, &_this2->var.dr);
#endif
		// 現在値を保存領域へコピー
		copyData(_this.get());
		copyData(_this2.get());
		_this->state_contact = 0;
#if finger2_use
		_this2->state_contact = 0;
#endif	
		_this->printInfo();
		// シミュレーションを１ステップ進行
		entity->update();
		sim->step++;
		

#if FLAG_DRAW_SIM
	// 終了設定
		if (sim->step == DATA_CNT_NUM)	dsStop();
#endif
	}
#if FLAG_DRAW_SIM
	//	drawRobot(); // ロボットの描画
	_this->draw();
#if finger2_use
	_this2->draw();
#endif;
	//finger2->plate.draw();


	std::cout << "step:" << sim->step << std::endl;

#if SIM_OBJ_IMPACT
	//	drawObject(); // 衝突対象の描画
	_this->getObj()->draw();
#elif SIM_ADD_EXT_FORCE
	//drawExtForce();		// 外力の描画
	//drawExtForce2();	// 外力の描画

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
	xyz[0] = -0.5;	xyz[1] = 0.0;	xyz[2] = 2.5;
	hpr[0] = 0.0;	hpr[1] = -90.0;	hpr[2] = 180;	// +zからの視点(右が+x,上が+y)
#endif
	dsSetViewpoint(xyz, hpr);               // 視点，視線の設定
	dsSetSphereQuality(3);					// 球の品質設定
#if	FLAG_SAVE_VIDEO
	init_video();
#endif
}

////////////////////////////////////////////////////////
// キーボードコマンドの処理関数
////////////////////////////////////////////////////////
void DrawStuff::command(int cmd) {
	auto _this = EntityManager::get();
	switch (cmd) {
	case 'x': xyz[0] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// x方向
	case 'X': xyz[0] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -x方向
	case 'y': xyz[1] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// y方向
	case 'Y': xyz[1] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -y方向
	case 'z': xyz[2] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// z方向
	case 'Z': xyz[2] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -z方向
//			case 'u':	dBodyAddForce(_this->getObj()->getBody(), 500.0, 0.0, 0.0);	break;
	case 'u':	_this->setAddForceObj(500.0, 0.0, 0.0);	break;
	case 'r':	restart();	break;
	case 'q':	dsStop();	break;
	default: std::cout << "key missed" << std::endl; break;
	}
}
