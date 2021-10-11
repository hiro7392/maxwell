#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "simMain.h"
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

// シミュレーション変数
SIM sim;
// ODE変数
dWorldID world;  // 動力学計算用ワールド
dSpaceID space;  // 衝突検出用スペース
dGeomID  ground; // 地面
dJointGroupID contactgroup; // コンタクトグループ
dJointFeedback force, *p_force;
MyObject base, arm[ARM_JNT], sensor, obj;    // 
dJointID f_joint, r_joint[ARM_JNT], f2_joint; // 固定関節と回転関節
// 描画変数
dsFunctions fn;
// 視点変数
static float xyz[3];
static float hpr[3];	// 単位はdeg

// 初期設定変数
//double	init_jnt_pos[ARM_JNT] = {3*PI/4.0, PI/2.0};	// ロボット初期姿勢
double	init_jnt_pos[ARM_JNT] = {4*PI/4.0, PI/4.0};	// ロボット初期姿勢
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

////////////////////////////////////////////////////////
// 衝突検出計算コールバック関数
// 摩擦係数(contact[cnt].surface.mu):この値は並進摩擦であり，回転摩擦は自分で設定する必要がある．無限大(dInfinity)では，衝突の際に対象orアームが動かないため過大な力が発生して，変な挙動を発生することがある．
// つまり，滑りが発生している時の摩擦値．円柱では接触線方向の運動時．
////////////////////////////////////////////////////////
static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	static const int N = 10;     // 接触点数
	int	n, cnt;
	int	flag_ground, flag_sensor;	// 衝突検出用フラグ
	dContact contact[N];
	dBodyID b1, b2;
	dJointID c;

	// 地面との衝突検出
	flag_ground = ((o1 == ground) || (o2 == ground));
	// アームリンクとの衝突検出
//	flag_arm = ((o1 == arm.geom) || (o2 == arm.geom));
	// アーム手先との衝突検出
	flag_sensor = ((o1 == sensor.geom) || (o2 == sensor.geom));
	// 2つのボディがジョイントで結合されていたら衝突検出しない
	b1 = dGeomGetBody(o1);	b2 = dGeomGetBody(o2);
	if (b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact)) return;
	// 衝突設定
	n =  dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));
	if(flag_ground){
		for(cnt=0; cnt<n; cnt++) {
			contact[cnt].surface.mode   = dContactBounce | dContactSoftERP | dContactSoftCFM;
			contact[cnt].surface.soft_erp   = 0.2;   // 接触点のERP
			contact[cnt].surface.soft_cfm   = 0.001; // 接触点のCFM
			contact[cnt].surface.mu     = 0.5; // 摩擦係数
			c = dJointCreateContact(world, contactgroup, &contact[cnt]);
			dJointAttach(c, dGeomGetBody(contact[cnt].geom.g1), dGeomGetBody(contact[cnt].geom.g2));
			// 転がり摩擦
//#define	COEF_FRIC_ROLL	0.006
#define	COEF_FRIC_ROLL	0.0006
			rolling_function( o1, COEF_FRIC_ROLL/n, contact+cnt );
			rolling_function( o2, COEF_FRIC_ROLL/n, contact+cnt );
		}
//		if(n!=0)	rolling_function( o1, COEF_FRIC_ROLL, contact );
	}
	if(flag_sensor){
		for(cnt=0; cnt<n; cnt++) {
			contact[cnt].surface.mode   = dContactBounce | dContactSoftERP | dContactSoftCFM;
//			contact[cnt].surface.soft_erp   = 0.45;   // 接触点のERP
//			contact[cnt].surface.soft_cfm   = 0.005; // 接触点のCFM
#if 1
			contact[cnt].surface.soft_erp   = 0.2;   // 接触点のERP
			contact[cnt].surface.soft_cfm   = 0.005; // 接触点のCFM
#else
			contact[cnt].surface.soft_erp   = 0.2;   // 接触点のERP
			contact[cnt].surface.soft_cfm   = 0.0005; // 接触点のCFM
#endif
			contact[cnt].surface.mu     = 0.5; // 摩擦係数
			contact[cnt].surface.bounce   = 0.5; // 反発係数
			c = dJointCreateContact(world, contactgroup, &contact[cnt]);
			dJointAttach(c, dGeomGetBody(contact[cnt].geom.g1), dGeomGetBody(contact[cnt].geom.g2));
		}
		if(n!=0)	sim.state_contact = 1;
	}
}

////////////////////////////////////////////////////////
// 関節制御
// 戻り値：関節一般化力（直動関節では力，回転関節ではトルク）
////////////////////////////////////////////////////////
int ctrlArm(dReal *jnt_force)
{
	int	jnt, crd;
	static Matrix	tau;
	// 初期化
	if(sim.step == 0){
		matInit(&tau,2,1);
		// 初期化
		ctrlInit(&sim);
		ctrlInitErr(&sim);		// パラメータ誤差を追加
		// 初期値保存
		for(crd=0;crd<DIM2;crd++){
			sim.var_init.q.el[crd][0] = sim.jnt_pos[crd]; sim.var_init.dq.el[crd][0] = sim.jnt_vel[crd];
			sim.var_init.r.el[crd][0] = sim.eff_pos[crd]; sim.var_init.dr.el[crd][0] = sim.eff_vel[crd];
			sim.var_init.F.el[crd][0] = sim.eff_force[crd];
		}
	}
	// 手先変数代入
	for(crd=0;crd<DIM2;crd++){
		sim.var.r.el[crd][0] = sim.eff_pos[crd]; sim.var.dr.el[crd][0] = sim.eff_vel[crd];
		sim.var.F.el[crd][0] = sim.eff_force[crd];
	}
	// 関節変数代入
	for(jnt=0;jnt<ARM_JNT;jnt++){
		sim.var.q.el[jnt][0] = sim.jnt_pos[jnt];	sim.var.dq.el[jnt][0] = sim.jnt_vel[jnt];
	}
	// パラメータセット
	armDynPara(&sim);

	// インピーダンス設定
	// 制御指令計算
#if SIM_CTRL_MODE_MAXWELL & SIM_ADD_EXT_FORCE
	double	impM[] = { 2.0, 2.0 }, impC[] = { 4.0, 4.0 }, impK[] = { 40.0, 40.0 }, impK0[] = { 10.0, 10.0 };
	//		double	impM[] = {2.0, 2.0}, impC[] = {4.0, 4.0}, impK[] = {1000.0, 1000.0}, impK0[] = {10.0, 10.0};
	//		double	impM[] = {2.0, 2.0}, impC[] = {4.0, 4.0}, impK[] = {100.0, 100.0}, impK0[] = {10.0, 10.0};
	//		double	impM[] = {1.0, 1.0}, impC[] = {10.0, 10.0}, impK[] = {10.0, 10.0}, impK0[] = {10.0, 10.0};
	matSetValDiag(&sim.imp.M, impM); matSetValDiag(&sim.imp.C, impC); matSetValDiag(&sim.imp.K, impK); matSetValDiag(&sim.imp.K0, impK0);	// ゲイン設定
	ctrlPreProcessing(&sim);		// インピーダンス補足処理（逆行列等）
#if 1
//	ctrlMaxwell(&sim, &tau);
	ctrlMaxwellWithoutInertiaShaping(&sim, &tau);
#elif 1
	ctrlMaxwellConv(&sim, &tau);
//	ctrlMaxwellConvRK(&sim, &tau);
//	ctrlMaxwellConvRK2(&sim, &tau);
//	ctrlMaxwellConvWithoutInertiaShaping(&sim, &tau);
//	ctrlMaxwellConvInnerLoop(&sim, &tau);
#else
	ctrlMaxwellInnerLoop(&sim, &tau);
#endif
//	ctrlMaxwellE(&sim, &tau);
//	ctrlMaxwellImplicit(&sim, &tau);
//	ctrlMaxwellInnerLoopImplicit(&sim, &tau);			// 制御指令計算
//	ctrlMaxwellInnerLoopJntSpace(&sim, &tau);		// デバッグ中
//	ctrlSLS(&sim, &tau);
//	ctrlMaxwellVar2(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Md, &Cd, &Kd);
//	ctrlMaxwellVar2(&sim, &tau, &sim.dyn.Mq, &sim.dyn.h, &sim.kine.J, &sim.kine.dJ, &sim.var.q, &sim.var.dq, &re, &dre, &sim.var.F, &sim.imp.M, &sim.imp.C, &sim.imp.K);
//	ctrlMaxwellVar(&sim, &tau);
#elif SIM_CTRL_MODE_MAXWELL & SIM_OBJ_IMPACT
//		sim.imp.m_d[CRD_X] = 0.5;	sim.imp.m_d[CRD_Y] = 0.5;	// 慣性
//		sim.imp.c_d[CRD_X] = 1.0;	sim.imp.c_d[CRD_Y] = 1.0;	// 粘性
//		sim.imp.k_d[CRD_X] = 10.0;	sim.imp.k_d[CRD_Y] = 10.0;	// 弾性
	sim.imp.M.el[CRD_X][CRD_X] = 0.5;	sim.imp.M.el[CRD_Y][CRD_Y] = 0.5;	// 慣性
	sim.imp.C.el[CRD_X][CRD_X] = 0.8;	sim.imp.C.el[CRD_Y][CRD_Y] = 0.8;	// 粘性
	sim.imp.K.el[CRD_X][CRD_X] = 5.2;	sim.imp.K.el[CRD_Y][CRD_Y] = 5.2;	// 弾性
	sim.imp.K0.el[CRD_X][CRD_X] = 10.0;	sim.imp.K0.el[CRD_Y][CRD_Y] = 10.0;	// 弾性(SLS並列バネ)
#elif SIM_CTRL_MODE_HYBRID & SIM_ADD_EXT_FORCE
	sim.imp.m_d[CRD_X] = 2.0;	sim.imp.m_d[CRD_Y] = 1.0;	// 慣性
	sim.imp.c_d[CRD_X] = 4.0;	sim.imp.c_d[CRD_Y] = 0.5;	// 粘性
	sim.imp.k_d[CRD_X] = 100.0;	sim.imp.k_d[CRD_Y] = 50.0;	// 弾性
	// MaxwellとVoigtの方向切替
	if (sim.step == IMP_SWITCH_STEP) {
		// 変数初期化
		for (crd = 0; crd<2; crd++)	Fint.el[crd][0] = 0.0;
		// 目標インピーダンス（手先）
		sim.imp.M.el[CRD_X][CRD_X] = 1.0;	sim.imp.M.el[CRD_Y][CRD_Y] = 2.0;	// 慣性
		sim.imp.C.el[CRD_X][CRD_X] = 0.5;	sim.imp.C.el[CRD_Y][CRD_Y] = 4.0;	// 粘性
		sim.imp.K.el[CRD_X][CRD_X] = 50.0;	sim.imp.K.el[CRD_Y][CRD_Y] = 100.0;	// 弾性
		for (crd = 0; crd<2; crd++) {
			sim.var_init.r.el[crd][0] = sim.eff_pos[crd];
}
	}
	ctrlHybrid(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Fint, &Md, &Cd, &Kd);
#elif 0
	sim.imp.m_d[CRD_X] = 200.0;	sim.imp.m_d[CRD_Y] = 2.0;	// 慣性
	sim.imp.c_d[CRD_X] = 4.0;	sim.imp.c_d[CRD_Y] = 4.0;	// 粘性
	sim.imp.k_d[CRD_X] = 10.0;	sim.imp.k_d[CRD_Y] = 100.0;	// 弾性
#elif 1
	sim.imp.m_d[CRD_X] = 10.0;	sim.imp.m_d[CRD_Y] = 10.0;	// 慣性
	sim.imp.c_d[CRD_X] = 100.0;	sim.imp.c_d[CRD_Y] = 1.0;	// 粘性
	sim.imp.k_d[CRD_X] = 10.0;	sim.imp.k_d[CRD_Y] = 10.0;	// 弾性
#elif SIM_CTRL_MODE_VOIGT
	ctrlVoigt(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Md, &Cd, &Kd);
#endif

	// 返り値に代入
	jnt_force[ARM_M1] = tau.el[ARM_M1][0];
	jnt_force[ARM_M2] = tau.el[ARM_M2][0];
//	if(sim.step == IMP_SWITCH_STEP){	jnt_force[ARM_M1] = 0.0;	jnt_force[ARM_M2] = 0.0;}
	// 駆動力制限
//	for(jnt=0;jnt<ARM_JNT;jnt++)	if(jnt_force[jnt] > 100 || jnt_force[jnt] < -100)	jnt_force[jnt] = 0.0;
	// 駆動力入力をODEに設定
	for(jnt=0;jnt<ARM_JNT;jnt++)	dJointAddHingeTorque(r_joint[jnt], jnt_force[jnt]);		// トルクは上書きではなくインクリメントされることに注意
	return	0;
}

////////////////////////////////////////////////////////
// シミュレーションループ
////////////////////////////////////////////////////////
static void simLoop(int pause)
{
	int	jnt, crd;

	if(!pause){
		// 初期設定
#if SIM_OBJ_IMPACT
		if(sim.step == 0)	dBodySetLinearVel(obj.body, sim.init_obj_att[AXIS_X][CRD_X]*SIM_OBJ_INIT_ABS_VEL, sim.init_obj_att[AXIS_X][CRD_Y]*SIM_OBJ_INIT_ABS_VEL, sim.init_obj_att[AXIS_X][CRD_Z]*SIM_OBJ_INIT_ABS_VEL);		// 対象速度
		if(sim.step == 0)	dBodySetAngularVel(obj.body, sim.init_obj_att[AXIS_Z][CRD_X]*SIM_OBJ_INIT_ABS_VEL/OBJ_RADIUS, sim.init_obj_att[AXIS_Z][CRD_Y]*SIM_OBJ_INIT_ABS_VEL/OBJ_RADIUS, sim.init_obj_att[AXIS_Z][CRD_Z]*SIM_OBJ_INIT_ABS_VEL/OBJ_RADIUS);		// 対象角速度
//		if(sim.step == 0)	dBodySetLinearVel(obj.body, 1, 0, 0);		// 対象速度
//		if(sim.step == 0)	dBodySetAngularVel(obj.body, -1, 0, 0);		// 対象速度
#elif SIM_ADD_EXT_FORCE
		if(sim.step == 0)	dBodyDisable(obj.body);		// 対象無効化
#endif
		// 状態取得
		for(jnt=0;jnt<ARM_JNT;jnt++){
			sim.jnt_pos[jnt] = dJointGetHingeAngle(r_joint[jnt]) + sim.init_jnt_pos[jnt];	// 関節位置（x軸が基準角0）
			sim.jnt_vel[jnt] = dJointGetHingeAngleRate(r_joint[jnt]);	// 関節速度
		}
		dBodyGetRelPointPos(sensor.body, 0.0, 0.0, sensor.l/2.0, sim.eff_pos);			// 手先位置
		dBodyGetRelPointVel(sensor.body, 0.0, 0.0, sensor.l/2.0, sim.eff_vel);			// 手先速度
		p_force = dJointGetFeedback(f2_joint);
		for(crd=0;crd<DIM3;crd++){
			sim.eff_force[crd] = -p_force->f1[crd];	// 対象がセンサに及ぼしている力=センサが関節に及ぼしている力
			sim.obj_pos[crd] = (dBodyGetPosition(obj.body))[crd];		// 対象位置
			sim.obj_vel[crd] = (dBodyGetLinearVel(obj.body))[crd];		// 対象速度
		}
		// 距離計算
		calcDist(&sim);
		// 外力設定
#if SIM_ADD_EXT_FORCE
		addExtForce(&sim);
#endif
		// ODE摩擦手動設定(粘性摩擦)
		setJntFric(&sim);
		// 力計算
		ctrlArm(sim.jnt_force);
		// 過去データとして代入
		for(jnt=0;jnt<ARM_JNT;jnt++)	sim.past_jnt_pos[jnt] = sim.jnt_pos[jnt];
		matCopy(&sim.var_prev2.r, &sim.var_prev.r); matCopy(&sim.var_prev2.dr, &sim.var_prev.dr);
		matCopy(&sim.var_prev.r, &sim.var.r); matCopy(&sim.var_prev.dr, &sim.var.dr);
		// 現在値を保存領域へコピー
		copyData(&sim);
		sim.state_contact = 0;
		// シミュレーションを１ステップ進行
		dSpaceCollide(space, 0, &nearCallback);
		dWorldStep(world, SIM_CYCLE_TIME);
		dJointGroupEmpty(contactgroup); // ジョイントグループを空にする
		sim.step++;
#if FLAG_DRAW_SIM
		// 終了設定
		if(sim.step == DATA_CNT_NUM)	dsStop();
#endif
	}
#if FLAG_DRAW_SIM
	drawRobot(); // ロボットの描画
#if SIM_OBJ_IMPACT
	drawObject(); // 衝突対象の描画
#elif SIM_ADD_EXT_FORCE
	drawExtForce(); // 外力の描画
#endif
#endif
#if FLAG_DRAW_SIM & FLAG_SAVE_IMAGE
	// 画像保存
	if(!pause){
		if(sim.step % SAVE_IMG_RATE == 0)	saveImage(640, 480);	// SAVE_IMG_RATE毎に画像保存
	}
#endif
#if	FLAG_SAVE_VIDEO
	if(sim.step % SAVE_VIDEO_RATE == 0)	save_video();
#endif
}

////////////////////////////////////////////////////////
// シミュレーションリスタート
////////////////////////////////////////////////////////
static void restart()
{
	int	jnt, crd;
	//変数初期化
	sim.step = 0;		//ステップ数初期化
	sim.state_contact = 0;				// 状態変数初期化
	sim.dist = 0.0;
	for(jnt=0;jnt<ARM_JNT;jnt++){
		sim.ref_jnt_pos[jnt] = 0.0;
		sim.jnt_pos[jnt] = 0.0;
		sim.jnt_vel[jnt] = 0.0;
		sim.jnt_force[jnt] = 0.0;
		sim.past_jnt_pos[jnt] = 0.0;
	}
	for(crd=0;crd<DIM3;crd++){
		sim.ref_eff_pos[crd] = 0.0;
		sim.eff_pos[crd] = 0.0;
		sim.eff_force[crd] = 0.0;
	}
	// ODE
	destroyRobot();  // ロボットの破壊
	destroyObject();  // 衝突対象の破壊
	dJointGroupDestroy(contactgroup);     // ジョイントグループの破壊
	contactgroup = dJointGroupCreate(0);  // ジョイントグループの生成
	createRobot(&sim);                      // ロボットの生成
	createObject(&sim);                      // 衝突対象の生成
}

////////////////////////////////////////////////////////
// キーボードコマンドの処理関数
////////////////////////////////////////////////////////
static void command(int cmd)
{
	switch(cmd){
		case 'x': xyz[0] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// x方向
		case 'X': xyz[0] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -x方向
		case 'y': xyz[1] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// y方向
		case 'Y': xyz[1] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -y方向
		case 'z': xyz[2] += 0.01; dsSetViewpoint(xyz, hpr);	break;		// z方向
		case 'Z': xyz[2] -= 0.01; dsSetViewpoint(xyz, hpr);	break;		// -z方向
		case 'u':	dBodyAddForce(obj.body, 500.0, 0.0, 0.0);	break;
		case 'r':	restart();	break;
		case 'q':	dsStop();	break;
		default :printf("key missed \n")             ; break;
	}
}

////////////////////////////////////////////////////////
// ODE初期設定
////////////////////////////////////////////////////////
static void start()
{
#if 0
	xyz[0] = -0.2;	xyz[1] = 2.5;	xyz[2] = 0.5;
	hpr[0] = -90.0;	hpr[1] = 0.0;	hpr[2] = 0.0;	// +yからの視点(右が-x,上が+z)
#elif 0
	xyz[0] = 2.5;	xyz[1] = 0.2;	xyz[2] = 0.5;
	hpr[0] = -180.0;	hpr[1] = 0.0;	hpr[2] = 0.0;	// +xからの視点(右が+y,上が+z)
#elif 1
	xyz[0] = -0.5;	xyz[1] = 0.0;	xyz[2] = 2.5;
	hpr[0] = 0.0;	hpr[1] = -90.0;	hpr[2] = 90.0;	// +zからの視点(右が+x,上が+y)
#endif
	dsSetViewpoint(xyz, hpr);               // 視点，視線の設定
	dsSetSphereQuality(3);                 // 球の品質設定
#if	FLAG_SAVE_VIDEO
	init_video();
#endif
}

////////////////////////////////////////////////////////
// 描画関数の設定
////////////////////////////////////////////////////////
void setDrawStuff()
{
	fn.version = DS_VERSION;    // ドロースタッフのバージョン
	fn.start   = &start;        // 前処理 start関数のポインタ
	fn.step    = &simLoop;      // simLoop関数のポインタ
	fn.command = &command;      // キー入力関数へのポインタ
	fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH; // テクスチャ
}

////////////////////////////////////////////////////////
// コマンド実行
////////////////////////////////////////////////////////
int exeCmd(int argc, char *argv[])
{
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
		dsSimulationLoop(argc, argv, DISPLAY_WIDTH, DISPLAY_HEIGHT, &fn);
#else
		while(1){
			simLoop(0);
			if(sim.step == DATA_CNT_NUM)	break;				// 終了設定
		}
#endif
		// ファイル保存
		sprintf(sim.data_file_name, FILENAME_DATA, incount);		// ファイル名を連番に設定
		sprintf(sim.filename_info, FILENAME_INFO, incount);		// ファイル名を連番に設定
		saveData(&sim);
		saveInfo(&sim);
		saveGraph(&sim);
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
// main関数
////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	int	jnt, crd, axis;
	// 初期設定
	dInitODE();
	world        = dWorldCreate();
	space        = dHashSpaceCreate(0);
	contactgroup = dJointGroupCreate(0);
#if FLAG_DRAW_SIM
	// 描画設定
	setDrawStuff();		// ドロースタッフ
#endif
	// 環境設定
	dWorldSetGravity(world, 0,0, -9.8);	// 重力設定
	dWorldSetERP(world, 0.9);          // ERPの設定
	dWorldSetCFM(world, 1e-4);         // CFMの設定
	// パラメータ設定
	for(jnt=0;jnt<ARM_JNT;jnt++)	sim.init_jnt_pos[jnt] = init_jnt_pos[jnt];
	for(crd=0;crd<DIM3;crd++){
		sim.init_obj_pos[crd] = init_obj_pos[crd];
		for(axis=0;axis<DIM3;axis++)	sim.init_obj_att[axis][crd] = init_obj_att[axis][crd];
	}
	// 物体生成
	ground = dCreatePlane(space, 0, 0, 1, 0);		// 地面の設定
	createRobot(&sim);
	createObject(&sim);
	// 乱数種設定
//	dRandSetSeed(0);
	// コマンド開始
	exeCmd(argc, argv);		// 内部でdsSimulationLoop()を実行
	// シミュレーションループ
//	dsSimulationLoop (argc, argv, 640, 480, &fn);
	// 終了設定
	dSpaceDestroy(space);
	dWorldDestroy(world);
	dCloseODE();
	// ファイル保存
//	saveData();
//	saveGraph();

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
