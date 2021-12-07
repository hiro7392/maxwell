#include "makeRobot.h"
#include "command.h"
#include "matBase.h"
#include "setRobot.h"
#include "ctrlRobot.h"

#pragma comment(lib,"C:\\ode-0.16.1\\lib\\DebugDoubleDLL\\ode_doubled.lib")
#pragma comment(lib,"C:\\ode-0.16.1\\lib\\DebugDoubleDLL\\drawstuffd.lib")

#include "texturepath.h"
#include "simMain.h"
#include "simMainF2.h"

#include "simMode.h"
#include "setEnv.h"

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


//カプセル用
#define DENSITY (5.0)	// 密度
// カプセル
static dMass mass ;
dReal radius = 0.25; // 半径
dReal length = 1.0;  // 長さ
const dReal* pos1=0, * R1, * pos2 , * R2, * pos3, * R3;    //　球の描画
struct MyObject {
	dBodyID body;		// ボディ（剛体）
	dGeomID geomBody;
};
static MyObject capsule;
static MyObject plateToGrasp;	//把持する用のプレート

// クラス静的メンバの初期化
dsFunctions DrawStuff::fn;
float DrawStuff::xyz[3];
float DrawStuff::hpr[3];

// 初期設定変数
//double	init_jnt_pos[ARM_JNT] = {3*PI/4.0, PI/2.0};	// ロボット初期姿勢
double	init_jnt_pos[ARM_JNT] = { 4 * PI / 4.0, PI / 4.0 };	// ロボット初期姿勢

#if SIM_OBJ_CASE1
double	init_obj_pos[DIM3] = { -0.8 - 2 * 0.75 / sqrt(2.0), -0.035, OBJ_RADIUS };	// 対象初期位置
double	init_obj_att[DIM3][DIM3] = { {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0} };	// 回転軸がy方向
#elif SIM_OBJ_CASE2
double	init_obj_pos[DIM3] = { -2 * 0.75 / sqrt(2.0) - 0.035, -0.8, OBJ_RADIUS };	// 対象初期位置
double	init_obj_att[DIM3][DIM3] = { {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}, {-1.0, 0.0, 0.0} };	// 回転軸がx方向
#elif SIM_OBJ_CASE3
double	init_obj_pos[DIM3] = { -0.8 / sqrt(2.0) - 2 * 0.75 / sqrt(2.0), -0.8 / sqrt(2.0), OBJ_RADIUS };	// 対象初期位置
double	init_obj_att[DIM3][DIM3] = { {sqrt(2.0) / 2, sqrt(2.0) / 2, 0.0}, {0.0, 0.0, -1.0}, {-sqrt(2.0) / 2, sqrt(2.0) / 2, 0.0} };	// 回転軸がx方向
#endif

#define practice 0			//ODE OpenGLの練習用 kawahara
////////////////////////////////////////////////////////
// 関節制御
// 戻り値：関節一般化力（直動関節では力，回転関節ではトルク）
////////////////////////////////////////////////////////
void cFinger::control() {
	auto entity = EntityManager::get();

	Matrix	tau;
	matInit(&tau, 2, 1);
	// 初期化
	if (entity->step == 0) {

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
	if (fingerID == 1) {
		//ctrlMaxwell(&tau);
		RestrictedCtrlMaxwell(&tau);
	}
	else {
		//ctrlMaxwell2(&tau);
		RestrictedCtrlMaxwell2(&tau);
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
		for (crd = 0; crd < 2; crd++)	Fint.el[crd][0] = 0.0;
		// 目標インピーダンス（手先）
		_this->imp.M.el[CRD_X][CRD_X] = 1.0;	_this->imp.M.el[CRD_Y][CRD_Y] = 2.0;	// 慣性
		_this->imp.C.el[CRD_X][CRD_X] = 0.5;	_this->imp.C.el[CRD_Y][CRD_Y] = 4.0;	// 粘性
		_this->imp.K.el[CRD_X][CRD_X] = 50.0;	_this->imp.K.el[CRD_Y][CRD_Y] = 100.0;	// 弾性
		for (crd = 0; crd < 2; crd++) {
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
#if print_debug
	std::cout << "tau fingerID :" << fingerID << std::endl;
	matPrint(&tau);
#endif
	// 返り値に代入
	jnt_force[ARM_M1] = tau.el[ARM_M1][0];
	jnt_force[ARM_M2] = tau.el[ARM_M2][0];

	//	if(_this->step == IMP_SWITCH_STEP){	_this->jnt_force[ARM_M1] = 0.0;	_this->jnt_force[ARM_M2] = 0.0;}
		// 駆動力制限
	//	for(jnt=0;jnt<ARM_JNT;jnt++)	if(_this->jnt_force[jnt] > 100 || _this->jnt_force[jnt] < -100)	_this->jnt_force[jnt] = 0.0;
		// 駆動力入力をODEに設定

	for (int jnt = 0; jnt < ARM_JNT; jnt++)	dJointAddHingeTorque(r_joint[jnt], jnt_force[jnt]);		// トルクは上E書きではなくインクリメントされることに注意
}

////////////////////////////////////////////////////////
// 外力描画
// 把持対象に加える外力を表示する
////////////////////////////////////////////////////////
int drawExtForcePlate() {
	int width;
	dJointFeedback* fb;
	dVector3	p_e;    // 線の始点と終点
	double k1 = 0.3;  // 線長の比例定数
	double line_w = 0.05;
	dVector3	arrow_center, arrow_r, arrow_l;    // 矢印の頂点
	dVector3	rect_ul, rect_ll, rect_ur, rect_lr;    // 矢印の頂点
	dVector3	line, line_e;    // 
	
	dMatrix3	R;
	double angArrow = PI / 6;  //矢印の角度 rad
	dJointFeedback* p_force;
	
	//	MyObject *sensor = &sim->sys.finger[ARM_N1].sensor;
	auto _this = EntityManager::get();
	//現在の位置
	const dReal* nowPos = dBodyGetPosition(plateToGrasp.body);
    //0.75はARM_LINK2_LENと同じ値
	//dBodyGetRelPointPos(plateToGrasp.body, 0.0, 0.0,0.75/2.0, p_s);			// 手先位置
	
	p_force = dJointGetFeedback(_this->getFinger()->f2_joint);
	//for (int crd = 0; crd < DIM3; crd++)	ext_f[crd] = -p_force->f1[crd];	// 対象がセンサに及ぼしている力=センサが関節に及ぼしている力
	auto sim = EntityManager::get();
	int forceDir = (sim->step % 500 == 0) ? -1 : 1;
	dVector3	ext_f = {0,2.0*forceDir,0};	// 外力の向きと強さ
	dVector3	p_s = { nowPos[0] - 0.5,nowPos[1], nowPos[2] };//外力を食わる位置(絶対座標)
	//0.75はARM_LINK2_RADと同じ値
	p_s[CRD_Z] += 0.10; //腕の上に表示
	for (int crd = 0; crd < DIM3; crd++)	p_e[crd] = p_s[crd] - k1 * ext_f[crd];
	//	p_e[CRD_Z] = p_s[CRD_Z] + sensor.r;	// 腕の上に表示
	p_e[CRD_Z] = p_s[CRD_Z];	// z方向の力は無視
	dsSetColor(1.0, 1.0, 1.0);                    // 
#if 1
//	arrow_l[CRD_X] = p_s[CRD_X]+k1/2*fb->f1[CRD_X]-k1/2*fb->f1[CRD_Y];
//	arrow_l[CRD_Y] = p_s[CRD_Y]+k1/2*fb->f1[CRD_Y]+k1/2*fb->f1[CRD_X];
//	arrow_l[CRD_Z] = p_s[CRD_Z];
//	arrow_r[CRD_X] = p_s[CRD_X]+k1/2*fb->f1[CRD_X]+k1/2*fb->f1[CRD_Y];
//	arrow_r[CRD_Y] = p_s[CRD_Y]+k1/2*fb->f1[CRD_Y]-k1/2*fb->f1[CRD_X];
//	arrow_r[CRD_Z] = p_s[CRD_Z];
	arrow_center[CRD_X] = 0;	arrow_center[CRD_Y] = 0.0;	arrow_center[CRD_Z] = 0;
	arrow_l[CRD_X] = sin(angArrow) * ext_f[0] * k1 / 3;
	arrow_l[CRD_Y] = cos(angArrow) * ext_f[0] * k1 / 3;
	arrow_l[CRD_Z] = 0;
	arrow_r[CRD_X] = -sin(angArrow) * ext_f[0] * k1 / 3;
	arrow_r[CRD_Y] = cos(angArrow) * ext_f[0] * k1 / 3;
	arrow_r[CRD_Z] = 0;
	rect_ul[CRD_X] = rect_ll[CRD_X] = sin(angArrow) * ext_f[0] * k1 / 8;
	rect_ur[CRD_X] = rect_lr[CRD_X] = -sin(angArrow) * ext_f[0] * k1 / 8;
	rect_ul[CRD_Y] = rect_ur[CRD_Y] = cos(angArrow) * ext_f[0] * k1 / 3;
	rect_ll[CRD_Y] = rect_lr[CRD_Y] = k1 * ext_f[0];
	rect_ul[CRD_Z] = rect_ll[CRD_Z] = rect_ur[CRD_Z] = rect_lr[CRD_Z] = 0.0;

	dRFromAxisAndAngle(R, 0, 0, 1, PI - atan2(p_s[CRD_Y] - p_e[CRD_Y], p_s[CRD_X] - p_e[CRD_X]));
	//	printf("%f %f %f\n\n", rect_ll[0],rect_ll[1],rect_ll[2]);

		// 矢印の頭
	dsDrawTriangle(p_s, R, arrow_center, arrow_l, arrow_r, 1); // 
	// 矢印の線（三角形を2つ合わせて幅の持つ線を四角形として表示）
	dsDrawTriangle(p_s, R, rect_ul, rect_ll, rect_ur, 1); // 
	dsDrawTriangle(p_s, R, rect_ur, rect_ll, rect_lr, 1); // 

//	dsDrawLine(p_s, p_e); // p_sからp_eまでの直線を描画
//	dsDrawLine(line_center, p_e); // p_sからp_eまでの直線を描画
#endif
	return	0;
}


