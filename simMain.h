#ifndef _INC_SIMMAIN
#define _INC_SIMMAIN

#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "setRobot.h"
#include "matBase.h"
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawLine dsDrawLineD
#define dsDrawTriangle dsDrawTriangleD
#endif

////////////////////////////////////////////////////////

// 行列計算Eigen(マクロ定義はインクルードより前に必要)
#define EIGEN_NO_DEBUG // コード内のassertを無効化．
#define EIGEN_DONT_VECTORIZE // SIMDを無効化．
#define EIGEN_DONT_PARALLELIZE // 並列を無効化．
#define EIGEN_MPL2_ONLY // LGPLライセンスのコードを使わない．
//#include "eigen-3.3.7/Eigen/Core"
//#include "eigen-3.3.7/Eigen/LU"		// 逆行列や行列式の計算に必要
//#include "c:/software/eigen-3.3.7/Eigen/Dense"		// 上2つの代わりにこれ1つでもOK

#include "c:/eigen-3.4.0/Eigen/Dense"
////////////////////////////////////////////////////////
// バイナリフラグ
// ON(1)とOFF(0)のみ設定可能
////////////////////////////////////////////////////////
#define	GLAPHIC_OPENGL		0		// OpenGLで描画
#define	FLAG_DRAW_SIM		1		// ODEの標準描画

#define	FLAG_SAVE_IMAGE		0		// 画像保存
#define	FLAG_SAVE_VIDEO		0		// 動画保存(OpenCVが必要)


////////////////////////////////////////////////////////
// define定義
////////////////////////////////////////////////////////
#ifndef PI
#define PI (3.14159265358979323846)
#endif
// 画面表示定義
#define	DISPLAY_WIDTH	640
#define	DISPLAY_HEIGHT	480

// 次元・座標識別定義
#define DIM2	2
#define	DIM3	3	// 3次元の位置や姿勢

#define	CRD_X	0
#define	CRD_Y	1
#define	CRD_Z	2
#define	AXIS_X	0
#define	AXIS_Y	1
#define	AXIS_Z	2

#define DIR_LONG_AXIS_Z	3	// 長軸方向(dMassSetCylinderTotalなどに利用)
// 定数定義
#define SYSTEM_CYCLE_TIME	(0.001)	// 実験用サイクルタイム
#define SIM_CYCLE_TIME	(0.001)	// シミュレーション用サイクルタイム
#define DATA_CNT_NUM	5000	// データ保存カウント数
#define SAVE_IMG_RATE	200		// 画像保存間隔カウント数
#define SAVE_VIDEO_RATE	33		// 動画保存間隔カウント数
// 文字列定義
//#define GNUPLOT_PATH	"\"C:\\Program Files\\gnuplot\\bin\\pgnuplot.exe\""	// パスに空白があるため[\"]を前後に追加
#define GNUPLOT_PATH	"\"C:\\Program Files (x86)\\gnuplot\\bin\\gnuplot.exe\""	// パスに空白があるため[\"]を前後に追加
#define FILE_SAVE_DIR	"./data/"		// ファイル保存ディレクトリ
#define FILENAME_DATA	FILE_SAVE_DIR "data_%.3d.txt"		// 連番3桁対応
#define FILENAME_INFO	FILE_SAVE_DIR "info_%.3d.txt"		// 連番3桁対応
#define FILENAME_GRAPH1	FILE_SAVE_DIR "img_jnt_pos.png"
#define FILENAME_GRAPH2	FILE_SAVE_DIR "img_jnt_vel.png"
#define FILENAME_GRAPH3	FILE_SAVE_DIR "img_jnt_force.png"
#define FILENAME_GRAPH4	FILE_SAVE_DIR "img_eff_force.png"
#define FILENAME_GRAPH5	FILE_SAVE_DIR "img_err.png"
#define	DATA_FILE_NAME_MAXLEN	20

#define FILENAME_VIDEO	FILE_SAVE_DIR "cap.mp4"		// ビデオ名

// アーム定義
// 2自由度アーム　各リンクは円柱で構成

#if 0
#define	ARM_JNT	2
#define	ARM_M1	0
#define	ARM_M2	1
#define	ARM_LINK1_LEN	0.75		// リンク長
#define	ARM_LINK2_LEN	0.75		// リンク長

#define	ARM_LINK1_COG_LEN	(ARM_LINK1_LEN/2.0)		// 質量中心
#define	ARM_LINK2_COG_LEN	(ARM_LINK2_LEN/2.0)		// 質量中心
#define	ARM_LINK1_RAD	0.125		// リンク半径
#define	ARM_LINK2_RAD	0.10		// リンク半径
#define	ARM_LINK1_MASS	1.0		// 質量
#define	ARM_LINK2_MASS	0.8		// 質量

#define	ARM_JNT1_VISCOUS	1.0		// 粘性係数
#define	ARM_JNT2_VISCOUS	1.0		// 粘性係数
#elif 1
constexpr int	ARM_NUM = 2;	// アーム本数
constexpr int	ARM_N1 = 0;	// アーム番号
constexpr int	ARM_N2 = 1;	// アーム番号

constexpr int	ARM_JNT = 2;	// アーム関節数
constexpr int	ARM_M1 = 0;	// アーム関節番号
constexpr int	ARM_M2 = 1;	// アーム関節番号
constexpr double	ARM_LINK1_LEN = 0.75;		// リンク長
constexpr double	ARM_LINK2_LEN = 0.75;		// リンク長

constexpr double	ARM_LINK1_COG_LEN = ARM_LINK1_LEN / 2.0;		// 質量中心
constexpr double	ARM_LINK2_COG_LEN = ARM_LINK2_LEN / 2.0;		// 質量中心
constexpr double	ARM_LINK1_RAD = 0.125;		// リンク半径
constexpr double	ARM_LINK2_RAD = 0.10;		// リンク半径
constexpr double	ARM_LINK1_MASS = 1.0;		// 質量
constexpr double	ARM_LINK2_MASS = 0.8;		// 質量

constexpr double	ARM_JNT1_VISCOUS = 1.0;		// 粘性係数
constexpr double	ARM_JNT2_VISCOUS = 1.0;		// 粘性係数

#endif

////////////////////////////////////////////////////////
// 構造体定義
////////////////////////////////////////////////////////

// 変数構造体
struct Variable {
	Matrix	q, dq, ddq;	// 関節角度，関節速度，関節加速度
	Matrix	r, dr, ddr;	// 手先位置，手先速度，手先加速度
	Matrix	F;	// 手先外力
//	Matrix	dq;	// 関節速度
//	Matrix	dr;	// 手先速度
//	Matrix	ddq;	// 関節加速度
//	Matrix	ddr;	// 手先加速度
	/*
	// 補足変数（初期値）
	Matrix	q0;	// 関節角
	Matrix	r0;	// 手先位置
	Matrix	F0;	// 手先外力
	Matrix	dq0;	// 関節速度
	Matrix	dr0;	// 手先速度
	*/
	Variable() {
		matInit(&q, ARM_JNT, 1); matInit(&dq, ARM_JNT, 1); matInit(&ddq, ARM_JNT, 1);
		matInit(&r, DIM2, 1); matInit(&dr, DIM2, 1); matInit(&ddr, DIM2, 1);
		matInit(&F, DIM2, 1);
	}
};


// 運動学構造体
struct Kinematics {       //
	double	l[ARM_JNT];		// リンク長
	double	lg[ARM_JNT];		// リンク重心位置までの長さ
	double	r[ARM_JNT];		// リンク半径（太さ方向）
	Matrix	J;	// ヤコビアン
	Matrix	dJ;	// ヤコビアン微分
	Matrix	Jt, Jinv;	// 転置行列，逆行列

	Kinematics(){
		this->l[ARM_M1] = ARM_LINK1_LEN;	this->l[ARM_M2] = ARM_LINK2_LEN;
		this->lg[ARM_M1] = ARM_LINK1_COG_LEN;	this->lg[ARM_M2] = ARM_LINK2_COG_LEN;
		this->r[ARM_M1] = ARM_LINK1_RAD;	this->r[ARM_M2] = ARM_LINK2_RAD;
		matInit(&J, ARM_JNT, ARM_JNT); matInit(&dJ, ARM_JNT, ARM_JNT);
		matInit(&Jt, ARM_JNT, ARM_JNT); matInit(&Jinv, ARM_JNT, ARM_JNT);
	}
};


// 動力学構造体
// Mq*ddq + h + V*dq = tau + Jt*F
struct Dynamics {       //
	double	m[ARM_JNT];		// リンク質量
	Matrix	Mq;		// 慣性項
	Matrix	h;	// コリオリ・遠心力項
	double	V[ARM_JNT];	// 粘性摩擦係数
	// 補足変数
	Matrix	dMq;		// 慣性項微分

	Dynamics() {
		this->m[ARM_M1] = ARM_LINK1_MASS;	this->m[ARM_M2] = ARM_LINK2_MASS;
		this->V[ARM_M1] = ARM_JNT1_VISCOUS;	this->V[ARM_M2] = ARM_JNT2_VISCOUS;
		matInit(&Mq, ARM_JNT, ARM_JNT); matInit(&h, ARM_JNT, 1);
		matInit(&dMq, ARM_JNT, ARM_JNT);
	}
};


// インピーダンス構造体
struct  Impedance {
	Matrix	M, C, K;	// 目標インピーダンス（手先座標）
	Matrix	K0;	// SLS用
	Matrix	Gp, Gv;	// インナーループ用ゲイン（比例ゲイン，微分ゲイン）
	double	T[DIM3];	// 周期
	// 補足変数
	Matrix	dM, dC, dK;	// 目標インピーダンス微分（手先座標）
	Matrix	Minv, Cinv, Kinv;	// 逆行列

	Impedance() {
		matInit(&M, DIM2, DIM2); matInit(&C, DIM2, DIM2); matInit(&K, DIM2, DIM2);
		matInit(&Minv, DIM2, DIM2); matInit(&Cinv, DIM2, DIM2); matInit(&Kinv, DIM2, DIM2);
		matInit(&Gp, DIM2, DIM2); matInit(&Gv, DIM2, DIM2);
		matInit(&dM, DIM2, DIM2); matInit(&dC, DIM2, DIM2); matInit(&dK, DIM2, DIM2);
		matInit(&K0, DIM2, DIM2);
	}
};

////////////////////////////////////////////////////////

// プロトタイプ
////////////////////////////////////////////////////////
struct Vec3 { double x, y, z; Vec3(double x, double y, double z) : x(x), y(y), z(z) {} };

// パーツクラス
class cParts {
protected:
	dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
	dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）

	dReal  m;       // 質量[kg]
	dMass mass;
	std::vector<float> color{1,1,1};
public:
	cParts(dReal m);
	cParts(dReal m, Vec3 init_pos);
	~cParts() { dBodyDestroy(this->body); dGeomDestroy(this->geom);	std::cout << "destroy" << std::endl; }
	dBodyID getBody() { return this->body; }
	dGeomID getGeom() { return this->geom; }
	// 位置を設定
	void setPosition(double x, double y, double z) { dBodySetPosition(getBody(), x, y, z); }
	// 位置を取得
	auto getPosition() { return dBodyGetPosition(getBody()); }
	// 回転を設定
	void setRotation(double ang) {
		dMatrix3	R;
		dRFromAxisAndAngle(R, -sin(ang), cos(ang), 0, PI / 2);
		dBodySetRotation(getBody(), R);
	}
	void setColor(float r, float g, float b) { color[0] = r; color[1] = g; color[2] = b; }
	// 回転を取得
	//	Quaternion getRotation() const;
	// サイズを取得
	//	Vec3 getSize() const { return this->size; }
	void destroy() { dBodyDestroy(getBody()); dGeomDestroy(getGeom()); }
	// 仮想関数
	virtual dReal getl() { return 0; }	// 取りあえずreturnで0を返している
	virtual dReal getr() { return 0; }	// 取りあえずreturnで0を返している
	virtual void draw() {}
};

// オブジェクト
class cPartsBox : public cParts {
	dReal	sides[DIM3];	// 直方体x,y,zの辺長
public:
	cPartsBox(dReal m, Vec3 l);
	cPartsBox(dReal m, Vec3 init_pos, Vec3 l);
	~cPartsBox() {}		// デストラクタ
	// 描画
	auto get() { return this->sides; }
	// 描画
	void draw() {		// 環境設定
		dsSetColor(color[0], color[1], color[2]);
		dsDrawBox(dBodyGetPosition(getBody()), dBodyGetRotation(getBody()), this->sides);
	}
};

// オブジェクト
class cPartsCylinder : public cParts {
	dReal  l, r;       // 長さ[m], 半径[m]
public:
	cPartsCylinder(dReal m, dReal l, dReal r);
	cPartsCylinder(dReal m, Vec3 init_pos, dReal l, dReal r);
	~cPartsCylinder() {}		// デストラクタ
	dReal getl() { return l; }
	dReal getr() { return r; }
	void draw() {
		dsSetColor(color[0], color[1], color[2]);
		dsDrawCylinder(dBodyGetPosition(getBody()), dBodyGetRotation(getBody()), this->l, this->r);
	}
};

// 指
class cFinger {

	cPartsBox	base{ 14.0, Vec3(0.3, 0.5, 0.4)};

	cPartsCylinder	link1{ ARM_LINK1_MASS, ARM_LINK1_LEN, ARM_LINK1_RAD };
	cPartsCylinder	link2{ ARM_LINK2_MASS, ARM_LINK2_LEN, ARM_LINK2_RAD };
//	std::vector<cParts*> finger{ 4 };	// 4 = ARM_JNT + base + sensor
	std::vector<cParts*> finger;
	//dReal x0 = 0.0, y0 = 0.0, z0 = 1.5;

	dReal x0 = 0.5, y0 = 0.0, z0 = 1.5;			//	書き換えた後1本目の指の土台の位置　kawahara
	dReal x1 = 0.5, y1 = -1.0, z1 = 1.5;		//	書き換えた後2本目の指の土台の位置　kawahara

	dReal px1 =-20, py1 = -20, pz1 = 0;
	double Z_OFFSET = 0.08;
	//double jnt_pos[ARM_JNT];
public:
	

	cPartsCylinder	sensor{ 0.0001 / ARM_LINK2_LEN * ARM_LINK2_MASS, 0.0001, ARM_LINK2_RAD };	// アームと密度をそろえる

	//把持するプレート{質量,初期位置(x,y,z),大きさ(x,y,z)}
	cPartsBox	plate{ 10.0, Vec3(px1,py1,pz1),Vec3(1.5,0.5,0.5) };

	dJointFeedback force, *p_force;
	dJointID f_joint, r_joint[ARM_JNT], f2_joint; // 固定関節と回転関節
	dJointID graspObj; 							  //把持対象のプレート kawahara

	// 指の制御用変数
	int fingerID;
	int state_contact;			// 接触状態(0:OFF, 1:ON)

	double	dist;				// アームと対象の距離
	double	jnt_pos[ARM_JNT] = {};
	double	jnt_vel[ARM_JNT] = {};
	double	jnt_force[ARM_JNT] = {};
	double	past_jnt_pos[ARM_JNT] = {};
	double	eff_pos[DIM3] = {};
	double	eff_vel[DIM3] = {};
	double	eff_force[DIM3] = {};
	double	obj_pos[DIM3] = {};
	double	obj_vel[DIM3] = {};
	// 目標変数
	double	ref_jnt_pos[ARM_JNT] = {};
	double	ref_jnt_vel[ARM_JNT] = {};
	double	ref_eff_pos[DIM3] = {};
	double	ref_eff_vel[DIM3] = {};
	// 初期変数
	double	init_jnt_pos[ARM_JNT] = {};
	double	init_obj_pos[DIM3] = {};

	double	init_obj_att[DIM3][DIM3] = {};	// 絶対座標における対象座標軸の姿勢（軸は正規直交基底）
	// 変数構造体
	Variable	var;			// 現在値
	Variable	var_prev;		// 過去値（1サイクル前）
	Variable	var_prev2;		// 過去値（2サイクル前）

	Variable	var_init;		// 初期値
	// 運動学変数
	Kinematics	kine;
	// 動力学変数
	Dynamics	dyn;
	// インピーダンス変数
	Impedance	imp;
	// 保存用データ変数

	int save_state_contact[DATA_CNT_NUM] = {};
	double	save_dist[DATA_CNT_NUM] = {};
	double	save_ref_jnt_pos[DATA_CNT_NUM][ARM_JNT] = {};
	double	save_ref_jnt_vel[DATA_CNT_NUM][ARM_JNT] = {};
	double	save_jnt_pos[DATA_CNT_NUM][ARM_JNT] = {};
	double	save_jnt_vel[DATA_CNT_NUM][ARM_JNT] = {};
	double	save_jnt_force[DATA_CNT_NUM][ARM_JNT] = {};
	double	save_ref_eff_pos[DATA_CNT_NUM][DIM3] = {};
	double	save_ref_eff_vel[DATA_CNT_NUM][DIM3] = {};
	double	save_eff_pos[DATA_CNT_NUM][DIM3] = {};
	double	save_eff_vel[DATA_CNT_NUM][DIM3] = {};
	double	save_eff_force[DATA_CNT_NUM][DIM3] = {};
	double	save_obj_pos[DATA_CNT_NUM][DIM3] = {};
	double	save_obj_vel[DATA_CNT_NUM][DIM3] = {};
	// 保存用ファイル名変数
	char	data_file_name[DATA_FILE_NAME_MAXLEN] = {};
	char	filename_info[DATA_FILE_NAME_MAXLEN] = {};
	char	filename_graph[DATA_FILE_NAME_MAXLEN] = {};
	// メンバ関数
	void initJntPos(double* init_jnt_pos) {}
	int armWithoutInertiaShaping();
	int ctrlPreProcessing();
	int armDynPara();
	int armInvKine(Kinematics* kine, Variable* var);
	int armJacob(Kinematics* kine, Variable* var);
	int armInitMat(Variable* var, Kinematics* kine, Dynamics* dyn, Impedance* imp);
	

	//kawaharaの変更以前からコメントアウト
	////	int armInitMatVar(Variable *var);
	////	int armInitMatKine(Kinematics *kine);

	int ctrlInitErr();
	int armCalcImpPeriod();
	void saveData();
	void saveInfo();
	void saveGraph();

	//debug用　kawaharaが追加
	void printInfo();

	////Finger classの中に移勁E
	//int ctrlMaxwell(Matrix* tau);

	
	cFinger(double* init_jnt_pos) 
		: jnt_pos{ init_jnt_pos[0], init_jnt_pos[1] }, 
		finger{&base, &link1, &link2, &sensor} {
		this->kine = Kinematics();
		this->dyn = Dynamics();
		this->imp = Impedance();
		this->var = Variable();
		this->var_prev = Variable();
		this->var_prev2 = Variable();
		this->var_init = Variable();
	}
~cFinger() {		// ジョイント破壊
		dJointDestroy(f_joint);   // 土台固定
		dJointDestroy(r_joint[ARM_M1]);   // アーム
		dJointDestroy(r_joint[ARM_M2]);   // アーム
		dJointDestroy(f2_joint);   // センサ固定

	}
	auto getParts() { return finger; }
	//	void setPosition(const dVector3 pos) {
	void setPosition() {
		//inger[0]->setPosition(x0, y0, 0.4 / 2);	// z:base->sides[CRD_Z]/2
		//finger[0]->setPosition(x0, y0, 0.4);	// z:base->sides[CRD_Z]/2

		finger[0]->setPosition(x0, y0, 0.2);	// z:base->sides[CRD_Z]/2
		finger[1]->setPosition(x0 + ARM_LINK1_LEN / 2.0*cos(jnt_pos[ARM_M1]), y0 + ARM_LINK1_LEN / 2.0*sin(jnt_pos[ARM_M1]), 0.4 / 2.0 - Z_OFFSET);
		finger[2]->setPosition(x0 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + ARM_LINK2_LEN / 2.0*cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), y0 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + ARM_LINK2_LEN / 2.0*sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), 0.4 / 2.0 - Z_OFFSET);
		finger[3]->setPosition(x0 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.0001 / 2.0)*cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), y0 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.0001 / 2.0)*sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), 0.4 / 2.0 - Z_OFFSET);
		finger[0]->setRotation(0);
		finger[1]->setRotation(jnt_pos[ARM_M1]);
		finger[2]->setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		finger[3]->setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
	}

	//kawaharaが追加　二本目の指用
	void setPosition2() {
		//finger[0]->setPosition(x0, y0, 0.4 / 2);	// z:base->sides[CRD_Z]/2
		//finger[0]->setPosition(x0, y0, 0.4);	// z:base->sides[CRD_Z]/2


		finger[0]->setPosition(x1,y1, 0.2);	// z:base->sides[CRD_Z]/2
		finger[1]->setPosition(x1 + ARM_LINK1_LEN / 2.0 * cos(jnt_pos[ARM_M1]), y1 + ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1]), 0.4 / 2.0 - Z_OFFSET);
		finger[2]->setPosition(x1 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + ARM_LINK2_LEN / 2.0 * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), y1 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), 0.4 / 2.0 - Z_OFFSET);
		finger[3]->setPosition(x1 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.0001 / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), y1 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.0001 / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), 0.4 / 2.0 - Z_OFFSET);
		finger[0]->setRotation(0);
		finger[1]->setRotation(jnt_pos[ARM_M1]);
		finger[2]->setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		finger[3]->setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
	}
	void setColor(std::vector<Vec3> color) {
		auto i = color.begin();
		for (auto j = finger.begin(); j != finger.end(); ++j, ++i) 	(*j)->setColor((*i).x, (*i).y, (*i).z);
	}

	void setJoint();	// 関節設定
	void setJoint2();	// 関節設定 2本目の指
	void setJntFric();	// 摩擦設定
	void addExtForce();		// 外力

	void addExtForce2();	// 外力
	//kawaharaが追加
	int calcDist();
	int ctrlMaxwell(Matrix* tau);
	int ctrlMaxwell2(Matrix* tau);		//kawaharaが追記　二本目の指用

	//制約条件つきMaxwellモデル
	int RestrictedCtrlMaxwell(Matrix* tau);
	int RestrictedCtrlMaxwell2(Matrix* tau);	//kawaharaが追記　二本目の指用
	void control();		// 制御
	void destroy() { for (auto &x : finger) { x->destroy(); } }
	void draw() { for (auto &x : finger) { x->draw(); } }
};

////////////////////////////////////////////////////////
// DrawStuffクラス
////////////////////////////////////////////////////////
class DrawStuff {
	static dsFunctions fn;	// 描画変数
	// 視点変数
	static float xyz[3];

	static float hpr[3];	// 単位はdeg
public:
	DrawStuff()	{	// 描画関数の設定
		fn.version = DS_VERSION;    // ドロースタッフのバージョン
		fn.start = &start;			// 前処理 start関数のポインタ
		fn.step = &simLoop;			// simLoop関数のポインタ
		fn.command = &command;      // キー入力関数へのポインタ
		fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH; // テクスチャ

	}
	static void start();
	static void simLoop(int pause);
	static void command(int cmd);
	auto getFn() { return &fn; }
};

////////////////////////////////////////////////////////
// ODEクラス
////////////////////////////////////////////////////////
class ODE {
	// ODE

	dWorldID world;  // 動力学計算用ワールド
	dSpaceID space;  // 衝突検出用スペース
public:
	dGeomID  ground; // 地面
	dJointGroupID contactgroup; // コンタクトグループ
	ODE() {		// デフォルトコンストラクタ

		dInitODE();
		this->world = dWorldCreate();
		this->space = dHashSpaceCreate(0);
		this->contactgroup = dJointGroupCreate(0);
	}

	~ODE() {		// デストラクタ
		dJointGroupDestroy(this->contactgroup);     // ジョイントグループの破壊

		dSpaceDestroy(this->space);
		dWorldDestroy(this->world);
		dCloseODE();
	}

	void setEnv() {		// 環境設定
		dWorldSetGravity(this->world, 0, 0, -9.8);	// 重力設定
		dWorldSetERP(this->world, 0.9);          // ERPの設定
		dWorldSetCFM(this->world, 1e-4);         // CFMの設定

	}
	auto getWorld() const -> decltype(world) { return this->world; }
	auto getSpace() const -> decltype(space) { return this->space; }
//	void step(dReal time) { dWorldStep(this->world, time); }
	static void nearCallback(void *data, dGeomID o1, dGeomID o2);
};

class EntityODE : public ODE {
//	std::unique_ptr<cFinger> pFinger;
//	std::unique_ptr<cPartsCylinder> pObj;
	std::shared_ptr<cFinger> pFinger;
	//std::shared_ptr<cFinger> pFinger;


	std::shared_ptr<cFinger> pFinger2;	//二本目の指　kawahara


	std::shared_ptr<cPartsCylinder> pObj;

	std::shared_ptr<cPartsCylinder> pObj2;

public:
	int FingerNum=0;
	void setup() {
		constexpr auto OBJ_RADIUS = 0.10;
		//double init_jnt_pos[2] = { 4 * PI / 4.0, PI/ 4.0 };
		//各関節の初期姿勢(角度)

		double init_jnt_pos[2] = {  4* PI / 4.0, PI/5.0 };
		double init_jnt_posF2[2] = { 4 * PI / 4.0, -PI / 5.0  };//二本目の指
		Vec3 obj_pos = { Vec3(-0.8 / sqrt(2.0) - 2 * 0.75 / sqrt(2.0), -0.8 / sqrt(2.0), OBJ_RADIUS) };
		

		//1本目の指
		this->pFinger = std::make_shared<cFinger>(init_jnt_pos);
		this->pFinger->fingerID = ++FingerNum;

		//2本目の指
		this->pFinger2 = std::make_shared<cFinger>(init_jnt_posF2);	
		this->pFinger2->fingerID = ++FingerNum;


		this->pObj = std::make_shared<cPartsCylinder>(0.2, obj_pos, 0.15, 0.10);
		this->pObj2 = std::make_shared<cPartsCylinder>(0.2, obj_pos, 0.15, 0.10);

		std::vector<Vec3> color{ Vec3(1, 0, 0), Vec3(0, 0, 1), Vec3(0, 0.5, 0.5), Vec3(0, 0.5, 0.5) };
		
		this->pFinger->setColor(color);

		this->pFinger2->setColor(color);	//二本目の指　kawaharaが追加


	}
	void createRobot();		// ロボット生成（ボディ・ジオメトリ・ジョイント）
	void createObject();	// 対象生成（ボディ・ジオメトリ）
	void destroyRobot() {	// ロボット破壊（ジョイント・ボディ・ジオメトリ）
		pFinger.reset();	// インスタンスの破壊
		pFinger2.reset();
		//this->pFinger->destroy();
	}
	void destroyObject() {	pObj.reset(); } // インスタンスの破壊	// 対象破壊（ボディ・ジオメトリ）
	void update() {		// シミュレーションを１ステップ進行
		dSpaceCollide(this->getSpace(), 0, &this->nearCallback);		// 衝突判定
		dWorldStep(this->getWorld(), SIM_CYCLE_TIME);	// 1ステップ進める

		dJointGroupEmpty(this->contactgroup); // ジョイントグループを空にする
	}
	auto getFinger() { return pFinger; }
	auto getFinger2() { return pFinger2; }


	auto getObj() { return pObj; }
	auto getObj2() { return pObj2; }
	void setAddForceObj(dReal fx, dReal fy, dReal fz) { dBodyAddForce(pObj->getBody(), fx, fy, fz); }
};

////////////////////////////////////////////////////////

// シミュレーション構造体

////////////////////////////////////////////////////////
class SIM: public EntityODE {

public:
	int	step;					//経過ステップ数
//	// 1本目の指用
//	// 制御用変数
//	int	step;					//経過ステップ数
//	int state_contact;			// 接触状態(0:OFF, 1:ON)

//	double	dist;				// アームと対象の距離

//	double	jnt_pos[ARM_JNT];
//	double	jnt_vel[ARM_JNT];
//	double	jnt_force[ARM_JNT];
//	double	past_jnt_pos[ARM_JNT];
//	double	eff_pos[DIM3];
//	double	eff_vel[DIM3];
//	double	eff_force[DIM3];
//	double	obj_pos[DIM3];
//	double	obj_vel[DIM3];
//	// 目標変数
//	double	ref_jnt_pos[ARM_JNT];
//	double	ref_jnt_vel[ARM_JNT];
//	double	ref_eff_pos[DIM3];
//	double	ref_eff_vel[DIM3];
//	// 初期変数
//	double	init_jnt_pos[ARM_JNT];
//	double	init_obj_pos[DIM3];

//	double	init_obj_att[DIM3][DIM3];	// 絶対座標における対象座標軸の姿勢（軸は正規直交基底）
//	// 変数構造体

//	Variable	var;			// 現在値

//	Variable	var_prev;		// 過去値（1サイクル前）
//	Variable	var_prev2;		// 過去値（2サイクル前）
//	Variable	var_init;		// 初期値
//	// 運動学変数
//	Kinematics	kine;
//	// 動力学変数
//	Dynamics	dyn;
//	// インピーダンス変数
//	Impedance	imp;
//	// 保存用データ変数

//	int save_state_contact[DATA_CNT_NUM];
//	double	save_dist[DATA_CNT_NUM];
//	double	save_ref_jnt_pos[DATA_CNT_NUM][ARM_JNT];
//	double	save_ref_jnt_vel[DATA_CNT_NUM][ARM_JNT];
//	double	save_jnt_pos[DATA_CNT_NUM][ARM_JNT];
//	double	save_jnt_vel[DATA_CNT_NUM][ARM_JNT];
//	double	save_jnt_force[DATA_CNT_NUM][ARM_JNT];
//	double	save_ref_eff_pos[DATA_CNT_NUM][DIM3];
//	double	save_ref_eff_vel[DATA_CNT_NUM][DIM3];
//	double	save_eff_pos[DATA_CNT_NUM][DIM3];
//	double	save_eff_vel[DATA_CNT_NUM][DIM3];
//	double	save_eff_force[DATA_CNT_NUM][DIM3];
//	double	save_obj_pos[DATA_CNT_NUM][DIM3];
//	double	save_obj_vel[DATA_CNT_NUM][DIM3];
//	// 保存用ファイル名変数
//	char	data_file_name[DATA_FILE_NAME_MAXLEN];
//	char	filename_info[DATA_FILE_NAME_MAXLEN];
//	char	filename_graph[DATA_FILE_NAME_MAXLEN];
//	// メンバ関数
//	void initJntPos(double *init_jnt_pos) {}
//	int armWithoutInertiaShaping();
//	int ctrlPreProcessing();
//	int armDynPara();
//	int armInvKine(Kinematics *kine, Variable *var);
//	int armJacob(Kinematics *kine, Variable *var);
//	int armInitMat(Variable *var, Kinematics *kine, Dynamics *dyn, Impedance *imp);

//////	int armInitMatVar(Variable *var);
//////	int armInitMatKine(Kinematics *kine);
//int ctrlInitErr();	

//	int armCalcImpPeriod();
//	void saveData();
//	void saveInfo();
//	void saveGraph();




};




// 単一インスタンス管理
template<typename WorldT>
struct Manager{
	static std::unique_ptr<WorldT> pWorldInstance;
public:
	static WorldT* init()
	{
		if (pWorldInstance == nullptr) pWorldInstance = std::make_unique<WorldT>();
		return pWorldInstance.get();
	}
	static WorldT* get() { return pWorldInstance.get(); }
};
template<typename WorldT> std::unique_ptr<WorldT> Manager<WorldT>::pWorldInstance; 
using EntityManager = Manager<SIM>;

////////////////////////////////////////////////////////

// プロトタイプ
////////////////////////////////////////////////////////
static void restart();
int exeCmd(int argc, char *argv[]);


#endif
