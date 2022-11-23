
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


//matplotlibでグラフ描画用
//#include <iostream>
//#include <cmath>
//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;

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
#define VIEW_FROM_X 1		//カメラの開始時点での向き
#define VIEW_FROM_Y 0		//カメラの開始時点での向き

#define ADD_EXT_FORCE 0
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
constexpr double	ARM_LINK1_LEN = 0.65;		// リンク長 0.75
constexpr double	ARM_LINK2_LEN = 0.55;		// リンク長

constexpr double	ARM_LINK1_COG_LEN = ARM_LINK1_LEN / 2.0;		// 質量中心
constexpr double	ARM_LINK2_COG_LEN = ARM_LINK2_LEN / 2.0;		// 質量中心

constexpr double	PLATE_MASS = 8.0;
constexpr double	PLATE_X_LEN = 1.5;
constexpr double	PLATE_Y_LEN = 0.5;
constexpr double	PLATE_Z_LEN = 0.5;

//	旋回関節とそのリンク
constexpr double	SENKAI_LINK_LEN = 0.3;
constexpr double	SENSOR_LEN = 0.01;	//センサの長さ等について設定

constexpr double	ARM_LINK1_RAD = 0.125;		// リンク半径
constexpr double	ARM_LINK2_RAD = 0.10;		// リンク半径
constexpr double	ARM_LINK1_MASS = 1.0;		// 質量
constexpr double	ARM_LINK2_MASS = 0.8;		// 質量
constexpr double	ARM_BASE_MASS = 1.0;		// オリジナル　14.0

constexpr double	ARM_JNT1_VISCOUS = 40.0;		// 粘性係数	//XY平面上で実験字は　1.0
constexpr double	ARM_JNT2_VISCOUS = 40.0;		// 粘性係数
constexpr double    OFFSET_VAL = -0.5;			//　実験では-0.3
//constexpr double    OFFSET_VAL = -0.0001;			//　実験では-0.3
constexpr double	OFFSET_VAL_SENKAI = PI / 8.0;	//旋回関節用のオフセット

std::string forceOutfilename1 = "./data/force_finger1.csv";
std::string forceOutfilename2 = "./data/force_finger2.csv";

std::string jntAngleOutfilename1 = "./data/finger1_info.csv";
std::string jntAngleOutfilename2 = "./data/finger2_info.csv";
#define addOffset		1			//把持力を生み出すオフセットを与えるとき

#endif


template <typename T>
T angToRad(T ang) {
	return (ang / 360.0) * 2 * PI;
}

template <typename T>
T radToAng(T rad) {
	return (rad / (2 * PI)) * 360.0;
}
//	平面上の２点間の距離を返す
template <typename T>
T getDistPlain(T x1, T y1, T x2, T y2) {
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

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

	//	3次元シミュレーション用の変数	川原が追加
	Matrix  G,G_xyz;					//	重力項　
	Matrix  oTs, sT1, T12;				//	同時行列(4×4)	
	Matrix  ss, s1, s2;					//  質量中心位置を格納する(4×1)
	Matrix  oT1, oT2;					//	旋回根元からの変換(同次)行列
	Matrix	dqs_oTs, dqs_oT1, dqs_oT2;	//	同次行列の旋回角による微分
	Matrix	dq1_oTs, dq1_oT1, dq1_oT2;	//	同次行列のリンク1による微分
	Matrix	dq2_oTs, dq2_oT1, dq2_oT2;	//	同次行列のリンク2による微分


	Impedance() {
		matInit(&M, DIM2, DIM2); matInit(&C, DIM2, DIM2); matInit(&K, DIM2, DIM2);
		matInit(&Minv, DIM2, DIM2); matInit(&Cinv, DIM2, DIM2); matInit(&Kinv, DIM2, DIM2);
		matInit(&Gp, DIM2, DIM2); matInit(&Gv, DIM2, DIM2);
		matInit(&dM, DIM2, DIM2); matInit(&dC, DIM2, DIM2); matInit(&dK, DIM2, DIM2);
		matInit(&K0, DIM2, DIM2);
		//	重力項の計算用
		// matInit(row,col)//el[_row][_col] 
		matInit(&oTs, 4, 4); matInit(&sT1, 4, 4); matInit(&T12, 4, 4);
		matInit(&G, 1,3); matInit(&G_xyz, 4, 1);
		matInit(&ss, 1, 4); matInit(&s1, 1, 4); matInit(&s2, 1, 4);
		matInit(&oT1, 4, 4); matInit(&oT2, 1, 4);
		//	関節角で微分した同次行列
		matInit(&dqs_oTs, 4, 4); matInit(&dqs_oT1, 4, 4); matInit(&dqs_oT2, 4, 4);
		matInit(&dq1_oTs, 4, 4); matInit(&dq1_oT1, 4, 4); matInit(&dq1_oT2, 4, 4);
		matInit(&dq2_oTs, 4, 4); matInit(&dq2_oT1, 4, 4); matInit(&dq2_oT2, 4, 4);
	}
};

struct RotImpedance {
	//double K = 2.0, C = 4.0, lg = 0.03;
	//double K = 50.0, C = 4.0, lg = SENKAI_LINK_LEN / 2.0;	//3次元で重力に対して水平維持
	double K =  50.0, C = 4.0, lg = SENKAI_LINK_LEN/2.0;
	// 把持物体の慣性モーメント
	double Id = (PLATE_MASS / 3.0) * (PLATE_Y_LEN * PLATE_Y_LEN + PLATE_Z_LEN * PLATE_Z_LEN);
	//double Id = (PLATE_MASS/PLATE_X_LEN) / PLATE_Y_LEN; //PLATE_MASS / (PLATE_Y_LEN* PLATE_X_LEN* PLATE_Z_LEN);	//把持物体の慣性モーメント
	
	//	ロボットの慣性モーメント
	double Iq = (ARM_LINK1_MASS + ARM_LINK2_MASS + ARM_BASE_MASS +/*指先のセンサ部分*/+(SENSOR_LEN / ARM_LINK2_LEN * ARM_LINK2_MASS)/*先端のカプセル部分*/ + ARM_LINK2_MASS)/(SENKAI_LINK_LEN);		
	//	コリオリ遠心力
	double Ja = 1.0;	// omega = Ja*dq なのでこの場合は1.0
	//	慣性H
	double h = 10.0;		//適当なので後で変更する
	//	重力項
	double g_senkai;
	double V= ARM_JNT1_VISCOUS;	// 粘性摩擦係数
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
	dReal getMass() { return this->m;}	//kawaharaが追加
	// 位置を設定
	void setPosition(double x, double y, double z) { dBodySetPosition(getBody(), x, y, z); }
	// 位置を取得
	auto getPosition() { return dBodyGetPosition(getBody()); }
	// 回転を設定
	void setRotation(double ang) {
		dMatrix3	R;
		//dRFromAxisAndAngle(dQuaternion q,dReal ax,dReal ay,aReal az,dReal angle)
		//=回転軸ベクトル(ax,ay,az)を中心にangle[rad]回転させたときの回転行列を取得する
		dRFromAxisAndAngle(R, -sin(ang), cos(ang), 0, PI / 2);
		dBodySetRotation(getBody(), R);//Bodyに回転行列Rを設定する
	}
	void setRotationSenkai(double ang,double angZ) {

		dMatrix3	R;
		//dRFromAxisAndAngle(dQuaternion q,dReal ax,dReal ay,aReal az,dReal angle)
		//=回転軸ベクトル(ax,ay,az)を中心にangle[rad]回転させたときの回転行列を取得する
		//dRFromAxisAndAngle(R, -sin(0.0), cos(0.0), 0, PI/2.0);
		dRFromAxisAndAngle(R, -sin(ang), cos(ang), 0, PI / 2.0+angZ);
		dBodySetRotation(getBody(), R);//Bodyに回転行列Rを設定する
	}
	void setRotationFrom2Points(double x1,double y1,double z1, double x2, double y2, double z2) {

		dMatrix3	R;
		dRFromZAxis(R, x1-x2, y1-y2, z1-z2);
		dBodySetRotation(getBody(), R);//Bodyに回転行列Rを設定する
	}
	void setRotation2(double ang) {
		dMatrix3	R;
		dRFromAxisAndAngle(R, -1,0,0, PI);
		dBodySetRotation(getBody(), R);	//Bodyに回転行列Rを設定する
	}
	void setColor(float r, float g, float b) { color[0] = r; color[1] = g; color[2] = b; }
	// 回転を取得
	//	Quaternion getRotation() const;
	// サイズを取得
	//Vec3 getSize() const { return this->size; }
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
	dReal getVolume() { return sides[0]*sides[1]*sides[2]; }	//体積を返す
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
	dReal getVolume() { return r * r * PI * l; }	//体積を返す
	dReal getl() { return l; }
	dReal getr() { return r; }
	void draw() {
		dsSetColor(color[0], color[1], color[2]);
		dsDrawCylinder(dBodyGetPosition(getBody()), dBodyGetRotation(getBody()), this->l, this->r);
	}
};
//指先部分はカプセルに変更
class cPartsCapsule : public cParts {
	dReal l, r;			// 長さ[m], 半径[m]
public:
	cPartsCapsule(dReal m, dReal l, dReal r);
	cPartsCapsule(dReal m, Vec3 init_pos,dReal l, dReal r);
	dReal getVolumeHalfSquare() { return ((4.0/3.0)*PI*r*r*r)/2.0; }	//先端の半球部分の体積を返す
	~cPartsCapsule() {}		// デストラクタ
	dReal getl() { return l; }
	dReal getr() { return r; }
	
	void draw() {
		dsSetColor(color[0], color[1], color[2]);
		dsDrawCapsule(dBodyGetPosition(getBody()), dBodyGetRotation(getBody()), this->l, this->r);
	}


};
#include"fingerDef.h"
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
	static void nearCallback2(void* data, dGeomID o1, dGeomID o2);

};


class EntityODE : public ODE {

	std::shared_ptr<cFinger> pFinger;
	std::shared_ptr<cFinger> pFinger2;	//二本目の指　kawahara

	std::shared_ptr<cPartsCylinder> pObj;
	std::shared_ptr<cPartsCylinder> pObj2;

public:
	int FingerNum=0;
	double angle_rate = 8.0;//大きいほど初期角度が小さくなる
	double finger_state = -1.0;	//ひじ上げとひじ下げを切り替える
	double	init_jnt_pos[ARM_JNT] = { 4 * PI / 4.0 + PI / angle_rate*finger_state ,-PI / angle_rate* finger_state };	// ロボット初期姿勢
	double	init_jnt_posF2[ARM_JNT] = { 4 * PI / 4.0-  PI / angle_rate* finger_state, PI / angle_rate* finger_state };	// ロボット初期姿勢
	//double	init_jnt_pos[ARM_JNT] = { 4 * PI / 4.0 + PI / 7,-PI/7 };	// ロボット初期姿勢
	//double	init_jnt_posF2[ARM_JNT] = { 4 * PI / 4.0 - PI / 7, +PI/7};	// ロボット初期姿勢

	void setup() {
		constexpr auto OBJ_RADIUS = 0.10;

		//各関節の初期姿勢(角度)
		Vec3 obj_pos = { Vec3(-0.8 / sqrt(2.0) - 2 * 0.75 / sqrt(2.0), -0.8 / sqrt(2.0), OBJ_RADIUS) };
		
		//1本目の指
		this->pFinger = std::make_shared<cFinger>(init_jnt_pos,forceOutfilename1);
		this->pFinger->fingerID = ++FingerNum;

		//2本目の指
		this->pFinger2 = std::make_shared<cFinger>(init_jnt_posF2,forceOutfilename2);	
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
		//dSpaceCollide(this->getSpace(), 0, &this->nearCallback2);		// 衝突判定

		dWorldStep(this->getWorld(), SIM_CYCLE_TIME);	// 1ステップ進める
		dJointGroupEmpty(this->contactgroup); // ジョイントグループを空にする
	}
	auto getFinger() { return pFinger; }
	auto getFinger2() { return pFinger2; }


	auto getObj() { return pObj; }
	auto getObj2() { return pObj2; }
	void setAddForceObj(dReal fx, dReal fy, dReal fz) { dBodyAddForce(pObj->getBody(), fx, fy, fz); }
};




//センサの値を記録
void cFinger::outputForce() {
	forceOutOfs.open(forceOutFilename);
	for (int k = 0; k < DATA_CNT_NUM; k++) {
		forceOutOfs << k + 1 << ",";
		for (int i = 0; i < 2; i++) {
			forceOutOfs << saveForce[k][i] << ",";
		}
		forceOutOfs << std::endl;
	}
	forceOutOfs.close();

}

//センサの値を記録
void cFinger::outputJntAngle(int end) {
	//auto entity = EntityManager::get();
	
	std::ofstream outfile;
	if (fingerID == 1) {
		outfile.open(jntAngleOutfilename1);
	}
	else {
		outfile.open(jntAngleOutfilename2);
	}

	for (int k = 0; k < end; k++) {
		outfile << k << ",";
		//各関節について
		for (int i = 0; i < 2; i++) {
			outfile << save_jnt_vel[k][i] << ",";			//関節角度[rad]
			outfile << radToAng(save_jnt_vel[k][i]) << ",";	//関節角度[度]

			outfile << save_jnt_dq[k][i] << ",";			//関節速度[rad/s]
			outfile << radToAng(save_jnt_dq[k][i]) << ",";	//関節速度[度/s]

			outfile << saveForce[k][i] << ",";			//力覚センサの値 Fx,Fy[N]
			outfile << save_jnt_force[k][i] << ",";		//対象の関節に抱える力

			outfile << save_eff_pos[k][i] << ",";		//対象の手先位置

			outfile << save_eff_vel[k][i] << ",";		//対象の手先速度

			outfile << save_ref_eff_pos[k][i] << ",";	//対象の目標位置(平衡位置)
			//outfile << ",";
		}
		outfile << std::endl;
	}
	outfile.close();
}

void cFinger::setNums(int step) {
	for (int i = 0; i < 2; i++) {
		
		//	力覚センサの値
		saveForce[step - 1][i] = eff_force[i];
		//	関節角度
		save_jnt_vel[step - 1][i] = var.q.el[i][0];
		//	関節速度
		save_jnt_dq[step - 1][i] = var.dq.el[i][0];
		save_jnt_dq[step - 1][i] = var.dq.el[i][0];

#if useContactPoint
		//  手先位置を取得する
		const dReal* finger1TopPos = dBodyGetPosition(_this->forceContactPoint.getBody());
		const dReal* finger2TopPos = dBodyGetPosition(_this2->forceContactPoint.getBody());
		_this->save_eff_pos[sim->step - 1][i] = finger1TopPos[i];
		_this2->save_eff_pos[sim->step - 1][i] = finger2TopPos[i];
#else
		save_eff_pos[step - 1][i] = var.r.el[i][0];
		save_eff_vel[step - 1][i] = var.dr.el[i][0];
		save_ref_eff_pos[step - 1][i] = var_init.r.el[i][0];
#endif
		save_jnt_force[step - 1][i] = jnt_force[i];
		save_jnt_force[step - 1][i] = jnt_force[i];
	}
}

////////////////////////////////////////////////////////

// シミュレーション構造体

////////////////////////////////////////////////////////
class SIM: public EntityODE {

public:
	int	step;					//経過ステップ数
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