#pragma once

// 指
class cFinger {
	//二次元での実験時　	cPartsBox	base{ 14.0, Vec3(0.3, 0.5, 0.4)};
	cPartsBox	base{ ARM_BASE_MASS, Vec3(0.22, 0.22, 0.4) };
	cPartsCylinder	link1{ ARM_LINK1_MASS, ARM_LINK1_LEN, ARM_LINK1_RAD };
	cPartsCylinder	link2{ ARM_LINK2_MASS, ARM_LINK2_LEN, ARM_LINK2_RAD };

	dReal px1 = -20, py1 = -20, pz1 = 0;
	double Z_OFFSET = 0.0;// 0.08

public:
	std::vector<cParts*> finger;
	dReal x0 = 0.5, y0 = 0.0, z0 = 1.5;			//	書き換えた後1本目の指の土台の位置　kawahara
	dReal x1 = 0.5, y1 = -1.2, z1 = 1.5;		//	書き換えた後2本目の指の土台の位置　kawahara
	double z_base_pos = 1.5;
	double z_senkai_base_pos = 0.5;

	//旋回関節の土台部分
	dReal senkai_base_x0 = 0.5, senkai_base_y0 = -0.3, senkai_base_z0 = z_base_pos;		//	旋回関節の土台
	dReal senkai_base_x1 = 0.5, senkai_base_y1 = -0.9, senkai_base_z1 = z_base_pos;		//	旋回関節の土台
	double senkai_init_jnt = PI / 2.0;	///水平になる位置は0.0
	double senkai_init_jnt2 = -PI / 2.0;	//指2は反対の姿勢をとる

	//指先のカプセル
	double fingerTopCapsuleLen = ARM_LINK2_LEN / 4.0;
	cPartsCapsule	fingerTopCapsule{ ARM_LINK2_MASS, fingerTopCapsuleLen, ARM_LINK2_RAD };
	double forceContactPointThickness = 0.001;	//力の作用点	極薄の円柱
#if useForceContactPoint
	cPartsCylinder forceContactPoint{ 0.0001 / ARM_LINK2_LEN * ARM_LINK2_MASS, forceContactPointThickness, ARM_LINK2_RAD / 10 };
#endif
	
	cPartsCylinder	sensor{ SENSOR_LEN / ARM_LINK2_LEN * ARM_LINK2_MASS, SENSOR_LEN, ARM_LINK2_RAD };	// アームと密度をそろえる

	// 旋回関節の土台部分
	cPartsBox	senkai_base{ 1.0, Vec3(0.22, 0.22, 0.22) };
	cPartsCylinder	senkai_link{ ARM_LINK2_MASS / 2.0, SENKAI_LINK_LEN, ARM_LINK2_RAD / 2.0 };

	//把持するプレート{質量,初期位置(x,y,z),大きさ(x,y,z)}
	cPartsBox	plate{ PLATE_MASS, Vec3(px1,py1,pz1),Vec3(PLATE_X_LEN,PLATE_Y_LEN,PLATE_Z_LEN) };

	dJointFeedback force, * p_force, senkai_force, * senkai_p_force;		//力センサ用
	dJointFeedback fingerTop2ForcePoint_joint;
	dJointFeedback  r_joint_feedback[ARM_JNT];
	dJointFeedback *r_joint_feedback_p[ARM_JNT];

	dJointID senkai_link_joint, senkai_base_joint, senkai_link2finger_joint;						//  旋回関節->指土台->指根本の結合点
	dJointID f_joint, r_joint[ARM_JNT], f2_joint;	//	固定関節と回転関節
	dJointID sensor2FingerTop;						//	センサの先端用　先端のカプセルとセンサの結合点
	dJointID FingerTop2ForcePoint;					//	指先カプセルと力の作用点をつなぐ関節
	dJointID graspObj; 								//	把持対象のプレート kawahara

	// 指の制御用変数
	int fingerID;
	int state_contact;			// 接触状態(0:OFF, 1:ON)


	std::string forceOutFilename;	//力覚センサのcsv出力用
	std::ofstream forceOutOfs;		//出力用ofstream

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
	// 回転運動用の変数
	RotImpedance rotImp;

	// 保存用データ変数
	int save_state_contact[DATA_CNT_NUM] = {};
	double	save_dist[DATA_CNT_NUM] = {};
	double	save_ref_jnt_pos[DATA_CNT_NUM][ARM_JNT] = {};
	double	save_ref_jnt_vel[DATA_CNT_NUM][ARM_JNT] = {};
	double	save_jnt_pos[DATA_CNT_NUM][ARM_JNT] = {};
	double	save_jnt_vel[DATA_CNT_NUM][ARM_JNT] = {};
	double	save_jnt_dq[DATA_CNT_NUM][ARM_JNT] = {};

	double	save_jnt_force[DATA_CNT_NUM][ARM_JNT] = {};
	double	save_ref_eff_pos[DATA_CNT_NUM][DIM3] = {};
	double	save_ref_eff_vel[DATA_CNT_NUM][DIM3] = {};
	double	save_eff_pos[DATA_CNT_NUM][DIM3] = {};
	double	save_eff_vel[DATA_CNT_NUM][DIM3] = {};
	double	save_eff_force[DATA_CNT_NUM][DIM3] = {};
	double	save_obj_pos[DATA_CNT_NUM][DIM3] = {};
	double	save_obj_vel[DATA_CNT_NUM][DIM3] = {};
	double  saveForce[DATA_CNT_NUM][ARM_JNT] = {};

	// 旋回関節角
	double senkai_base_jnt = 0;
	double senkai_base_vel = 0.0;
	double senkai_base_ddq = 0.0;
	double senkai_base_jnt_init = 0;
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

	//	川原が作成	3次元のシミュレーション用
	int setTransMatrix();
	int setMassCenterPosition(Matrix& mat, double mx, double my, double mz);
	int senkaiDynPara();			//	旋回関節のパラメータ
	int calculateGravity();			//	重力項を計算
	int setTransMatrixDq();			//	同次行列の関節角による微分を計算
	//kawaharaの変更以前からコメントアウト
	////	int armInitMatVar(Variable *var);
	////	int armInitMatKine(Kinematics *kine);

	int ctrlInitErr();
	int armCalcImpPeriod();
	void saveData();
	void saveInfo();
	void saveGraph();


	//debug用　kawaharaが追加
	void setNums(int step);		//出力用配列に各種センサ値を格納
	void printInfo();

	////Finger classの中に移勁E
	//int ctrlMaxwell(Matrix* tau);

	//コンストラクタ
	cFinger(double* init_jnt_pos, std::string forceFilename)
		: jnt_pos{ init_jnt_pos[0], init_jnt_pos[1] },
		finger{ &base, &link1, &link2, &sensor } {
		this->kine = Kinematics();
		this->dyn = Dynamics();
		this->imp = Impedance();
		this->var = Variable();
		this->var_prev = Variable();
		this->var_prev2 = Variable();
		this->var_init = Variable();
		this->forceOutFilename = forceFilename;
	}
	~cFinger() {		// ジョイント破壊
		dJointDestroy(f_joint);				// 土台固定
		dJointDestroy(r_joint[ARM_M1]);		// アーム
		dJointDestroy(r_joint[ARM_M2]);		// アーム
		dJointDestroy(f2_joint);   // センサ固定
		dJointDestroy(sensor2FingerTop);
		dJointDestroy(senkai_base_joint);
		dJointDestroy(senkai_link_joint);
		dJointDestroy(senkai_link2finger_joint);
	}
	auto getParts() { return finger; }
	//	void setPosition(const dVector3 pos) {

	// エンドエフェクタの位置を計算
	void setEffPos() {
		//	指先先端のカプセルの座標を採用する
		const dReal* nowPos = dBodyGetPosition(fingerTopCapsule.getBody());
		//	x座標
		eff_pos[0] = nowPos[0];
		eff_pos[1] = nowPos[1];
		eff_pos[2] = nowPos[2];

		return;
	}
	//指１の初期位置と初期姿勢	指1と指2で共通
	void setPosition(double senkai_base_x0, double senkai_base_y0, double senkai_base_z0) {
		//	リンク1の土台
		double base_x = senkai_base_x0;
		double base_y = senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN * cos(senkai_base_jnt);
		double base_z = senkai_base_z0 - SENKAI_LINK_LEN * sin(senkai_base_jnt);
		finger[0]->setPosition(base_x, base_y, base_z);
		
		//	リンク１
		double link1_x = base_x + (ARM_LINK1_LEN / 2.0 * cos(jnt_pos[ARM_M1]));
		double link1_y = base_y +  cos(senkai_base_jnt) * (ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1]));
		double link1_z = base_z -  sin(senkai_base_jnt) * (ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1]));
		finger[1]->setPosition(link1_x, link1_y, link1_z);

		double link1_top_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]);
		double link1_top_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));
		double link1_top_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));
		
		//	リンク2
		double link2_x = link1_top_x + (ARM_LINK2_LEN / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		//double link2_y = base_y +abs(cos(senkai_base_jnt)) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double link2_y = link1_top_y + cos(senkai_base_jnt) * (ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double link2_z = base_z - sin(senkai_base_jnt) *(ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		//printf("setposition fingerID =%d link1_angle=%.2lf link2 angle=%.2lf\n", fingerID, radToAng(jnt_pos[0]), radToAng(jnt_pos[1]));

		finger[2]->setPosition(link2_x, link2_y, link2_z);

		//	センサ
		double sensor_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + SENSOR_LEN / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double sensor_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + SENSOR_LEN / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double sensor_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1])+(ARM_LINK2_LEN + SENSOR_LEN / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		finger[3]->setPosition(sensor_x, sensor_y, sensor_z);
		
		//指先のカプセル　センサと同じ位置
		/*double capsule_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN +0.1) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double capsule_y = base_y + abs(cos(senkai_base_jnt)) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double capsule_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1])) + sin(senkai_base_jnt) * ((ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));*/
		double capsule_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double capsule_y = base_y + abs(cos(senkai_base_jnt)) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double capsule_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + ((ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2])));
		fingerTopCapsule.setPosition(capsule_x, capsule_y, capsule_z);
		//　旋回関節
		senkai_base.setPosition(senkai_base_x0, senkai_base_y0, senkai_base_z0);	// z:base->sides[CRD_Z]/2
		if (fingerID == 1) {
			senkai_link.setPosition(senkai_base_x0, 
				senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN/2.0 * cos(senkai_base_jnt),
				senkai_base_z0 - SENKAI_LINK_LEN/2.0 * sin(senkai_base_jnt));
		}
		else {
			senkai_link.setPosition(senkai_base_x0,
				senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN / 2.0 * cos(senkai_base_jnt),
				senkai_base_z0 - SENKAI_LINK_LEN / 2.0 * sin(senkai_base_jnt));
		}
#if 0
		fingerTopCapsule.setPosition(x0 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.3) / 2.0 * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]),
			y0 + (ARM_LINK1_LEN)*sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.3) / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), 0.4 / 2.0 - Z_OFFSET),//指先のカプセル
#else
		//指先のカプセル　センサと同じ位置
		//fingerTopCapsule.setPosition(base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), base_y + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), base_z - Z_OFFSET);
#if forceContactPoint
		//力の作用点を指先端に固定
		forceContactPoint.setPosition(x0 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1 + fingerTopCapsuleLen / 2.0 + ARM_LINK2_RAD + forceContactPointThickness / 2) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), y0 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1 + fingerTopCapsuleLen / 2.0 + ARM_LINK2_RAD + forceContactPointThickness / 2) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), 0.4 / 2.0 - Z_OFFSET);
#endif

#endif
		finger[0]->setRotation(0);
		//finger[1]->setRotation(jnt_pos[ARM_M1]);
		finger[1]->setRotationFrom2Points(base_x, base_y, base_z,link1_x,link1_y,link1_z);
		finger[2]->setRotationFrom2Points(link1_top_x, link1_top_y, link1_top_z, link2_x, link2_y, link2_z);
		finger[3]->setRotationFrom2Points(link1_top_x, link1_top_y, link1_top_z, link2_x, link2_y, link2_z);
		fingerTopCapsule.setRotationFrom2Points(link1_top_x, link1_top_y, link1_top_z, link2_x, link2_y, link2_z);
		//finger[1]->setRotationSenkai2(jnt_pos[ARM_M1]-PI/2.0, fingerID == 1 ? senkai_base_jnt : -senkai_base_jnt);
		//finger[2]->setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		//finger[3]->setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);

		//旋回関節
		senkai_base.setRotation(0);
		senkai_link.setRotationSenkai(PI/2.0,fingerID == 1 ?senkai_base_jnt : -senkai_base_jnt);

		
#if forceContactPoint
		forceContactPoint.setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
#endif
	}

	//kawaharaが追加　二本目の指用
	//指１の初期位置と初期姿勢	指1と指2で共通
	void setPosition2(double senkai_base_x0, double senkai_base_y0, double senkai_base_z0) {
		//	リンク1の土台
		double base_x = senkai_base_x0;
		double base_y = senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN * cos(senkai_base_jnt);
		double base_z = senkai_base_z0 + SENKAI_LINK_LEN * sin(-senkai_base_jnt);
		finger[0]->setPosition(base_x, base_y, base_z);

		//	リンク１
		double link1_x = base_x + (ARM_LINK1_LEN / 2.0 * cos(jnt_pos[ARM_M1]));
		double link1_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1]));
		double link1_z = base_z + sin(senkai_base_jnt) * ((ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1])));
		finger[1]->setPosition(link1_x, link1_y, link1_z);

		double link1_top_x = base_x + (ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]));
		double link1_top_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));
		double link1_top_z = base_z + sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));

		//	リンク2
		double link2_x = link1_top_x + (ARM_LINK2_LEN / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double link2_y = link1_top_y + cos(senkai_base_jnt) * (ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double link2_z = link1_top_z + sin(senkai_base_jnt) * (ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));

		finger[2]->setPosition(link2_x, link2_y, link2_z);
		
		//	センサ
		double sensor_x = link1_top_x + (ARM_LINK2_LEN + SENSOR_LEN / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double sensor_y = link1_top_y + cos(senkai_base_jnt) * ((ARM_LINK2_LEN + SENSOR_LEN / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double sensor_z = link1_top_z + sin(senkai_base_jnt) * ((ARM_LINK2_LEN + SENSOR_LEN / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		finger[3]->setPosition(sensor_x, sensor_y, sensor_z);

		//指先のカプセル　センサと同じ位置
		double capsule_x = link1_top_x + (ARM_LINK2_LEN + 0.1) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double capsule_y = link1_top_y + (cos(senkai_base_jnt)) * ((ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double capsule_z = link1_top_z + sin(senkai_base_jnt) * ((ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		fingerTopCapsule.setPosition(capsule_x, capsule_y, capsule_z);
		//　旋回関節
		senkai_base.setPosition(senkai_base_x0, senkai_base_y0, senkai_base_z0);	// z:base->sides[CRD_Z]/2
		if (fingerID == 1) {
			senkai_link.setPosition(senkai_base_x0,
				senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN / 2.0 * cos(senkai_base_jnt),
				senkai_base_z0 - SENKAI_LINK_LEN / 2.0 * sin(senkai_base_jnt));
		}
		else {
			senkai_link.setPosition(senkai_base_x0,
				senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN / 2.0 * cos(senkai_base_jnt),
				senkai_base_z0 - SENKAI_LINK_LEN / 2.0 * sin(senkai_base_jnt));
		}
#if 0
		fingerTopCapsule.setPosition(x0 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.3) / 2.0 * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]),
			y0 + (ARM_LINK1_LEN)*sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.3) / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), 0.4 / 2.0 - Z_OFFSET),//指先のカプセル
#else
		//指先のカプセル　センサと同じ位置
		//fingerTopCapsule.setPosition(base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), base_y + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), base_z - Z_OFFSET);
#if forceContactPoint
		//力の作用点を指先端に固定
		forceContactPoint.setPosition(x0 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1 + fingerTopCapsuleLen / 2.0 + ARM_LINK2_RAD + forceContactPointThickness / 2) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), y0 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1 + fingerTopCapsuleLen / 2.0 + ARM_LINK2_RAD + forceContactPointThickness / 2) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), 0.4 / 2.0 - Z_OFFSET);
#endif

#endif
		finger[0]->setRotation(0);
		//finger[1]->setRotation(jnt_pos[ARM_M1]);
		finger[1]->setRotationFrom2Points(base_x, base_y, base_z, link1_x, link1_y, link1_z);
		finger[2]->setRotationFrom2Points(link1_top_x, link1_top_y, link1_top_z, link2_x, link2_y, link2_z);
		finger[3]->setRotationFrom2Points(link1_top_x, link1_top_y, link1_top_z, link2_x, link2_y, link2_z);
		fingerTopCapsule.setRotationFrom2Points(link1_top_x, link1_top_y, link1_top_z, link2_x, link2_y, link2_z);
		//finger[1]->setRotationSenkai2(jnt_pos[ARM_M1]-PI/2.0, fingerID == 1 ? senkai_base_jnt : -senkai_base_jnt);
		//finger[2]->setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		//finger[3]->setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);

		//旋回関節
		senkai_base.setRotation(0);
		senkai_link.setRotationSenkai(PI / 2.0, fingerID == 1 ? senkai_base_jnt : -senkai_base_jnt);


#if forceContactPoint
		forceContactPoint.setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
#endif
	}
	void setColor(std::vector<Vec3> color) {
		auto i = color.begin();
		int cnt = 0;
		//fingerTopCapsule->setColor((*i).x, (*i).y, (*i).z);	//指先のカプセル用
		if (fingerID == 1)senkai_base.setColor((*i).x, (*i).y, (*i).z);
		else {
			senkai_base.setColor((*(i + 1)).x, (*(i + 1)).y, (*(i + 1)).z);
		}
		//　指2については同じ色
		senkai_link.setColor((*(i + 1)).x, (*(i + 1)).y, (*(i + 1)).z);
		for (auto j = finger.begin(); j != finger.end(); ++j, ++i) {

			if (cnt < 3)(*j)->setColor((*i).x, (*i).y, (*i).z);
			else {
				(*j)->setColor(0, 0, 0);
			}
			cnt++;
		}
	}

	void setJoint();		// 関節設定
	void setJoint2();		// 関節設定 2本目の指
	void setJntFric();		// 摩擦設定
	void addExtForce();		// 外力

	void addExtForce2();	// 外力

	void outputForce();		//デバッグ用	外力をcsv出力する
	void outputJntAngle(int end);	//関節角情報を出力する
	//kawaharaが追加
	int calcDist();
	int ctrlMaxwell(Matrix* tau);
	int ctrlMaxwell2(Matrix* tau);	//kawaharaが追記　二本目の指用

	//制約条件つきMaxwellモデル
	int RestrictedCtrlMaxwell(Matrix* tau);
	int RestrictedCtrlMaxwell2(Matrix* tau);	//kawaharaが追記　二本目の指用

	int moveEqPointCtrlMaxwell(Matrix* tau);	//maxwell制御+把持の平衡点を移動する
	int moveEqPointCtrlMaxwell2(Matrix* tau);

	int RotRestrictedCtrlMaxwell(double* tau);

	void control();		// 制御
	void destroy() { for (auto& x : finger) { x->destroy(); } }
	void draw() {
		fingerTopCapsule.draw();
		senkai_base.draw();
		senkai_link.draw();
#if useForceContactPoint 
		forceContactPoint.draw();
#endif
		for (auto& x : finger) {
			x->draw();
		}
		//for (int i = 1; i < finger.size(); i++)finger[i]->draw();
	}
};
