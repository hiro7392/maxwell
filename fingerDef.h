#pragma once

// �w
class cFinger {
	//�񎟌��ł̎������@	cPartsBox	base{ 14.0, Vec3(0.3, 0.5, 0.4)};
	cPartsBox	base{ ARM_BASE_MASS, Vec3(0.22, 0.22, 0.4) };
	cPartsCylinder	link1{ ARM_LINK1_MASS, ARM_LINK1_LEN, ARM_LINK1_RAD };
	cPartsCylinder	link2{ ARM_LINK2_MASS, ARM_LINK2_LEN, ARM_LINK2_RAD };

	dReal px1 = -20, py1 = -20, pz1 = 0;
	double Z_OFFSET = 0.0;// 0.08

public:
	std::vector<cParts*> finger;
	dReal x0 = 0.5, y0 = 0.0, z0 = 1.5;			//	������������1�{�ڂ̎w�̓y��̈ʒu�@kawahara
	dReal x1 = 0.5, y1 = -1.2, z1 = 1.5;		//	������������2�{�ڂ̎w�̓y��̈ʒu�@kawahara
	double z_base_pos = 1.5;
	double z_senkai_base_pos = 0.5;

	//����֐߂̓y�䕔��
	dReal senkai_base_x0 = 0.5, senkai_base_y0 = -0.3, senkai_base_z0 = z_base_pos;		//	����֐߂̓y��
	dReal senkai_base_x1 = 0.5, senkai_base_y1 = -0.9, senkai_base_z1 = z_base_pos;		//	����֐߂̓y��
	double senkai_init_jnt = PI / 2.0;	///�����ɂȂ�ʒu��0.0
	double senkai_init_jnt2 = -PI / 2.0;	//�w2�͔��΂̎p�����Ƃ�

	//�w��̃J�v�Z��
	double fingerTopCapsuleLen = ARM_LINK2_LEN / 4.0;
	cPartsCapsule	fingerTopCapsule{ ARM_LINK2_MASS, fingerTopCapsuleLen, ARM_LINK2_RAD };
	double forceContactPointThickness = 0.001;	//�͂̍�p�_	�ɔ��̉~��
#if useForceContactPoint
	cPartsCylinder forceContactPoint{ 0.0001 / ARM_LINK2_LEN * ARM_LINK2_MASS, forceContactPointThickness, ARM_LINK2_RAD / 10 };
#endif
	
	cPartsCylinder	sensor{ SENSOR_LEN / ARM_LINK2_LEN * ARM_LINK2_MASS, SENSOR_LEN, ARM_LINK2_RAD };	// �A�[���Ɩ��x�����낦��

	// ����֐߂̓y�䕔��
	cPartsBox	senkai_base{ 1.0, Vec3(0.22, 0.22, 0.22) };
	cPartsCylinder	senkai_link{ ARM_LINK2_MASS / 2.0, SENKAI_LINK_LEN, ARM_LINK2_RAD / 2.0 };

	//�c������v���[�g{����,�����ʒu(x,y,z),�傫��(x,y,z)}
	cPartsBox	plate{ PLATE_MASS, Vec3(px1,py1,pz1),Vec3(PLATE_X_LEN,PLATE_Y_LEN,PLATE_Z_LEN) };

	dJointFeedback force, * p_force, senkai_force, * senkai_p_force;		//�̓Z���T�p
	dJointFeedback fingerTop2ForcePoint_joint;
	dJointFeedback  r_joint_feedback[ARM_JNT];
	dJointFeedback *r_joint_feedback_p[ARM_JNT];

	dJointID senkai_link_joint, senkai_base_joint, senkai_link2finger_joint;						//  ����֐�->�w�y��->�w���{�̌����_
	dJointID f_joint, r_joint[ARM_JNT], f2_joint;	//	�Œ�֐߂Ɖ�]�֐�
	dJointID sensor2FingerTop;						//	�Z���T�̐�[�p�@��[�̃J�v�Z���ƃZ���T�̌����_
	dJointID FingerTop2ForcePoint;					//	�w��J�v�Z���Ɨ͂̍�p�_���Ȃ��֐�
	dJointID graspObj; 								//	�c���Ώۂ̃v���[�g kawahara

	// �w�̐���p�ϐ�
	int fingerID;
	int state_contact;			// �ڐG���(0:OFF, 1:ON)


	std::string forceOutFilename;	//�͊o�Z���T��csv�o�͗p
	std::ofstream forceOutOfs;		//�o�͗pofstream

	double	dist;				// �A�[���ƑΏۂ̋���
	double	jnt_pos[ARM_JNT] = {};
	double	jnt_vel[ARM_JNT] = {};
	double	jnt_force[ARM_JNT] = {};
	double	past_jnt_pos[ARM_JNT] = {};
	double	eff_pos[DIM3] = {};
	double	eff_vel[DIM3] = {};
	double	eff_force[DIM3] = {};
	double	obj_pos[DIM3] = {};
	double	obj_vel[DIM3] = {};
	// �ڕW�ϐ�
	double	ref_jnt_pos[ARM_JNT] = {};
	double	ref_jnt_vel[ARM_JNT] = {};
	double	ref_eff_pos[DIM3] = {};
	double	ref_eff_vel[DIM3] = {};
	// �����ϐ�
	double	init_jnt_pos[ARM_JNT] = {};
	double	init_obj_pos[DIM3] = {};

	double	init_obj_att[DIM3][DIM3] = {};	// ��΍��W�ɂ�����Ώۍ��W���̎p���i���͐��K�������j
	// �ϐ��\����
	Variable	var;			// ���ݒl
	Variable	var_prev;		// �ߋ��l�i1�T�C�N���O�j
	Variable	var_prev2;		// �ߋ��l�i2�T�C�N���O�j

	Variable	var_init;		// �����l
	// �^���w�ϐ�
	Kinematics	kine;
	// ���͊w�ϐ�
	Dynamics	dyn;
	// �C���s�[�_���X�ϐ�
	Impedance	imp;
	// ��]�^���p�̕ϐ�
	RotImpedance rotImp;

	// �ۑ��p�f�[�^�ϐ�
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

	// ����֐ߊp
	double senkai_base_jnt = 0;
	double senkai_base_vel = 0.0;
	double senkai_base_ddq = 0.0;
	double senkai_base_jnt_init = 0;
	// �ۑ��p�t�@�C�����ϐ�
	char	data_file_name[DATA_FILE_NAME_MAXLEN] = {};
	char	filename_info[DATA_FILE_NAME_MAXLEN] = {};
	char	filename_graph[DATA_FILE_NAME_MAXLEN] = {};

	// �����o�֐�
	void initJntPos(double* init_jnt_pos) {}
	int armWithoutInertiaShaping();
	int ctrlPreProcessing();
	int armDynPara();

	int armInvKine(Kinematics* kine, Variable* var);
	int armJacob(Kinematics* kine, Variable* var);
	int armInitMat(Variable* var, Kinematics* kine, Dynamics* dyn, Impedance* imp);

	//	�쌴���쐬	3�����̃V�~�����[�V�����p
	int setTransMatrix();
	int setMassCenterPosition(Matrix& mat, double mx, double my, double mz);
	int senkaiDynPara();			//	����֐߂̃p�����[�^
	int calculateGravity();			//	�d�͍����v�Z
	int setTransMatrixDq();			//	�����s��̊֐ߊp�ɂ��������v�Z
	//kawahara�̕ύX�ȑO����R�����g�A�E�g
	////	int armInitMatVar(Variable *var);
	////	int armInitMatKine(Kinematics *kine);

	int ctrlInitErr();
	int armCalcImpPeriod();
	void saveData();
	void saveInfo();
	void saveGraph();


	//debug�p�@kawahara���ǉ�
	void setNums(int step);		//�o�͗p�z��Ɋe��Z���T�l���i�[
	void printInfo();

	////Finger class�̒��Ɉڙ�E
	//int ctrlMaxwell(Matrix* tau);

	//�R���X�g���N�^
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
	~cFinger() {		// �W���C���g�j��
		dJointDestroy(f_joint);				// �y��Œ�
		dJointDestroy(r_joint[ARM_M1]);		// �A�[��
		dJointDestroy(r_joint[ARM_M2]);		// �A�[��
		dJointDestroy(f2_joint);   // �Z���T�Œ�
		dJointDestroy(sensor2FingerTop);
		dJointDestroy(senkai_base_joint);
		dJointDestroy(senkai_link_joint);
		dJointDestroy(senkai_link2finger_joint);
	}
	auto getParts() { return finger; }
	//	void setPosition(const dVector3 pos) {

	// �G���h�G�t�F�N�^�̈ʒu���v�Z
	void setEffPos() {
		//	�w���[�̃J�v�Z���̍��W���̗p����
		const dReal* nowPos = dBodyGetPosition(fingerTopCapsule.getBody());
		//	x���W
		eff_pos[0] = nowPos[0];
		eff_pos[1] = nowPos[1];
		eff_pos[2] = nowPos[2];

		return;
	}
	//�w�P�̏����ʒu�Ə����p��	�w1�Ǝw2�ŋ���
	void setPosition(double senkai_base_x0, double senkai_base_y0, double senkai_base_z0) {
		//	�����N1�̓y��
		double base_x = senkai_base_x0;
		double base_y = senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN * cos(senkai_base_jnt);
		double base_z = senkai_base_z0 - SENKAI_LINK_LEN * sin(senkai_base_jnt);
		finger[0]->setPosition(base_x, base_y, base_z);
		
		//	�����N�P
		double link1_x = base_x + (ARM_LINK1_LEN / 2.0 * cos(jnt_pos[ARM_M1]));
		double link1_y = base_y +  cos(senkai_base_jnt) * (ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1]));
		double link1_z = base_z -  sin(senkai_base_jnt) * (ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1]));
		finger[1]->setPosition(link1_x, link1_y, link1_z);

		double link1_top_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]);
		double link1_top_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));
		double link1_top_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));
		
		//	�����N2
		double link2_x = link1_top_x + (ARM_LINK2_LEN / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		//double link2_y = base_y +abs(cos(senkai_base_jnt)) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double link2_y = link1_top_y + cos(senkai_base_jnt) * (ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double link2_z = base_z - sin(senkai_base_jnt) *(ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		//printf("setposition fingerID =%d link1_angle=%.2lf link2 angle=%.2lf\n", fingerID, radToAng(jnt_pos[0]), radToAng(jnt_pos[1]));

		finger[2]->setPosition(link2_x, link2_y, link2_z);

		//	�Z���T
		double sensor_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + SENSOR_LEN / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double sensor_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + SENSOR_LEN / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double sensor_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1])+(ARM_LINK2_LEN + SENSOR_LEN / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		finger[3]->setPosition(sensor_x, sensor_y, sensor_z);
		
		//�w��̃J�v�Z���@�Z���T�Ɠ����ʒu
		/*double capsule_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN +0.1) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double capsule_y = base_y + abs(cos(senkai_base_jnt)) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double capsule_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1])) + sin(senkai_base_jnt) * ((ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));*/
		double capsule_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double capsule_y = base_y + abs(cos(senkai_base_jnt)) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double capsule_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + ((ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2])));
		fingerTopCapsule.setPosition(capsule_x, capsule_y, capsule_z);
		//�@����֐�
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
			y0 + (ARM_LINK1_LEN)*sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.3) / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), 0.4 / 2.0 - Z_OFFSET),//�w��̃J�v�Z��
#else
		//�w��̃J�v�Z���@�Z���T�Ɠ����ʒu
		//fingerTopCapsule.setPosition(base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), base_y + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), base_z - Z_OFFSET);
#if forceContactPoint
		//�͂̍�p�_���w��[�ɌŒ�
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

		//����֐�
		senkai_base.setRotation(0);
		senkai_link.setRotationSenkai(PI/2.0,fingerID == 1 ?senkai_base_jnt : -senkai_base_jnt);

		
#if forceContactPoint
		forceContactPoint.setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
#endif
	}

	//kawahara���ǉ��@��{�ڂ̎w�p
	//�w�P�̏����ʒu�Ə����p��	�w1�Ǝw2�ŋ���
	void setPosition2(double senkai_base_x0, double senkai_base_y0, double senkai_base_z0) {
		//	�����N1�̓y��
		double base_x = senkai_base_x0;
		double base_y = senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN * cos(senkai_base_jnt);
		double base_z = senkai_base_z0 + SENKAI_LINK_LEN * sin(-senkai_base_jnt);
		finger[0]->setPosition(base_x, base_y, base_z);

		//	�����N�P
		double link1_x = base_x + (ARM_LINK1_LEN / 2.0 * cos(jnt_pos[ARM_M1]));
		double link1_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1]));
		double link1_z = base_z + sin(senkai_base_jnt) * ((ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1])));
		finger[1]->setPosition(link1_x, link1_y, link1_z);

		double link1_top_x = base_x + (ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]));
		double link1_top_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));
		double link1_top_z = base_z + sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));

		//	�����N2
		double link2_x = link1_top_x + (ARM_LINK2_LEN / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double link2_y = link1_top_y + cos(senkai_base_jnt) * (ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double link2_z = link1_top_z + sin(senkai_base_jnt) * (ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));

		finger[2]->setPosition(link2_x, link2_y, link2_z);
		
		//	�Z���T
		double sensor_x = link1_top_x + (ARM_LINK2_LEN + SENSOR_LEN / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double sensor_y = link1_top_y + cos(senkai_base_jnt) * ((ARM_LINK2_LEN + SENSOR_LEN / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double sensor_z = link1_top_z + sin(senkai_base_jnt) * ((ARM_LINK2_LEN + SENSOR_LEN / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		finger[3]->setPosition(sensor_x, sensor_y, sensor_z);

		//�w��̃J�v�Z���@�Z���T�Ɠ����ʒu
		double capsule_x = link1_top_x + (ARM_LINK2_LEN + 0.1) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
		double capsule_y = link1_top_y + (cos(senkai_base_jnt)) * ((ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		double capsule_z = link1_top_z + sin(senkai_base_jnt) * ((ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
		fingerTopCapsule.setPosition(capsule_x, capsule_y, capsule_z);
		//�@����֐�
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
			y0 + (ARM_LINK1_LEN)*sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.3) / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), 0.4 / 2.0 - Z_OFFSET),//�w��̃J�v�Z��
#else
		//�w��̃J�v�Z���@�Z���T�Ɠ����ʒu
		//fingerTopCapsule.setPosition(base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), base_y + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + 0.1) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]), base_z - Z_OFFSET);
#if forceContactPoint
		//�͂̍�p�_���w��[�ɌŒ�
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

		//����֐�
		senkai_base.setRotation(0);
		senkai_link.setRotationSenkai(PI / 2.0, fingerID == 1 ? senkai_base_jnt : -senkai_base_jnt);


#if forceContactPoint
		forceContactPoint.setRotation(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
#endif
	}
	void setColor(std::vector<Vec3> color) {
		auto i = color.begin();
		int cnt = 0;
		//fingerTopCapsule->setColor((*i).x, (*i).y, (*i).z);	//�w��̃J�v�Z���p
		if (fingerID == 1)senkai_base.setColor((*i).x, (*i).y, (*i).z);
		else {
			senkai_base.setColor((*(i + 1)).x, (*(i + 1)).y, (*(i + 1)).z);
		}
		//�@�w2�ɂ��Ă͓����F
		senkai_link.setColor((*(i + 1)).x, (*(i + 1)).y, (*(i + 1)).z);
		for (auto j = finger.begin(); j != finger.end(); ++j, ++i) {

			if (cnt < 3)(*j)->setColor((*i).x, (*i).y, (*i).z);
			else {
				(*j)->setColor(0, 0, 0);
			}
			cnt++;
		}
	}

	void setJoint();		// �֐ߐݒ�
	void setJoint2();		// �֐ߐݒ� 2�{�ڂ̎w
	void setJntFric();		// ���C�ݒ�
	void addExtForce();		// �O��

	void addExtForce2();	// �O��

	void outputForce();		//�f�o�b�O�p	�O�͂�csv�o�͂���
	void outputJntAngle(int end);	//�֐ߊp�����o�͂���
	//kawahara���ǉ�
	int calcDist();
	int ctrlMaxwell(Matrix* tau);
	int ctrlMaxwell2(Matrix* tau);	//kawahara���ǋL�@��{�ڂ̎w�p

	//���������Maxwell���f��
	int RestrictedCtrlMaxwell(Matrix* tau);
	int RestrictedCtrlMaxwell2(Matrix* tau);	//kawahara���ǋL�@��{�ڂ̎w�p

	int moveEqPointCtrlMaxwell(Matrix* tau);	//maxwell����+�c���̕��t�_���ړ�����
	int moveEqPointCtrlMaxwell2(Matrix* tau);

	int RotRestrictedCtrlMaxwell(double* tau);

	void control();		// ����
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
