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
// �s��v�ZEigen(�}�N����`�̓C���N���[�h���O�ɕK�v)
#define EIGEN_NO_DEBUG // �R�[�h����assert�𖳌����D
#define EIGEN_DONT_VECTORIZE // SIMD�𖳌����D
#define EIGEN_DONT_PARALLELIZE // ����𖳌����D
#define EIGEN_MPL2_ONLY // LGPL���C�Z���X�̃R�[�h���g��Ȃ��D
//#include "eigen-3.3.7/Eigen/Core"
//#include "eigen-3.3.7/Eigen/LU"		// �t�s���s�񎮂̌v�Z�ɕK�v
//#include "c:/software/eigen-3.3.7/Eigen/Dense"		// ��2�̑���ɂ���1�ł�OK
#include "c:/eigen-3.4.0/Eigen/Dense"
////////////////////////////////////////////////////////
// �o�C�i���t���O
// ON(1)��OFF(0)�̂ݐݒ�\
////////////////////////////////////////////////////////
#define	GLAPHIC_OPENGL		0		// OpenGL�ŕ`��
#define	FLAG_DRAW_SIM		1		// ODE�̕W���`��
#define	FLAG_SAVE_IMAGE		0		// �摜�ۑ�
#define	FLAG_SAVE_VIDEO		0		// ����ۑ�(OpenCV���K�v)

////////////////////////////////////////////////////////
// define��`
////////////////////////////////////////////////////////
#ifndef PI
#define PI (3.14159265358979323846)
#endif
// ��ʕ\����`
#define	DISPLAY_WIDTH	640
#define	DISPLAY_HEIGHT	480
// �����E���W���ʒ�`
#define DIM2	2
#define	DIM3	3	// 3�����̈ʒu��p��
#define	CRD_X	0
#define	CRD_Y	1
#define	CRD_Z	2
#define	AXIS_X	0
#define	AXIS_Y	1
#define	AXIS_Z	2
#define DIR_LONG_AXIS_Z	3	// ��������(dMassSetCylinderTotal�Ȃǂɗ��p)
// �萔��`
#define SYSTEM_CYCLE_TIME	(0.001)	// �����p�T�C�N���^�C��
#define SIM_CYCLE_TIME	(0.001)	// �V�~�����[�V�����p�T�C�N���^�C��
#define DATA_CNT_NUM	5000	// �f�[�^�ۑ��J�E���g��
#define SAVE_IMG_RATE	200		// �摜�ۑ��Ԋu�J�E���g��
#define SAVE_VIDEO_RATE	33		// ����ۑ��Ԋu�J�E���g��
// �������`
//#define GNUPLOT_PATH	"\"C:\\Program Files\\gnuplot\\bin\\pgnuplot.exe\""	// �p�X�ɋ󔒂����邽��[\"]��O��ɒǉ�
#define GNUPLOT_PATH	"\"C:\\Program Files (x86)\\gnuplot\\bin\\gnuplot.exe\""	// �p�X�ɋ󔒂����邽��[\"]��O��ɒǉ�
#define FILE_SAVE_DIR	"./data/"		// �t�@�C���ۑ��f�B���N�g��
#define FILENAME_DATA	FILE_SAVE_DIR "data_%.3d.txt"		// �A��3���Ή�
#define FILENAME_INFO	FILE_SAVE_DIR "info_%.3d.txt"		// �A��3���Ή�
#define FILENAME_GRAPH1	FILE_SAVE_DIR "img_jnt_pos.png"
#define FILENAME_GRAPH2	FILE_SAVE_DIR "img_jnt_vel.png"
#define FILENAME_GRAPH3	FILE_SAVE_DIR "img_jnt_force.png"
#define FILENAME_GRAPH4	FILE_SAVE_DIR "img_eff_force.png"
#define FILENAME_GRAPH5	FILE_SAVE_DIR "img_err.png"
#define	DATA_FILE_NAME_MAXLEN	20
#define FILENAME_VIDEO	FILE_SAVE_DIR "cap.mp4"		// �r�f�I��

// �A�[����`
// 2���R�x�A�[���@�e�����N�͉~���ō\��
#if 0
#define	ARM_JNT	2
#define	ARM_M1	0
#define	ARM_M2	1
#define	ARM_LINK1_LEN	0.75		// �����N��
#define	ARM_LINK2_LEN	0.75		// �����N��
#define	ARM_LINK1_COG_LEN	(ARM_LINK1_LEN/2.0)		// ���ʒ��S
#define	ARM_LINK2_COG_LEN	(ARM_LINK2_LEN/2.0)		// ���ʒ��S
#define	ARM_LINK1_RAD	0.125		// �����N���a
#define	ARM_LINK2_RAD	0.10		// �����N���a
#define	ARM_LINK1_MASS	1.0		// ����
#define	ARM_LINK2_MASS	0.8		// ����
#define	ARM_JNT1_VISCOUS	1.0		// �S���W��
#define	ARM_JNT2_VISCOUS	1.0		// �S���W��
#elif 1
constexpr int	ARM_NUM = 2;	// �A�[���{��
constexpr int	ARM_N1 = 0;	// �A�[���ԍ�
constexpr int	ARM_N2 = 1;	// �A�[���ԍ�

constexpr int	ARM_JNT = 2;	// �A�[���֐ߐ�
constexpr int	ARM_M1 = 0;	// �A�[���֐ߔԍ�
constexpr int	ARM_M2 = 1;	// �A�[���֐ߔԍ�
constexpr double	ARM_LINK1_LEN = 0.75;		// �����N��
constexpr double	ARM_LINK2_LEN = 0.75;		// �����N��
constexpr double	ARM_LINK1_COG_LEN = ARM_LINK1_LEN / 2.0;		// ���ʒ��S
constexpr double	ARM_LINK2_COG_LEN = ARM_LINK2_LEN / 2.0;		// ���ʒ��S
constexpr double	ARM_LINK1_RAD = 0.125;		// �����N���a
constexpr double	ARM_LINK2_RAD = 0.10;		// �����N���a
constexpr double	ARM_LINK1_MASS = 1.0;		// ����
constexpr double	ARM_LINK2_MASS = 0.8;		// ����
constexpr double	ARM_JNT1_VISCOUS = 1.0;		// �S���W��
constexpr double	ARM_JNT2_VISCOUS = 1.0;		// �S���W��

#endif

////////////////////////////////////////////////////////
// �\���̒�`
////////////////////////////////////////////////////////
// �ϐ��\����
struct Variable {
	Matrix	q, dq, ddq;	// �֐ߊp�x�C�֐ߑ��x�C�֐߉����x
	Matrix	r, dr, ddr;	// ���ʒu�C��摬�x�C�������x
	Matrix	F;	// ���O��
//	Matrix	dq;	// �֐ߑ��x
//	Matrix	dr;	// ��摬�x
//	Matrix	ddq;	// �֐߉����x
//	Matrix	ddr;	// �������x
	/*
	// �⑫�ϐ��i�����l�j
	Matrix	q0;	// �֐ߊp
	Matrix	r0;	// ���ʒu
	Matrix	F0;	// ���O��
	Matrix	dq0;	// �֐ߑ��x
	Matrix	dr0;	// ��摬�x
	*/
	Variable() {
		matInit(&q, ARM_JNT, 1); matInit(&dq, ARM_JNT, 1); matInit(&ddq, ARM_JNT, 1);
		matInit(&r, DIM2, 1); matInit(&dr, DIM2, 1); matInit(&ddr, DIM2, 1);
		matInit(&F, DIM2, 1);
	}
};

// �^���w�\����
struct Kinematics {       //
	double	l[ARM_JNT];		// �����N��
	double	lg[ARM_JNT];		// �����N�d�S�ʒu�܂ł̒���
	double	r[ARM_JNT];		// �����N���a�i���������j
	Matrix	J;	// ���R�r�A��
	Matrix	dJ;	// ���R�r�A������
	Matrix	Jt, Jinv;	// �]�u�s��C�t�s��
	Kinematics(){
		this->l[ARM_M1] = ARM_LINK1_LEN;	this->l[ARM_M2] = ARM_LINK2_LEN;
		this->lg[ARM_M1] = ARM_LINK1_COG_LEN;	this->lg[ARM_M2] = ARM_LINK2_COG_LEN;
		this->r[ARM_M1] = ARM_LINK1_RAD;	this->r[ARM_M2] = ARM_LINK2_RAD;
		matInit(&J, ARM_JNT, ARM_JNT); matInit(&dJ, ARM_JNT, ARM_JNT);
		matInit(&Jt, ARM_JNT, ARM_JNT); matInit(&Jinv, ARM_JNT, ARM_JNT);
	}
};

// ���͊w�\����
// Mq*ddq + h + V*dq = tau + Jt*F
struct Dynamics {       //
	double	m[ARM_JNT];		// �����N����
	Matrix	Mq;		// ������
	Matrix	h;	// �R���I���E���S�͍�
	double	V[ARM_JNT];	// �S�����C�W��
	// �⑫�ϐ�
	Matrix	dMq;		// ����������
	Dynamics() {
		this->m[ARM_M1] = ARM_LINK1_MASS;	this->m[ARM_M2] = ARM_LINK2_MASS;
		this->V[ARM_M1] = ARM_JNT1_VISCOUS;	this->V[ARM_M2] = ARM_JNT2_VISCOUS;
		matInit(&Mq, ARM_JNT, ARM_JNT); matInit(&h, ARM_JNT, 1);
		matInit(&dMq, ARM_JNT, ARM_JNT);
	}
};

// �C���s�[�_���X�\����
struct  Impedance {
	Matrix	M, C, K;	// �ڕW�C���s�[�_���X�i�����W�j
	Matrix	K0;	// SLS�p
	Matrix	Gp, Gv;	// �C���i�[���[�v�p�Q�C���i���Q�C���C�����Q�C���j
	double	T[DIM3];	// ����
	// �⑫�ϐ�
	Matrix	dM, dC, dK;	// �ڕW�C���s�[�_���X�����i�����W�j
	Matrix	Minv, Cinv, Kinv;	// �t�s��
	Impedance() {
		matInit(&M, DIM2, DIM2); matInit(&C, DIM2, DIM2); matInit(&K, DIM2, DIM2);
		matInit(&Minv, DIM2, DIM2); matInit(&Cinv, DIM2, DIM2); matInit(&Kinv, DIM2, DIM2);
		matInit(&Gp, DIM2, DIM2); matInit(&Gv, DIM2, DIM2);
		matInit(&dM, DIM2, DIM2); matInit(&dC, DIM2, DIM2); matInit(&dK, DIM2, DIM2);
		matInit(&K0, DIM2, DIM2);
	}
};

////////////////////////////////////////////////////////
// �v���g�^�C�v
////////////////////////////////////////////////////////
struct Vec3 { double x, y, z; Vec3(double x, double y, double z) : x(x), y(y), z(z) {} };

// �p�[�c�N���X
class cParts {
protected:
	dBodyID body;        // �{�f�B(����)��ID�ԍ��i���͊w�v�Z�p�j
	dGeomID geom;        // �W�I���g����ID�ԍ�(�Փˌ��o�v�Z�p�j
	dReal  m;       // ����[kg]
	dMass mass;
	std::vector<float> color{1,1,1};
public:
	cParts(dReal m);
	cParts(dReal m, Vec3 init_pos);
	~cParts() { dBodyDestroy(this->body); dGeomDestroy(this->geom);	std::cout << "destroy" << std::endl; }
	dBodyID getBody() { return this->body; }
	dGeomID getGeom() { return this->geom; }
	// �ʒu��ݒ�
	void setPosition(double x, double y, double z) { dBodySetPosition(getBody(), x, y, z); }
	// �ʒu���擾
	auto getPosition() { return dBodyGetPosition(getBody()); }
	// ��]��ݒ�
	void setRotation(double ang) {
		dMatrix3	R;
		dRFromAxisAndAngle(R, -sin(ang), cos(ang), 0, PI / 2);
		dBodySetRotation(getBody(), R);
	}
	void setColor(float r, float g, float b) { color[0] = r; color[1] = g; color[2] = b; }
	// ��]���擾
	//	Quaternion getRotation() const;
	// �T�C�Y���擾
	//	Vec3 getSize() const { return this->size; }
	void destroy() { dBodyDestroy(getBody()); dGeomDestroy(getGeom()); }
	// ���z�֐�
	virtual dReal getl() { return 0; }	// ��肠����return��0��Ԃ��Ă���
	virtual dReal getr() { return 0; }	// ��肠����return��0��Ԃ��Ă���
	virtual void draw() {}
};

// �I�u�W�F�N�g
class cPartsBox : public cParts {
	dReal	sides[DIM3];	// ������x,y,z�̕Ӓ�
public:
	cPartsBox(dReal m, Vec3 l);
	cPartsBox(dReal m, Vec3 init_pos, Vec3 l);
	~cPartsBox() {}		// �f�X�g���N�^
	// �`��
	auto get() { return this->sides; }
	// �`��
	void draw() {		// ���ݒ�
		dsSetColor(color[0], color[1], color[2]);
		dsDrawBox(dBodyGetPosition(getBody()), dBodyGetRotation(getBody()), this->sides);
	}
};

// �I�u�W�F�N�g
class cPartsCylinder : public cParts {
	dReal  l, r;       // ����[m], ���a[m]
public:
	cPartsCylinder(dReal m, dReal l, dReal r);
	cPartsCylinder(dReal m, Vec3 init_pos, dReal l, dReal r);
	~cPartsCylinder() {}		// �f�X�g���N�^
	dReal getl() { return l; }
	dReal getr() { return r; }
	void draw() {
		dsSetColor(color[0], color[1], color[2]);
		dsDrawCylinder(dBodyGetPosition(getBody()), dBodyGetRotation(getBody()), this->l, this->r);
	}
};

// �w
class cFinger {

	cPartsBox	base{ 14.0, Vec3(0.3, 0.5, 0.4)};

	cPartsCylinder	link1{ ARM_LINK1_MASS, ARM_LINK1_LEN, ARM_LINK1_RAD };
	cPartsCylinder	link2{ ARM_LINK2_MASS, ARM_LINK2_LEN, ARM_LINK2_RAD };
	cPartsCylinder	sensor{ 0.0001 / ARM_LINK2_LEN * ARM_LINK2_MASS, 0.0001, ARM_LINK2_RAD };	// �A�[���Ɩ��x�����낦��
//	std::vector<cParts*> finger{ 4 };	// 4 = ARM_JNT + base + sensor
	std::vector<cParts*> finger;
	//dReal x0 = 0.0, y0 = 0.0, z0 = 1.5;
	dReal x0 = 0.5, y0 = 0.0, z0 = 1.5;			//	������������1�{�ڂ̎w�̓y��̈ʒu�@kawahara
	dReal x1 = 0.5, y1 = -1.0, z1 = 1.5;		//	������������1�{�ڂ̎w�̓y��̈ʒu�@kawahara

	//	constexpr double Z_OFFSET = 0.08;
	double Z_OFFSET = 0.08;
	//double jnt_pos[ARM_JNT];
public:
	//{����,�����ʒu(x,y,z),�傫��(x,y,z)}
	cPartsBox	plate{ 100.0, Vec3(-1.2,-0.5, 0.0),Vec3(1.5,0.2,0.5) };


	dJointFeedback force, *p_force;

	dJointID f_joint, r_joint[ARM_JNT], f2_joint; // �Œ�֐߂Ɖ�]�֐�
	dJointID graspObj; 							  //�c���Ώۂ̃v���[�g

	// �w�̐���p�ϐ�
	int	step;					//�o�߃X�e�b�v��
	int state_contact;			// �ڐG���(0:OFF, 1:ON)
	double	dist;				// �A�[���ƑΏۂ̋���
	double	jnt_pos[ARM_JNT];
	double	jnt_vel[ARM_JNT];
	double	jnt_force[ARM_JNT];
	double	past_jnt_pos[ARM_JNT];
	double	eff_pos[DIM3];
	double	eff_vel[DIM3];
	double	eff_force[DIM3];
	double	obj_pos[DIM3];
	double	obj_vel[DIM3];
	// �ڕW�ϐ�
	double	ref_jnt_pos[ARM_JNT];
	double	ref_jnt_vel[ARM_JNT];
	double	ref_eff_pos[DIM3];
	double	ref_eff_vel[DIM3];
	// �����ϐ�
	double	init_jnt_pos[ARM_JNT];
	double	init_obj_pos[DIM3];
	double	init_obj_att[DIM3][DIM3];	// ��΍��W�ɂ�����Ώۍ��W���̎p���i���͐��K�������j
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
	// �ۑ��p�f�[�^�ϐ�
	int save_state_contact[DATA_CNT_NUM];
	double	save_dist[DATA_CNT_NUM];
	double	save_ref_jnt_pos[DATA_CNT_NUM][ARM_JNT];
	double	save_ref_jnt_vel[DATA_CNT_NUM][ARM_JNT];
	double	save_jnt_pos[DATA_CNT_NUM][ARM_JNT];
	double	save_jnt_vel[DATA_CNT_NUM][ARM_JNT];
	double	save_jnt_force[DATA_CNT_NUM][ARM_JNT];
	double	save_ref_eff_pos[DATA_CNT_NUM][DIM3];
	double	save_ref_eff_vel[DATA_CNT_NUM][DIM3];
	double	save_eff_pos[DATA_CNT_NUM][DIM3];
	double	save_eff_vel[DATA_CNT_NUM][DIM3];
	double	save_eff_force[DATA_CNT_NUM][DIM3];
	double	save_obj_pos[DATA_CNT_NUM][DIM3];
	double	save_obj_vel[DATA_CNT_NUM][DIM3];
	// �ۑ��p�t�@�C�����ϐ�
	char	data_file_name[DATA_FILE_NAME_MAXLEN];
	char	filename_info[DATA_FILE_NAME_MAXLEN];
	char	filename_graph[DATA_FILE_NAME_MAXLEN];
	// �����o�֐�
	void initJntPos(double* init_jnt_pos) {}
	int armWithoutInertiaShaping();
	int ctrlPreProcessing();
	int armDynPara();
	int armInvKine(Kinematics* kine, Variable* var);
	int armJacob(Kinematics* kine, Variable* var);
	int armInitMat(Variable* var, Kinematics* kine, Dynamics* dyn, Impedance* imp);
	
	//kawahara�̕ύX�ȑO����R�����g�A�E�g
	////	int armInitMatVar(Variable *var);
	////	int armInitMatKine(Kinematics *kine);

	//int ctrlInitErr();
	int armCalcImpPeriod();
	void saveData();
	void saveInfo();
	void saveGraph();
	////Finger class�̒��Ɉړ�
	//int ctrlMaxwell(Matrix* tau);

	//	cFinger(double* init_jnt_pos) : jnt_pos{init_jnt_pos[0], init_jnt_pos[1]} { finger[0] = &base; finger[1] = &link1; finger[2] = &link2; finger[3] = &sensor; }
	//�R���X�g���N�^
	cFinger(double* init_jnt_pos) : jnt_pos{ init_jnt_pos[0], init_jnt_pos[1] }, finger{&base, &link1, &link2, &sensor} {}
	~cFinger() {		// �W���C���g�j��
		dJointDestroy(f_joint);   // �y��Œ�
		dJointDestroy(r_joint[ARM_M1]);   // �A�[��
		dJointDestroy(r_joint[ARM_M2]);   // �A�[��
		dJointDestroy(f2_joint);   // �Z���T�Œ�
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
	//kawahara���ǉ��@��{�ڂ̎w�p
	void setPosition2() {
		//inger[0]->setPosition(x0, y0, 0.4 / 2);	// z:base->sides[CRD_Z]/2
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
	void setJoint();	// �֐ߐݒ�
	void setJoint2();	// �֐ߐݒ� 2�{�ڂ̎w
	void setJntFric();	// ���C�ݒ�
	void addExtForce();		// �O��
	void addExtForce2();		// �O��

	void control();		// ����
	void destroy() { for (auto &x : finger) { x->destroy(); } }
	void draw() { for (auto &x : finger) { x->draw(); } }
};

////////////////////////////////////////////////////////
// DrawStuff�N���X
////////////////////////////////////////////////////////
class DrawStuff {
	static dsFunctions fn;	// �`��ϐ�
	// ���_�ϐ�
	static float xyz[3];
	static float hpr[3];	// �P�ʂ�deg
public:
	DrawStuff()	{	// �`��֐��̐ݒ�
		fn.version = DS_VERSION;    // �h���[�X�^�b�t�̃o�[�W����
		fn.start = &start;			// �O���� start�֐��̃|�C���^
		fn.step = &simLoop;			// simLoop�֐��̃|�C���^
		fn.command = &command;      // �L�[���͊֐��ւ̃|�C���^
		fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH; // �e�N�X�`��
	}
	static void start();
	static void simLoop(int pause);
	static void command(int cmd);
	auto getFn() { return &fn; }
};

////////////////////////////////////////////////////////
// ODE�N���X
////////////////////////////////////////////////////////
class ODE {
	// ODE
	dWorldID world;  // ���͊w�v�Z�p���[���h
	dSpaceID space;  // �Փˌ��o�p�X�y�[�X
public:
	dGeomID  ground; // �n��
	dJointGroupID contactgroup; // �R���^�N�g�O���[�v
	ODE() {		// �f�t�H���g�R���X�g���N�^
		dInitODE();
		this->world = dWorldCreate();
		this->space = dHashSpaceCreate(0);
		this->contactgroup = dJointGroupCreate(0);
	}
	~ODE() {		// �f�X�g���N�^
		dJointGroupDestroy(this->contactgroup);     // �W���C���g�O���[�v�̔j��
		dSpaceDestroy(this->space);
		dWorldDestroy(this->world);
		dCloseODE();
	}
	void setEnv() {		// ���ݒ�
		dWorldSetGravity(this->world, 0, 0, -9.8);	// �d�͐ݒ�
		dWorldSetERP(this->world, 0.9);          // ERP�̐ݒ�
		dWorldSetCFM(this->world, 1e-4);         // CFM�̐ݒ�
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

	std::shared_ptr<cFinger> pFinger2;	//��{�ڂ̎w�@kawahara

	std::shared_ptr<cPartsCylinder> pObj;
	//std::shared_ptr<cFinger> pFinger2;
	//std::shared_ptr<cPartsCylinder> pObj2;

public:
	void setup() {
		constexpr auto OBJ_RADIUS = 0.10;
		//double init_jnt_pos[2] = { 4 * PI / 4.0, PI/ 4.0 };
		//�e�֐߂̏����p��(�p�x)
		double init_jnt_pos[2] = { 4 * PI / 4.0, PI / 8.0 };
		double init_jnt_posF2[2] = { 4 * PI / 4.0, -PI / 6.0 };//��{�ڂ̎w


		Vec3 obj_pos = { Vec3(-0.8 / sqrt(2.0) - 2 * 0.75 / sqrt(2.0), -0.8 / sqrt(2.0), OBJ_RADIUS) };
		this->pFinger = std::make_shared<cFinger>(init_jnt_pos);
		this->pFinger2 = std::make_shared<cFinger>(init_jnt_posF2);	//��{�ڂ̎w

		this->pObj = std::make_shared<cPartsCylinder>(0.2, obj_pos, 0.15, 0.10);
		std::vector<Vec3> color{ Vec3(1, 0, 0), Vec3(0, 0, 1), Vec3(0, 0.5, 0.5), Vec3(0, 0.5, 0.5) };

		

		
		this->pFinger->setColor(color);
		this->pFinger2->setColor(color);	//��{�ڂ̎w�@kawahara���ǉ�

#if 0

		obj_pos = { Vec3(-0.8 / sqrt(2.0) - 2 * 0.75 / sqrt(2.0)+5, -0.8 / sqrt(2.0), OBJ_RADIUS) };
		this->pFinger2 = std::make_shared<cFinger>(init_jnt_pos);
		this->pObj2 = std::make_shared<cPartsCylinder>(0.2, obj_pos, 0.15, 0.10);
		//std::vector<Vec3> color{ Vec3(1, 0, 0), Vec3(0, 0, 1), Vec3(0, 0.5,.5), Vec3(0, 0.5, 0.5) };
		this->pFinger->setColor(color);
#endif 0
	}
	void createRobot();	// ���{�b�g�����i�{�f�B�E�W�I���g���E�W���C���g�j
	void createObject();	// �Ώې����i�{�f�B�E�W�I���g���j
	void destroyRobot() {	// ���{�b�g�j��i�W���C���g�E�{�f�B�E�W�I���g���j
		pFinger.reset();  // �C���X�^���X�̔j��
		pFinger2.reset();
//		this->pFinger->destroy();
	}
	void destroyObject() {	pObj.reset(); } // �C���X�^���X�̔j��	// �Ώ۔j��i�{�f�B�E�W�I���g���j
	void update() {		// �V�~�����[�V�������P�X�e�b�v�i�s
		dSpaceCollide(this->getSpace(), 0, &this->nearCallback);		// �Փ˔���
		dWorldStep(this->getWorld(), SIM_CYCLE_TIME);	// 1�X�e�b�v�i�߂�
		dJointGroupEmpty(this->contactgroup); // �W���C���g�O���[�v����ɂ���
	}
	auto getFinger() { return pFinger; }
	auto getFinger2() { return pFinger2; }

	auto getFingers() { return (pFinger,pFinger2); }

	auto getObj() { return pObj; }
	void setAddForceObj(dReal fx, dReal fy, dReal fz) { dBodyAddForce(pObj->getBody(), fx, fy, fz); }
};

////////////////////////////////////////////////////////
// �V�~�����[�V�����\����
////////////////////////////////////////////////////////
class SIM: public EntityODE {
public:
//
//	// 1�{�ڂ̎w�p
//	// ����p�ϐ�
//	int	step;					//�o�߃X�e�b�v��
//	int state_contact;			// �ڐG���(0:OFF, 1:ON)
//	double	dist;				// �A�[���ƑΏۂ̋���
//	double	jnt_pos[ARM_JNT];
//	double	jnt_vel[ARM_JNT];
//	double	jnt_force[ARM_JNT];
//	double	past_jnt_pos[ARM_JNT];
//	double	eff_pos[DIM3];
//	double	eff_vel[DIM3];
//	double	eff_force[DIM3];
//	double	obj_pos[DIM3];
//	double	obj_vel[DIM3];
//	// �ڕW�ϐ�
//	double	ref_jnt_pos[ARM_JNT];
//	double	ref_jnt_vel[ARM_JNT];
//	double	ref_eff_pos[DIM3];
//	double	ref_eff_vel[DIM3];
//	// �����ϐ�
//	double	init_jnt_pos[ARM_JNT];
//	double	init_obj_pos[DIM3];
//	double	init_obj_att[DIM3][DIM3];	// ��΍��W�ɂ�����Ώۍ��W���̎p���i���͐��K�������j
//	// �ϐ��\����
//	Variable	var;			// ���ݒl
//	Variable	var_prev;		// �ߋ��l�i1�T�C�N���O�j
//	Variable	var_prev2;		// �ߋ��l�i2�T�C�N���O�j
//	Variable	var_init;		// �����l
//	// �^���w�ϐ�
//	Kinematics	kine;
//	// ���͊w�ϐ�
//	Dynamics	dyn;
//	// �C���s�[�_���X�ϐ�
//	Impedance	imp;
//	// �ۑ��p�f�[�^�ϐ�
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
//	// �ۑ��p�t�@�C�����ϐ�
//	char	data_file_name[DATA_FILE_NAME_MAXLEN];
//	char	filename_info[DATA_FILE_NAME_MAXLEN];
//	char	filename_graph[DATA_FILE_NAME_MAXLEN];
//	// �����o�֐�
//	void initJntPos(double *init_jnt_pos) {}
//	int armWithoutInertiaShaping();
//	int ctrlPreProcessing();
	int armDynPara();
//	int armInvKine(Kinematics *kine, Variable *var);
//	int armJacob(Kinematics *kine, Variable *var);
//	int armInitMat(Variable *var, Kinematics *kine, Dynamics *dyn, Impedance *imp);
//////	int armInitMatVar(Variable *var);
//////	int armInitMatKine(Kinematics *kine);
int ctrlInitErr();	
//	int armCalcImpPeriod();
//	void saveData();
//	void saveInfo();
//	void saveGraph();



};


struct Fparams {
	
};


// �P��C���X�^���X�Ǘ�
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
// �v���g�^�C�v
////////////////////////////////////////////////////////
static void restart();
int exeCmd(int argc, char *argv[]);

#endif
