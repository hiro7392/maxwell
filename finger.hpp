#ifndef _H_FINGER_
#define _H_FINGER_

#include "makeRobot.h"
#include "command.h"
#include "matBase.h"
#include "setRobot.h"
#include "ctrlRobot.h"

#pragma comment(lib,"C:\\ode-0.16.1\\lib\\DebugDoubleDLL\\ode_doubled.lib")
#pragma comment(lib,"C:\\ode-0.16.1\\lib\\DebugDoubleDLL\\drawstuffd.lib")

#include "texturepath.h"
#include "simMain.h"


#include "simMode.h"
#include "setEnv.h"

#if	GLAPHIC_OPENGL 
#include <GL/glut.h>	// stdlib.h����ɓǂݍ��ޕK�v����
#include "graphic.h"
#endif
#if	FLAG_SAVE_VIDEO
#include "video.h"
#endif

//�Z���T�̃m�C�Y�����p
#include "generateNoise.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif




#define finger2_use 1				//��{�ڂ̎w���g���Ƃ�
#define print_debug 0
#define usePlateToGrasp 1			//�c������v���[�g��u���Ƃ�
#define useForceContactPoint 0		//�͂̐ڐG�_�𗘗p����Ƃ�

//�J�v�Z���p
#define DENSITY (5.0)	// ���x
// �J�v�Z��
static dMass mass ;
dReal radius = 0.25; // ���a
dReal length = 1.0;  // ����
const dReal* pos1=0, * R1, * pos2 , * R2, * pos3, * R3;    //�@���̕`��
struct MyObject {
	dBodyID body;		// �{�f�B�i���́j
	dGeomID geomBody;
};
static MyObject capsule;
static MyObject plateToGrasp;	//�c������p�̃v���[�g

// �N���X�ÓI�����o�̏�����
dsFunctions DrawStuff::fn;
float DrawStuff::xyz[3];
float DrawStuff::hpr[3];

// �����ݒ�ϐ�
//double	init_jnt_pos[ARM_JNT] = {3*PI/4.0, PI/2.0};	// ���{�b�g�����p��
double	init_jnt_pos[ARM_JNT] = { 4 * PI / 4.0 - PI / 7, PI / 3.0 };	// ���{�b�g�����p��
double	init_jnt_posF2[ARM_JNT] = { 4 * PI / 4.0 + PI / 7, -PI / 3.0 };	// ���{�b�g�����p��
#if SIM_OBJ_CASE1
double	init_obj_pos[DIM3] = { -0.8 - 2 * 0.75 / sqrt(2.0), -0.035, OBJ_RADIUS };	// �Ώۏ����ʒu
double	init_obj_att[DIM3][DIM3] = { {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}, {0.0, 1.0, 0.0} };	// ��]����y����
#elif SIM_OBJ_CASE2
double	init_obj_pos[DIM3] = { -2 * 0.75 / sqrt(2.0) - 0.035, -0.8, OBJ_RADIUS };	// �Ώۏ����ʒu
double	init_obj_att[DIM3][DIM3] = { {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}, {-1.0, 0.0, 0.0} };	// ��]����x����
#elif SIM_OBJ_CASE3
double	init_obj_pos[DIM3] = { -0.8 / sqrt(2.0) - 2 * 0.75 / sqrt(2.0), -0.8 / sqrt(2.0), OBJ_RADIUS };	// �Ώۏ����ʒu
double	init_obj_att[DIM3][DIM3] = { {sqrt(2.0) / 2, sqrt(2.0) / 2, 0.0}, {0.0, 0.0, -1.0}, {-sqrt(2.0) / 2, sqrt(2.0) / 2, 0.0} };	// ��]����x����
#endif

#define practice 0			//ODE OpenGL�̗��K�p kawahara
////////////////////////////////////////////////////////
// �֐ߐ���
// �߂�l�F�֐߈�ʉ��́i�����֐߂ł͗́C��]�֐߂ł̓g���N�j
////////////////////////////////////////////////////////

void cFinger::control() {
	auto entity = EntityManager::get();

	Matrix	tau;
	double rotTau=0.0;
	matInit(&tau, 2, 1);
	// ������
	if (entity->step == 0) {

		// ������
		ctrlInitErr();		// �p�����[�^�덷��ǉ�	���͉�������Ă��Ȃ��̂Ŗ���
		// �����l�ۑ�
		for (int crd = 0; crd < DIM2; crd++) {
			var_init.q.el[crd][0] = jnt_pos[crd]; var_init.dq.el[crd][0] = jnt_vel[crd] = 0.0;
			var_init.r.el[crd][0] = eff_pos[crd]; var_init.dr.el[crd][0] = eff_vel[crd] = 0.0;
			var_init.F.el[crd][0] = eff_force[crd] = 0.0;
		}
	}

	// ���ϐ����
	for (int crd = 0; crd < DIM2; crd++) {
		var.r.el[crd][0] = eff_pos[crd]; var.dr.el[crd][0] = eff_vel[crd];
		var.F.el[crd][0] = eff_force[crd];
	}
	// �֐ߕϐ����
	for (int jnt = 0; jnt < ARM_JNT; jnt++) {
		var.q.el[jnt][0] = jnt_pos[jnt]; var.dq.el[jnt][0] = jnt_vel[jnt];
	}

	// �p�����[�^�Z�b�g
	armDynPara();

	// �C���s�[�_���X�ݒ�
	// ����w�ߌv�Z
#if SIM_CTRL_MODE_MAXWELL & SIM_ADD_EXT_FORCE
	double	impM[] = { 2.0, 2.0 }, impC[] = { 4.0, 4.0 }, impK[] = { 40.0, 40.0 }, impK0[] = { 10.0, 10.0 };

	matSetValDiag(&imp.M, impM); matSetValDiag(&imp.C, impC); matSetValDiag(&imp.K, impK); matSetValDiag(&imp.K0, impK0);	// �Q�C���ݒ�

	//K_p�Ȃǂ̋t�s��̌v�Z
	ctrlPreProcessing();


#if 1
	if (fingerID == 1) {
		//ctrlMaxwell(&tau);
		RestrictedCtrlMaxwell(&tau);
		RotRestrictedCtrlMaxwell(&rotTau);
	}
	else {
		//ctrlMaxwell2(&tau);
		RestrictedCtrlMaxwell2(&tau);
		RotRestrictedCtrlMaxwell(&rotTau);
	}
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
	//	ctrlMaxwellInnerLoopImplicit(_this, &tau);			// ����w�ߌv�Z
	//	ctrlMaxwellInnerLoopJntSpace(_this, &tau);		// �f�o�b�O��
	//	ctrlSLS(_this, &tau);
	//	ctrlMaxwellVar2(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Md, &Cd, &Kd);
	//	ctrlMaxwellVar2(_this, &tau, &_this->dyn.Mq, &_this->dyn.h, &_this->kine.J, &_this->kine.dJ, &_this->var.q, &_this->var.dq, &re, &dre, &_this->var.F, &_this->imp.M, &_this->imp.C, &_this->imp.K);
	//	ctrlMaxwellVar(_this, &tau);
#elif SIM_CTRL_MODE_MAXWELL & SIM_OBJ_IMPACT
	_this->imp.M.el[CRD_X][CRD_X] = 0.5;	_this->imp.M.el[CRD_Y][CRD_Y] = 0.5;	// ����
	_this->imp.C.el[CRD_X][CRD_X] = 0.8;	_this->imp.C.el[CRD_Y][CRD_Y] = 0.8;	// �S��
	_this->imp.K.el[CRD_X][CRD_X] = 5.2;	_this->imp.K.el[CRD_Y][CRD_Y] = 5.2;	// �e��
	_this->imp.K0.el[CRD_X][CRD_X] = 10.0;	_this->imp.K0.el[CRD_Y][CRD_Y] = 10.0;	// �e��(SLS����o�l)
#elif SIM_CTRL_MODE_HYBRID & SIM_ADD_EXT_FORCE
	_this->imp.m_d[CRD_X] = 2.0;	_this->imp.m_d[CRD_Y] = 1.0;	// ����
	_this->imp.c_d[CRD_X] = 4.0;	_this->imp.c_d[CRD_Y] = 0.5;	// �S��
	_this->imp.k_d[CRD_X] = 100.0;	_this->imp.k_d[CRD_Y] = 50.0;	// �e��
	// Maxwell��Voigt�̕����ؑ�
	if (_this->step == IMP_SWITCH_STEP) {
		// �ϐ�������
		for (crd = 0; crd < 2; crd++)	Fint.el[crd][0] = 0.0;
		// �ڕW�C���s�[�_���X�i���j
		_this->imp.M.el[CRD_X][CRD_X] = 1.0;	_this->imp.M.el[CRD_Y][CRD_Y] = 2.0;	// ����
		_this->imp.C.el[CRD_X][CRD_X] = 0.5;	_this->imp.C.el[CRD_Y][CRD_Y] = 4.0;	// �S��
		_this->imp.K.el[CRD_X][CRD_X] = 50.0;	_this->imp.K.el[CRD_Y][CRD_Y] = 100.0;	// �e��
		for (crd = 0; crd < 2; crd++) {
			_this->var_init.r.el[crd][0] = _this->eff_pos[crd];
		}
	}
	ctrlHybrid(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Fint, &Md, &Cd, &Kd);
#elif 0
	_this->imp.m_d[CRD_X] = 200.0;	_this->imp.m_d[CRD_Y] = 2.0;	// ����
	_this->imp.c_d[CRD_X] = 4.0;	_this->imp.c_d[CRD_Y] = 4.0;	// �S��
	_this->imp.k_d[CRD_X] = 10.0;	_this->imp.k_d[CRD_Y] = 100.0;	// �e��
#elif 1
	_this->imp.m_d[CRD_X] = 10.0;	_this->imp.m_d[CRD_Y] = 10.0;	// ����
	_this->imp.c_d[CRD_X] = 100.0;	_this->imp.c_d[CRD_Y] = 1.0;	// �S��
	_this->imp.k_d[CRD_X] = 10.0;	_this->imp.k_d[CRD_Y] = 10.0;	// �e��
#elif SIM_CTRL_MODE_VOIGT
	ctrlVoigt(&tau, &Mq, &h, &J, &dJ, &q, &dq, &re, &dre, &F, &Md, &Cd, &Kd);
#endif
#if print_debug
	std::cout << "tau fingerID :" << fingerID << std::endl;
	matPrint(&tau);
#endif
	// �Ԃ�l�ɑ��
	jnt_force[ARM_M1] = tau.el[ARM_M1][0];
	jnt_force[ARM_M2] = tau.el[ARM_M2][0];
	std::cout << "eff_force " << std::endl;
	std::cout << eff_force[0] << " " << eff_force[1] << std::endl;

	// �w�̎p���ɂ���Č�����ω�
	// ����֐߂������ԃR�����g�A�E�g
	for (int jnt = 0; jnt < ARM_JNT; jnt++)	dJointAddHingeTorque(r_joint[jnt], jnt_force[jnt]);		// �g���N�͏㏑���ł͂Ȃ��C���N�������g����邱�Ƃɒ���
	
	printf("FingerID = %d torque =%lf\n", fingerID, rotTau);
	double MAX = 10000.0, MIN = -10000.0;
	if (rotTau > MAX )rotTau = MAX;
	if (rotTau < MIN)rotTau = MIN;
	printf("FingerID = %d torque =%lf\n", fingerID, rotTau);
	dJointAddHingeTorque(senkai_link_joint, fingerID==1? rotTau:rotTau);
}

#endif