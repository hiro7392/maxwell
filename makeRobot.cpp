//#include <ode/ode.h>
//#include <drawstuff/drawstuff.h>
#include <stdio.h>
#include <math.h>
#include "texturepath.h"
#include "simMain.h"
#include "makeRobot.h"



////////////////////////////////////////////////////////
// ���{�b�g�쐬
// ����2���R�x
////////////////////////////////////////////////////////
int makeFinger(SIM* sim, int idx);
#if 0
int createRobot(SIM* sim)
{
	makeFinger(sim, 0);
	return 0;
}

int makeFinger(SIM* sim, int idx)
{
	auto _this = EntityManager::get();
	dMass mass;
	dReal x0 = 0.0, y0 = 0.0, z0 = 1.5;
	dMatrix3	R;
	//
	MyObject* arm = sim->sys.finger[idx].arm;
	MyObject* base = &sim->sys.finger[idx].base;
	MyObject* sensor = &sim->sys.finger[idx].sensor;
	// �p�����[�^�ݒ�
#define	Z_OFFSET	0.08
	base->sides[CRD_X] = 0.3;	base->sides[CRD_Y] = 0.3;	base->sides[CRD_Z] = 0.4;	base->m = 14.0;	// �y�䒼����
	arm[ARM_M1].l = ARM_LINK1_LEN;	arm[ARM_M1].r = ARM_LINK1_RAD;	arm[ARM_M1].m = ARM_LINK1_MASS;	// �A�[�������N1�~��
	arm[ARM_M2].l = ARM_LINK2_LEN;	arm[ARM_M2].r = ARM_LINK2_RAD;	arm[ARM_M2].m = ARM_LINK2_MASS;	// �A�[�������N2�~��
	sensor->l = 0.0001;    // �����i�T�C�Y�̉e�����o�Ȃ��悤�ɏ����߂ɐݒ�j
	sensor->r = ARM_LINK2_RAD;
	sensor->m = sensor->l / ARM_LINK2_LEN * ARM_LINK2_MASS;		// �A�[���Ɩ��x�����낦��

	// �y�䐶��
	base->body = dBodyCreate(_this->world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, base->m, base->sides[CRD_X], base->sides[CRD_Y], base->sides[CRD_Z]);
	dBodySetMass(base->body, &mass);
	dBodySetPosition(base->body, x0, y0, base->sides[CRD_Z] / 2);
	base->geom = dCreateBox(_this->space, base->sides[CRD_X], base->sides[CRD_Y], base->sides[CRD_Z]);
	dGeomSetBody(base->geom, base->body);
	// �A�[�������N1����
	arm[ARM_M1].body = dBodyCreate(_this->world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, arm[ARM_M1].m, DIR_LONG_AXIS_Z, arm[ARM_M1].r, arm[ARM_M1].l);
	dBodySetMass(arm[ARM_M1].body, &mass);
	dBodySetPosition(arm[ARM_M1].body, x0 + arm[ARM_M1].l / 2.0 * cos(sim->init_jnt_pos[ARM_M1]), y0 + arm[ARM_M1].l / 2.0 * sin(sim->init_jnt_pos[ARM_M1]), base->sides[CRD_Z] / 2.0 - Z_OFFSET);
	dRFromAxisAndAngle(R, -sin(sim->init_jnt_pos[ARM_M1]), cos(sim->init_jnt_pos[ARM_M1]), 0, PI / 2);
	dBodySetRotation(arm[ARM_M1].body, R);
	arm[ARM_M1].geom = dCreateCylinder(_this->space, arm[ARM_M1].r, arm[ARM_M1].l);
	dGeomSetBody(arm[ARM_M1].geom, arm[ARM_M1].body);
	// �A�[�������N2����
	arm[ARM_M2].body = dBodyCreate(_this->world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, arm[ARM_M2].m, DIR_LONG_AXIS_Z, arm[ARM_M2].r, arm[ARM_M2].l);
	dBodySetMass(arm[ARM_M2].body, &mass);
	dBodySetPosition(arm[ARM_M2].body, x0 + arm[ARM_M1].l * cos(sim->init_jnt_pos[ARM_M1]) + arm[ARM_M2].l / 2.0 * cos(sim->init_jnt_pos[ARM_M1] + sim->init_jnt_pos[ARM_M2]), y0 + arm[ARM_M1].l * sin(sim->init_jnt_pos[ARM_M1]) + arm[ARM_M2].l / 2.0 * sin(sim->init_jnt_pos[ARM_M1] + sim->init_jnt_pos[ARM_M2]), base->sides[CRD_Z] / 2.0 - Z_OFFSET);
	dRFromAxisAndAngle(R, -sin(sim->init_jnt_pos[ARM_M1] + sim->init_jnt_pos[ARM_M2]), cos(sim->init_jnt_pos[ARM_M1] + sim->init_jnt_pos[ARM_M2]), 0, PI / 2);
	dBodySetRotation(arm[ARM_M2].body, R);
	arm[ARM_M2].geom = dCreateCylinder(_this->space, arm[ARM_M2].r, arm[ARM_M2].l);
	dGeomSetBody(arm[ARM_M2].geom, arm[ARM_M2].body);
	// �Z���T�i���ɕt�����C�T�C�Y�̉e�����o�Ȃ��悤�ɏ����߂ɐݒ�j
	sensor->body = dBodyCreate(_this->world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, sensor->m, DIR_LONG_AXIS_Z, sensor->r, sensor->l);
	dBodySetMass(sensor->body, &mass);
	dBodySetPosition(sensor->body, x0 + arm[ARM_M1].l * cos(sim->init_jnt_pos[ARM_M1]) + (arm[ARM_M2].l + sensor->l / 2.0) * cos(sim->init_jnt_pos[ARM_M1] + sim->init_jnt_pos[ARM_M2]), y0 + arm[ARM_M1].l * sin(sim->init_jnt_pos[ARM_M1]) + (arm[ARM_M2].l + sensor->l / 2.0) * sin(sim->init_jnt_pos[ARM_M1] + sim->init_jnt_pos[ARM_M2]), base->sides[CRD_Z] / 2.0 - Z_OFFSET);
	dRFromAxisAndAngle(R, -sin(sim->init_jnt_pos[ARM_M1] + sim->init_jnt_pos[ARM_M2]), cos(sim->init_jnt_pos[ARM_M1] + sim->init_jnt_pos[ARM_M2]), 0, PI / 2);
	dBodySetRotation(sensor->body, R);
	sensor->geom = dCreateCylinder(_this->space, sensor->r, sensor->l);
	dGeomSetBody(sensor->geom, sensor->body);
	// �Œ�W���C���g
	_this->f_joint = dJointCreateFixed(_this->world, 0);
	dJointAttach(_this->f_joint, base->body, 0);
	dJointSetFixed(_this->f_joint);
	// �q���W�W���C���g1
	_this->r_joint[ARM_M1] = dJointCreateHinge(_this->world, 0);
	dJointAttach(_this->r_joint[ARM_M1], arm[ARM_M1].body, base->body);
	dJointSetHingeAnchor(_this->r_joint[ARM_M1], x0, y0, base->sides[CRD_Z] / 2);
	dJointSetHingeAxis(_this->r_joint[ARM_M1], 0, 0, 1);
	dJointSetHingeParam(_this->r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(_this->r_joint[ARM_M1], dParamHiStop, M_PI);
	//	dJointSetHingeParam(_this->r_joint[ARM_M1], dParamFudgeFactor, 0.1);
	//	dJointSetHingeParam(_this->r_joint[ARM_M1], dParamStopERP, 0.8);
		// �q���W�W���C���g2
	_this->r_joint[ARM_M2] = dJointCreateHinge(_this->world, 0);
	dJointAttach(_this->r_joint[ARM_M2], arm[ARM_M2].body, arm[ARM_M1].body);
	dJointSetHingeAnchor(_this->r_joint[ARM_M2], x0 + arm[ARM_M1].l * cos(sim->init_jnt_pos[ARM_M1]), y0 + arm[ARM_M1].l * sin(sim->init_jnt_pos[ARM_M1]), base->sides[CRD_Z] / 2);
	dJointSetHingeAxis(_this->r_joint[ARM_M2], 0, 0, 1);
	dJointSetHingeParam(_this->r_joint[ARM_M2], dParamLoStop, -M_PI);
	dJointSetHingeParam(_this->r_joint[ARM_M2], dParamHiStop, M_PI);
	//	dJointSetHingeParam(_this->r_joint[ARM_M2], dParamFudgeFactor, 0.1);
	//	dJointSetHingeParam(_this->r_joint[ARM_M2], dParamStopERP, 0.8);
		// �Œ�W���C���g
	_this->f2_joint = dJointCreateFixed(_this->world, 0);  // �Œ�W���C���g
	dJointAttach(_this->f2_joint, sensor->body, arm[ARM_M2].body);
	dJointSetFixed(_this->f2_joint);
	// �Z���T�ݒ�i�͂ƃg���N�̎擾�ɕK�v�j
	dJointSetFeedback(_this->f2_joint, &_this->force);

	return	0;
}
#endif

////////////////////////////////////////////////////////
// ���{�b�g�`��
////////////////////////////////////////////////////////
int drawRobot()
{
	const dReal* pos, * R;
	//  const dReal sides[3] = {0.5,0.5,0.5};

#if 0
	MyObject* arm = sim->sys.finger[idx].arm;
	MyObject* base = &sim->sys.finger[idx].base;
	MyObject* sensor = &sim->sys.finger[idx].sensor;
#else
	auto base = EntityManager::get()->getFinger()->getParts()[0];
	auto link1 = EntityManager::get()->getFinger()->getParts()[1];
	auto link2 = EntityManager::get()->getFinger()->getParts()[2];
	auto sensor = EntityManager::get()->getFinger()->getParts()[3];
#endif
#if 0
	// �y��`��
	dsSetColor(1.0, 0.0, 0.0);                       // �ԐF
	pos = dBodyGetPosition(base->getBody());
	R = dBodyGetRotation(base->getBody());
	dsDrawBox(pos, R, base->sides);
	// �A�[�������N1�`��
	pos = dBodyGetPosition(link1->getBody());
	R = dBodyGetRotation(link1->getBody());
	dsSetColor(0.0, 0.0, 1.0);                    // �F
	dsDrawCylinder(pos, R, arm[ARM_M1].l, arm[ARM_M1].r);
	// �A�[�������N2�`��
	pos = dBodyGetPosition(link2->getBody());
	R = dBodyGetRotation(link2->getBody());
	dsSetColor(0.0, 0.5, 0.5);                    // �F
	dsDrawCylinder(pos, R, arm[ARM_M2].l, arm[ARM_M2].r);
	//      dsSetColor(1.2,1.2,1.2);                   // ���F
	//      dsDrawCylinder(pos2, R2, arm.l, arm.r);
	// �Z���T�`��
	pos = dBodyGetPosition(sensor->getBody());
	R = dBodyGetRotation(sensor->getBody());
	dsSetColor(0.0, 0.5, 0.5);                    // 
	dsDrawCylinder(pos, R, sensor->l, sensor->r);
#else
	//EntityManager::get()->getFinger()->draw();
	//EntityManager::get()->getFingers()->draw();		//kawahara

#endif
	return	0;
}

////////////////////////////////////////////////////////
// ���ʒ��S�\��
////////////////////////////////////////////////////////
/*
int drawCoM()
{
	int i,j,k;
	double pos_CoM[3]={0.0};
	double m_all=0.0;
	dMatrix3 dammy_R;

	dRSetIdentity(dammy_R);
	for (i=0;i<2;i++){
		for (j=0;j<5;j++){
			for (k=0;k<DIM_THREE;k++) pos_CoM[k] += *(dBodyGetPosition(leg[i][j].body)+k) *  leg[i][j].m;
			m_all += leg[i][j].m;
		}
	}

	for (i=0;i<2;i++){
		for (k=0;k<DIM_THREE;k++) pos_CoM[k] +=*(dBodyGetPosition(hip[i].body)+k) *  hip[i].m;
		m_all += hip[i].m;
	}

	for (i=0;i<3;i++)  pos_CoM[i] /= m_all;

	dsSetColor(0.5, 0.0, 0.0);
	dsDrawSphere(pos_CoM, dammy_R , 0.01);

	pos_CoM[2]=0.001;
	dsSetColor(0.2, 0.0, 0.1);
	dsDrawSphere(pos_CoM, dammy_R , 0.01);

	// �f�o�b�O�\��
	printf("mass=%f ", m_all);
	return 0;
}
*/

////////////////////////////////////////////////////////
// �O�͕`��
// �̓Z���T�̒l�ɔ�Ⴕ��������\������
////////////////////////////////////////////////////////
int drawExtForce() {

	auto _this = EntityManager::get();

	//	�Z���T�ɂ�����͂�`��
	cParts* sensor = _this->getFinger()->getParts()[3];		//�Z���T����擾����ꍇ
	drawForceCylinder(sensor, dJointGetFeedback(_this->getFinger()->sensor2FingerTop));

	//	�w��ɂ���
	//cPartsCylinder&  forcePoint = _this->getFinger()->forceContactPoint;			//�Z���T����擾����ꍇ
	//drawForceFingerTop(forcePoint, dJointGetFeedback(_this->getFinger()->sensor2FingerTop));

	//	�Z���T�ɂ�����͂�`��
	cParts* sensor2 = _this->getFinger2()->getParts()[3];		//�Z���T����擾����ꍇ
	drawForceCylinder(sensor2, dJointGetFeedback(_this->getFinger2()->sensor2FingerTop));

	return	0;
}

void drawForceCylinder(cParts* sensor,dJointFeedback* p_force) {
	int width;
	dJointFeedback* fb;
	dVector3	p_s, p_e;    // ���̎n�_�ƏI�_
	double k1 = 0.3;  // �����̔��萔
	double line_w = 0.05;

	dVector3	ext_f;	// �O��
	auto _this = EntityManager::get();

	dBodyGetRelPointPos(sensor->getBody(), 0.0, 0.0, sensor->getl() / 2.0, p_s);			// ���ʒu
	
	//std::cout << "p_force f1" << p_force->f1[0] << std::endl;
	for (int crd = 0; crd < DIM3; crd++)	ext_f[crd] = -p_force->f1[crd];	// �Ώۂ��Z���T�ɋy�ڂ��Ă����=�Z���T���֐߂ɋy�ڂ��Ă����
	p_s[CRD_Z] += sensor->getr(); //�r�̏�ɕ\��
	for (int crd = 0; crd < DIM3; crd++)	p_e[crd] = p_s[crd] - k1 * ext_f[crd];
	drawArrowOriginal(p_s, p_e, ext_f);		//�Z���T�Ɋւ��Ă�����͂�`��
}

void drawForceFingerTop(cPartsCylinder& sensor, dJointFeedback* p_force) {
	int width;
	dJointFeedback* fb;
	dVector3	p_s, p_e;    // ���̎n�_�ƏI�_
	double k1 = 0.3;  // �����̔��萔
	double line_w = 0.05;

	dVector3	ext_f;	// �O��
	//double angArrow = PI / 6;  //���̊p�x rad
	double angArrow = 0;
	auto _this = EntityManager::get();

	dBodyGetRelPointPos(sensor.getBody(), 0.0, 0.0, sensor.getl() / 2.0, p_s);			// ���ʒu
	
	//std::cout << "p_force f1" << p_force->f1[0] << std::endl;
	for (int crd = 0; crd < DIM3; crd++)	ext_f[crd] = -p_force->f1[crd];	// �Ώۂ��Z���T�ɋy�ڂ��Ă����=�Z���T���֐߂ɋy�ڂ��Ă����
	p_s[CRD_Z] += sensor.getr(); //�r�̏�ɕ\��
	for (int crd = 0; crd < DIM3; crd++)	p_e[crd] = p_s[crd] - k1 * ext_f[crd];
	drawArrowOriginal(p_s, p_e, ext_f);	//�Z���T�Ɋւ��Ă�����͂�`��
}


//����`�悷��֐�
//����(���̎n�_,���̏I�_,������͂̕���)
void drawArrow(dVector3	p_s, dVector3 p_e, dVector3 ext_f) {
	p_e[CRD_Z] = p_s[CRD_Z];	// z�����̗͖͂���
	double k1 = 0.3;  // �����̔��萔
	double line_w = 0.05;
	dVector3	arrow_center, arrow_r, arrow_l;    // ���̒��_
	dVector3	rect_ul, rect_ll, rect_ur, rect_lr;    // ���̒��_
	dVector3	line, line_e;    // 
	dMatrix3	R;
	double angArrow = PI / 4;  //���̊p�x rad	
	//double angArrow = atan2(-ext_f[0],-ext_f[1]);  //
	dsSetColor(1.0,1.0,1.0);                    // 
#if 1
//	arrow_l[CRD_X] = p_s[CRD_X]+k1/2*fb->f1[CRD_X]-k1/2*fb->f1[CRD_Y];
//	arrow_l[CRD_Y] = p_s[CRD_Y]+k1/2*fb->f1[CRD_Y]+k1/2*fb->f1[CRD_X];
//	arrow_l[CRD_Z] = p_s[CRD_Z];
//	arrow_r[CRD_X] = p_s[CRD_X]+k1/2*fb->f1[CRD_X]+k1/2*fb->f1[CRD_Y];
//	arrow_r[CRD_Y] = p_s[CRD_Y]+k1/2*fb->f1[CRD_Y]-k1/2*fb->f1[CRD_X];
//	arrow_r[CRD_Z] = p_s[CRD_Z];
	arrow_center[CRD_X] = 0;	arrow_center[CRD_Y] = 0.0;	arrow_center[CRD_Z] = 0;

	//���̍��̒��_
	arrow_l[CRD_X] =sin(angArrow)* ext_f[0] * k1 / 3;
	arrow_l[CRD_Y] =cos(angArrow)* ext_f[0] * k1 / 3;
	arrow_l[CRD_Z] = 0;
	//���̉E�̒��_
	arrow_r[CRD_X] = -sin(angArrow)* ext_f[0] * k1 / 3;
	arrow_r[CRD_Y] = cos(angArrow)* ext_f[0] * k1 / 3;
	arrow_r[CRD_Z] = 0;

	//���̒����`����
	//=���p�O�p�`�~2�ŕ`��
	rect_ul[CRD_X] = rect_ll[CRD_X] = sin(angArrow) * ext_f[0] * k1 / 8;
	
	rect_ur[CRD_X] = rect_lr[CRD_X] = -sin(angArrow) * ext_f[0] * k1 / 8;
	rect_ul[CRD_Y] = rect_ur[CRD_Y] = cos(angArrow) * ext_f[0] * k1 / 3;
	rect_ll[CRD_Y] = rect_lr[CRD_Y] = k1 * ext_f[0];
	rect_ul[CRD_Z] = rect_ll[CRD_Z] = rect_ur[CRD_Z] = rect_lr[CRD_Z] = 0.0;

	//(0,0,1)�����ɃΉ�]����
	dRFromAxisAndAngle(R, 0, 0, 1, -PI - atan2(p_s[CRD_Y] - p_e[CRD_Y], p_s[CRD_X] - p_e[CRD_X]));
	//dRFromAxisAndAngle(R, 0, 0, 1, -PI - atan2(p_e[CRD_Y],p_e[CRD_X]));

	printf("atan2 =%f\n",radToAng(atan2(p_s[CRD_Y] - p_e[CRD_Y], p_s[CRD_X] - p_e[CRD_X])));
	printf("atan2 ext_f =%f\n", radToAng(atan2(ext_f[CRD_Y], ext_f[CRD_X])));

	//	printf("%f %f %f\n\n", rect_ll[0],rect_ll[1],rect_ll[2]);

	// ���̓�
	dsDrawTriangle(p_s, R, arrow_center, arrow_l, arrow_r, 1); // 
	// ���̐��i�O�p�`��2���킹�ĕ��̎������l�p�`�Ƃ��ĕ\���j
	dsDrawTriangle(p_s, R, rect_ul, rect_ll, rect_ur, 1); // 
	dsDrawTriangle(p_s, R, rect_ur, rect_ll, rect_lr, 1); // 
	

//	dsDrawLine(p_s, p_e); // p_s����p_e�܂ł̒�����`��
//	dsDrawLine(line_center, p_e); // p_s����p_e�܂ł̒�����`��
#endif
	return;
}


//����`�悷��֐�
//����(���̎n�_,���̏I�_,������͂̕���)
void drawArrowOriginal(dVector3	p_s, dVector3 p_e, dVector3 ext_f) {

	
	p_e[CRD_Z] = p_s[CRD_Z];	// z�����̗͖͂���
	double k1 = 0.3;  // �����̔��萔
	double line_w = 0.05;
	dVector3	arrow_center, arrow_r, arrow_l;    // ���̒��_
	dVector3	rect_ul, rect_ll, rect_ur, rect_lr;    // ���̒��_
	dVector3	line, line_e;    // 
	dMatrix3	R;
	double angArrow = PI / 4;  //���̊p�x rad	

	dsSetColor(0.0, 1.0, 0.0);                    // 
	arrow_center[CRD_X] = 0;	arrow_center[CRD_Y] = 0.0;	arrow_center[CRD_Z] = 0;

	double L2Norm = sqrt(ext_f[CRD_X] * ext_f[CRD_X] + ext_f[CRD_Y] * ext_f[CRD_Y]);
	//���̍��̒��_
	arrow_l[CRD_X] = sin(angArrow) * L2Norm * k1 / 3;
	arrow_l[CRD_Y] = cos(angArrow) * L2Norm * k1 / 3;
	arrow_l[CRD_Z] = 0;
	//���̉E�̒��_
	arrow_r[CRD_X] = -sin(angArrow) * L2Norm * k1 / 3;
	arrow_r[CRD_Y] = cos(angArrow) * L2Norm * k1 / 3;
	arrow_r[CRD_Z] = 0;

	//���̒����`����
	//=���p�O�p�`�~2�ŕ`��
	rect_ul[CRD_X] = rect_ll[CRD_X] = sin(angArrow) * L2Norm * k1 / 8;

	rect_ur[CRD_X] = rect_lr[CRD_X] = -sin(angArrow) * L2Norm * k1 / 8;
	rect_ul[CRD_Y] = rect_ur[CRD_Y] = cos(angArrow) * L2Norm * k1 / 3;
	rect_ll[CRD_Y] = rect_lr[CRD_Y] = k1 * L2Norm;
	rect_ul[CRD_Z] = rect_ll[CRD_Z] = rect_ur[CRD_Z] = rect_lr[CRD_Z] = 0.0;

	//(0,0,1)�����ɃΉ�]����
	//dRFromAxisAndAngle(R, 0, 0, 1, -PI - atan2(p_s[CRD_Y] - p_e[CRD_Y], p_s[CRD_X] - p_e[CRD_X]));

	//�Z���T�l��fx,fy�ɉ����Č�����ύX����
	dRFromAxisAndAngle(R, 0, 0, 1, -PI - atan2(ext_f[CRD_X], ext_f[CRD_Y]));

	printf("atan2 =%f degree\n", radToAng(atan2(p_s[CRD_X] - p_e[CRD_X], p_s[CRD_Y] - p_e[CRD_X])));
	printf("atan2 ext_f =%f degree\n", radToAng(atan2(ext_f[CRD_X], ext_f[CRD_Y])));

	//	printf("%f %f %f\n\n", rect_ll[0],rect_ll[1],rect_ll[2]);

	// ���̓�
	dsDrawTriangle(p_s, R, arrow_center, arrow_l, arrow_r, 1); // 
	// ���̐��i�O�p�`��2���킹�ĕ��̎������l�p�`�Ƃ��ĕ\���j
	dsDrawTriangle(p_s, R, rect_ul, rect_ll, rect_ur, 1); // 
	dsDrawTriangle(p_s, R, rect_ur, rect_ll, rect_lr, 1); // 



	return;
}




////////////////////////////////////////////////////////
// ���{�b�g�j��
////////////////////////////////////////////////////////
#if 0
int destroyRobot()
{
	auto _this = EntityManager::get();
#if 0
	MyObject* arm = sim->sys.finger[idx].arm;
	MyObject* base = &sim->sys.finger[idx].base;
	MyObject* sensor = &sim->sys.finger[idx].sensor;
#else
	auto base = _this->getFinger()->getParts()[0];
	auto link1 = _this->getFinger()->getParts()[1];
	auto link2 = _this->getFinger()->getParts()[2];
	auto sensor = _this->getFinger()->getParts()[3];
#endif
	// �W���C���g�j��
	dJointDestroy(_this->f_joint);   // �y��Œ�
	dJointDestroy(_this->r_joint[ARM_M1]);   // �A�[��
	dJointDestroy(_this->r_joint[ARM_M2]);   // �A�[��
	dJointDestroy(_this->f2_joint);   // �Z���T�Œ�
	// �{�f�B�j��
	dBodyDestroy(base->getBody()); // �y��
	dBodyDestroy(link1->getBody());  // �A�[��
	dBodyDestroy(link2->getBody());  // �A�[��
	dBodyDestroy(sensor->getBody());  // �Z���T
	// �W�I���g���j��
	dGeomDestroy(base->getGeom()); // �y��
	dGeomDestroy(link1->getGeom());  // �A�[��
	dGeomDestroy(link2->getGeom());  // �A�[��
	dGeomDestroy(sensor->getGeom());  // �Z���T
	return	0;
}
#endif

////////////////////////////////////////////////////////
// �Ώۍ쐬
////////////////////////////////////////////////////////
#if 0
int createObject(SIM* sim)
{
	auto _this = EntityManager::get();
	dMass mass;
	dMatrix3	R;
	MyObject* obj = &sim->sys.obj;

	// �p�����[�^�ݒ�
	obj->l = 0.15;	obj->r = 0.10;	obj->m = 0.2;	// �~��
#if 1
	// �~��
	obj->body = dBodyCreate(_this->world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, obj->m, DIR_LONG_AXIS_Z, obj->r, obj->l);
	dBodySetMass(obj->body, &mass);
	dBodySetPosition(obj->body, sim->init_obj_pos[CRD_X], sim->init_obj_pos[CRD_Y], sim->init_obj_pos[CRD_Z]);
	//	dRFromAxisAndAngle(R, 1, 0, 0, PI/2);	// ������y�����ɉ�]
	//	dRFromAxisAndAngle(R, 0, 1, 0, PI/2);	// ������x�����ɉ�]
	dRFrom2Axes(R, sim->init_obj_att[AXIS_X][CRD_X], sim->init_obj_att[AXIS_X][CRD_Y], sim->init_obj_att[AXIS_X][CRD_Z], sim->init_obj_att[AXIS_Y][CRD_X], sim->init_obj_att[AXIS_Y][CRD_Y], sim->init_obj_att[AXIS_Y][CRD_Z]);
	dBodySetRotation(obj->body, R);
	obj->geom = dCreateCylinder(_this->space, obj->r, obj->l);
	dGeomSetBody(obj->geom, obj->body);
#else
	// ��
	obj->body = dBodyCreate(_this->world);
	dMassSetZero(&mass);
	dMassSetSphereTotal(&mass, obj->m, obj->r);
	dBodySetMass(obj->body, &mass);
	dBodySetPosition(obj->body, sim->init_obj_pos[CRD_X], sim->init_obj_pos[CRD_Y], sim->init_obj_pos[CRD_Z]);
	obj->geom = dCreateSphere(_this->space, obj->r);
	dGeomSetBody(obj->geom, obj->body);
#endif
	return	0;
}
#endif

////////////////////////////////////////////////////////
// �Ώە`��
////////////////////////////////////////////////////////
#if 0
int drawObject(SIM* sim)
{
	int	i;
	const dReal* pos, * R;
	MyObject* obj = &sim->sys.obj;
	// �ՓˑΏە`��
	pos = dBodyGetPosition(obj->body);
	R = dBodyGetRotation(obj->body);
	dsSetColor(0.5, 0.5, 1.0);                    // 
#if 1	// �~��
	dsDrawCylinder(pos, R, obj->l, obj->r);
#else	// ��
	dsDrawSphere(pos, R, obj->r);
#endif
	return	0;
}
#endif

////////////////////////////////////////////////////////
// �Ώ۔j��
////////////////////////////////////////////////////////
#if 0
int destroyObject(SIM* sim)
{
	MyObject* obj = &sim->sys.obj;
	// �{�f�B�j��
	dBodyDestroy(obj->body);
	// �W�I���g���j��
	dGeomDestroy(obj->geom);
	return	0;
}
#endif

////////////////////////////////////////////////////////
// ���{�b�g�쐬
// ����2���R�x
////////////////////////////////////////////////////////
void EntityODE::createRobot() {

	this->pFinger->setPosition(this->pFinger->senkai_base_x0, this->pFinger->senkai_base_y0, this->pFinger->senkai_base_z0);
	this->pFinger->setJoint();

	this->pFinger2->setPosition2(this->pFinger2->senkai_base_x1, this->pFinger2->senkai_base_y1, this->pFinger2->senkai_base_z1);
	this->pFinger2->setJoint2();

}

void EntityODE::createObject() {
	dMatrix3	R;
	double	init_obj_att[DIM3][DIM3] = { { sqrt(2.0) / 2, sqrt(2.0) / 2, 0.0 },{ 0.0, 0.0, -1.0 },{ -sqrt(2.0) / 2, sqrt(2.0) / 2, 0.0 } };	// ��]����x����
	dRFrom2Axes(R, init_obj_att[AXIS_X][CRD_X], init_obj_att[AXIS_X][CRD_Y], init_obj_att[AXIS_X][CRD_Z], init_obj_att[AXIS_Y][CRD_X], init_obj_att[AXIS_Y][CRD_Y], init_obj_att[AXIS_Y][CRD_Z]);
	dBodySetRotation(this->pObj->getBody(), R);
}
#if 0
//�w�P�̏����ʒu�Ə����p��	�w1�Ǝw2�ŋ���
void setPosition(double senkai_base_x0, double senkai_base_y0, double senkai_base_z0) {
	//	�����N1�̓y��
	double base_x = senkai_base_x0;
	double base_y = senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN * cos(senkai_base_jnt);
	double base_z = senkai_base_z0 - SENKAI_LINK_LEN * sin(senkai_base_jnt);
	finger[0]->setPosition(base_x, base_y, base_z);

	//	�����N�P
	double link1_x = base_x + (ARM_LINK1_LEN / 2.0 * cos(jnt_pos[ARM_M1]));
	double link1_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1]));
	double link1_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN / 2.0 * sin(jnt_pos[ARM_M1]));
	finger[1]->setPosition(link1_x, link1_y, link1_z);

	double link1_top_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]);
	double link1_top_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));
	double link1_top_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));

	//	�����N2
	double link2_x = link1_top_x + (ARM_LINK2_LEN / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
	double link2_y = link1_top_y + cos(senkai_base_jnt) * (ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
	double link2_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + ARM_LINK2_LEN / 2.0 * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
	finger[2]->setPosition(link2_x, link2_y, link2_z);

	//	�Z���T
	double sensor_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + SENSOR_LEN / 2.0) * cos(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]);
	double sensor_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + SENSOR_LEN / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
	double sensor_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]) + (ARM_LINK2_LEN + SENSOR_LEN / 2.0) * sin(jnt_pos[ARM_M1] + jnt_pos[ARM_M2]));
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
void cFinger::setJoint() {

	auto sim = EntityManager::get();
	double base_x = senkai_base_x0;
	double base_y = senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN * cos(senkai_base_jnt);
	double base_z = senkai_base_z0 - SENKAI_LINK_LEN * sin(senkai_base_jnt);

	// �q���W�W���C���g1
	r_joint[ARM_M1] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M1], finger[1]->getBody(), finger[0]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M1], base_x, base_y, base_z);
	//dJointSetHingeAxis(r_joint[ARM_M1], 0, 0, 1);
	dJointSetHingeAxis(r_joint[ARM_M1], 0, sin(senkai_base_jnt), cos(senkai_base_jnt));	//xy���ʏ�ŉ�]����Ƃ�

	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);
	dJointSetFeedback(r_joint[ARM_M1], &r_joint_feedback[ARM_M1]);

	double link1_top_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]);
	double link1_top_y = base_y + cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));
	double link1_top_z = base_z - sin(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));

	// �q���W�W���C���g2
	r_joint[ARM_M2] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M2], finger[2]->getBody(), finger[1]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M2], link1_top_x, link1_top_y, link1_top_z);
	dJointSetHingeAxis(r_joint[ARM_M2], 0, sin(senkai_base_jnt), cos(senkai_base_jnt));	//xy���ʏ�ŉ�]����Ƃ�

	//dJointSetHingeAxis(r_joint[ARM_M2], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M2], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M2], dParamHiStop, M_PI);
	dJointSetFeedback(r_joint[ARM_M2], &r_joint_feedback[ARM_M2]);

	// �Œ�W���C���g
	f2_joint = dJointCreateFixed(sim->getWorld(), 0);  // �Œ�W���C���g
	dJointAttach(f2_joint, finger[3]->getBody(), finger[2]->getBody());
	dJointSetFixed(f2_joint);

	// �Œ�W���C���g	�Z���T�Ǝw��̉~��	
	sensor2FingerTop = dJointCreateFixed(sim->getWorld(), 0);  // �Œ�W���C���g
	dJointAttach(sensor2FingerTop, fingerTopCapsule.getBody(), finger[3]->getBody());
	dJointSetFixed(sensor2FingerTop);

	// �Z���T�ݒ�i�͂ƃg���N�̎擾�ɕK�v�j
	dJointSetFeedback(sensor2FingerTop, &force);


	//	�w����֐߂̓y��
	senkai_base_joint = dJointCreateFixed(sim->getWorld(), 0);
	dJointAttach(senkai_base_joint, senkai_base.getBody(), 0);
	dJointSetFixed(senkai_base_joint);

	//	�쓮���Ă���g���N�𑪒�
	dJointSetFeedback(senkai_base_joint, &senkai_force);

	// �q���W�W���C���g	�w����֐�
	senkai_link_joint = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(senkai_link_joint, senkai_link.getBody(), senkai_base.getBody());
	dJointSetHingeAnchor(senkai_link_joint, senkai_base_x0, senkai_base_y0, senkai_base_z0);
	dJointSetHingeAxis(senkai_link_joint, -1, 0, 0);
	dJointSetHingeParam(senkai_link_joint, dParamLoStop, -M_PI / 2.0);
	dJointSetHingeParam(senkai_link_joint, dParamHiStop, 0.0);


	// �Œ�W���C���g �w����֐߂̃����N->�w���{�֐�
	senkai_link2finger_joint = dJointCreateFixed(sim->getWorld(), 0);  // �Œ�W���C���g
	dJointAttach(senkai_link2finger_joint, finger[0]->getBody(), senkai_link.getBody());
	dJointSetFixed(senkai_link2finger_joint);
#if useForceContactPoint
	//�w��J�v�Z���ƃZ���T��ڑ�	//�Œ�W���C���g
	FingerTop2ForcePoint = dJointCreateFixed(sim->getWorld(), 0);  // �Œ�W���C���g
	dJointAttach(FingerTop2ForcePoint, forceContactPoint.getBody(), fingerTopCapsule.getBody());
	dJointSetFixed(FingerTop2ForcePoint);

	dJointSetFeedback(FingerTop2ForcePoint, &fingerTop2ForcePoint_joint);
#endif
	// �Z���T�ݒ�i�͂ƃg���N�̂ɕK�v�j
	//dJointSetFeedback(f2_joint, &force);


}
#endif