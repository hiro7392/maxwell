#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <stdio.h>
#include <math.h>
#include "texturepath.h"
#include "simMain.h"
#include "makeRobot.h"

// �O���ϐ�
extern dWorldID world;
extern dSpaceID space;
extern	dJointID f_joint, r_joint[ARM_JNT], f2_joint; // �Œ�֐߂Ɖ�]�֐�
extern	MyObject base, arm[ARM_JNT], sensor, obj;    // 
extern dJointFeedback force;

////////////////////////////////////////////////////////
// ���{�b�g�쐬
// ����2���R�x
////////////////////////////////////////////////////////
int createRobot(SIM *sim)
{
	dMass mass;
	dReal x0 = 0.0, y0 = 0.0, z0 = 1.5;
	dMatrix3	R;

	// �p�����[�^�ݒ�
#define	Z_OFFSET	0.08
	base.sides[CRD_X] = 0.3;	base.sides[CRD_Y] = 0.3;	base.sides[CRD_Z] = 0.4;	base.m = 14.0;	// �y�䒼����
	arm[ARM_M1].l = ARM_LINK1_LEN;	arm[ARM_M1].r = ARM_LINK1_RAD;	arm[ARM_M1].m = ARM_LINK1_MASS;	// �A�[�������N1�~��
	arm[ARM_M2].l = ARM_LINK2_LEN;	arm[ARM_M2].r = ARM_LINK2_RAD;	arm[ARM_M2].m = ARM_LINK2_MASS;	// �A�[�������N2�~��
	sensor.l = 0.0001;    // �����i�T�C�Y�̉e�����o�Ȃ��悤�ɏ����߂ɐݒ�j
	sensor.r = ARM_LINK2_RAD;
	sensor.m   = sensor.l/ARM_LINK2_LEN * ARM_LINK2_MASS;		// �A�[���Ɩ��x�����낦��

	// �y�䐶��
	base.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetBoxTotal(&mass, base.m, base.sides[CRD_X], base.sides[CRD_Y], base.sides[CRD_Z]);
	dBodySetMass(base.body, &mass);
	dBodySetPosition(base.body, x0, y0, base.sides[CRD_Z]/2);
	base.geom = dCreateBox(space, base.sides[CRD_X], base.sides[CRD_Y], base.sides[CRD_Z]);
	dGeomSetBody(base.geom, base.body);
	// �A�[�������N1����
	arm[ARM_M1].body   = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, arm[ARM_M1].m, DIR_LONG_AXIS_Z, arm[ARM_M1].r, arm[ARM_M1].l);
	dBodySetMass(arm[ARM_M1].body, &mass);
	dBodySetPosition(arm[ARM_M1].body, x0+arm[ARM_M1].l/2.0*cos(sim->init_jnt_pos[ARM_M1]), y0+arm[ARM_M1].l/2.0*sin(sim->init_jnt_pos[ARM_M1]), base.sides[CRD_Z]/2.0-Z_OFFSET);
	dRFromAxisAndAngle(R, -sin(sim->init_jnt_pos[ARM_M1]), cos(sim->init_jnt_pos[ARM_M1]), 0, PI/2);
	dBodySetRotation(arm[ARM_M1].body, R);
	arm[ARM_M1].geom = dCreateCylinder(space, arm[ARM_M1].r, arm[ARM_M1].l);
	dGeomSetBody(arm[ARM_M1].geom, arm[ARM_M1].body);
	// �A�[�������N2����
	arm[ARM_M2].body   = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, arm[ARM_M2].m, DIR_LONG_AXIS_Z, arm[ARM_M2].r, arm[ARM_M2].l);
	dBodySetMass(arm[ARM_M2].body, &mass);
	dBodySetPosition(arm[ARM_M2].body, x0+arm[ARM_M1].l*cos(sim->init_jnt_pos[ARM_M1])+arm[ARM_M2].l/2.0*cos(sim->init_jnt_pos[ARM_M1]+sim->init_jnt_pos[ARM_M2]), y0+arm[ARM_M1].l*sin(sim->init_jnt_pos[ARM_M1])+arm[ARM_M2].l/2.0*sin(sim->init_jnt_pos[ARM_M1]+sim->init_jnt_pos[ARM_M2]), base.sides[CRD_Z]/2.0-Z_OFFSET);
	dRFromAxisAndAngle(R, -sin(sim->init_jnt_pos[ARM_M1]+sim->init_jnt_pos[ARM_M2]), cos(sim->init_jnt_pos[ARM_M1]+sim->init_jnt_pos[ARM_M2]), 0, PI/2);
	dBodySetRotation(arm[ARM_M2].body, R);
	arm[ARM_M2].geom = dCreateCylinder(space, arm[ARM_M2].r, arm[ARM_M2].l);
	dGeomSetBody(arm[ARM_M2].geom, arm[ARM_M2].body);
	// �Z���T�i���ɕt�����C�T�C�Y�̉e�����o�Ȃ��悤�ɏ����߂ɐݒ�j
	sensor.body = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, sensor.m, DIR_LONG_AXIS_Z, sensor.r, sensor.l);
	dBodySetMass(sensor.body, &mass);
	dBodySetPosition(sensor.body, x0+arm[ARM_M1].l*cos(sim->init_jnt_pos[ARM_M1])+(arm[ARM_M2].l+sensor.l/2.0)*cos(sim->init_jnt_pos[ARM_M1]+sim->init_jnt_pos[ARM_M2]), y0+arm[ARM_M1].l*sin(sim->init_jnt_pos[ARM_M1])+(arm[ARM_M2].l+sensor.l/2.0)*sin(sim->init_jnt_pos[ARM_M1]+sim->init_jnt_pos[ARM_M2]), base.sides[CRD_Z]/2.0-Z_OFFSET);
	dRFromAxisAndAngle(R, -sin(sim->init_jnt_pos[ARM_M1]+sim->init_jnt_pos[ARM_M2]), cos(sim->init_jnt_pos[ARM_M1]+sim->init_jnt_pos[ARM_M2]), 0, PI/2);
	dBodySetRotation(sensor.body, R);
	sensor.geom = dCreateCylinder(space, sensor.r, sensor.l);
	dGeomSetBody(sensor.geom, sensor.body);
	// �Œ�W���C���g
	f_joint = dJointCreateFixed(world, 0);
	dJointAttach(f_joint, base.body, 0);
	dJointSetFixed(f_joint);
	// �q���W�W���C���g1
	r_joint[ARM_M1] = dJointCreateHinge(world, 0);
	dJointAttach(r_joint[ARM_M1], arm[ARM_M1].body, base.body);
	dJointSetHingeAnchor(r_joint[ARM_M1], x0, y0, base.sides[CRD_Z]/2);
	dJointSetHingeAxis(r_joint[ARM_M1], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);
//	dJointSetHingeParam(r_joint[ARM_M1], dParamFudgeFactor, 0.1);
//	dJointSetHingeParam(r_joint[ARM_M1], dParamStopERP, 0.8);
	// �q���W�W���C���g2
	r_joint[ARM_M2] = dJointCreateHinge(world, 0);
	dJointAttach(r_joint[ARM_M2], arm[ARM_M2].body, arm[ARM_M1].body);
	dJointSetHingeAnchor(r_joint[ARM_M2], x0+arm[ARM_M1].l*cos(sim->init_jnt_pos[ARM_M1]), y0+arm[ARM_M1].l*sin(sim->init_jnt_pos[ARM_M1]), base.sides[CRD_Z]/2);
	dJointSetHingeAxis(r_joint[ARM_M2], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M2], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M2], dParamHiStop, M_PI);
//	dJointSetHingeParam(r_joint[ARM_M2], dParamFudgeFactor, 0.1);
//	dJointSetHingeParam(r_joint[ARM_M2], dParamStopERP, 0.8);
	// �Œ�W���C���g
	f2_joint = dJointCreateFixed(world, 0);  // �Œ�W���C���g
	dJointAttach(f2_joint, sensor.body, arm[ARM_M2].body);
	dJointSetFixed(f2_joint);
	// �Z���T�ݒ�i�͂ƃg���N�̎擾�ɕK�v�j
	dJointSetFeedback(f2_joint, &force);

	return	0;
}

////////////////////////////////////////////////////////
// ���{�b�g�`��
////////////////////////////////////////////////////////
int drawRobot()
{
	const dReal *pos, *R;
	//  const dReal sides[3] = {0.5,0.5,0.5};

	// �y��`��
	dsSetColor(1.0, 0.0, 0.0);                       // �ԐF
	pos = dBodyGetPosition(base.body);
	R   = dBodyGetRotation(base.body);
	dsDrawBox(pos, R, base.sides);
	// �A�[�������N1�`��
	pos = dBodyGetPosition(arm[ARM_M1].body);
	R   = dBodyGetRotation(arm[ARM_M1].body);
	dsSetColor(0.0,0.0,1.0);                    // �F
	dsDrawCylinder(pos, R, arm[ARM_M1].l, arm[ARM_M1].r);
	// �A�[�������N2�`��
	pos = dBodyGetPosition(arm[ARM_M2].body);
	R   = dBodyGetRotation(arm[ARM_M2].body);
	dsSetColor(0.0,0.5,0.5);                    // �F
	dsDrawCylinder(pos, R, arm[ARM_M2].l, arm[ARM_M2].r);
	//      dsSetColor(1.2,1.2,1.2);                   // ���F
	//      dsDrawCylinder(pos2, R2, arm.l, arm.r);
	// �Z���T�`��
	pos = dBodyGetPosition(sensor.body);
	R   = dBodyGetRotation(sensor.body);
	dsSetColor(0.0,0.5,0.5);                    // 
	dsDrawCylinder(pos, R, sensor.l, sensor.r);
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
int drawExtForce()
{
	int crd, width;
	dJointFeedback *fb;
	dVector3	p_s, p_e;    // ���̎n�_�ƏI�_
	double k1 = 0.3;  // �����̔��萔
	double line_w = 0.05;
	dVector3	arrow_center, arrow_r, arrow_l;    // ���̒��_
	dVector3	rect_ul, rect_ll, rect_ur, rect_lr;    // ���̒��_
	dVector3	line, line_e;    // 
	dVector3	ext_f;	// �O��
	dMatrix3	R;
	double angArrow = PI/6;  //���̊p�x rad
	dJointFeedback *p_force;
	
	dBodyGetRelPointPos(sensor.body, 0.0, 0.0, sensor.l/2.0, p_s);			// ���ʒu
//		endP[0] = pos[0] + k1*sensor[jnt].f1[0];
//		endP[1] = pos[1] + k1*sensor[jnt].f1[1];
//		endP[2] = pos[2] + k1*sensor[jnt].f1[2];
	p_force = dJointGetFeedback(f2_joint);
	for(crd=0;crd<DIM3;crd++)	ext_f[crd] = -p_force->f1[crd];	// �Ώۂ��Z���T�ɋy�ڂ��Ă����=�Z���T���֐߂ɋy�ڂ��Ă����
	p_s[CRD_Z] += sensor.r; //�r�̏�ɕ\��
	for(crd=0;crd<DIM3;crd++)	p_e[crd] = p_s[crd] - k1*ext_f[crd];
//	p_e[CRD_Z] = p_s[CRD_Z] + sensor.r;	// �r�̏�ɕ\��
	p_e[CRD_Z] = p_s[CRD_Z];	// z�����̗͖͂���
	dsSetColor(1.0, 1.0, 1.0);                    // 
#if 1
//	arrow_l[CRD_X] = p_s[CRD_X]+k1/2*fb->f1[CRD_X]-k1/2*fb->f1[CRD_Y];
//	arrow_l[CRD_Y] = p_s[CRD_Y]+k1/2*fb->f1[CRD_Y]+k1/2*fb->f1[CRD_X];
//	arrow_l[CRD_Z] = p_s[CRD_Z];
//	arrow_r[CRD_X] = p_s[CRD_X]+k1/2*fb->f1[CRD_X]+k1/2*fb->f1[CRD_Y];
//	arrow_r[CRD_Y] = p_s[CRD_Y]+k1/2*fb->f1[CRD_Y]-k1/2*fb->f1[CRD_X];
//	arrow_r[CRD_Z] = p_s[CRD_Z];
	arrow_center[CRD_X] = 0;	arrow_center[CRD_Y] = 0.0;	arrow_center[CRD_Z] = 0;
	arrow_l[CRD_X] = sin(angArrow)* ext_f[0] * k1 / 3;
	arrow_l[CRD_Y] = cos(angArrow)* ext_f[0] * k1 / 3;
	arrow_l[CRD_Z] = 0;
	arrow_r[CRD_X] = -sin(angArrow)* ext_f[0] * k1 / 3;
	arrow_r[CRD_Y] = cos(angArrow)* ext_f[0] * k1 / 3;
	arrow_r[CRD_Z] = 0;
	rect_ul[CRD_X] = rect_ll[CRD_X] = sin(angArrow)* ext_f[0] * k1 / 8;
	rect_ur[CRD_X] = rect_lr[CRD_X] = -sin(angArrow)* ext_f[0] * k1 / 8;
	rect_ul[CRD_Y] = rect_ur[CRD_Y] = cos(angArrow)* ext_f[0] * k1 / 3;
	rect_ll[CRD_Y] = rect_lr[CRD_Y] = k1 * ext_f[0];
	rect_ul[CRD_Z] = rect_ll[CRD_Z] = rect_ur[CRD_Z] = rect_lr[CRD_Z] = 0.0;

	dRFromAxisAndAngle(R, 0, 0, 1, PI - atan2(p_s[CRD_Y] - p_e[CRD_Y], p_s[CRD_X] - p_e[CRD_X]));
//	printf("%f %f %f\n\n", rect_ll[0],rect_ll[1],rect_ll[2]);

	// ���̓�
	dsDrawTriangle(p_s, R, arrow_center, arrow_l, arrow_r, 1); // 
	// ���̐��i�O�p�`��2���킹�ĕ��̎������l�p�`�Ƃ��ĕ\���j
	dsDrawTriangle(p_s, R, rect_ul, rect_ll, rect_ur, 1); // 
	dsDrawTriangle(p_s, R, rect_ur, rect_ll, rect_lr, 1); // 

//	dsDrawLine(p_s, p_e); // p_s����p_e�܂ł̒�����`��
//	dsDrawLine(line_center, p_e); // p_s����p_e�܂ł̒�����`��
#endif
	return	0;
}

////////////////////////////////////////////////////////
// ���{�b�g�j��
////////////////////////////////////////////////////////
int destroyRobot()
{
	// �W���C���g�j��
	dJointDestroy(f_joint);   // �y��Œ�
	dJointDestroy(r_joint[ARM_M1]);   // �A�[��
	dJointDestroy(r_joint[ARM_M2]);   // �A�[��
	dJointDestroy(f2_joint);   // �Z���T�Œ�
	// �{�f�B�j��
	dBodyDestroy(base.body); // �y��
	dBodyDestroy(arm[ARM_M1].body);  // �A�[��
	dBodyDestroy(arm[ARM_M2].body);  // �A�[��
	dBodyDestroy(sensor.body);  // �Z���T
	// �W�I���g���j��
	dGeomDestroy(base.geom); // �y��
	dGeomDestroy(arm[ARM_M1].geom);  // �A�[��
	dGeomDestroy(arm[ARM_M2].geom);  // �A�[��
	dGeomDestroy(sensor.geom);  // �Z���T
	return	0;
}

////////////////////////////////////////////////////////
// �Ώۍ쐬
////////////////////////////////////////////////////////
int createObject(SIM *sim)
{
	dMass mass;
	dMatrix3	R;
	// �p�����[�^�ݒ�
//	obj.l = 0.15;	obj.r = 0.10;	obj.m = 0.5;	// �~��
	obj.l = 0.15;	obj.r = 0.10;	obj.m = 0.2;	// �~��
#if 1
	// �~��
	obj.body   = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetCylinderTotal(&mass, obj.m, DIR_LONG_AXIS_Z, obj.r, obj.l);
	dBodySetMass(obj.body, &mass);
	dBodySetPosition(obj.body, sim->init_obj_pos[CRD_X], sim->init_obj_pos[CRD_Y], sim->init_obj_pos[CRD_Z]);
//	dRFromAxisAndAngle(R, 1, 0, 0, PI/2);	// ������y�����ɉ�]
//	dRFromAxisAndAngle(R, 0, 1, 0, PI/2);	// ������x�����ɉ�]
	dRFrom2Axes(R, sim->init_obj_att[AXIS_X][CRD_X], sim->init_obj_att[AXIS_X][CRD_Y], sim->init_obj_att[AXIS_X][CRD_Z], sim->init_obj_att[AXIS_Y][CRD_X], sim->init_obj_att[AXIS_Y][CRD_Y], sim->init_obj_att[AXIS_Y][CRD_Z]);
	dBodySetRotation(obj.body, R);
	obj.geom = dCreateCylinder(space, obj.r, obj.l);
	dGeomSetBody(obj.geom, obj.body);
#else
	// ��
	obj.body   = dBodyCreate(world);
	dMassSetZero(&mass);
	dMassSetSphereTotal(&mass, obj.m, obj.r);
	dBodySetMass(obj.body, &mass);
	dBodySetPosition(obj.body, sim->init_obj_pos[CRD_X], sim->init_obj_pos[CRD_Y], sim->init_obj_pos[CRD_Z]);
	obj.geom = dCreateSphere(space, obj.r);
	dGeomSetBody(obj.geom, obj.body);
#endif
	return	0;
}

////////////////////////////////////////////////////////
// �Ώە`��
////////////////////////////////////////////////////////
int drawObject()
{
	int	i;
	const dReal *pos,*R;
	// �ՓˑΏە`��
	pos = dBodyGetPosition(obj.body);
	R   = dBodyGetRotation(obj.body);
	dsSetColor(0.5, 0.5, 1.0);                    // 
#if 1	// �~��
	dsDrawCylinder(pos, R, obj.l, obj.r);
#else	// ��
	dsDrawSphere(pos, R, obj.r);
#endif
	return	0;
}

////////////////////////////////////////////////////////
// �Ώ۔j��
////////////////////////////////////////////////////////
int destroyObject()
{
	// �{�f�B�j��
	dBodyDestroy(obj.body);
	// �W�I���g���j��
	dGeomDestroy(obj.geom);
	return	0;
}