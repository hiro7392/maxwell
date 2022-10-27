#pragma once
////////////////////////////////////////////////////////
// ���̒�`
////////////////////////////////////////////////////////
cParts::cParts(dReal m) : m(m) {
	this->body = dBodyCreate(EntityManager::get()->getWorld()); dMassSetZero(&mass);
	//this->geom = dCreateBox(EntityManager::get()->getSpace(), 1.0, 1.0, 1.0);
}


cParts::cParts(dReal m, Vec3 init_pos) : cParts(m) {
	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
}

cPartsBox::cPartsBox(dReal m, Vec3 l) : cParts(m), sides{ l.x, l.y, l.z } {
	dMassSetBoxTotal(&this->mass, this->m, this->sides[CRD_X], this->sides[CRD_Y], this->sides[CRD_Z]);
	dBodySetMass(this->body, &mass);
	this->geom = dCreateBox(EntityManager::get()->getSpace(), this->sides[CRD_X], this->sides[CRD_Y], this->sides[CRD_Z]);
	dGeomSetBody(this->geom, this->body);
}

cPartsBox::cPartsBox(dReal m, Vec3 init_pos, Vec3 l) : cPartsBox(m, l) {
	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
	//this->geom = dCreateBox(EntityManager::get()->getSpace(), init_pos.x, init_pos.y, init_pos.z);
}

cPartsCylinder::cPartsCylinder(dReal m, dReal l, dReal r) : cParts(m), l(l), r(r) {

	dMassSetCylinderTotal(&this->mass, this->m, DIR_LONG_AXIS_Z, this->r, this->l);
	dBodySetMass(this->body, &mass);
	//�W�I���g���̐���
	this->geom = dCreateCylinder(EntityManager::get()->getSpace(), this->r, this->l);
	dGeomSetBody(this->geom, this->body);
}

cPartsCapsule::cPartsCapsule(dReal m, dReal l, dReal r) : cParts(m), l(l), r(r) {

	//	��������J�v�Z���֘A
	dMassSetZero(&mass);

	//	Cylinder�̕����Ƃقړ���
	//this->body = dBodyCreate(EntityManager::get()->getWorld());
	dMassSetCapsuleTotal(&this->mass, this->m, DIR_LONG_AXIS_Z, this->r, this->l);
	dBodySetMass(this->body, &mass);

	//	�W�I���g���̐���
	this->geom = dCreateCapsule(EntityManager::get()->getSpace(), this->r, this->l);
	dGeomCapsuleSetParams(this->geom, this->r, this->l);
	//�W�I���g���ƃ{�f�B�̑Ή��t��
	dGeomSetBody(this->geom, this->body);

}

cPartsCylinder::cPartsCylinder(dReal m, Vec3 init_pos, dReal l, dReal r) : cPartsCylinder(m, l, r) {		// �f�t�H���g�R���X�g���N�^
	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
}

//�e�֐߂�����
//HingeJoint:=�ғ�����֐�
void cFinger::setJoint() {

	auto sim = EntityManager::get();
	double base_x = senkai_base_x0;
	double base_y = senkai_base_y0 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN * cos(senkai_base_jnt);
	double base_z = senkai_base_z0 - SENKAI_LINK_LEN * sin(senkai_base_jnt);

	// �q���W�W���C���g1
	r_joint[ARM_M1] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M1], finger[1]->getBody(), finger[0]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M1], base_x,base_y,base_z);
	//dJointSetHingeAxis(r_joint[ARM_M1], 0, 0, 1);
	dJointSetHingeAxis(r_joint[ARM_M1], 0, -sin(-senkai_base_jnt), cos(-senkai_base_jnt));	//xy���ʏ�ŉ�]����Ƃ�

	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);

	double link1_top_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]);
	double link1_top_y = base_y + abs(cos(senkai_base_jnt)) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));
	double link1_top_z = base_z - sin(senkai_base_jnt) * abs(ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));

	// �q���W�W���C���g2
	r_joint[ARM_M2] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M2], finger[2]->getBody(), finger[1]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M2], link1_top_x,link1_top_y, link1_top_z);
	dJointSetHingeAxis(r_joint[ARM_M2], 0, -sin(-senkai_base_jnt), cos(-senkai_base_jnt));	//xy���ʏ�ŉ�]����Ƃ�

	//dJointSetHingeAxis(r_joint[ARM_M2], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M2], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M2], dParamHiStop, M_PI);

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
	dJointSetHingeParam(senkai_link_joint, dParamHiStop, M_PI / 2.0);

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
//�c�����̂̏����ʒu
dReal capX = -1.2, capY = -0.60, capZ = 0.3;//0.3(�񎟌�) z=1.5(3����)
//�c�����̂̑傫��
const dReal plateX = 1.5, plateY = 0.50, plateZ = 0.4;//z=1.2
// const dReal plateX = 1.5, plateY = 0.50, plateZ = 0.4;


//��{�ڂ̎w�̏����ʒu�ݒ�
void cFinger::setJoint2() {

	//���ۂɔc������p�̃v���[�g�̐���
#if usePlateToGrasp
	plateToGrasp.body = dBodyCreate(EntityManager::get()->getWorld());
	dMassSetBox(&mass, DENSITY, plateX, plateY, plateZ);

	//Body�ňʒu�Ǝ��ʂ̐ݒ�
	dMass massPlate;
	dReal newMass = 0.5;
	dMassSetZero(&massPlate);
	dMassSetBoxTotal(&massPlate, newMass, plateX, plateY, plateZ);
	dBodySetPosition(plateToGrasp.body, capX, capY, capZ);	//�ʒu //x=-1.1
	//�W�I���g���̐���
	auto geomBodyPlate = dCreateBox(EntityManager::get()->getSpace(), plateX, plateY, plateZ);
	//	���͊wBody�ƏՓˌv�Z�W�I���g���̑Ή�
	dGeomSetBody(geomBodyPlate, plateToGrasp.body);
#endif	
	auto sim = EntityManager::get();
	double base_x = senkai_base_x1;
	double base_y = senkai_base_y1 + (fingerID == 1 ? 1 : -1) * SENKAI_LINK_LEN * cos(senkai_base_jnt);
	double base_z = senkai_base_z1 - SENKAI_LINK_LEN * sin(senkai_base_jnt);

	// �q���W�W���C���g1
	r_joint[ARM_M1] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M1], finger[1]->getBody(), finger[0]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M1], base_x, base_y, base_z);
	//	����֐߂̕��Ax�����S�ɉ�]
	dJointSetHingeAxis(r_joint[ARM_M1], 0, -sin(senkai_base_jnt), cos(senkai_base_jnt));
	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);

	double link1_top_x = base_x + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]);
	double link1_top_y = base_y + abs(cos(senkai_base_jnt)) * (ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));
	double link1_top_z = base_z - sin(senkai_base_jnt) * abs(ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]));

	// �q���W�W���C���g2
	r_joint[ARM_M2] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M2], finger[2]->getBody(), finger[1]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M2], link1_top_x, link1_top_y, link1_top_z);
	//	����֐߂̕��Ax�����S�ɉ�]
	dJointSetHingeAxis(r_joint[ARM_M2], 0, -sin(senkai_base_jnt), cos(senkai_base_jnt));
	dJointSetHingeParam(r_joint[ARM_M2], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M2], dParamHiStop, M_PI);

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

	// �쓮���Ă���g���N�𑪒�
	dJointSetFeedback(senkai_base_joint, &senkai_force);

	// �q���W�W���C���g	�w����֐�
	senkai_link_joint = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(senkai_link_joint, senkai_link.getBody(), senkai_base.getBody());
	dJointSetHingeAnchor(senkai_link_joint, senkai_base_x1, senkai_base_y1, senkai_base_z1);
	dJointSetHingeAxis(senkai_link_joint, 1, 0, 0);
	dJointSetHingeParam(senkai_link_joint, dParamLoStop, -M_PI / 4.0);
	dJointSetHingeParam(senkai_link_joint, dParamHiStop, M_PI / 4.0);

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

	// �Z���T�ݒ�i�͂ƃg���N�̎擾�ɕK�v�j
	//dJointSetFeedback(f2_joint, &force);

}