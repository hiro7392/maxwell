//
//////////////////////////////////////////////////////////
//// ���̒�`
//////////////////////////////////////////////////////////
//cParts::cParts(dReal m) : m(m) {
//	this->body = dBodyCreate(EntityManager::get()->getWorld()); dMassSetZero(&mass);
//	//this->geom = dCreateBox(EntityManager::get()->getSpace(), 1.0, 1.0, 1.0);
//}
//
//
//cParts::cParts(dReal m, Vec3 init_pos) : cParts(m) {
//	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
//}
//
//cPartsBox::cPartsBox(dReal m, Vec3 l) : cParts(m), sides{ l.x, l.y, l.z } {
//	dMassSetBoxTotal(&this->mass, this->m, this->sides[CRD_X], this->sides[CRD_Y], this->sides[CRD_Z]);
//	dBodySetMass(this->body, &mass);
//	this->geom = dCreateBox(EntityManager::get()->getSpace(), this->sides[CRD_X], this->sides[CRD_Y], this->sides[CRD_Z]);
//	dGeomSetBody(this->geom, this->body);
//}
//
//cPartsBox::cPartsBox(dReal m, Vec3 init_pos, Vec3 l) : cPartsBox(m, l) {
//	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
//	//this->geom = dCreateBox(EntityManager::get()->getSpace(), init_pos.x, init_pos.y, init_pos.z);
//}
//
//cPartsCylinder::cPartsCylinder(dReal m, dReal l, dReal r) : cParts(m), l(l), r(r) {
//
//	dMassSetCylinderTotal(&this->mass, this->m, DIR_LONG_AXIS_Z, this->r, this->l);
//	dBodySetMass(this->body, &mass);
//	this->geom = dCreateCylinder(EntityManager::get()->getSpace(), this->r, this->l);
//	dGeomSetBody(this->geom, this->body);
//}
//
//cPartsCapsule::cPartsCapsule(dReal m, dReal l, dReal r) : cParts(m), l(l), r(r) {
//
//	//	��������J�v�Z���֘A
//	dMassSetZero(&mass);
//
//	//	Cylinder�̕����Ƃقړ���
//	//this->body = dBodyCreate(EntityManager::get()->getWorld());
//	dMassSetCapsuleTotal(&this->mass, this->m, DIR_LONG_AXIS_Z, this->r, this->l);
//	dBodySetMass(this->body, &mass);
//
//	//	�W�I���g���̐���
//	this->geom = dCreateCapsule(EntityManager::get()->getSpace(), this->r, this->l);
//	dGeomCapsuleSetParams(this->geom, this->r, this->l);
//	//�W�I���g���ƃ{�f�B�̑Ή��t��
//	dGeomSetBody(this->geom, this->body);
//
//}
//
//cPartsCylinder::cPartsCylinder(dReal m, Vec3 init_pos, dReal l, dReal r) : cPartsCylinder(m, l, r) {		// �f�t�H���g�R���X�g���N�^
//	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
//}
//
////�e�֐߂�����
////HingeJoint:=�ғ�����֐�
//void cFinger::setJoint() {
//
//	auto sim = EntityManager::get();
//
//	// �Œ�W���C���g
//	f_joint = dJointCreateFixed(sim->getWorld(), 0);
//	dJointAttach(f_joint, finger[0]->getBody(), 0);
//	dJointSetFixed(f_joint);
//	// �q���W�W���C���g1
//	r_joint[ARM_M1] = dJointCreateHinge(sim->getWorld(), 0);
//	dJointAttach(r_joint[ARM_M1], finger[1]->getBody(), finger[0]->getBody());
//	dJointSetHingeAnchor(r_joint[ARM_M1], x0, y0, 0.4 / 2);
//	dJointSetHingeAxis(r_joint[ARM_M1], 0, 0, 1);
//	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
//	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);
//	// �q���W�W���C���g2
//	r_joint[ARM_M2] = dJointCreateHinge(sim->getWorld(), 0);
//	dJointAttach(r_joint[ARM_M2], finger[2]->getBody(), finger[1]->getBody());
//	dJointSetHingeAnchor(r_joint[ARM_M2], x0 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]), y0 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]), 0.4 / 2);
//	dJointSetHingeAxis(r_joint[ARM_M2], 0, 0, 1);
//	dJointSetHingeParam(r_joint[ARM_M2], dParamLoStop, -M_PI);
//	dJointSetHingeParam(r_joint[ARM_M2], dParamHiStop, M_PI);
//
//	// �Œ�W���C���g
//	f2_joint = dJointCreateFixed(sim->getWorld(), 0);  // �Œ�W���C���g
//	dJointAttach(f2_joint, finger[3]->getBody(), finger[2]->getBody());
//	dJointSetFixed(f2_joint);
//
//	// �Z���T�p�̌Œ�W���C���g
//	//f3_joint = dJointCreateFixed(sim->getWorld(), 0);  // �Œ�W���C���g
//	//dJointAttach(f3_joint, finger[4]->getBody(), finger[3]->getBody());
//	//dJointSetFixed(f3_joint);
//
//
//	// �Z���T�ݒ�i�͂ƃg���N�̂ɕK�v�j
//	dJointSetFeedback(f2_joint, &force);
//}
////�c�����̂̏����ʒu
//dReal capX = -2.0, capY = -0.5, capZ = 0.3;
////
//const dReal plateX = 1.5, plateY = 0.58, plateZ = 0.4;
////��{�ڂ̎w�̏����ʒu�ݒ�
//void cFinger::setJoint2() {
//
//	//	�ڐG��������邽�߂̃J�v�Z���̐���
//	capsule.body = dBodyCreate(EntityManager::get()->getWorld());
//	dMassSetCapsule(&mass, DENSITY, 3, ARM_LINK2_RAD, ARM_LINK2_LEN);
//	dMass massPlate;
//	dReal newMass = 0.5;
//	dMassSetZero(&massPlate);
//	dMassSetCapsuleTotal(&massPlate, newMass, 3, ARM_LINK2_RAD, ARM_LINK2_LEN);				//����
//	dBodySetPosition(capsule.body, capX, capY, capZ);	//�ʒu
//	auto geomBodySample = dCreateCapsule(EntityManager::get()->getSpace(), ARM_LINK2_RAD, ARM_LINK2_LEN);
//	//	���͊wBody�ƏՓˌv�Z�W�I���g���̑Ή�
//	dGeomSetBody(geomBodySample, capsule.body);
//
//	//���ۂɔc������p�̃v���[�g�̐���
//	plateToGrasp.body = dBodyCreate(EntityManager::get()->getWorld());
//	dMassSetBox(&mass, DENSITY, plateX, plateY, plateZ);
//
//	//Body�ňʒu�Ǝ��ʂ̐ݒ�
//	dMassSetZero(&massPlate);
//	dMassSetBoxTotal(&massPlate, newMass, plateX, plateY, plateZ);
//	dBodySetPosition(plateToGrasp.body, -0.8, capY, capZ);	//�ʒu //x=-1.1
//	//�W�I���g���̐���
//	auto geomBodyPlate = dCreateBox(EntityManager::get()->getSpace(), plateX, plateY, plateZ);
//	//	���͊wBody�ƏՓˌv�Z�W�I���g���̑Ή�
//	dGeomSetBody(geomBodyPlate, plateToGrasp.body);
//
//
//
//	auto sim = EntityManager::get();
//
//	//�c�������ݒu(�ʒu�͉�)
//	graspObj = dJointCreateHinge(sim->getWorld(), 0);
//	dJointAttach(graspObj, plate.getBody(), 0);
//	dJointSetHingeAnchor(graspObj, px1, py1, pz1);
//
//	// �Œ�W���C���g
//	f_joint = dJointCreateFixed(sim->getWorld(), 0);
//	dJointAttach(f_joint, finger[0]->getBody(), 0);
//	dJointSetFixed(f_joint);
//
//	// �q���W�W���C���g1
//	r_joint[ARM_M1] = dJointCreateHinge(sim->getWorld(), 0);
//	dJointAttach(r_joint[ARM_M1], finger[1]->getBody(), finger[0]->getBody());
//	dJointSetHingeAnchor(r_joint[ARM_M1], x1, y1, 0.4 / 2);
//	dJointSetHingeAxis(r_joint[ARM_M1], 0, 0, 1);
//	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
//	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);
//
//	// �q���W�W���C���g2
//	r_joint[ARM_M2] = dJointCreateHinge(sim->getWorld(), 0);
//	dJointAttach(r_joint[ARM_M2], finger[2]->getBody(), finger[1]->getBody());
//	dJointSetHingeAnchor(r_joint[ARM_M2], x1 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]), y1 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]), 0.4 / 2);
//	dJointSetHingeAxis(r_joint[ARM_M2], 0, 0, 1);
//	dJointSetHingeParam(r_joint[ARM_M2], dParamLoStop, -M_PI);
//	dJointSetHingeParam(r_joint[ARM_M2], dParamHiStop, M_PI);
//
//	// �Œ�W���C���g
//	f2_joint = dJointCreateFixed(sim->getWorld(), 0);  // �Œ�W���C���g
//	dJointAttach(f2_joint, finger[3]->getBody(), finger[2]->getBody());
//	dJointSetFixed(f2_joint);
//
//	// �Z���T�ݒ�i�͂ƃg���N�̎擾�ɕK�v�j
//	dJointSetFeedback(f2_joint, &force);
//}