#pragma once
int cFinger::armDynPara() {
	int	jnt;
	//static double	m1, m2, l1, l2, lg1, lg2, I1, I2;
	double	m1, m2, l1, l2, lg1, lg2, I1, I2;

	double	C1, C2, S1, S2, C12, S12;
	auto entity = EntityManager::get();
	const int ARM_JNT_NUM = 3;
	// �p�����[�^�ݒ�
	//if(entity->step == 0){
	m1 = this->dyn.m[ARM_M1]; m2 = this->dyn.m[ARM_M2];
	l1 = this->kine.l[ARM_M1]; l2 = this->kine.l[ARM_M2];
	lg1 = this->kine.lg[ARM_M1]; lg2 = this->kine.lg[ARM_M2];
	I1 = (this->kine.r[ARM_M1] * this->kine.r[ARM_M1] / 4 + l1 * l1 / 12) * m1;
	I2 = (this->kine.r[ARM_M2] * this->kine.r[ARM_M2] / 4 + l2 * l2 / 12) * m2;
	//}
	// �O�p�֐�
	C1 = cos(this->var.q.el[0][0]); C2 = cos(this->var.q.el[1][0]); S1 = sin(this->var.q.el[0][0]); S2 = sin(this->var.q.el[1][0]);
	C12 = cos(this->var.q.el[0][0] + this->var.q.el[1][0]); S12 = sin(this->var.q.el[0][0] + this->var.q.el[1][0]);
	// �����s��
	this->dyn.Mq.el[0][0] = m1 * lg1 * lg1 + I1 + m2 * (l1 * l1 + lg2 * lg2 + 2 * l1 * lg2 * C2) + I2;
	this->dyn.Mq.el[0][1] = this->dyn.Mq.el[1][0] = m2 * (lg2 * lg2 + l1 * lg2 * C2) + I2;
	this->dyn.Mq.el[1][1] = m2 * lg2 * lg2 + I2;
	// ���S�E�R���I����
	this->dyn.h.el[0][0] = -m2 * l1 * lg2 * S2 * (this->var.dq.el[1][0] * this->var.dq.el[1][0] + 2 * this->var.dq.el[0][0] * this->var.dq.el[1][0]);
	this->dyn.h.el[1][0] = m2 * l1 * lg2 * S2 * this->var.dq.el[0][0] * this->var.dq.el[0][0];
	// �֐ߔS�����C�͂�h�ɒǉ�
	//for(jnt=0;jnt<ARM_JNT;jnt++)	this->dyn.h.el[jnt][0] += this->dyn.V[jnt] * this->jnt_vel[jnt];	//�I���W�i��
	for (jnt = 0; jnt < ARM_JNT; jnt++)	this->dyn.h.el[jnt][0] -= this->dyn.V[jnt] * this->jnt_vel[jnt];

	// ���R�r�A�� 2�����o�[�W����
	this->kine.J.el[0][0] = -(l1 * S1 + l2 * S12);	this->kine.J.el[0][1] = -l2 * S12;
	this->kine.J.el[1][0] = l1 * C1 + l2 * C12;	this->kine.J.el[1][1] = l2 * C12;


	// �����s�����
	this->dyn.dMq.el[0][0] = -2 * m2 * l1 * lg2 * S2 * this->var.dq.el[1][0];
	this->dyn.dMq.el[0][1] = this->dyn.dMq.el[1][0] = -m2 * l1 * lg2 * S2 * this->var.dq.el[1][0];
	this->dyn.dMq.el[1][1] = 0.0;
	// ���R�r�A������
	this->kine.dJ.el[0][0] = -this->eff_vel[CRD_Y]; this->kine.dJ.el[0][1] = -l2 * C12 * (this->var.dq.el[0][0] + this->var.dq.el[1][0]);
	this->kine.dJ.el[1][0] = this->eff_vel[CRD_X]; this->kine.dJ.el[1][1] = -l2 * S12 * (this->var.dq.el[0][0] + this->var.dq.el[1][0]);
	// ���R�r�A���]�u�C���R�r�A���t�s��
	matTrans(&this->kine.Jt, &this->kine.J);		// J^{T}
	matInv(&this->kine.Jinv, NULL, &this->kine.J);		// J^{-1}�i�����̏ꍇ�̂ݑΉ��j

	//�@�d�͍��̌v�Z
	//  �v�Z�����d�͍���h�ɑ������킹��
	// 
	// ����֐߂̓��I�p�����[�^
	const dReal* senkai_base_pos = dBodyGetPosition(this->senkai_base.getBody());

	//	���񃊃��N�̏d�S�ʒu
	const dReal* senkai_link_pos = dBodyGetPosition(this->senkai_link.getBody());

	//	�����N1�̏d�S�ʒu
	const dReal* link1_pos = dBodyGetPosition(this->link1.getBody());
	//	�����N2�̏d�S�ʒu
	const dReal* link2_pos = dBodyGetPosition(this->link2.getBody());
	//	�J�v�Z���̏d�S�ʒu
	const dReal* capsule_pos = dBodyGetPosition(this->fingerTopCapsule.getBody());
	//	�Z���T�̏d�S�ʒu
	const dReal* sensor_pos = dBodyGetPosition(this->sensor.getBody());

	//	����֐ߊp�Ɋւ��銵�����[�����g���߂�

	//	����֐ߊp����̋������v�Z����
	static double link1_dist, link2_dist, capsule_dist, sensor_dist;
	link1_dist = getDistPlain(senkai_base_pos[0], link1_pos[0], senkai_base_pos[1], link1_pos[1]);
	link2_dist = getDistPlain(senkai_base_pos[0], link2_pos[0], senkai_base_pos[1], link2_pos[1]);
	capsule_dist = getDistPlain(senkai_base_pos[0], capsule_pos[0], senkai_base_pos[1], capsule_pos[1]);
	sensor_dist = getDistPlain(senkai_base_pos[0], sensor_pos[0], senkai_base_pos[1], sensor_pos[1]);

	// �e�����N�̐ړ_����̋����~����
	static double senkai_link_Ix, base_Ix, link1_Ix, link2_Ix, capsule_Ix, sensor_Ix;
	senkai_link_Ix = senkai_link.getMass() * SENKAI_LINK_LEN / 2.0;
	base_Ix = base.getMass() * SENKAI_LINK_LEN;
	link1_Ix = link1_dist * link1.getMass();
	link2_Ix = link2_dist * link2.getMass();
	capsule_Ix = capsule_dist * fingerTopCapsule.getMass();
	sensor_Ix = sensor_dist * sensor.getMass();
	//	�e�֐߂ɂ��ā@(zy���ʂɂ����鋗��r^2)*����m �𑫂����킹��
	this->rotImp.Iq = senkai_link_Ix + base_Ix + link1_Ix + link2_Ix + capsule_Ix + sensor_Ix;
	printf("rotImp.Iq =%lf\n ", this->rotImp.Iq);

	//	�����s��̌v�Z
	setTransMatrix();

	//	�e�����N�̎��ʒ��S��ݒ�
	setMassCenterPosition(imp.s1, link1_pos[0], link1_pos[1], link1_pos[2]);
	setMassCenterPosition(imp.s2, link2_pos[0], link2_pos[1], link2_pos[2]);
	setMassCenterPosition(imp.ss, senkai_link_pos[0], senkai_link_pos[1], senkai_link_pos[2]);

	//MatPrintDebug4x1(imp.s1, "s1");
	//MatPrintDebug4x1(imp.s2, "s2");
	//MatPrintDebug4x1(imp.ss, "ss");

	//	�e�֐߂ɂ��ďd�͍����v�Z
	calculateGravity();
	printf("fingerID = %d\n", fingerID);
	MatPrintDebug3x1(imp.G, "G");
	//	�d�͍���h�ɑ���
	dyn.h.el[0][0] += imp.G.el[0][0];
	dyn.h.el[1][0] += imp.G.el[0][1];
	rotImp.h += imp.G.el[0][2];
	return	0;
}