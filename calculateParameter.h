#pragma once
void MatPrintDebug4x4(Matrix& mat, std::string name) {
	std::cout << name << "= " << std::endl;

	for (int col = 0; col < 4; col++) {
		for (int row = 0; row < 4; row++) {
			printf("%lf ", mat.el[row][col]);
		}
		printf("\n");
	}
	return;
}
// mat.el[��][�s]
void MatPrintDebug4x1(Matrix& mat, std::string name) {
	std::cout << name << "= " << std::endl;
	for (int col = 0; col < 4; col++) {
		for (int row = 0; row < 1; row++) {
			printf("%lf ", mat.el[row][col]);
		}
		printf("\n");
	}
	return;
}
void MatPrintDebug1x4(Matrix& mat, std::string name) {
	std::cout << name << "= " << std::endl;

	for (int col = 0; col < 1; col++) {
		for (int row = 0; row < 4; row++) {
			printf("%lf ", mat.el[row][col]);
		}
		printf("\n");
	}
	return;
}
void MatPrintDebug3x1(Matrix& mat, std::string name) {
	std::cout << name << " = " << std::endl;
	for (int col = 0; col < 3; col++) {
		for (int row = 0; row < 1; row++) {
			printf("%.5lf ", mat.el[row][col]);
		}
		printf("\n");
	}
	return;
}
void MatPrintDebugAll(Matrix& mat, std::string name,int Col,int Row) {
	std::cout << name << " = " << std::endl;
	for (int col = 0; col < Col; col++) {
		for (int row = 0; row < Row; row++) {
			printf("%lf ", mat.el[row][col]);
		}
		printf("\n");
	}
	return;
}
//	�����s��̌v�Z
int cFinger::setTransMatrix() {
	double sig = (fingerID == 1 ? 1.0 : -1.0);
	//	�����s��̌v�Z
	//	�����N1 1�s��
	this->imp.sT1.el[0][0] = cos(jnt_pos[0]);
	imp.sT1.el[1][0] = -sin(jnt_pos[0]);
	imp.sT1.el[2][0] = 0;
	imp.sT1.el[3][0] = ARM_LINK1_LEN * cos(jnt_pos[0])*sig;

	// 2�s��
	imp.sT1.el[0][1] = sin(jnt_pos[0]);
	imp.sT1.el[1][1] = cos(jnt_pos[0]);
	imp.sT1.el[2][1] = 0;
	imp.sT1.el[3][1] = ARM_LINK1_LEN * sin(jnt_pos[0]);

	// 3�s�ڂ�3��ڈȊO0
	imp.sT1.el[2][2] = 1.0;
	// 4�s�ڂ�4��ڈȊO���ׂ�0
	imp.sT1.el[3][3] = 1.0;

	//	�����N2
	//	1�s��
	imp.T12.el[0][0] = cos(jnt_pos[1]);
	imp.T12.el[1][0] = -sin(jnt_pos[1]);
	imp.T12.el[2][0] = 0;
	imp.T12.el[3][0] = ARM_LINK2_LEN * cos(jnt_pos[0] + jnt_pos[1]);

	// 2�s��
	imp.T12.el[0][1] = sin(jnt_pos[1]);
	imp.T12.el[1][1] = cos(jnt_pos[1]);
	imp.T12.el[2][1] = 0;
	imp.T12.el[3][1] = ARM_LINK2_LEN * sin(jnt_pos[0] + jnt_pos[1]);

	// 3�s�ڂ�3��ڈȊO0
	imp.T12.el[2][2] = 1.0;
	// 4�s�ڂ�4��ڈȊO���ׂ�0
	imp.T12.el[3][3] = 1.0;

	//	����֐�
	//	1�s��
	imp.oTs.el[0][0] = 1.0;
	//	2�s��
	imp.oTs.el[0][1] = 0.0;
	imp.oTs.el[1][1] = cos(senkai_base_jnt);
	imp.oTs.el[2][1] = -sin(senkai_base_jnt);
	imp.oTs.el[3][1] = SENKAI_LINK_LEN * cos(senkai_base_jnt);

	//	3�s��
	imp.oTs.el[0][2] = 0.0;
	imp.oTs.el[1][2] = sin(senkai_base_jnt);
	imp.oTs.el[2][2] = cos(senkai_base_jnt);
	imp.oTs.el[3][2] = SENKAI_LINK_LEN * (-sin(senkai_base_jnt));

	// 3�s�ڂ͂��ׂ�0�Ȃ̂ŉ������Ȃ�
	// 4�s�ڂ�4��ڈȊO���ׂ�0
	imp.oTs.el[3][3] = 1.0;
#if 0 //�f�o�b�O�p
	
#endif

	//	���񍪌�����̕ϊ��s����v�Z
	matMul(&imp.oT1, &imp.oTs, &imp.sT1);	//���_���烊���N1�̓����ϊ�
	matMul(&imp.oT2, &imp.oT1, &imp.T12);	//���_���烊���N2�̓����ϊ�
	//	MatPrintDebug4x4(imp.oT1, "oT1");
	//	MatPrintDebug4x4(imp.oT2, "oT2");


	return 0;
}
//	3�������W���󂯎���ăZ�b�g����(�s���4�~1)
int cFinger::setMassCenterPosition(Matrix& mat,double mx, double my, double mz) {
	mat.el[0][0] = mx;
	mat.el[0][1] = my;
	mat.el[0][2] = mz;
	mat.el[0][3] = 0.0;
#if 1 //�f�o�b�O�p
	//MatPrintDebug4x1(mat, "MassCenter=");
	
#endif
	return 0;
}
int cFinger::calculateGravity() {

	imp.G_xyz.el[2][0] = -9.8;
	//MatPrintDebug4x1(imp.G_xyz, "G_xyz");

	setTransMatrixDq();
	for (int i = 0; i < 3; i++)imp.G.el[0][i] = 0.0;
	//	�d�͍������߂�
	Matrix tmp,tmp2;
	matInit(&tmp, 1, 4); matInit(&tmp2, 1, 1);
	//	����֐߂ɂ���
	matMul(&tmp, &imp.ss, &imp.dqs_oTs);
	//	MatPrintDebug4x4(imp.dqs_oTs, "oTs");
	//	MatPrintDebug4x1(imp.ss, "ss");
	//	MatPrintDebug4x1(tmp, "tmp");
	//	MatPrintDebugAll(imp.G_xyz, "G_xyz",1,4);
	matMul(&tmp2, &tmp, &imp.G_xyz);
	//	MatPrintDebugAll(tmp2, "tmp2",1,1);
	imp.G.el[0][0] += (ARM_LINK2_MASS / 2.0) * tmp2.el[0][0];	//���񕔕��̃����N���ʂ�(ARM_LINK2_MASS / 2.0) 
	//	printf("G[0][0] = %lf\n", imp.G.el[0][0]);

	matMul(&tmp,  &imp.s1, &imp.dqs_oT1);
	matMul(&tmp2, &tmp ,&imp.G_xyz);
	//	MatPrintDebugAll(imp.G_xyz, "G_xyz", 1, 4);
	//	MatPrintDebugAll(tmp2, "tmp2", 1, 1);
	imp.G.el[0][0] += ARM_LINK1_MASS * tmp2.el[0][0];
	//	printf("G[0][0] = %lf\n", imp.G.el[0][0]);

	matMul(&tmp, &imp.s2 ,&imp.dqs_oT2);
	matMul(&tmp2, &tmp, &imp.G_xyz);
	//	MatPrintDebugAll(imp.G_xyz, "G_xyz", 1, 4);
	//	MatPrintDebugAll(tmp2, "tmp2", 1, 1);
	imp.G.el[0][0] += ARM_LINK2_MASS * tmp2.el[0][0];
	//	printf("G[0][0] = %lf\n", imp.G.el[0][0]);

	//	�֐�1�ɂ���
	matMul(&tmp, &imp.s1, &imp.dq1_oT1);
	matMul(&tmp2, &tmp ,&imp.G_xyz);
	imp.G.el[0][1] += ARM_LINK1_MASS * tmp2.el[0][0];

	matMul(&tmp, &imp.s2 ,&imp.dq1_oT2);
	matMul(&tmp2, &tmp ,&imp.G_xyz);
	imp.G.el[0][1] += ARM_LINK2_MASS * tmp2.el[0][0];
	//printf("G[0][1] = %lf\n", imp.G.el[0][1]);
	//	�֐߂Q�ɂ���
	matMul(&tmp, &imp.s2 ,&imp.dq2_oT2);
	matMul(&tmp2, &tmp ,&imp.G_xyz);
	imp.G.el[0][2] += ARM_LINK2_MASS * tmp2.el[0][0];
	//printf("G[0][2] = %lf\n", imp.G.el[0][2]);
	//MatPrintDebug3x1(imp.G, "G");

	matFree(&tmp);

	return 0;
}
//	�����s��̔����̌v�Z
int cFinger::setTransMatrixDq() {
	//	�s��ɂ���
	//	A[row][col]	A[��][�s]
	

	//	����֐߂Ɋւ������
	// oTs��1�s�ڂ͂��ׂ�0
	// 2�s��
	imp.dqs_oTs.el[0][1] = 0;
	imp.dqs_oTs.el[1][1] = -cos(senkai_base_jnt);
	imp.dqs_oTs.el[2][1] = sin(senkai_base_jnt);
	imp.dqs_oTs.el[3][1] = SENKAI_LINK_LEN * (fingerID == 1 ? (-1) : 1) * cos(senkai_base_jnt);

	// 3�s��
	imp.dqs_oTs.el[0][2] = 0;
	imp.dqs_oTs.el[1][2] = -sin(senkai_base_jnt);
	imp.dqs_oTs.el[2][2] = -cos(senkai_base_jnt);
	imp.dqs_oTs.el[3][2] = sin(senkai_base_jnt);

	// 4�s��
	imp.dqs_oTs.el[0][3] = sin(jnt_pos[0]);
	imp.dqs_oTs.el[1][3] = cos(jnt_pos[0]);
	imp.dqs_oTs.el[2][3] = 0;
	imp.dqs_oTs.el[3][3] = ARM_LINK1_LEN * sin(jnt_pos[0]);

	//	oT1�̐���֐߂Ɋւ������
	//	1�s�ڂ͂��ׂ�0

	// 2�s��
	imp.dqs_oT1.el[0][1] = -sin(jnt_pos[0])*sin(senkai_base_jnt);
	imp.dqs_oT1.el[1][1] = -cos(jnt_pos[0]) * sin(senkai_base_jnt);
	imp.dqs_oT1.el[2][1] = -cos(senkai_base_jnt);
	imp.dqs_oT1.el[3][1] = -ARM_LINK1_LEN*sin(jnt_pos[0])*sin(senkai_base_jnt) + SENKAI_LINK_LEN * (fingerID == 1 ? (-1) : 1) * sin(senkai_base_jnt);

	// 3�s��
	imp.dqs_oT1.el[0][2] = sin(jnt_pos[0]) * cos(senkai_base_jnt);
	imp.dqs_oT1.el[1][2] = cos(jnt_pos[0]) * cos(senkai_base_jnt);
	imp.dqs_oT1.el[2][2] = -sin(senkai_base_jnt);
	imp.dqs_oT1.el[3][2] = cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[0] - 1.0));

	// 4�s�ڂ͂��ׂ�0

	//	oT2�̐���֐߂Ɋւ������
	//	1�s�ڂ͂��ׂ�0
	imp.dqs_oT2.el[0][1] = -sin(senkai_base_jnt)*(sin(jnt_pos[0])*cos(jnt_pos[1])+ cos(jnt_pos[0]) * sin(jnt_pos[1]));
	imp.dqs_oT2.el[1][1] = sin(senkai_base_jnt) * (sin(jnt_pos[0]) * sin(jnt_pos[1]) - cos(jnt_pos[0]) * cos(jnt_pos[1]));
	double tmp1, tmp2;
	tmp1 = -ARM_LINK2_LEN * sin(senkai_base_jnt) * (sin(jnt_pos[0]) * cos(jnt_pos[0] + jnt_pos[1]) + cos(jnt_pos[0]) * sin(jnt_pos[0] + jnt_pos[1]));
	tmp2 = -ARM_LINK1_LEN * sin(jnt_pos[0]) * sin(senkai_base_jnt) + (fingerID == 1 ? (-1) : 1) * SENKAI_LINK_LEN * sin(senkai_base_jnt);

	imp.dqs_oT2.el[2][1] = cos(senkai_base_jnt);
	imp.dqs_oT2.el[3][1] = tmp1 + tmp2;
	// 3�s��
	imp.dqs_oT2.el[0][2] =	cos(senkai_base_jnt) * (sin(jnt_pos[0]) * cos(jnt_pos[1]) + cos(jnt_pos[0]) * sin(jnt_pos[1]));
	imp.dqs_oT2.el[1][2] =	cos(senkai_base_jnt) * (cos(jnt_pos[0]) * cos(jnt_pos[1]) - sin(jnt_pos[0]) * sin(jnt_pos[1]));
	imp.dqs_oT2.el[2][2] = -sin(senkai_base_jnt);
	tmp1 = -ARM_LINK2_LEN * cos(senkai_base_jnt) * (sin(jnt_pos[0]) * cos(jnt_pos[0] + jnt_pos[1]) + cos(jnt_pos[0]) * sin(jnt_pos[0] + jnt_pos[1]));
	tmp2 = cos(senkai_base_jnt)*(ARM_LINK1_LEN*sin(jnt_pos[0]-1.0));
	imp.dqs_oT2.el[3][2] = tmp1 + tmp2;
	// 4�s�ڂ͂��ׂ�0

	//�֐�1�Ɋւ������
	//dq1_oTs oTs�̊֐�1�Ɋւ��������0		�������ł��ł�0�ɂȂ��Ă���
	//dq1_oT1 oT1�̊֐�1�Ɋւ������
	imp.dq1_oT1.el[0][0] = -sin(jnt_pos[0]);
	imp.dq1_oT1.el[1][0] = -cos(jnt_pos[0]);
	imp.dq1_oT1.el[2][0] = 0.0;
	imp.dq1_oT1.el[3][0] = SENKAI_LINK_LEN  * sin(jnt_pos[0]);
	//	2�s��
	imp.dq1_oT1.el[0][1] = cos(senkai_base_jnt)*cos(jnt_pos[0]);
	imp.dq1_oT1.el[1][1] = -cos(senkai_base_jnt) * sin(jnt_pos[0]);
	imp.dq1_oT1.el[2][1] = 0.0;
	imp.dq1_oT1.el[3][1] = SENKAI_LINK_LEN  * cos(senkai_base_jnt)*cos(jnt_pos[0]);
	//	3�s��
	imp.dq1_oT1.el[0][2] = sin(senkai_base_jnt) * cos(jnt_pos[0]);
	imp.dq1_oT1.el[1][2] = -sin(senkai_base_jnt) * sin(jnt_pos[0]);
	imp.dq1_oT1.el[2][2] = 0.0;
	imp.dq1_oT1.el[3][2] = SENKAI_LINK_LEN * sin(senkai_base_jnt) * cos(jnt_pos[0]);

	//dq1_oT2 oT2�̊֐�1�Ɋւ������
	imp.dq1_oT2.el[0][0] = -sin(jnt_pos[0])*cos(jnt_pos[1])+ cos(jnt_pos[0]) * sin(jnt_pos[1]);
	imp.dq1_oT2.el[1][0] = -cos(jnt_pos[0]) * cos(jnt_pos[1]) + sin(jnt_pos[0]) * sin(jnt_pos[1]);
	imp.dq1_oT2.el[2][0] = 0.0;
	imp.dq1_oT2.el[3][0] = -2.0 * ARM_LINK2_LEN * sin(2 * jnt_pos[0] + jnt_pos[1]);

	imp.dq1_oT2.el[0][1] = cos(senkai_base_jnt) * (cos(jnt_pos[0]) * cos(jnt_pos[1]) - sin(jnt_pos[0]) * sin(jnt_pos[1]));
	imp.dq1_oT2.el[1][1] = -cos(senkai_base_jnt) * (cos(jnt_pos[0]) * sin(jnt_pos[1]) + sin(jnt_pos[0]) * cos(jnt_pos[1]));
	imp.dq1_oT2.el[2][1] = 0.0;
	imp.dq1_oT2.el[3][1] = 2.0 * ARM_LINK2_LEN * cos(senkai_base_jnt) * cos(2 * jnt_pos[0] + jnt_pos[1]) + ARM_LINK1_LEN * cos(senkai_base_jnt) * cos(jnt_pos[0]);

	imp.dq1_oT2.el[0][2] = sin(senkai_base_jnt) * (cos(jnt_pos[0]) * cos(jnt_pos[1]) - sin(jnt_pos[0]) * sin(jnt_pos[1]));
	imp.dq1_oT2.el[1][2] = -sin(senkai_base_jnt) * (cos(jnt_pos[0]) * sin(jnt_pos[1]) + sin(jnt_pos[0]) * cos(jnt_pos[1]));
	imp.dq1_oT2.el[2][2] = 0.0;
	imp.dq1_oT2.el[3][2] = 2.0 * ARM_LINK2_LEN * sin(senkai_base_jnt) * cos(2 * jnt_pos[0] + jnt_pos[1]) + ARM_LINK1_LEN * sin(senkai_base_jnt) * cos(jnt_pos[0]);

	//dq2_oTs oTs�̊֐�2�Ɋւ��������0		�������ł��ł�0�ɂȂ��Ă���
	//dq2_oT1 oT1�̊֐�2�Ɋւ��������0		�������ł��ł�0�ɂȂ��Ă���
	
	///dq2_oT2 oT2�̊֐�2�Ɋւ������
	imp.dq2_oT2.el[0][0] = -cos(jnt_pos[0]) * sin(jnt_pos[1]) + sin(jnt_pos[0]) * cos(jnt_pos[1]);
	imp.dq2_oT2.el[1][0] = -cos(jnt_pos[0]) * cos(jnt_pos[1]) + sin(jnt_pos[0]) * sin(jnt_pos[1]);
	imp.dq2_oT2.el[2][0] = 0.0;
	imp.dq2_oT2.el[3][0] = -2.0 * ARM_LINK2_LEN * sin(2 * jnt_pos[0] + jnt_pos[1]);

	imp.dq2_oT2.el[0][1] = cos(senkai_base_jnt) * (sin(jnt_pos[0]) * sin(jnt_pos[1])- cos(jnt_pos[0]) * cos(jnt_pos[1]));
	imp.dq2_oT2.el[1][1] = -cos(senkai_base_jnt) * (sin(jnt_pos[0]) * cos(jnt_pos[1]) + cos(jnt_pos[0]) * sin(jnt_pos[1]));
	imp.dq2_oT2.el[2][1] = 0.0;
	imp.dq2_oT2.el[3][1] = 2.0 * ARM_LINK2_LEN * cos(senkai_base_jnt) * cos(2 * jnt_pos[0] + jnt_pos[1]);

	imp.dq2_oT2.el[0][2] = sin(senkai_base_jnt) * (cos(jnt_pos[0]) * cos(jnt_pos[1]) - sin(jnt_pos[0]) * sin(jnt_pos[1]));
	imp.dq2_oT2.el[1][2] = -sin(senkai_base_jnt) * (cos(jnt_pos[0]) * sin(jnt_pos[1]) + sin(jnt_pos[0]) * cos(jnt_pos[1]));
	imp.dq2_oT2.el[2][2] = 0.0;
	imp.dq2_oT2.el[3][2] = 2.0 * ARM_LINK2_LEN * sin(senkai_base_jnt) * cos(2 * jnt_pos[0] + jnt_pos[1]);

	//	���񍪌�����̕ϊ��s����v�Z
	matMul(&imp.oT1, &imp.oTs, &imp.sT1);	//���_���烊���N1�̓����ϊ�
	matMul(&imp.oT2, &imp.oT1, &imp.T12);	//���_���烊���N2�̓����ϊ�
	//MatPrintDebug4x4(imp.dq1_oT1, "dq1_oT1");
	//MatPrintDebug4x4(imp.dq1_oT2, "dq1_oT2");
	//MatPrintDebug4x4(imp.dq2_oT1, "dq2_oT1");
	//MatPrintDebug4x4(imp.dq2_oT2, "dq2_oT2");
	//MatPrintDebug4x4(imp.dqs_oTs, "dqs_oTs");
	
	//	�����s��̊e�֐ߔ������i�[
	//	dqkoTi[i][k]�Ŋ֐�i�̓����s��oTi��qk�Ɋւ������
	imp.dqkoTi[0][0] = imp.dqs_oTs;	//dqkoTi[i][k]�Ŋ֐�i�̓����s��oTi��qk�Ɋւ������
	imp.dqkoTi[0][1] = imp.dq1_oTs;	
	imp.dqkoTi[0][2] = imp.dq2_oTs;	

	imp.dqkoTi[1][0] = imp.dqs_oT1;	//dqkoTi[i][k]�Ŋ֐�i�̓����s��oTi��qk�Ɋւ������
	imp.dqkoTi[1][1] = imp.dq1_oT1;
	imp.dqkoTi[1][2] = imp.dq2_oT1;

	imp.dqkoTi[2][0] = imp.dqs_oT2;	//dqkoTi[i][k]�Ŋ֐�i�̓����s��oTi��qk�Ɋւ������
	imp.dqkoTi[2][1] = imp.dq1_oT2;
	imp.dqkoTi[2][2] = imp.dq2_oT2;
#if INVERSE
	for (int i = 0; i < 3; i++) {
		for (int k = 0; k < 3; k++) {
			for (int l = 0; l < 3; l++) {
				matTrans(&imp.dqkoTi[i][k], &imp.dqkoTi[i][k]);
			}
		}
	}
#endif
	return 0;
}
// �����s��̓������̌v�Z
int cFinger::getdqjdqkoTi() {
	
	double sig = (fingerID == 1 ? 1.0 : -1.0);
	double	l0, l1, l2;
	l0 = SENKAI_LINK_LEN; l1 = this->kine.l[ARM_M1]; l2 = this->kine.l[ARM_M2];
	double	C0, S0, C1, C2, S1, S2, C12, S12;
	double th22_th1 = this->var.q.el[0][0] * 2.0 + this->var.q.el[1][0];
	// �O�p�֐�
	C0 = cos(this->senkai_base_jnt); C1 = cos(this->var.q.el[0][0]); C2 = cos(this->var.q.el[1][0]);
	S0 = sin(this->senkai_base_jnt); S1 = sin(this->var.q.el[0][0]); S2 = sin(this->var.q.el[1][0]);
	C12 = cos(this->var.q.el[0][0] + this->var.q.el[1][0]); S12 = sin(this->var.q.el[0][0] + this->var.q.el[1][0]);

	//	dqjdqkoTi[i][j][k]�Ŋ֐�i�̓����s��oTi��qk�Ɋւ��������qj�Ɋւ������
	//	Matrix dqjdqk0Ti[3][3][3];
	Matrix zeroMat; matInit(&zeroMat, 4, 4);
	// mat.el[��][�s]
	//	oTs�ɂ���
	imp.dqjdqk0Ti[0][0][0].el[1][1] = -C0; imp.dqjdqk0Ti[0][0][0].el[2][1] = S0;	imp.dqjdqk0Ti[0][0][0].el[3][1] = l0 * C0;
	imp.dqjdqk0Ti[0][0][0].el[1][2] = -S0; imp.dqjdqk0Ti[0][0][0].el[2][2] = -C0;	imp.dqjdqk0Ti[0][0][0].el[3][2] = l0 * S0;
	//	oTs�͐���֐߈ȊO�͊֗^���Ȃ��̂ő��͂��ׂă[��

	//	oT1�ɂ���
	imp.dqjdqk0Ti[1][1][0].el[0][1] = -S0 * C1;	imp.dqjdqk0Ti[1][1][0].el[1][1] = -S0 * S1; imp.dqjdqk0Ti[1][1][0].el[3][1] = l1*S0 * S1;
	imp.dqjdqk0Ti[1][1][0].el[0][2] = C0 * C1;	imp.dqjdqk0Ti[1][1][0].el[1][2] = -C0 * S1; imp.dqjdqk0Ti[1][1][0].el[3][2] = -l1 * C0 * S1;
	imp.dqjdqk0Ti[1][0][1] = imp.dqjdqk0Ti[1][1][0];//(j,k������ւ���Ă����ʂ͓���)
	// ����֐߂œ�����
	imp.dqjdqk0Ti[1][0][0].el[0][1] = -C0 * S1;	imp.dqjdqk0Ti[1][0][0].el[1][1] = -C0 * C1; imp.dqjdqk0Ti[1][0][0].el[2][1] =S0;		imp.dqjdqk0Ti[1][0][0].el[3][1] =-C0*(l0*sig+l1*C1);
	imp.dqjdqk0Ti[1][0][0].el[0][2] = -S0 * S1;	imp.dqjdqk0Ti[1][0][0].el[1][2] = -S0 * C1; imp.dqjdqk0Ti[1][0][0].el[2][2] = -C0;		imp.dqjdqk0Ti[1][0][0].el[3][1] = S0 * (l0 - l1 * C1);
	//	�֐�1�œ�����
	imp.dqjdqk0Ti[1][1][1].el[0][0] = -C1;		imp.dqjdqk0Ti[1][1][1].el[1][0] = S1;		imp.dqjdqk0Ti[1][1][1].el[3][0] = -l1 * C1;
	imp.dqjdqk0Ti[1][1][1].el[0][1] = -C0 * S1; imp.dqjdqk0Ti[1][1][1].el[1][1] = -C0 * C1; imp.dqjdqk0Ti[1][1][1].el[3][1] = -l1 * C1 *C0;
	imp.dqjdqk0Ti[1][1][1].el[0][2] = -S0 * C1; imp.dqjdqk0Ti[1][1][1].el[1][2] = -S0 * C1; imp.dqjdqk0Ti[1][1][1].el[3][2] = -l1 * C1 * S0;

	//	0T2�ɂ���
	//	����֐߂Ɗ֐�1�Ɋւ��Ĕ���
	imp.dqjdqk0Ti[2][1][0].el[0][1] = -S0 * C12;	imp.dqjdqk0Ti[2][1][0].el[1][1] = S0 * S12;		imp.dqjdqk0Ti[2][1][0].el[3][1] = l1*(S0*S1-2*l2*cos(th22_th1));
	imp.dqjdqk0Ti[2][1][0].el[0][2] = C0 * C12;		imp.dqjdqk0Ti[2][1][0].el[1][2] = -C0 * S12;	imp.dqjdqk0Ti[2][1][0].el[3][2] = -l1 * C0 * S1 + 2 * l2 * C0 * cos(th22_th1);
	imp.dqjdqk0Ti[2][0][1] = imp.dqjdqk0Ti[2][1][0];//(j,k������ւ���Ă����ʂ͓���)

	//	����֐߂Ɗ֐�2�Ɋւ��Ĕ���
	imp.dqjdqk0Ti[2][2][0].el[0][1] = -S0 * C12;	imp.dqjdqk0Ti[2][2][0].el[1][1] = S0 * S12;		imp.dqjdqk0Ti[2][2][0].el[3][1] = l1 * (S0 * S1 - 2 * l2 * cos(th22_th1));
	imp.dqjdqk0Ti[2][2][0].el[0][2] = C0 * C12;		imp.dqjdqk0Ti[2][2][0].el[1][2] = -C0 * S12;	imp.dqjdqk0Ti[2][2][0].el[3][2] = -l1 * C0 * S1 + 2 * l2 * C0 * cos(th22_th1);
	imp.dqjdqk0Ti[2][0][2] = imp.dqjdqk0Ti[2][2][0];//(j,k������ւ���Ă����ʂ͓���)
	//	�֐�1�Ɗ֐�2�Ɋւ��Ĕ���
	imp.dqjdqk0Ti[2][2][1].el[0][0] = -C12;			imp.dqjdqk0Ti[2][2][1].el[1][0] = S12;			imp.dqjdqk0Ti[2][2][1].el[3][0] = -2*l2*cos(th22_th1);
	imp.dqjdqk0Ti[2][2][1].el[0][1] = -C0 * S12;	imp.dqjdqk0Ti[2][2][1].el[1][1] = -C0 * C12;	imp.dqjdqk0Ti[2][2][1].el[3][1] = -2 * l2 *C0* sin(th22_th1);
	imp.dqjdqk0Ti[2][2][1].el[0][2] = -S0 * S12;	imp.dqjdqk0Ti[2][2][1].el[1][2] = -S0 * C12;	imp.dqjdqk0Ti[2][2][1].el[3][2] = -2 * l2 *S0* sin(th22_th1);
	
	imp.dqjdqk0Ti[2][1][2] = imp.dqjdqk0Ti[2][2][1];//(j,k������ւ���Ă����ʂ͓���)
	// ����֐߂œ�����
	imp.dqjdqk0Ti[2][0][0].el[0][1] = -C0 * S12;	imp.dqjdqk0Ti[2][0][0].el[1][1] = -C0 * C12;	imp.dqjdqk0Ti[2][0][0].el[2][1] = S0;	imp.dqjdqk0Ti[2][0][0].el[3][1] = -C0 * (l2 * sin(th22_th1) + sig * l0 + l1 * C1);
	imp.dqjdqk0Ti[2][0][0].el[0][2] = -S0 * S12;	imp.dqjdqk0Ti[2][0][0].el[1][2] = -S0 * C12;	imp.dqjdqk0Ti[2][0][0].el[2][2] = -C0;	imp.dqjdqk0Ti[2][0][0].el[3][2] = S0 * (-l2 * sin(th22_th1) + l0 + l1 * C1);
	// �֐�1�œ�����
	imp.dqjdqk0Ti[2][1][1].el[0][0] = -C12;			imp.dqjdqk0Ti[2][1][1].el[1][0] = S12;			imp.dqjdqk0Ti[2][1][1].el[3][0] = -4 * l2 * cos(th22_th1) - l1 * C1;
	imp.dqjdqk0Ti[2][1][1].el[0][1] = -C0 * S12;	imp.dqjdqk0Ti[2][1][1].el[1][1] = -C0 * C12;	imp.dqjdqk0Ti[2][1][1].el[3][1] = -C0 * (4 * l2 * sin(th22_th1) + l1 * C1);
	imp.dqjdqk0Ti[2][1][1].el[0][2] = -S0 * S12;	imp.dqjdqk0Ti[2][1][1].el[1][2] = -S0 * C12;	imp.dqjdqk0Ti[2][1][1].el[3][2] = -S0 * (4 * l2 * sin(th22_th1) + l1 * C1);
	// �֐�2�œ�����
	imp.dqjdqk0Ti[2][2][2].el[0][0] = -C12;			imp.dqjdqk0Ti[2][2][2].el[1][0] = S12;			imp.dqjdqk0Ti[2][2][2].el[3][0] = -l2 * cos(th22_th1);
	imp.dqjdqk0Ti[2][2][2].el[0][1] = -C0 * S12;	imp.dqjdqk0Ti[2][2][2].el[1][1] = -C0 * C12;	imp.dqjdqk0Ti[2][2][2].el[3][1] = -C0 *l2 * sin(th22_th1);
	imp.dqjdqk0Ti[2][2][2].el[0][2] = -S0 * S12;	imp.dqjdqk0Ti[2][2][2].el[1][2] = -S0 * C12;	imp.dqjdqk0Ti[2][2][2].el[3][2] = -S0 * l2 * sin(th22_th1);
#if INVERSE
	for (int i = 0; i < 3; i++) {
		for (int k = 0; k < 3; k++) {
			for (int l = 0; l < 3; l++) {
				matTrans(&imp.dqjdqk0Ti[i][k][l], &imp.dqjdqk0Ti[i][k][l]);
			}
		}
	}
#endif
	return 0;
}

Matrix getHi(double sx, double sy, double sz, double Ixx, double Iyy, double Izz,double m) {
	Matrix H;
	matInit(&H, 4, 4);
	// mat.el[��][�s]
	H.el[0][0] = (-Ixx + Iyy + Izz) / 2.0;	H.el[1][0] = 0.0;						H.el[2][0] = 0.0;						H.el[3][0] = m*sx;
	H.el[0][1] = 0;							H.el[1][1] = (Ixx - Iyy + Izz) / 2.0;	H.el[2][1] = 0.0;						H.el[3][1] = m * sy;
	H.el[0][2] = 0;							H.el[1][2] = 0;							H.el[2][2] = (Ixx - Iyy + Izz) / 2.0;	H.el[3][2]= m * sz;
	H.el[0][3] = m * sx;					H.el[1][3] = m * sy;					H.el[2][3] = m * sz;					H.el[3][3] = m;
	return H;

}
void cFinger::showPos() {
	//	����֐߂̓��I�p�����[�^
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
	printf("finger ID : %d\n", this->fingerID);
	printf("senkai_base(%lf,%lf,%lf)\n", senkai_base_pos[0], senkai_base_pos[1], senkai_base_pos[2]);
	printf("link1(%lf,%lf,%lf)\n", link1_pos[0], link1_pos[1], link1_pos[2]);
	printf("link1_x = base_x + ARM_LINK1_LEN *  (ARM_LINK1_LEN / 2.0 * cos(jnt_pos[ARM_M1])=%lf + %lf/2.0*%lf=%lf\n",
		0.5, ARM_LINK1_LEN, cos(jnt_pos[ARM_M1]), 0.5+ (ARM_LINK1_LEN/2.0)* cos(jnt_pos[ARM_M1]));
	double estimated_theta = acos(2.0*(link1_pos[0]-0.5)/ARM_LINK1_LEN);
	printf("estimated theta1 = %lf or %lf real theta1 = %lf\n", radToAng(estimated_theta), radToAng(2*PI-estimated_theta),radToAng(jnt_pos[ARM_M1]));
	//printf("link_base(%lf,%lf,%lf)\n", link1_pos[0], link1_pos[1], link1_pos[2]);

}
//	�^�������s��Hi_hat�̌v�Z
int cFinger::setHi() {
	//	����֐߂̓��I�p�����[�^
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

	double	m0, m1, m2, l0, l1, l2, lg0, lg1, lg2;
	double	I0xx, I0yy, I0zz, I1xx, I1yy, I1zz, I2xx, I2yy, I2zz;
	m0 = this->senkai_link.getMass(); m1 = this->dyn.m[ARM_M1]; m2 = this->dyn.m[ARM_M2];
	l0 = SENKAI_LINK_LEN; l1 = this->kine.l[ARM_M1]; l2 = this->kine.l[ARM_M2];
	lg0 = SENKAI_LINK_LEN / 2.0; lg1 = this->kine.lg[ARM_M1]; lg2 = this->kine.lg[ARM_M2];

	//	�e���Ɋւ��銵�����[�����g(�����N���~���Ƃ��Čv�Z)
	I0xx = I0yy=(SENKAI_LINK_RAD * SENKAI_LINK_RAD / 4 + l1 * l1 / 12) * m0;//I0xx=I0yy
	I0zz = (1 / 2.0) * m0 * SENKAI_LINK_RAD * SENKAI_LINK_RAD;

	I1yy = I1zz=(this->kine.r[ARM_M1] * this->kine.r[ARM_M1] / 4 + l1 * l1 / 12) * m1;	//I1yy=I1zz
	I1xx = (1 / 2.0) * m1 * this->kine.r[ARM_M1] * this->kine.r[ARM_M1];

	I2yy = I2zz = (this->kine.r[ARM_M2] * this->kine.r[ARM_M2] / 4 + l2 * l2 / 12) * m2;	//I2yy=I2zz
	I2xx = (1 / 2.0) * m2 * this->kine.r[ARM_M2] * this->kine.r[ARM_M2];
	

	//	4x4�̋^�������s����v�Z����
	double sx0, sy0, sz0, sx1, sy1, sz1, sx2, sy2, sz2;
	sx0 = senkai_link_pos[0]; sy0 = senkai_link_pos[1]; sz0 = senkai_link_pos[2];
	sx1 = link1_pos[0]; sy1 = link1_pos[1]; sz1 = link1_pos[2];
	sx2 = link2_pos[0]; sy2 = link2_pos[1]; sz2 = link2_pos[2];
	imp.H_hat[0] = getHi(sx0, sy0, sz0, I0xx + m0 * (sy0 * sy0 + sz0 * sz0), I0yy + m0 * (sx0 * sx0 + sz0 * sz0), I0zz + m0 * (sx0 * sx0 + sy0 * sy0),m0);
	imp.H_hat[1] = getHi(sx1, sy1, sz1, I1xx + m1 * (sy1 * sy1 + sz1 * sz1), I1yy + m1 * (sx1 * sx1 + sz1 * sz1), I1zz + m1 * (sx1 * sx1 + sy1 * sy1), m1);
	imp.H_hat[2] = getHi(sx2, sy2, sz2, I2xx + m2 * (sy2 * sy2 + sz2 * sz2), I2yy + m2 * (sx2 * sx2 + sz2 * sz2), I2zz + m2 * (sx2 * sx2 + sy2 * sy2), m2);

	
	return 0;
}

//	�����s��̌v�Z	3x3
int cFinger::setMq() {
	getdqjdqkoTi();
	// mat.el[��][�s]
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			int k = std::max(i, j);
			Matrix tmp1, tmp2,dqkTrans,res;
			matInit(&tmp1, 4, 4); matInit(&tmp2, 4, 4);
			matInit(&dqkTrans, 4, 4); matInit(&res, 4, 4);

			//	dqkoTi[i][k]�Ŋ֐�i�̓����s��oTi��qk�Ɋւ������
			matMul(&tmp1, &imp.dqkoTi[k][j], &imp.H_hat[k]);
			matTrans(&dqkTrans, &imp.dqkoTi[k][j]);		//�s��̓]�u
			matMul(&tmp2, &tmp1,&dqkTrans);
			dyn.Mq.el[j][i] = matTrace(&tmp2);
		}
	}

	return 0;
}


// ���S�E�R���I���͂̌v�Z
int cFinger::seth() {
	int ARM_JNT_NUM = 3;
	Matrix tmp1, tmp2;
	matInit(&tmp1, 4, 4); matInit(&tmp2, 4, 4);
	for (int i = 0; i < 3; i++) {
		//hi=3�����x�N�g��h(q)�̑�i�v�f
		double result = 0.0;
		
		for (int j = 0; j < ARM_JNT_NUM; j++) {
			for (int m = 0; m < ARM_JNT_NUM; m++) {
				int k = std::max(i, std::max(j, m));
				double now_iteration_sum = 0.0;
				matZero(&tmp1); matZero(&tmp2);
				//	��K�����~Hk
				matMul(&tmp1, &imp.dqjdqk0Ti[k][j][m], &imp.H_hat[k]);
				matMul(&tmp2, &tmp1, &imp.dqkoTi[k][i]);
				now_iteration_sum=matTrace(&tmp2);	//trace���Ƃ�
				result += now_iteration_sum * this->var.dq.el[j][0] * this->var.dq.el[m][0];
				
			}
		}

		dyn.h.el[0][i] = result;
	}
	matFree(&tmp1);	matFree(&tmp2);


	return 0;
}