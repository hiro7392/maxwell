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
// mat.el[列][行]
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
//	同次行列の計算
int cFinger::setTransMatrix() {
	double sig = (fingerID == 1 ? 1.0 : -1.0);
	//	同次行列の計算
	//	リンク1 1行目
	this->imp.sT1.el[0][0] = cos(jnt_pos[0]);
	imp.sT1.el[1][0] = -sin(jnt_pos[0]);
	imp.sT1.el[2][0] = 0;
	imp.sT1.el[3][0] = ARM_LINK1_LEN * cos(jnt_pos[0])*sig;

	// 2行目
	imp.sT1.el[0][1] = sin(jnt_pos[0]);
	imp.sT1.el[1][1] = cos(jnt_pos[0]);
	imp.sT1.el[2][1] = 0;
	imp.sT1.el[3][1] = ARM_LINK1_LEN * sin(jnt_pos[0]);

	// 3行目は3列目以外0
	imp.sT1.el[2][2] = 1.0;
	// 4行目は4列目以外すべて0
	imp.sT1.el[3][3] = 1.0;

	//	リンク2
	//	1行目
	imp.T12.el[0][0] = cos(jnt_pos[1]);
	imp.T12.el[1][0] = -sin(jnt_pos[1]);
	imp.T12.el[2][0] = 0;
	imp.T12.el[3][0] = ARM_LINK2_LEN * cos(jnt_pos[0] + jnt_pos[1]);

	// 2行目
	imp.T12.el[0][1] = sin(jnt_pos[1]);
	imp.T12.el[1][1] = cos(jnt_pos[1]);
	imp.T12.el[2][1] = 0;
	imp.T12.el[3][1] = ARM_LINK2_LEN * sin(jnt_pos[0] + jnt_pos[1]);

	// 3行目は3列目以外0
	imp.T12.el[2][2] = 1.0;
	// 4行目は4列目以外すべて0
	imp.T12.el[3][3] = 1.0;

	//	旋回関節
	//	1行目
	imp.oTs.el[0][0] = 1.0;
	//	2行目
	imp.oTs.el[0][1] = 0.0;
	imp.oTs.el[1][1] = cos(senkai_base_jnt);
	imp.oTs.el[2][1] = -sin(senkai_base_jnt);
	imp.oTs.el[3][1] = SENKAI_LINK_LEN * cos(senkai_base_jnt);

	//	3行目
	imp.oTs.el[0][2] = 0.0;
	imp.oTs.el[1][2] = sin(senkai_base_jnt);
	imp.oTs.el[2][2] = cos(senkai_base_jnt);
	imp.oTs.el[3][2] = SENKAI_LINK_LEN * (-sin(senkai_base_jnt));

	// 3行目はすべて0なので何もしない
	// 4行目は4列目以外すべて0
	imp.oTs.el[3][3] = 1.0;
#if 0 //デバッグ用
	
#endif

	//	旋回根元からの変換行列を計算
	matMul(&imp.oT1, &imp.oTs, &imp.sT1);	//原点からリンク1の同次変換
	matMul(&imp.oT2, &imp.oT1, &imp.T12);	//原点からリンク2の同次変換
	//	MatPrintDebug4x4(imp.oT1, "oT1");
	//	MatPrintDebug4x4(imp.oT2, "oT2");


	return 0;
}
//	3次元座標を受け取ってセットする(行列は4×1)
int cFinger::setMassCenterPosition(Matrix& mat,double mx, double my, double mz) {
	mat.el[0][0] = mx;
	mat.el[0][1] = my;
	mat.el[0][2] = mz;
	mat.el[0][3] = 0.0;
#if 1 //デバッグ用
	//MatPrintDebug4x1(mat, "MassCenter=");
	
#endif
	return 0;
}
int cFinger::calculateGravity() {

	imp.G_xyz.el[2][0] = -9.8;
	//MatPrintDebug4x1(imp.G_xyz, "G_xyz");

	setTransMatrixDq();
	for (int i = 0; i < 3; i++)imp.G.el[0][i] = 0.0;
	//	重力項を求める
	Matrix tmp,tmp2;
	matInit(&tmp, 1, 4); matInit(&tmp2, 1, 1);
	//	旋回関節について
	matMul(&tmp, &imp.ss, &imp.dqs_oTs);
	//	MatPrintDebug4x4(imp.dqs_oTs, "oTs");
	//	MatPrintDebug4x1(imp.ss, "ss");
	//	MatPrintDebug4x1(tmp, "tmp");
	//	MatPrintDebugAll(imp.G_xyz, "G_xyz",1,4);
	matMul(&tmp2, &tmp, &imp.G_xyz);
	//	MatPrintDebugAll(tmp2, "tmp2",1,1);
	imp.G.el[0][0] += (ARM_LINK2_MASS / 2.0) * tmp2.el[0][0];	//旋回部分のリンク質量は(ARM_LINK2_MASS / 2.0) 
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

	//	関節1について
	matMul(&tmp, &imp.s1, &imp.dq1_oT1);
	matMul(&tmp2, &tmp ,&imp.G_xyz);
	imp.G.el[0][1] += ARM_LINK1_MASS * tmp2.el[0][0];

	matMul(&tmp, &imp.s2 ,&imp.dq1_oT2);
	matMul(&tmp2, &tmp ,&imp.G_xyz);
	imp.G.el[0][1] += ARM_LINK2_MASS * tmp2.el[0][0];
	//printf("G[0][1] = %lf\n", imp.G.el[0][1]);
	//	関節２について
	matMul(&tmp, &imp.s2 ,&imp.dq2_oT2);
	matMul(&tmp2, &tmp ,&imp.G_xyz);
	imp.G.el[0][2] += ARM_LINK2_MASS * tmp2.el[0][0];
	//printf("G[0][2] = %lf\n", imp.G.el[0][2]);
	//MatPrintDebug3x1(imp.G, "G");

	matFree(&tmp);

	return 0;
}
//	同次行列の微分の計算
int cFinger::setTransMatrixDq() {
	//	行列について
	//	A[row][col]	A[列][行]
	

	//	旋回関節に関する微分
	// oTsの1行目はすべて0
	// 2行目
	imp.dqs_oTs.el[0][1] = 0;
	imp.dqs_oTs.el[1][1] = -cos(senkai_base_jnt);
	imp.dqs_oTs.el[2][1] = sin(senkai_base_jnt);
	imp.dqs_oTs.el[3][1] = SENKAI_LINK_LEN * (fingerID == 1 ? (-1) : 1) * cos(senkai_base_jnt);

	// 3行目
	imp.dqs_oTs.el[0][2] = 0;
	imp.dqs_oTs.el[1][2] = -sin(senkai_base_jnt);
	imp.dqs_oTs.el[2][2] = -cos(senkai_base_jnt);
	imp.dqs_oTs.el[3][2] = sin(senkai_base_jnt);

	// 4行目
	imp.dqs_oTs.el[0][3] = sin(jnt_pos[0]);
	imp.dqs_oTs.el[1][3] = cos(jnt_pos[0]);
	imp.dqs_oTs.el[2][3] = 0;
	imp.dqs_oTs.el[3][3] = ARM_LINK1_LEN * sin(jnt_pos[0]);

	//	oT1の旋回関節に関する微分
	//	1行目はすべて0

	// 2行目
	imp.dqs_oT1.el[0][1] = -sin(jnt_pos[0])*sin(senkai_base_jnt);
	imp.dqs_oT1.el[1][1] = -cos(jnt_pos[0]) * sin(senkai_base_jnt);
	imp.dqs_oT1.el[2][1] = -cos(senkai_base_jnt);
	imp.dqs_oT1.el[3][1] = -ARM_LINK1_LEN*sin(jnt_pos[0])*sin(senkai_base_jnt) + SENKAI_LINK_LEN * (fingerID == 1 ? (-1) : 1) * sin(senkai_base_jnt);

	// 3行目
	imp.dqs_oT1.el[0][2] = sin(jnt_pos[0]) * cos(senkai_base_jnt);
	imp.dqs_oT1.el[1][2] = cos(jnt_pos[0]) * cos(senkai_base_jnt);
	imp.dqs_oT1.el[2][2] = -sin(senkai_base_jnt);
	imp.dqs_oT1.el[3][2] = cos(senkai_base_jnt) * (ARM_LINK1_LEN * sin(jnt_pos[0] - 1.0));

	// 4行目はすべて0

	//	oT2の旋回関節に関する微分
	//	1行目はすべて0
	imp.dqs_oT2.el[0][1] = -sin(senkai_base_jnt)*(sin(jnt_pos[0])*cos(jnt_pos[1])+ cos(jnt_pos[0]) * sin(jnt_pos[1]));
	imp.dqs_oT2.el[1][1] = sin(senkai_base_jnt) * (sin(jnt_pos[0]) * sin(jnt_pos[1]) - cos(jnt_pos[0]) * cos(jnt_pos[1]));
	double tmp1, tmp2;
	tmp1 = -ARM_LINK2_LEN * sin(senkai_base_jnt) * (sin(jnt_pos[0]) * cos(jnt_pos[0] + jnt_pos[1]) + cos(jnt_pos[0]) * sin(jnt_pos[0] + jnt_pos[1]));
	tmp2 = -ARM_LINK1_LEN * sin(jnt_pos[0]) * sin(senkai_base_jnt) + (fingerID == 1 ? (-1) : 1) * SENKAI_LINK_LEN * sin(senkai_base_jnt);

	imp.dqs_oT2.el[2][1] = cos(senkai_base_jnt);
	imp.dqs_oT2.el[3][1] = tmp1 + tmp2;
	// 3行目
	imp.dqs_oT2.el[0][2] =	cos(senkai_base_jnt) * (sin(jnt_pos[0]) * cos(jnt_pos[1]) + cos(jnt_pos[0]) * sin(jnt_pos[1]));
	imp.dqs_oT2.el[1][2] =	cos(senkai_base_jnt) * (cos(jnt_pos[0]) * cos(jnt_pos[1]) - sin(jnt_pos[0]) * sin(jnt_pos[1]));
	imp.dqs_oT2.el[2][2] = -sin(senkai_base_jnt);
	tmp1 = -ARM_LINK2_LEN * cos(senkai_base_jnt) * (sin(jnt_pos[0]) * cos(jnt_pos[0] + jnt_pos[1]) + cos(jnt_pos[0]) * sin(jnt_pos[0] + jnt_pos[1]));
	tmp2 = cos(senkai_base_jnt)*(ARM_LINK1_LEN*sin(jnt_pos[0]-1.0));
	imp.dqs_oT2.el[3][2] = tmp1 + tmp2;
	// 4行目はすべて0

	//関節1に関する微分
	//dq1_oTs oTsの関節1に関する微分は0		初期化ですでに0になっている
	//dq1_oT1 oT1の関節1に関する微分
	imp.dq1_oT1.el[0][0] = -sin(jnt_pos[0]);
	imp.dq1_oT1.el[1][0] = -cos(jnt_pos[0]);
	imp.dq1_oT1.el[2][0] = 0.0;
	imp.dq1_oT1.el[3][0] = SENKAI_LINK_LEN  * sin(jnt_pos[0]);
	//	2行目
	imp.dq1_oT1.el[0][1] = cos(senkai_base_jnt)*cos(jnt_pos[0]);
	imp.dq1_oT1.el[1][1] = -cos(senkai_base_jnt) * sin(jnt_pos[0]);
	imp.dq1_oT1.el[2][1] = 0.0;
	imp.dq1_oT1.el[3][1] = SENKAI_LINK_LEN  * cos(senkai_base_jnt)*cos(jnt_pos[0]);
	//	3行目
	imp.dq1_oT1.el[0][2] = sin(senkai_base_jnt) * cos(jnt_pos[0]);
	imp.dq1_oT1.el[1][2] = -sin(senkai_base_jnt) * sin(jnt_pos[0]);
	imp.dq1_oT1.el[2][2] = 0.0;
	imp.dq1_oT1.el[3][2] = SENKAI_LINK_LEN * sin(senkai_base_jnt) * cos(jnt_pos[0]);

	//dq1_oT2 oT2の関節1に関する微分
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

	//dq2_oTs oTsの関節2に関する微分は0		初期化ですでに0になっている
	//dq2_oT1 oT1の関節2に関する微分は0		初期化ですでに0になっている
	
	///dq2_oT2 oT2の関節2に関する微分
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

	//	旋回根元からの変換行列を計算
	matMul(&imp.oT1, &imp.oTs, &imp.sT1);	//原点からリンク1の同次変換
	matMul(&imp.oT2, &imp.oT1, &imp.T12);	//原点からリンク2の同次変換
	//MatPrintDebug4x4(imp.dq1_oT1, "dq1_oT1");
	//MatPrintDebug4x4(imp.dq1_oT2, "dq1_oT2");
	//MatPrintDebug4x4(imp.dq2_oT1, "dq2_oT1");
	//MatPrintDebug4x4(imp.dq2_oT2, "dq2_oT2");
	//MatPrintDebug4x4(imp.dqs_oTs, "dqs_oTs");
	
	//	同次行列の各関節微分を格納
	//	dqkoTi[i][k]で関節iの同次行列oTiのqkに関する微分
	imp.dqkoTi[0][0] = imp.dqs_oTs;	//dqkoTi[i][k]で関節iの同次行列oTiのqkに関する微分
	imp.dqkoTi[0][1] = imp.dq1_oTs;	
	imp.dqkoTi[0][2] = imp.dq2_oTs;	

	imp.dqkoTi[1][0] = imp.dqs_oT1;	//dqkoTi[i][k]で関節iの同次行列oTiのqkに関する微分
	imp.dqkoTi[1][1] = imp.dq1_oT1;
	imp.dqkoTi[1][2] = imp.dq2_oT1;

	imp.dqkoTi[2][0] = imp.dqs_oT2;	//dqkoTi[i][k]で関節iの同次行列oTiのqkに関する微分
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
// 同次行列の二回微分の計算
int cFinger::getdqjdqkoTi() {
	
	double sig = (fingerID == 1 ? 1.0 : -1.0);
	double	l0, l1, l2;
	l0 = SENKAI_LINK_LEN; l1 = this->kine.l[ARM_M1]; l2 = this->kine.l[ARM_M2];
	double	C0, S0, C1, C2, S1, S2, C12, S12;
	double th22_th1 = this->var.q.el[0][0] * 2.0 + this->var.q.el[1][0];
	// 三角関数
	C0 = cos(this->senkai_base_jnt); C1 = cos(this->var.q.el[0][0]); C2 = cos(this->var.q.el[1][0]);
	S0 = sin(this->senkai_base_jnt); S1 = sin(this->var.q.el[0][0]); S2 = sin(this->var.q.el[1][0]);
	C12 = cos(this->var.q.el[0][0] + this->var.q.el[1][0]); S12 = sin(this->var.q.el[0][0] + this->var.q.el[1][0]);

	//	dqjdqkoTi[i][j][k]で関節iの同次行列oTiのqkに関する微分とqjに関する微分
	//	Matrix dqjdqk0Ti[3][3][3];
	Matrix zeroMat; matInit(&zeroMat, 4, 4);
	// mat.el[列][行]
	//	oTsについて
	imp.dqjdqk0Ti[0][0][0].el[1][1] = -C0; imp.dqjdqk0Ti[0][0][0].el[2][1] = S0;	imp.dqjdqk0Ti[0][0][0].el[3][1] = l0 * C0;
	imp.dqjdqk0Ti[0][0][0].el[1][2] = -S0; imp.dqjdqk0Ti[0][0][0].el[2][2] = -C0;	imp.dqjdqk0Ti[0][0][0].el[3][2] = l0 * S0;
	//	oTsは旋回関節以外は関与しないので他はすべてゼロ

	//	oT1について
	imp.dqjdqk0Ti[1][1][0].el[0][1] = -S0 * C1;	imp.dqjdqk0Ti[1][1][0].el[1][1] = -S0 * S1; imp.dqjdqk0Ti[1][1][0].el[3][1] = l1*S0 * S1;
	imp.dqjdqk0Ti[1][1][0].el[0][2] = C0 * C1;	imp.dqjdqk0Ti[1][1][0].el[1][2] = -C0 * S1; imp.dqjdqk0Ti[1][1][0].el[3][2] = -l1 * C0 * S1;
	imp.dqjdqk0Ti[1][0][1] = imp.dqjdqk0Ti[1][1][0];//(j,kが入れ替わっても結果は同じ)
	// 旋回関節で二回微分
	imp.dqjdqk0Ti[1][0][0].el[0][1] = -C0 * S1;	imp.dqjdqk0Ti[1][0][0].el[1][1] = -C0 * C1; imp.dqjdqk0Ti[1][0][0].el[2][1] =S0;		imp.dqjdqk0Ti[1][0][0].el[3][1] =-C0*(l0*sig+l1*C1);
	imp.dqjdqk0Ti[1][0][0].el[0][2] = -S0 * S1;	imp.dqjdqk0Ti[1][0][0].el[1][2] = -S0 * C1; imp.dqjdqk0Ti[1][0][0].el[2][2] = -C0;		imp.dqjdqk0Ti[1][0][0].el[3][1] = S0 * (l0 - l1 * C1);
	//	関節1で二回微分
	imp.dqjdqk0Ti[1][1][1].el[0][0] = -C1;		imp.dqjdqk0Ti[1][1][1].el[1][0] = S1;		imp.dqjdqk0Ti[1][1][1].el[3][0] = -l1 * C1;
	imp.dqjdqk0Ti[1][1][1].el[0][1] = -C0 * S1; imp.dqjdqk0Ti[1][1][1].el[1][1] = -C0 * C1; imp.dqjdqk0Ti[1][1][1].el[3][1] = -l1 * C1 *C0;
	imp.dqjdqk0Ti[1][1][1].el[0][2] = -S0 * C1; imp.dqjdqk0Ti[1][1][1].el[1][2] = -S0 * C1; imp.dqjdqk0Ti[1][1][1].el[3][2] = -l1 * C1 * S0;

	//	0T2について
	//	旋回関節と関節1に関して微分
	imp.dqjdqk0Ti[2][1][0].el[0][1] = -S0 * C12;	imp.dqjdqk0Ti[2][1][0].el[1][1] = S0 * S12;		imp.dqjdqk0Ti[2][1][0].el[3][1] = l1*(S0*S1-2*l2*cos(th22_th1));
	imp.dqjdqk0Ti[2][1][0].el[0][2] = C0 * C12;		imp.dqjdqk0Ti[2][1][0].el[1][2] = -C0 * S12;	imp.dqjdqk0Ti[2][1][0].el[3][2] = -l1 * C0 * S1 + 2 * l2 * C0 * cos(th22_th1);
	imp.dqjdqk0Ti[2][0][1] = imp.dqjdqk0Ti[2][1][0];//(j,kが入れ替わっても結果は同じ)

	//	旋回関節と関節2に関して微分
	imp.dqjdqk0Ti[2][2][0].el[0][1] = -S0 * C12;	imp.dqjdqk0Ti[2][2][0].el[1][1] = S0 * S12;		imp.dqjdqk0Ti[2][2][0].el[3][1] = l1 * (S0 * S1 - 2 * l2 * cos(th22_th1));
	imp.dqjdqk0Ti[2][2][0].el[0][2] = C0 * C12;		imp.dqjdqk0Ti[2][2][0].el[1][2] = -C0 * S12;	imp.dqjdqk0Ti[2][2][0].el[3][2] = -l1 * C0 * S1 + 2 * l2 * C0 * cos(th22_th1);
	imp.dqjdqk0Ti[2][0][2] = imp.dqjdqk0Ti[2][2][0];//(j,kが入れ替わっても結果は同じ)
	//	関節1と関節2に関して微分
	imp.dqjdqk0Ti[2][2][1].el[0][0] = -C12;			imp.dqjdqk0Ti[2][2][1].el[1][0] = S12;			imp.dqjdqk0Ti[2][2][1].el[3][0] = -2*l2*cos(th22_th1);
	imp.dqjdqk0Ti[2][2][1].el[0][1] = -C0 * S12;	imp.dqjdqk0Ti[2][2][1].el[1][1] = -C0 * C12;	imp.dqjdqk0Ti[2][2][1].el[3][1] = -2 * l2 *C0* sin(th22_th1);
	imp.dqjdqk0Ti[2][2][1].el[0][2] = -S0 * S12;	imp.dqjdqk0Ti[2][2][1].el[1][2] = -S0 * C12;	imp.dqjdqk0Ti[2][2][1].el[3][2] = -2 * l2 *S0* sin(th22_th1);
	
	imp.dqjdqk0Ti[2][1][2] = imp.dqjdqk0Ti[2][2][1];//(j,kが入れ替わっても結果は同じ)
	// 旋回関節で二回微分
	imp.dqjdqk0Ti[2][0][0].el[0][1] = -C0 * S12;	imp.dqjdqk0Ti[2][0][0].el[1][1] = -C0 * C12;	imp.dqjdqk0Ti[2][0][0].el[2][1] = S0;	imp.dqjdqk0Ti[2][0][0].el[3][1] = -C0 * (l2 * sin(th22_th1) + sig * l0 + l1 * C1);
	imp.dqjdqk0Ti[2][0][0].el[0][2] = -S0 * S12;	imp.dqjdqk0Ti[2][0][0].el[1][2] = -S0 * C12;	imp.dqjdqk0Ti[2][0][0].el[2][2] = -C0;	imp.dqjdqk0Ti[2][0][0].el[3][2] = S0 * (-l2 * sin(th22_th1) + l0 + l1 * C1);
	// 関節1で二回微分
	imp.dqjdqk0Ti[2][1][1].el[0][0] = -C12;			imp.dqjdqk0Ti[2][1][1].el[1][0] = S12;			imp.dqjdqk0Ti[2][1][1].el[3][0] = -4 * l2 * cos(th22_th1) - l1 * C1;
	imp.dqjdqk0Ti[2][1][1].el[0][1] = -C0 * S12;	imp.dqjdqk0Ti[2][1][1].el[1][1] = -C0 * C12;	imp.dqjdqk0Ti[2][1][1].el[3][1] = -C0 * (4 * l2 * sin(th22_th1) + l1 * C1);
	imp.dqjdqk0Ti[2][1][1].el[0][2] = -S0 * S12;	imp.dqjdqk0Ti[2][1][1].el[1][2] = -S0 * C12;	imp.dqjdqk0Ti[2][1][1].el[3][2] = -S0 * (4 * l2 * sin(th22_th1) + l1 * C1);
	// 関節2で二回微分
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
	// mat.el[列][行]
	H.el[0][0] = (-Ixx + Iyy + Izz) / 2.0;	H.el[1][0] = 0.0;						H.el[2][0] = 0.0;						H.el[3][0] = m*sx;
	H.el[0][1] = 0;							H.el[1][1] = (Ixx - Iyy + Izz) / 2.0;	H.el[2][1] = 0.0;						H.el[3][1] = m * sy;
	H.el[0][2] = 0;							H.el[1][2] = 0;							H.el[2][2] = (Ixx - Iyy + Izz) / 2.0;	H.el[3][2]= m * sz;
	H.el[0][3] = m * sx;					H.el[1][3] = m * sy;					H.el[2][3] = m * sz;					H.el[3][3] = m;
	return H;

}
void cFinger::showPos() {
	//	旋回関節の動的パラメータ
	const dReal* senkai_base_pos = dBodyGetPosition(this->senkai_base.getBody());
	//	旋回リンクの重心位置
	const dReal* senkai_link_pos = dBodyGetPosition(this->senkai_link.getBody());
	//	リンク1の重心位置
	const dReal* link1_pos = dBodyGetPosition(this->link1.getBody());
	//	リンク2の重心位置
	const dReal* link2_pos = dBodyGetPosition(this->link2.getBody());
	//	カプセルの重心位置
	const dReal* capsule_pos = dBodyGetPosition(this->fingerTopCapsule.getBody());
	//	センサの重心位置
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
//	疑似慣性行列Hi_hatの計算
int cFinger::setHi() {
	//	旋回関節の動的パラメータ
	const dReal* senkai_base_pos = dBodyGetPosition(this->senkai_base.getBody());
	//	旋回リンクの重心位置
	const dReal* senkai_link_pos = dBodyGetPosition(this->senkai_link.getBody());
	//	リンク1の重心位置
	const dReal* link1_pos = dBodyGetPosition(this->link1.getBody());
	//	リンク2の重心位置
	const dReal* link2_pos = dBodyGetPosition(this->link2.getBody());
	//	カプセルの重心位置
	const dReal* capsule_pos = dBodyGetPosition(this->fingerTopCapsule.getBody());
	//	センサの重心位置
	const dReal* sensor_pos = dBodyGetPosition(this->sensor.getBody());

	double	m0, m1, m2, l0, l1, l2, lg0, lg1, lg2;
	double	I0xx, I0yy, I0zz, I1xx, I1yy, I1zz, I2xx, I2yy, I2zz;
	m0 = this->senkai_link.getMass(); m1 = this->dyn.m[ARM_M1]; m2 = this->dyn.m[ARM_M2];
	l0 = SENKAI_LINK_LEN; l1 = this->kine.l[ARM_M1]; l2 = this->kine.l[ARM_M2];
	lg0 = SENKAI_LINK_LEN / 2.0; lg1 = this->kine.lg[ARM_M1]; lg2 = this->kine.lg[ARM_M2];

	//	各軸に関する慣性モーメント(リンクを円柱として計算)
	I0xx = I0yy=(SENKAI_LINK_RAD * SENKAI_LINK_RAD / 4 + l1 * l1 / 12) * m0;//I0xx=I0yy
	I0zz = (1 / 2.0) * m0 * SENKAI_LINK_RAD * SENKAI_LINK_RAD;

	I1yy = I1zz=(this->kine.r[ARM_M1] * this->kine.r[ARM_M1] / 4 + l1 * l1 / 12) * m1;	//I1yy=I1zz
	I1xx = (1 / 2.0) * m1 * this->kine.r[ARM_M1] * this->kine.r[ARM_M1];

	I2yy = I2zz = (this->kine.r[ARM_M2] * this->kine.r[ARM_M2] / 4 + l2 * l2 / 12) * m2;	//I2yy=I2zz
	I2xx = (1 / 2.0) * m2 * this->kine.r[ARM_M2] * this->kine.r[ARM_M2];
	

	//	4x4の疑似慣性行列を計算する
	double sx0, sy0, sz0, sx1, sy1, sz1, sx2, sy2, sz2;
	sx0 = senkai_link_pos[0]; sy0 = senkai_link_pos[1]; sz0 = senkai_link_pos[2];
	sx1 = link1_pos[0]; sy1 = link1_pos[1]; sz1 = link1_pos[2];
	sx2 = link2_pos[0]; sy2 = link2_pos[1]; sz2 = link2_pos[2];
	imp.H_hat[0] = getHi(sx0, sy0, sz0, I0xx + m0 * (sy0 * sy0 + sz0 * sz0), I0yy + m0 * (sx0 * sx0 + sz0 * sz0), I0zz + m0 * (sx0 * sx0 + sy0 * sy0),m0);
	imp.H_hat[1] = getHi(sx1, sy1, sz1, I1xx + m1 * (sy1 * sy1 + sz1 * sz1), I1yy + m1 * (sx1 * sx1 + sz1 * sz1), I1zz + m1 * (sx1 * sx1 + sy1 * sy1), m1);
	imp.H_hat[2] = getHi(sx2, sy2, sz2, I2xx + m2 * (sy2 * sy2 + sz2 * sz2), I2yy + m2 * (sx2 * sx2 + sz2 * sz2), I2zz + m2 * (sx2 * sx2 + sy2 * sy2), m2);

	
	return 0;
}

//	慣性行列の計算	3x3
int cFinger::setMq() {
	getdqjdqkoTi();
	// mat.el[列][行]
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			int k = std::max(i, j);
			Matrix tmp1, tmp2,dqkTrans,res;
			matInit(&tmp1, 4, 4); matInit(&tmp2, 4, 4);
			matInit(&dqkTrans, 4, 4); matInit(&res, 4, 4);

			//	dqkoTi[i][k]で関節iの同次行列oTiのqkに関する微分
			matMul(&tmp1, &imp.dqkoTi[k][j], &imp.H_hat[k]);
			matTrans(&dqkTrans, &imp.dqkoTi[k][j]);		//行列の転置
			matMul(&tmp2, &tmp1,&dqkTrans);
			dyn.Mq.el[j][i] = matTrace(&tmp2);
		}
	}

	return 0;
}


// 遠心・コリオリ力の計算
int cFinger::seth() {
	int ARM_JNT_NUM = 3;
	Matrix tmp1, tmp2;
	matInit(&tmp1, 4, 4); matInit(&tmp2, 4, 4);
	for (int i = 0; i < 3; i++) {
		//hi=3次元ベクトルh(q)の第i要素
		double result = 0.0;
		
		for (int j = 0; j < ARM_JNT_NUM; j++) {
			for (int m = 0; m < ARM_JNT_NUM; m++) {
				int k = std::max(i, std::max(j, m));
				double now_iteration_sum = 0.0;
				matZero(&tmp1); matZero(&tmp2);
				//	二階部分×Hk
				matMul(&tmp1, &imp.dqjdqk0Ti[k][j][m], &imp.H_hat[k]);
				matMul(&tmp2, &tmp1, &imp.dqkoTi[k][i]);
				now_iteration_sum=matTrace(&tmp2);	//traceをとる
				result += now_iteration_sum * this->var.dq.el[j][0] * this->var.dq.el[m][0];
				
			}
		}

		dyn.h.el[0][i] = result;
	}
	matFree(&tmp1);	matFree(&tmp2);


	return 0;
}