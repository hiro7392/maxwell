#pragma once
void MatPrintDebug4x4(Matrix& mat, std::string name) {
	printf("%s = \n", &name);
	for (int col = 0; col < 4; col++) {
		for (int row = 0; row < 4; row++) {
			printf("%lf ", mat.el[row][col]);
		}
		printf("\n");
	}
	return;
}
void MatPrintDebug4x1(Matrix& mat, std::string name) {
	printf("%s = \n", &name);
	for (int col = 0; col < 4; col++) {
		for (int row = 0; row < 1; row++) {
			printf("%lf ", mat.el[row][col]);
		}
		printf("\n");
	}
	return;
}
//	同次行列の計算
int cFinger::setTransMatrix() {
	//	同次行列の計算
	//	リンク1 1行目
	this->imp.sT1.el[0][0] = cos(jnt_pos[0]);
	imp.sT1.el[1][0] = -sin(jnt_pos[0]);
	imp.sT1.el[2][0] = 0;
	imp.sT1.el[3][0] = ARM_LINK1_LEN * cos(jnt_pos[0]);

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
	MatPrintDebug4x1(mat, "MassCenter=");
	
#endif
	return 0;
}
int cFinger::calculateGravity() {

	imp.G_xyz.el[0][2] = 9.8;
	MatPrintDebug4x1(imp.G_xyz, "G_xyz");

	//	同次行列の計算


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
	//	MatPrintDebug4x4(imp.oT1, "oT1");
	//	MatPrintDebug4x4(imp.oT2, "oT2");


	return 0;
}