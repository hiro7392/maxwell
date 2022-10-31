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
	imp.Ts.el[0][0] = 1.0;
	//	2行目
	imp.Ts.el[0][1] = 0.0;
	imp.Ts.el[1][1] = cos(senkai_base_jnt);
	imp.Ts.el[2][1] = -sin(senkai_base_jnt);
	imp.Ts.el[3][1] = SENKAI_LINK_LEN * cos(senkai_base_jnt);

	//	3行目
	imp.Ts.el[0][2] = 0.0;
	imp.Ts.el[1][2] = sin(senkai_base_jnt);
	imp.Ts.el[2][2] = cos(senkai_base_jnt);
	imp.Ts.el[3][2] = SENKAI_LINK_LEN * (-sin(senkai_base_jnt));

	// 3行目はすべて0なので何もしない
	// 4行目は4列目以外すべて0
	imp.Ts.el[3][3] = 1.0;
#if 0 //デバッグ用
	
#endif

	//	旋回根元からの変換行列を計算
	matMul(&imp.oT1, &imp.Ts, &imp.sT1);	//原点からリンク1の同次変換
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

	
	return 0;
}