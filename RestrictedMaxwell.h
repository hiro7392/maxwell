#pragma once
////////////////////////////////////////////////////////
// 制約条件付きのMaxwell制御則
// M*ddx + K*C^{-1}*M*dx + K*x = F + K*C^{-1}∫Fdt
// x_1=x_2=x_0, P_1=P_2
////////////////////////////////////////////////////////
int cFinger::RestrictedCtrlMaxwell(Matrix* tau)
{

	int	jnt, crd;

	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp22_2(2, 2);
	static Matrix	tauNC(2, 1), tauVE(2, 1), tauIN(2, 1), tauPL(2, 1), E(2, 2);
	static Matrix	Integ(2, 1);
	static Matrix	re(2, 1), dre(2, 1);	// 手先位置変位，手先速度変位

	auto entity = EntityManager::get();
	if (entity->step == 0) {
		armCalcImpPeriod();		// 周期計算
		Matrix Offset(2, 1);
		Offset.el[0][0] = -0.1;//x軸なので0
		Offset.el[1][0] = -OFFSET_VAL;
		matSub(&var_init.r, &var_init.r, &Offset);	//平衡位置からオフセットの分をずらしておく
	}
	//ゲインを変更してみるとき
	// 前処理
	matSub(&re, &var.r, &var_init.r);			// 手先位置変位
	matSub(&dre, &var.dr, &var_init.dr);		// 手先速度変位

	Matrix F1 = EntityManager::get()->getFinger()->var.F;
	Matrix F2 = EntityManager::get()->getFinger2()->var.F;

	//Fの部分を(F1-F2)/2に変更
	Matrix F12;
	matAdd(&F12, &F1, &F2);	//加わる力は反転しているので足してもF1-F2になる
	Matrix half(2, 2);
	half.el[0][0] = 0.5;
	half.el[1][1] = 0.5;

#if 1//debug
	printf("F1=\n");
	matPrint(&F1);

	printf("F2=\n");
	matPrint(&F2);

	printf("F12=(F1-F2)\n");
	matPrint(&F12);
	matMul(&F12, &half, &F12);
	printf("F12=(F1-F2)/2\n");
	matPrint(&F12);
#endif
	//matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &var.F));		// Integ = ∫Fdt
	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &F12));		// 制約条件付きの時


	// 制御則
	matMul3(&E, &dyn.Mq, &kine.Jinv, &imp.Minv);	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, &dyn.h, matMul4(&Tmp21, &dyn.Mq, &kine.Jinv, &kine.dJ, &var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd(&Tmp21, matMul4(&tauVE, &imp.K, &imp.Cinv, &imp.M, &dre), matMul(&Tmp21, &imp.K, &re));		// Kd*Cd^{-1}*Md*dr+Kd*r
	matSignInv(matMul(&tauVE, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*dr+Kd*r}
	//matMul(&tauIN, matSub(&Tmp22, &E, &kine.Jt), &var.F);		// tauIN = (E-J^T)F
	matSub(&tauIN, matMul(&Tmp22, &E, &F12), matMul(&Tmp22_2, &kine.Jt, &var.F));		// tauIN = (E*((F1+F2)/2) -J^T*F)	制約条件付き

	matMul4(&tauPL, &E, &imp.K, &imp.Cinv, &Integ);		// tauPL = E*Kd*Cd^{-1}∫Fdt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);

	/*printf("tau =\n");
	matPrint(tau);
	Matrix minus(2, 2);
	minus.el[0][0] = -1.0;
	minus.el[1][1] = -1.0;
	matMul(tau, &minus, tau);
	printf("-tau =\n");
	matPrint(tau);*/
	// デバッグ
//	matPrint(&sim->imp.M);	matPrint(&sim->imp.C);	matPrint(&sim->imp.K);
#if 1// print_debug
	std::cout << "fingerID : " << fingerID << " tau = " << std::endl;
	matPrint(tau);		// Inertia Shaping無しの場合は0になればOK
#endif
	return	0;
}



int cFinger::RestrictedCtrlMaxwell2(Matrix* tau)
{

	int	jnt, crd;
	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp22_2(2, 2);
	static Matrix	tauNC(2, 1), tauVE(2, 1), tauIN(2, 1), tauPL(2, 1), E(2, 2);
	static Matrix	Integ(2, 1);
	static Matrix	re(2, 1), dre(2, 1);	// 手先位置変位，手先速度変位
	auto Finger1 = EntityManager::get()->getFinger();
	auto entity = EntityManager::get();
	if (entity->step == 0) {
		armCalcImpPeriod();		// 周期計算
		Matrix Offset(2, 1);
		Offset.el[0][0] = -0.1;//x軸なので0
		Offset.el[1][0] = OFFSET_VAL;

		matSub(&var_init.r, &var_init.r, &Offset);
		printf("initialized ! \n");

	}
	//ゲインを変更するとき
	//imp.K.el[0][0] = 10;
	//imp.K.el[1][1] = 10;
	//比例ゲインを出力
	//printf("impedance K=\n");
	//matPrint(&imp.K);

	matSub(&re, &var.r, &var_init.r);			// 手先位置変位
	matSub(&dre, &var.dr, &var_init.dr);		// 手先速度変位

	//Fの部分を(F1-F2)/2に変更
	Matrix F1 = EntityManager::get()->getFinger()->var.F;
	Matrix F2 = EntityManager::get()->getFinger2()->var.F;

	Matrix F12;				//(F2-F1)/2
	matAdd(&F12, &F2, &F1);
	Matrix half(2, 2);
	half.el[0][0] = 0.5;
	half.el[1][1] = 0.5;
	matMul(&F12, &half, &F12);
#if 0//debug
	printf("F1=\n");
	matPrint(&F1);

	printf("F2=\n");
	matPrint(&F2);

	printf("F12=(F2-F1)\n");
	matPrint(&F12);

	printf("F12=(F2-F1)/2\n");
	matPrint(&F12);
#endif

	//外力Fについて
	//matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &var.F));		// Integ = ∫Fdt
	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &F12));		// 制約条件付きの時


	// 制御則
	matMul3(&E, &dyn.Mq, &kine.Jinv, &imp.Minv);	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, &dyn.h, matMul4(&Tmp21, &dyn.Mq, &kine.Jinv, &kine.dJ, &var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd(&Tmp21, matMul4(&tauVE, &imp.K, &imp.Cinv, &imp.M, &dre), matMul(&Tmp21, &imp.K, &re));		// Kd*Cd^{-1}*Md*dr+Kd*r
	matSignInv(matMul(&tauVE, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*dr+Kd*r}
	//matMul(&tauIN, matSub(&Tmp22, &E, &kine.Jt), &var.F);		// tauIN = (E-J^T)F
	matSub(&tauIN, matMul(&Tmp22, &E, &F12), matMul(&Tmp22_2, &kine.Jt, &var.F));		// tauIN = (E*((F1+F2)/2) -J^T*F)

	matMul4(&tauPL, &E, &imp.K, &imp.Cinv, &Integ);		// tauPL = E*Kd*Cd^{-1}∫Fdt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);


	// デバッグ
#if 1//print_debug
	std::cout << "fingerID : " << fingerID << " tau = " << std::endl;
	matPrint(tau);		// Inertia Shaping無しの場合は0になればOK
#endif
//	matPrint(&sim->imp.M);	matPrint(&sim->imp.C);	matPrint(&sim->imp.K);
	return	0;
}

