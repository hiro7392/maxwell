#pragma once
////////////////////////////////////////////////////////
// 制約条件付きのMaxwell制御則
// M*ddx + K*C^{-1}*M*dx + K*x = F + K*C^{-1}∫Fdt
// x_1=x_2=x_0, P_1=P_2
////////////////////////////////////////////////////////
int cFinger::RestrictedCtrlMaxwell(Matrix* tau)
{
	int	jnt, crd;
	int DIM2_ = 3;
	static Matrix	Tmp21(DIM2_, 1), Tmp22(DIM2_, DIM2_), Tmp22_2(DIM2_, DIM2_);
	static Matrix	tauNC(DIM2_, 1), tauVE(DIM2_, 1), tauIN(DIM2_, 1), tauPL(DIM2_, 1), E(DIM2_, DIM2_);
	static Matrix	Integ(DIM2_, 1);
	static Matrix	re(DIM2_, 1), dre(DIM2_, 1);	// 手先位置変位，手先速度変位

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
	Matrix half(DIM2_, DIM2_);
	half.el[0][0] = 0.5;
	half.el[1][1] = 0.5;
	half.el[2][2] = 0.5;

#if 1//debug
	printf("F1=\n");
	matPrint(&F1);

	printf("F2=\n");
	matPrint(&F2);

	printf("F12=(F2-F1)\n");
	matPrint(&F12);
	matMul(&F12, &F12, &half);
	printf("F12=(F2-F1)/2\n");
	matPrint(&F12);
	//matZero(&F12);
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
#if PRINT_TORQUE

	std::cout << "fingerID : " << fingerID << " tau = " << std::endl;
	matPrint(tau);		// Inertia Shaping無しの場合は0になればOK
#endif
	return	0;
}

int cFinger::RestrictedCtrlMaxwell2(Matrix* tau)
{

	int	jnt, crd;
	//static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp22_2(2, 2);
	//static Matrix	tauNC(2, 1), tauVE(2, 1), tauIN(2, 1), tauPL(2, 1), E(2, 2);
	//static Matrix	Integ(2, 1);
	//static Matrix	re(2, 1), dre(2, 1);	// 手先位置変位，手先速度変位
	int DIM2_ = 3;
	static Matrix	Tmp21(DIM2_, 1), Tmp22(DIM2_, DIM2_), Tmp22_2(DIM2_, DIM2_);
	static Matrix	tauNC(DIM2_, 1), tauVE(DIM2_, 1), tauIN(DIM2_, 1), tauPL(DIM2_, 1), E(DIM2_, DIM2_);
	static Matrix	Integ(DIM2_, 1);
	static Matrix	re(DIM2_, 1), dre(DIM2_, 1);	// 手先位置変位，手先速度変位
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
	matSub(&re, &var.r, &var_init.r);			// 手先位置変位
	matSub(&dre, &var.dr, &var_init.dr);		// 手先速度変位

	//Fの部分を(F1-F2)/2に変更
	Matrix F1 = EntityManager::get()->getFinger()->var.F;
	Matrix F2 = EntityManager::get()->getFinger2()->var.F;

	Matrix F12;				//(F2-F1)/2
	matAdd(&F12, &F2, &F1);
	Matrix half(DIM2_, DIM2_);
	half.el[0][0] = 0.5;
	half.el[1][1] = 0.5;
	half.el[2][2] = 0.5;
	matMul(&F12, &half, &F12);
#if 1//debug
	printf("F1=\n");
	matPrint(&F1);

	printf("F2=\n");
	matPrint(&F2);

	printf("F12=(F2-F1)\n");
	matPrint(&F12);

	matMul(&F12, &F12, &half);

	printf("F12=(F2-F1)/2\n");
	matPrint(&F12);
	//matZero(&F12);
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
	tau->el[0][0] += imp.G.el[0][1];
	tau->el[0][1] += imp.G.el[0][2];
#if PRINT_MAXWELL_PARAM
	//printf("mu1 = %lf mu2=%lf\n", mu1, mu2);
	printf("heishin tauNC %lf tauPL %lf tauIN %lf tauVE %lf\n", tauNC.el[0][0], tauPL.el[0][0], tauIN.el[0][0], tauVE.el[0][0]/10);
	printf("heishin E = %lf Mq = %lf Id = %lf\n", E.el[0][0], dyn.Mq.el[0][0], rotImp.Ja, imp.M.el[0][0]);
#endif
	// デバッグ
#if PRINT_TORQUE
	std::cout << "fingerID : " << fingerID << " tau = " << std::endl;
	matPrint(tau);		// Inertia Shaping無しの場合は0になればOK
#endif
//	matPrint(&sim->imp.M);	matPrint(&sim->imp.C);	matPrint(&sim->imp.K);
	return	0;
}

//旋回関節について制約付きmaxwell制御
int cFinger::RotRestrictedCtrlMaxwell(double* tau)
{
	//	関節の粘性摩擦を追加
	senkaiDynPara();
	auto entity = EntityManager::get();
	/*if (entity->step == 0) {
		senkai_base_jnt_init -= (fingerID==1?1:-1)*OFFSET_VAL_SENKAI;
	}*/
	// Erの値がおかしいので修正する
	double mu1, mu2, mu_ave;
	auto Finger1 = EntityManager::get()->getFinger();
	auto Finger2 = EntityManager::get()->getFinger2();
	double scale = 100.0;
	mu1 = Finger1->senkai_p_force->t1[0]/scale; mu2 = Finger2->senkai_p_force->t1[0]/scale;
	mu_ave = (fingerID == 1 ? ((mu1 - mu2)/2.0) : ((mu2 - mu1)/2.0));
	static double Integ=0.0;
	Integ +=(mu_ave*SIM_CYCLE_TIME);
#if PRINT_MAXWELL_PARAM
	printf("mu1 = %lf mu2=%lf\n", mu1, mu2);
	printf("Integ = %lf\n", Integ);
#endif
	//	関節の粘性摩擦力を追加
	rotImp.h = senkai_base_vel * rotImp.V*(fingerID ==1?1:-1);
	double dphi =  senkai_base_jnt-senkai_base_jnt_init+ (fingerID == 1 ? 1 : -1) * OFFSET_VAL_SENKAI;

	static double tauNC, tauPL, tauIN, tauVE, Er;
	Er = (rotImp.Iq / rotImp.Ja) * rotImp.Id;
#if PRINT_MAXWELL_PARAM
	printf("Er = %lf Iq = %lf Ja = %lf Id = %lf,dphi = %lf\n", Er, rotImp.Iq, rotImp.Ja, rotImp.Id,dphi);
#endif
	tauNC = rotImp.h - rotImp.Iq * rotImp.Ja * 0.0 * senkai_base_vel;
	tauPL = ((Er * rotImp.lg * rotImp.K) / (rotImp.C * rotImp.lg))*Integ;
	tauIN = Er * mu_ave - rotImp.Ja * (fingerID == 1 ? mu1 : mu2);
	tauVE = -Er * rotImp.lg * rotImp.K*(rotImp.Id * senkai_base_vel / (rotImp.C * rotImp.lg) + (dphi));
#if PRINT_MAXWELL_PARAM
	printf("tauNC %lf tauPL %lf tauIN %lf tauVE %lf\n", tauNC, tauPL, tauIN, tauVE);
#endif
	*tau = tauNC + tauPL + tauIN + tauVE+imp.G.el[0][0];//重力項を足す
	//*tau = 2.0;
	
	return 0;
}
