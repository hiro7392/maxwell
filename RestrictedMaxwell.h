#pragma once
////////////////////////////////////////////////////////
// ��������t����Maxwell���䑥
// M*ddx + K*C^{-1}*M*dx + K*x = F + K*C^{-1}��Fdt
// x_1=x_2=x_0, P_1=P_2
////////////////////////////////////////////////////////
int cFinger::RestrictedCtrlMaxwell(Matrix* tau)
{
	int	jnt, crd;

	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp22_2(2, 2);
	static Matrix	tauNC(2, 1), tauVE(2, 1), tauIN(2, 1), tauPL(2, 1), E(2, 2);
	static Matrix	Integ(2, 1);
	static Matrix	re(2, 1), dre(2, 1);	// ���ʒu�ψʁC��摬�x�ψ�

	auto entity = EntityManager::get();
	if (entity->step == 0) {
		armCalcImpPeriod();		// �����v�Z
		Matrix Offset(2, 1);
		Offset.el[0][0] = -0.1;//x���Ȃ̂�0
		Offset.el[1][0] = -OFFSET_VAL;
		matSub(&var_init.r, &var_init.r, &Offset);	//���t�ʒu����I�t�Z�b�g�̕������炵�Ă���
	}
	//�Q�C����ύX���Ă݂�Ƃ�
	// �O����
	matSub(&re, &var.r, &var_init.r);			// ���ʒu�ψ�
	matSub(&dre, &var.dr, &var_init.dr);		// ��摬�x�ψ�

	Matrix F1 = EntityManager::get()->getFinger()->var.F;
	Matrix F2 = EntityManager::get()->getFinger2()->var.F;

	//F�̕�����(F1-F2)/2�ɕύX
	Matrix F12;
	matAdd(&F12, &F1, &F2);	//�����͔͂��]���Ă���̂ő����Ă�F1-F2�ɂȂ�
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
	//matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &var.F));		// Integ = ��Fdt
	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &F12));		// ��������t���̎�


	// ���䑥
	matMul3(&E, &dyn.Mq, &kine.Jinv, &imp.Minv);	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, &dyn.h, matMul4(&Tmp21, &dyn.Mq, &kine.Jinv, &kine.dJ, &var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd(&Tmp21, matMul4(&tauVE, &imp.K, &imp.Cinv, &imp.M, &dre), matMul(&Tmp21, &imp.K, &re));		// Kd*Cd^{-1}*Md*dr+Kd*r
	matSignInv(matMul(&tauVE, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*dr+Kd*r}
	//matMul(&tauIN, matSub(&Tmp22, &E, &kine.Jt), &var.F);		// tauIN = (E-J^T)F
	matSub(&tauIN, matMul(&Tmp22, &E, &F12), matMul(&Tmp22_2, &kine.Jt, &var.F));		// tauIN = (E*((F1+F2)/2) -J^T*F)	��������t��

	matMul4(&tauPL, &E, &imp.K, &imp.Cinv, &Integ);		// tauPL = E*Kd*Cd^{-1}��Fdt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);

	/*printf("tau =\n");
	matPrint(tau);
	Matrix minus(2, 2);
	minus.el[0][0] = -1.0;
	minus.el[1][1] = -1.0;
	matMul(tau, &minus, tau);
	printf("-tau =\n");
	matPrint(tau);*/
	// �f�o�b�O
//	matPrint(&sim->imp.M);	matPrint(&sim->imp.C);	matPrint(&sim->imp.K);
#if 1// print_debug
	std::cout << "fingerID : " << fingerID << " tau = " << std::endl;
	matPrint(tau);		// Inertia Shaping�����̏ꍇ��0�ɂȂ��OK
#endif
	return	0;
}

int cFinger::RestrictedCtrlMaxwell2(Matrix* tau)
{

	int	jnt, crd;
	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp22_2(2, 2);
	static Matrix	tauNC(2, 1), tauVE(2, 1), tauIN(2, 1), tauPL(2, 1), E(2, 2);
	static Matrix	Integ(2, 1);
	static Matrix	re(2, 1), dre(2, 1);	// ���ʒu�ψʁC��摬�x�ψ�
	auto Finger1 = EntityManager::get()->getFinger();
	auto entity = EntityManager::get();
	if (entity->step == 0) {
		armCalcImpPeriod();		// �����v�Z
		Matrix Offset(2, 1);
		Offset.el[0][0] = -0.1;//x���Ȃ̂�0
		Offset.el[1][0] = OFFSET_VAL;

		matSub(&var_init.r, &var_init.r, &Offset);
		printf("initialized ! \n");

	}
	matSub(&re, &var.r, &var_init.r);			// ���ʒu�ψ�
	matSub(&dre, &var.dr, &var_init.dr);		// ��摬�x�ψ�

	//F�̕�����(F1-F2)/2�ɕύX
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

	//�O��F�ɂ���
	//matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &var.F));		// Integ = ��Fdt
	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &F12));		// ��������t���̎�


	// ���䑥
	matMul3(&E, &dyn.Mq, &kine.Jinv, &imp.Minv);	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, &dyn.h, matMul4(&Tmp21, &dyn.Mq, &kine.Jinv, &kine.dJ, &var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd(&Tmp21, matMul4(&tauVE, &imp.K, &imp.Cinv, &imp.M, &dre), matMul(&Tmp21, &imp.K, &re));		// Kd*Cd^{-1}*Md*dr+Kd*r
	matSignInv(matMul(&tauVE, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*dr+Kd*r}
	//matMul(&tauIN, matSub(&Tmp22, &E, &kine.Jt), &var.F);		// tauIN = (E-J^T)F
	matSub(&tauIN, matMul(&Tmp22, &E, &F12), matMul(&Tmp22_2, &kine.Jt, &var.F));		// tauIN = (E*((F1+F2)/2) -J^T*F)

	matMul4(&tauPL, &E, &imp.K, &imp.Cinv, &Integ);		// tauPL = E*Kd*Cd^{-1}��Fdt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);
	tau->el[0][0] += imp.G.el[0][1];
	tau->el[0][0] += imp.G.el[0][2];

	printf("heishin tauNC %lf tauPL %lf tauIN %lf tauVE %lf\n", tauNC.el[0][0], tauPL.el[0][0], tauIN.el[0][0], tauVE.el[0][0]/10);
	printf("heishin E = %lf Mq = %lf Id = %lf\n", E.el[0][0], dyn.Mq.el[0][0], rotImp.Ja, &imp.M.el[0][0]);
	// �f�o�b�O
#if 1//print_debug
	std::cout << "fingerID : " << fingerID << " tau = " << std::endl;
	matPrint(tau);		// Inertia Shaping�����̏ꍇ��0�ɂȂ��OK
#endif
//	matPrint(&sim->imp.M);	matPrint(&sim->imp.C);	matPrint(&sim->imp.K);
	return	0;
}

//����֐߂ɂ��Đ���t��maxwell����
int cFinger::RotRestrictedCtrlMaxwell(double* tau)
{
	//	�֐߂̔S�����C��ǉ�
	senkaiDynPara();
	auto entity = EntityManager::get();
	static double allMass = 0.0;
	static double allVolume = 0.0;
	if (entity->step == 0) {
		allMass = senkai_link.getMass() + base.getMass() + link1.getMass() + link2.getMass() + sensor.getMass() + fingerTopCapsule.getMass();
		allVolume = senkai_link.getVolume() + base.getVolume() + link1.getVolume() + link2.getVolume() + fingerTopCapsule.getVolumeHalfSquare();
		//rotImp.Iq = 5.0;//allMass / allVolume;
	}
	// Er�̒l�����������̂ŏC������
	double mu1, mu2, mu_ave;
	auto Finger1 = EntityManager::get()->getFinger();
	auto Finger2 = EntityManager::get()->getFinger2();
	double scale = 100.0;
	mu1 = Finger1->senkai_p_force->t1[0]/scale; mu2 = Finger2->senkai_p_force->t1[0]/scale;
	mu_ave = (fingerID == 1 ? ((mu1 - mu2)/2.0) : ((mu2 - mu1)/2.0));
	printf("mu1 = %lf mu2=%lf\n", mu1, mu2);
	static double Integ=0.0;
	Integ +=(mu_ave*SIM_CYCLE_TIME);
	printf("Integ = %lf\n", Integ);

	rotImp.h = senkai_base_vel;
	double dphi =  senkai_base_jnt-senkai_base_jnt_init;

	static double tauNC, tauPL, tauIN, tauVE, Er;
	printf("allMass = %lf allVolume = %lf \n", allMass, allVolume);
	Er = (rotImp.Iq / rotImp.Ja) * rotImp.Id;
	printf("Er = %lf Iq = %lf Ja = %lf Id = %lf\n", Er, rotImp.Iq, rotImp.Ja, rotImp.Id);
	tauNC = rotImp.h - rotImp.Iq * rotImp.Ja * 0.0 * senkai_base_vel;
	tauPL = ((Er * rotImp.lg * rotImp.K) / (rotImp.C * rotImp.lg))*Integ;
	tauIN = Er * mu_ave - rotImp.Ja * (fingerID == 1 ? mu1 : mu2);
	tauVE = -Er * rotImp.lg * rotImp.K*(rotImp.Id * senkai_base_vel / (rotImp.C * rotImp.lg) + (dphi));
	printf("tauNC %lf tauPL %lf tauIN %lf tauVE %lf\n", tauNC, tauPL, tauIN, tauVE);
	*tau = tauNC + tauPL + tauIN + tauVE+imp.G.el[0][0];//�d�͍��𑫂�
	
	return 0;
}
