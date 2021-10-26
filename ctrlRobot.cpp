#include "simMain.h"
#include "ctrlRobot.h"
#include "setRobot.h"

////////////////////////////////////////////////////////
// 弾塑性ハイブリッド制御則(Maxwell+Voigt)
////////////////////////////////////////////////////////
int ctrlHybrid(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Fint, const Matrix *Md, const Matrix *Cd, const Matrix *Kd)
{
	auto _this = EntityManager::get()->getFinger();
	int	jnt, crd;
	static Matrix	S, I;	// Maxwell制御がS, Voigt制御がI-S
	static Matrix	Jinv, Jt, Tmp21, Tmp21_1, Tmp21_2, Tmp22, Tmp22_1, Tmp22_2;
	static Matrix	tauNC, tauVE, tauIN, tauPL, E;
	auto entity = EntityManager::get();
	// 初期化

	if(entity->step == 0){

		matInit(&S,2,2); matUnit(matInit(&I,2,2));
		matInit(&Jinv,2,2);	matInit(&Jt,2,2);
		matInit(&Tmp21,2,1); matInit(&Tmp21_1,2,1); matInit(&Tmp21_2,2,1); matInit(&Tmp22,2,2); matInit(&Tmp22_1,2,2); matInit(&Tmp22_2,2,2);
		matInit(&tauNC,2,1); matInit(&tauVE,2,1); matInit(&tauIN,2,1); matInit(&tauPL,2,1); matInit(&E,2,2);
		S.el[0][0] = 1.0;	S.el[1][1] = 0.0;	// x方向がMaxwell制御, y方向がVoigt制御
		for(crd=0;crd<2;crd++){
			if(fabs(S.el[crd][crd]) < MAT_EPS && _this->imp.C.el[crd][crd]*_this->imp.C.el[crd][crd] < 4*_this->imp.M.el[crd][crd]*_this->imp.K.el[crd][crd] ){		// Voigt
				_this->imp.T[crd] = 2*PI/sqrt(_this->imp.K.el[crd][crd]/_this->imp.M.el[crd][crd]-_this->imp.C.el[crd][crd]*_this->imp.C.el[crd][crd]/(4*_this->imp.M.el[crd][crd]*_this->imp.M.el[crd][crd]));
				printf("T[%d]=%f\n", crd, _this->imp.T[crd]);
			}else if(_this->imp.C.el[crd][crd]*_this->imp.C.el[crd][crd] > _this->imp.M.el[crd][crd]*_this->imp.K.el[crd][crd]/4 ){		// Maxwell
				_this->imp.T[crd] = 2*PI/sqrt(_this->imp.K.el[crd][crd]/_this->imp.M.el[crd][crd]-_this->imp.K.el[crd][crd]*_this->imp.K.el[crd][crd]/(4*_this->imp.C.el[crd][crd]*_this->imp.C.el[crd][crd]));
				printf("T[%d]=%f\n", crd, _this->imp.T[crd]);
			}
		}
	}
	// MaxwellとVoigtの方向切替
	if(entity->step == IMP_SWITCH_STEP){ S.el[0][0] = 0.0;	S.el[1][1] = 1.0; }		// x方向がVoigt制御, y方向がMaxwell制御
	// 制御則
	matTrans(&Jt, J);	matInv(&Jinv, NULL, J);	// J^T, J^{-1}
	matMul3(&E, Mq, &Jinv, matInv(&Tmp22, NULL, Md));	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, h, matMul4(&Tmp21, Mq, &Jinv, dJ, dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd3(&Tmp21, matMul5(&Tmp21_1, Kd, matInv(&Tmp22, NULL, Cd), Md, &S, dre), matMul3(&Tmp21_2, Cd, matSub(&Tmp22, &I, &S), dre), matMul(&Tmp21, Kd, re));
	matMulScl(&tauVE, -1, matMul(&Tmp21, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*S*dr+Cd*(I-S)*dr+Kd*r}
	matMul(&tauIN, matSub(&Tmp22, &E, &Jt), F);		// tauIN = (E-J^T)F
	matMul5(&tauPL, &E, Kd, matInv(&Tmp22, NULL, Cd), &S, Fint);	// tauPL = Kd*Cd^{-1}*S*Fint
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);

	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則
// 可変インピーダンス
////////////////////////////////////////////////////////
int ctrlMaxwellVar(cFinger *sim, Matrix *tau)
{
	int	jnt, crd;
	static Matrix	Tmp21, Tmp22;
	static Matrix	tauNC, tauVE, tauIN, tauPL, E;
	static Matrix	Integ, Integ2, Diff;
	static Matrix	re, dre;	// 手先位置変位，手先速度変位

	auto entity = EntityManager::get();
	if (entity->step == 0){
		matInit(&Tmp21, 2, 1); matInit(&Tmp22, 2, 2);
		matInit(&tauNC, 2, 1); matInit(&tauVE, 2, 1); matInit(&tauIN, 2, 1); matInit(&tauPL, 2, 1); matInit(&E, 2, 2);
		matInit(&Integ, 2, 1);	matInit(&Integ2, 2, 1);	matInit(&Diff, 2, 2);
		matInit(&re,2,1); matInit(&dre,2,1);
		sim->armCalcImpPeriod();		// 周期計算
	}
	// 前処理
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matCopy(&dre, &sim->var.dr);	// 手先速度変位（初期速度が0の場合のみ適用）
	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, matMul(&Tmp21, &sim->imp.Cinv, &sim->var.F)));		// Integ = ∫[Cd^{-1}*F]dt
	matMul(&Diff, &sim->imp.Cinv, matSub(&Tmp22, &sim->imp.dM, matMul3(&Tmp22, &sim->imp.dC, &sim->imp.Cinv, &sim->imp.M)));	// Diff = d[Cd^{-1}*Md] = Cd^{-1}*[dMd-dCd*Cd^{-1}*Md]
	matAdd(&Integ2, &Integ2, matMulScl(&Tmp21, SIM_CYCLE_TIME, matMul(&Tmp21, &Diff, &dre)));		// Integ2 = ∫[d[Cd^{-1}*Md]*dr]dt
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &sim->kine.Jinv, &sim->imp.Minv);	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, &sim->dyn.h, matMul4(&Tmp21, &sim->dyn.Mq, &sim->kine.Jinv, &sim->kine.dJ, &sim->var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd(&Tmp21, matMul4(&tauVE, &sim->imp.K, &sim->imp.Cinv, &sim->imp.M, &dre), matMul(&Tmp21, &sim->imp.K, &re));		// Kd*Cd^{-1}*Md*dr+Kd*r
//	matMulScl(&tauVE, -1, matMul(&Tmp21, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*dr+Kd*r}
	matSub(&tauVE, &Integ2, matMul(&Tmp21, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*dr+Kd*r} + ∫[d[Cd^{-1}*Md]*dr]dt
	matMul(&tauIN, matSub(&Tmp22, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matMul3(&tauPL, &E, &sim->imp.K, &Integ);		// tauPL = E*Kd∫[Cd^{-1}*F]dt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);
	// デバッグ
	matPrint(&tauIN);		// Inertia Shaping無しの場合は0になればOK
	return	0;
}

////////////////////////////////////////////////////////
// SLS制御則(標準線形固体モデル)
////////////////////////////////////////////////////////
int ctrlSLS(cFinger *sim, Matrix *tau)
{
	int	jnt, crd;
	static Matrix	Tmp21, Tmp22;
	static Matrix	tauNC, tauVE, tauIN, tauPL, E;
	static Matrix	tauP, tauI, tauD;		// tauVEのPID成分
	static Matrix	IntegF, IntegX;
	static Matrix	re, dre;	// 手先位置変位，手先速度変位

	auto entity = EntityManager::get();
	if (entity->step == 0){
		matInit(&Tmp21, 2, 1); matInit(&Tmp22, 2, 2);
		matInit(&tauNC, 2, 1); matInit(&tauVE, 2, 1); matInit(&tauIN, 2, 1); matInit(&tauPL, 2, 1); matInit(&E, 2, 2);
		matInit(&tauP, 2, 1); matInit(&tauI, 2, 1); matInit(&tauD, 2, 1);
		matInit(&IntegF, 2, 1);	matInit(&IntegX, 2, 1);
		matInit(&re,2,1); matInit(&dre,2,1);
		sim->armCalcImpPeriod();		// 周期計算
	}
	// 前処理
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matCopy(&dre, &sim->var.dr);	// 手先速度変位（初期速度が0の場合のみ適用）
	matAdd(&IntegF, &IntegF, matMulScl(&Tmp21, SIM_CYCLE_TIME, &sim->var.F));		// IntegF = ∫Fdt
	matAdd(&IntegX, &IntegX, matMulScl(&Tmp21, SIM_CYCLE_TIME, &re));		// IntegX = ∫rdt
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &sim->kine.Jinv, &sim->imp.Minv);	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, &sim->dyn.h, matMul4(&Tmp21, &sim->dyn.Mq, &sim->kine.Jinv, &sim->kine.dJ, &sim->var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matMul4(&tauD, &sim->imp.K, &sim->imp.Cinv, &sim->imp.M, &dre);	// tauD = Kd*Cd^{-1}*Md*dr
	matMul(&tauP, matAdd(&Tmp22, &sim->imp.K, &sim->imp.K0), &re);	// tauP = (Kd+K0d)*r
	matMul4(&tauI, &sim->imp.K, &sim->imp.Cinv, &sim->imp.K0, &IntegX);	// tauI = Kd*Cd^{-1}*K0d∫rdt
	matMulScl(&tauVE, -1, matMul(&Tmp21, &E, matAdd3(&Tmp21, &tauP, &tauI, &tauD)));	// tauVE = -E{tauP+tauI+tauD}
	matMul(&tauIN, matSub(&Tmp22, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matMul4(&tauPL, &E, &sim->imp.K, &sim->imp.Cinv, &IntegF);		// tauPL = E*Kd*Cd^{-1}∫Fdt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);
	// デバッグ
//	matPrint(&tauIN);		// Inertia Shaping無しの場合は0になればOK
//	matPrint(&sim->imp.M);	matPrint(&sim->imp.C);	matPrint(&sim->imp.K);
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則+位置制御インナーループ
////////////////////////////////////////////////////////

// 4次ルンゲクッタ法(状態空間表現 dX = AX+BU )
int rk4(cFinger *sim, Matrix *Xnext, Matrix *X, Matrix *A, Matrix *B, Matrix *U, double dt)
{
	auto entity = EntityManager::get();
	static Matrix	K1, K2, K3, K4, K;
	static Matrix	Tmp41_1, Tmp41_2;		// 途中計算
	if (entity->step == 0){
		matInit(&K1,4,1); matInit(&K2,4,1); matInit(&K3,4,1); matInit(&K4,4,1); matInit(&K,4,1); 
		matInit(&Tmp41_1,4,1); matInit(&Tmp41_2,4,1);
	}
	matAdd(&K1, matMul(&Tmp41_1, A, X), matMul(&Tmp41_2, B, U));		// K1 = A*X+B*U
	matAdd(&K2, &K1, matMulScl(&Tmp41_1, 0.5*dt, matMul(&Tmp41_2, A, &K1)));		// K2 = A*(X+0.5*K1*dt)+B*U = K1+0.5*dt*A*K1
	matAdd(&K3, &K1, matMulScl(&Tmp41_1, 0.5*dt, matMul(&Tmp41_2, A, &K2)));		// K3 = A*(X+0.5*K2*dt)+B*U = K1+0.5*dt*A*K2
	matAdd(&K4, &K1, matMulScl(&Tmp41_1, dt, matMul(&Tmp41_2, A, &K3)));		// K4 = A*(X+K3*dt)+B*U = K1+dt*A*K3
	matMulScl(&K, dt/6.0, matAdd4(&Tmp41_1, &K1, matMulScl(&Tmp41_1, 2.0, &K2), matMulScl(&Tmp41_2, 2.0, &K3), &K4));		// K = (K1+2.0*K2+2.0*K3+K4)*dt/6.0
	matAdd(Xnext, X, &K);		// X(t+dt) = X+K
	return	0;
}

int ctrlMaxwellInnerLoop(cFinger *sim, Matrix *tau)
{
	int	jnt, crd;
	static Matrix	Tmp21, Tmp22, Tmp21_2;
	static Matrix	tauNC, tauVE, tauIN, tauPL, E;
	static Matrix	Integ;
	static Matrix	re, dre, rc, drc;	// 手先位置速度変位，目標位置速度変位
	static Matrix	A, B, X, U, Xnext;	// 状態空間
//#define	GAIN_INNERLOOP	20		// 
	constexpr double GAIN_INNERLOOP = 20;		// 
	double	GpVal[] = {GAIN_INNERLOOP*GAIN_INNERLOOP/4.0, GAIN_INNERLOOP*GAIN_INNERLOOP/4.0};	// 臨界減衰
	double	GvVal[] = {GAIN_INNERLOOP, GAIN_INNERLOOP};

	auto entity = EntityManager::get();
	if (entity->step == 0){
		// 行列初期化
		matInit(&Tmp21, 2, 1); matInit(&Tmp22, 2, 2); matInit(&Tmp21_2, 2, 1); 
		matInit(&tauNC, 2, 1); matInit(&tauVE, 2, 1); matInit(&tauIN, 2, 1); matInit(&tauPL, 2, 1); matInit(&E, 2, 2);
		matInit(&Integ, 2, 1);
		matInit(&re,2,1); matInit(&dre,2,1);
		matInit(&A,4,4); matInit(&B,4,2); matInit(&X,4,1); matInit(&U,2,1); matInit(&Xnext,4,1);
		// 初期設定
		matShareInit(&rc,2,1); matShareInit(&drc,2,1); matShareBlock(&rc,&X,0,0); matShareBlock(&drc,&X,2,0);	// X = [rc; drc]
		matSetValDiag(&sim->imp.Gp, GpVal); matSetValDiag(&sim->imp.Gv, GvVal);	// ゲイン設定
		matAssign(&A, matUnit(&Tmp22), 0, 2);		// I
		matAssign(&A, matSignInv(matMul(&Tmp22, &sim->imp.Minv, &sim->imp.K)), 2, 0);		// -M^{-1}*K
		matAssign(&A, matSignInv(matMul4(&Tmp22, &sim->imp.Minv, &sim->imp.K, &sim->imp.Cinv, &sim->imp.M)), 2, 2);		// -M^{-1}*K*C^{-1}*M
		matAssign(&B, &sim->imp.Minv, 2, 0);		// M^{-1}
		sim->armCalcImpPeriod();		// 周期計算
	}
	// 前処理
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matCopy(&dre, &sim->var.dr);	// 手先速度変位（初期速度が0の場合のみ適用）
#if 0
	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &sim->var.F));		// Integ = ∫Fdt
#else
	if (entity->step > 0)	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &sim->var.F));		// Integ = ∫Fdt
#endif
	// インナーループ目標値計算
	matCopy(&X, &Xnext);	// 1サイクル前の計算値を初期値として代入
	matAdd(&U, &sim->var.F, matMul3(&Tmp22, &sim->imp.K, &sim->imp.Cinv, &Integ));		// U = F+K*C^{-1}*∫Fdt
	rk4(sim, &Xnext, &X, &A, &B, &U, SIM_CYCLE_TIME);
	for (crd = 0; crd<DIM2; crd++)	sim->ref_eff_pos[crd] = rc.el[crd][0];		// 変数保存
	for (crd = 0; crd<DIM2; crd++)	sim->ref_eff_vel[crd] = drc.el[crd][0];		// 変数保存
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &sim->kine.Jinv, &sim->imp.Minv);	// E = Mq*J^{-1}*M^{-1}
	matAdd(&Tmp21, matMulSub(&Tmp21, &sim->imp.Gv, &drc, &dre), matMulSub(&Tmp21_2, &sim->imp.Gp, &rc, &re));		// Gv*(drc-dre)+Gp*(rc-re)
	matSub(&Tmp21, &Tmp21, matMul(&Tmp21_2, &sim->kine.dJ, &sim->var.dq));		// Gv*(drc-dre)+Gp*(rc-re)-dJ*dq
	matAdd(&tauNC, &sim->dyn.h, matMul3(&Tmp21_2, &sim->dyn.Mq, &sim->kine.Jinv, &Tmp21));	// tauNC = h+Mq*J^{-1}*(Gv*(drc-dre)+Gp*(rc-re)-dJ*dq)
	matAdd(&Tmp21, matMul4(&tauVE, &sim->imp.K, &sim->imp.Cinv, &sim->imp.M, &drc), matMul(&Tmp21, &sim->imp.K, &rc));		// K*C^{-1}*M*drc+K*rc ← 通常のMaxwellと違い目標値を代入
	matSignInv(matMul(&tauVE, &E, &Tmp21));	// tauVE = -E{K*C^{-1}*M*drc+K*rc}
	matMul(&tauIN, matSub(&Tmp22, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matMul4(&tauPL, &E, &sim->imp.K, &sim->imp.Cinv, &Integ);		// tauPL = E*K*C^{-1}*∫Fdt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則(作業座標)+インナーループ(関節座標)
// デバッグ中．初期トルクが大きい．偏差が残る．ゲインを大きくすると発散．
////////////////////////////////////////////////////////
int ctrlMaxwellInnerLoopJntSpace(cFinger *sim, Matrix *tau)
{
	static Matrix	Tmp21, Tmp22, Tmp21_2;
	static Matrix	tauNC, tauVE, tauIN, tauPL, Ec;
	static Matrix	Integ;
	static Matrix	re, dre, rc, drc;	// 手先位置速度変位，目標位置速度変位
	static Matrix	qc, dqc;	// 目標関節角変位，目標関節速度変位
	static Matrix	A, B, X, U, Xnext, dX;	// 状態空間
	static Variable	var_c, var_e;		// 目標値変数, 変位変数
	static Kinematics	kine_c;		// 目標値運動学変数
//#define	GAIN_INNERLOOP	20		// 
//	double	GpVal[] = {GAIN_INNERLOOP*GAIN_INNERLOOP/4.0, GAIN_INNERLOOP*GAIN_INNERLOOP/4.0};	// 臨界減衰
//	double	GvVal[] = {GAIN_INNERLOOP, GAIN_INNERLOOP};
	double	GpVal[] = {800, 800};
	double	GvVal[] = {800, 800};

	auto entity = EntityManager::get();
	if (entity->step == 0){
		// 行列初期化
		matInit(&Tmp21, 2, 1); matInit(&Tmp22, 2, 2); matInit(&Tmp21_2, 2, 1); 
		matInit(&tauNC, 2, 1); matInit(&tauVE, 2, 1); matInit(&tauIN, 2, 1); matInit(&tauPL, 2, 1); matInit(&Ec, 2, 2);
		matInit(&Integ, 2, 1);
		matInit(&re,2,1); matInit(&dre,2,1);
//		matInit(&var_c,2,1); matInit(&drc,2,1);
		matInit(&A,4,4); matInit(&B,4,2); matInit(&X,4,1); matInit(&U,2,1); matInit(&Xnext,4,1); matInit(&dX,4,1);
//		sim->armInitMatVar(&var_c); //sim->armInitMatVar(&var_e); //sim->armInitMatKine(&kine_c);
		// 初期設定
		matShareInit(&rc,2,1); matShareInit(&drc,2,1); matShareBlock(&rc,&X,0,0); matShareBlock(&drc,&X,2,0);	// X = [rc; drc]
//		matShareInit(&var_c.r,2,1); matShareInit(&var_c.dr,2,1); matShareBlock(&var_c.r,&X,0,0); matShareBlock(&var_c.dr,&X,2,0);	// X = [rc; drc]
//		matShareInit(&var_c.ddr,2,1); matShareBlock(&var_c.ddr,&dX,2,0);	// dX = [drc; ddrc]
		matSetValDiag(&sim->imp.Gp, GpVal); matSetValDiag(&sim->imp.Gv, GvVal);	// ゲイン設定
		matAssign(&A, matUnit(&Tmp22), 0, 2);		// I
		matAssign(&A, matSignInv(matMul(&Tmp22, &sim->imp.Minv, &sim->imp.K)), 2, 0);		// -M^{-1}*K
		matAssign(&A, matSignInv(matMul4(&Tmp22, &sim->imp.Minv, &sim->imp.K, &sim->imp.Cinv, &sim->imp.M)), 2, 2);		// -M^{-1}*K*C^{-1}*M
		matAssign(&B, &sim->imp.Minv, 2, 0);		// M^{-1}
		sim->armCalcImpPeriod();		// 周期計算
	}
	// 前処理
//	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
//	matCopy(&dre, &sim->var.dr);	// 手先速度変位
	matSub(&var_e.q, &sim->var.q, &sim->var_init.q);		// 関節角変位
	matSub(&var_e.dq, &sim->var.dq, &sim->var_init.dq);		// 関節速度変位
#if 0
	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &sim->var.F));		// Integ = ∫Fdt
#else
	if (entity->step > 0)	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &sim->var.F));		// Integ = ∫Fdt
#endif
	// インナーループ目標値計算
	matCopy(&X, &Xnext);	// 1サイクル前の計算値を初期値として代入(初回は0で上書き)
	matAdd(&U, &sim->var.F, matMul3(&Tmp22, &sim->imp.K, &sim->imp.Cinv, &Integ));		// U = F+K*C^{-1}*∫Fdt
	rk4(sim, &Xnext, &X, &A, &B, &U, SIM_CYCLE_TIME);
	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_pos[crd] = rc.el[crd][0];		// 変数保存
	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_vel[crd] = drc.el[crd][0];		// 変数保存
	matAdd(&var_c.r, &rc, &sim->var_init.r); matAdd(&var_c.dr, &drc, &sim->var_init.dr);	// 変位から絶対値へ変換
//	armJacob(sim, &kine_c, &var_c);
	sim->armInvKine(&kine_c, &var_c);
	for (int crd = 0; crd<ARM_JNT; crd++)	sim->ref_jnt_pos[crd] = var_c.q.el[crd][0];		// 変数保存
	for (int crd = 0; crd<ARM_JNT; crd++)	sim->ref_jnt_vel[crd] = var_c.dq.el[crd][0];		// 変数保存
	matPrint(&var_c.r);
	// 制御則
	matMul3(&Ec, &sim->dyn.Mq, &kine_c.Jinv, &sim->imp.Minv);	// Ec = Mq*Jc^{-1}*M^{-1}
	matAdd(&Tmp21, matMulSub(&Tmp21, &sim->imp.Gv, &var_c.dq, &sim->var.dq), matMulSub(&Tmp21_2, &sim->imp.Gp, &var_c.q, &sim->var.q));		// Gv*(dqc-dqe)+Gp*(qc-qe)
	matSub(&Tmp21, &Tmp21, matMul3(&Tmp21_2, &kine_c.Jinv, &kine_c.dJ, &var_c.dq));		// Gv*(dqc-dqe)+Gp*(qc-qe)-Jc^{-1}*dJc*dqc
	matAdd(&tauNC, &sim->dyn.h, matMul(&Tmp21_2, &sim->dyn.Mq, &Tmp21));	// tauNC = h+Mq*{Gv*(dqc-dqe)+Gp*(qc-qe)-Jc^{-1}*dJc*dqc}
	matAdd(&Tmp21, matMul4(&tauVE, &sim->imp.K, &sim->imp.Cinv, &sim->imp.M, &var_c.dr), matMul(&Tmp21, &sim->imp.K, &var_c.r));		// K*C^{-1}*M*drc+K*rc ← 通常のMaxwellと違い目標値を代入
	matSignInv(matMul(&tauVE, &Ec, &Tmp21));	// tauVE = -Ec{K*C^{-1}*M*drc+K*rc}
	matMul(&tauIN, matSub(&Tmp22, &Ec, &sim->kine.Jt), &sim->var.F);		// tauIN = (Ec-J^T)F
	matMul4(&tauPL, &Ec, &sim->imp.K, &sim->imp.Cinv, &Integ);		// tauPL = Ec*K*C^{-1}*∫Fdt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則+位置制御インナーループ（可変インピーダンス対応）
// 目標値を Mddr = F-K(r-p), K(r-p) = Cdp から計算
// 定数インピーダンスのときはデバッグ済み
// ctrlMaxwellInnerLoopと計算方法が違い加速度の値が少しずつずれていくので，インナーループゲインが0のとき(フィードバックがかかっていない)は値がずれてくる
////////////////////////////////////////////////////////
// 4次ルンゲクッタ法(状態空間表現 dX = AX+BU )
int rk4Acc(cFinger *sim, Matrix *Xnext, Matrix *dX, Matrix *X, Matrix *A, Matrix *B, Matrix *U, double dt)
{
	static Matrix	K1, K2, K3, K4, K;
	static Matrix	Tmp_1, Tmp_2;		// 
	auto entity = EntityManager::get();
	if (entity->step == 0) {
		matInit(&K1, X->row, X->col); matInit(&K2, X->row, X->col); matInit(&K3, X->row, X->col); matInit(&K4, X->row, X->col); matInit(&K, X->row, X->col);
		matInit(&Tmp_1, X->row, X->col); matInit(&Tmp_2, X->row, X->col);
	}
	matAdd(&K1, matMul(&Tmp_1, A, X), matMul(&Tmp_2, B, U));		// K1 = A*X+B*U
	matAdd(&K2, &K1, matMulScl(&Tmp_1, 0.5*dt, matMul(&Tmp_2, A, &K1)));		// K2 = A*(X+0.5*K1*dt)+B*U = K1+0.5*dt*A*K1
	matAdd(&K3, &K1, matMulScl(&Tmp_1, 0.5*dt, matMul(&Tmp_2, A, &K2)));		// K3 = A*(X+0.5*K2*dt)+B*U = K1+0.5*dt*A*K2
	matAdd(&K4, &K1, matMulScl(&Tmp_1, dt, matMul(&Tmp_2, A, &K3)));		// K4 = A*(X+K3*dt)+B*U = K1+dt*A*K3
	matMulScl(&K, dt / 6.0, matAdd4(&Tmp_1, &K1, matMulScl(&Tmp_1, 2.0, &K2), matMulScl(&Tmp_2, 2.0, &K3), &K4));		// K = (K1+2.0*K2+2.0*K3+K4)*dt/6.0
	matAdd(Xnext, X, &K);		// X(t+dt) = X+K
	matCopy(dX, &K1);		// dX = A*X+B*U
	return	0;
}

int ctrlMaxwellInnerLoopImplicit(cFinger *sim, Matrix *tau)
{
	int	jnt, crd;
	static Matrix	Tmp21, Tmp22, Tmp21_2;
	static Matrix	tauNC, tauVE, tauIN, tauPL, E;
	static Matrix	re, dre, rc, drc, ddrc;	// 手先位置速度変位，目標位置速度変位
	static Matrix	A, B, X, U, Xnext, dX;	// 状態空間
//#define	GAIN_INNERLOOP	20		// 
	constexpr double GAIN_INNERLOOP = 20;		// 
	double	GpVal[] = {GAIN_INNERLOOP*GAIN_INNERLOOP/4.0, GAIN_INNERLOOP*GAIN_INNERLOOP/4.0};	// 臨界減衰
	double	GvVal[] = {GAIN_INNERLOOP, GAIN_INNERLOOP};

	auto entity = EntityManager::get();
	if (entity->step == 0){
		// 行列初期化
		matInit(&Tmp21, 2, 1); matInit(&Tmp22, 2, 2); matInit(&Tmp21_2, 2, 1); 
		matInit(&tauNC, 2, 1); matInit(&tauVE, 2, 1); matInit(&tauIN, 2, 1); matInit(&tauPL, 2, 1); matInit(&E, 2, 2);
		matInit(&re,2,1); matInit(&dre,2,1);
		matInit(&A,6,6); matInit(&B,6,2); matInit(&X,6,1); matInit(&U,2,1); matInit(&Xnext,6,1); matInit(&dX, 6, 1);
		// 初期設定
		matShareInit(&rc,2,1); matShareInit(&drc,2,1); matShareBlock(&rc,&X,0,0); matShareBlock(&drc,&X,2,0);	// X = [rc; drc; p]
		matShareInit(&ddrc, 2, 1); matShareBlock(&ddrc, &dX, 2, 0);	// dX = [drc; ddrc; dp]
		matSetValDiag(&sim->imp.Gp, GpVal); matSetValDiag(&sim->imp.Gv, GvVal);	// ゲイン設定
		matAssign(&A, matUnit(&Tmp22), 0, 2);		// I
		sim->armCalcImpPeriod();		// 周期計算
	}
	// 前処理
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matCopy(&dre, &sim->var.dr);	// 手先速度変位（初期速度が0の場合のみ適用）
	// インナーループ目標値計算
	matAssign(&A, matSignInv(matMul(&Tmp22, &sim->imp.Minv, &sim->imp.K)), 2, 0);		// -M^{-1}*K
	matAssign(&A, matMul(&Tmp22, &sim->imp.Minv, &sim->imp.K), 2, 4);		// M^{-1}*K
	matAssign(&A, matMul(&Tmp22, &sim->imp.Cinv, &sim->imp.K), 4, 0);		// C^{-1}*K
	matAssign(&A, matSignInv(matMul(&Tmp22, &sim->imp.Cinv, &sim->imp.K)), 4, 4);		// -C^{-1}*K
	matAssign(&B, &sim->imp.Minv, 2, 0);		// M^{-1}
	matCopy(&X, &Xnext);	// 1サイクル前の計算値を初期値として代入
	rk4Acc(sim, &Xnext, &dX, &X, &A, &B, &sim->var.F, SIM_CYCLE_TIME);		// Xを次のサイクルの値で更新
	for (crd = 0; crd<DIM2; crd++)	sim->ref_eff_pos[crd] = rc.el[crd][0];		// 変数保存
	for (crd = 0; crd<DIM2; crd++)	sim->ref_eff_vel[crd] = drc.el[crd][0];		// 変数保存
//	for (crd = 0; crd<DIM2; crd++)	sim->ref_eff_acc[crd] = ddrc.el[crd][0];		// 変数保存
	// 制御則
	matAdd3(&Tmp21, &ddrc, matMulSub(&Tmp21, &sim->imp.Gv, &drc, &dre), matMulSub(&Tmp21_2, &sim->imp.Gp, &rc, &re));		// ddrc+Gv*(drc-dre)+Gp*(rc-re)
	matSub(&Tmp21, &Tmp21, matMul(&Tmp21_2, &sim->kine.dJ, &sim->var.dq));		// ddrc+Gv*(drc-dre)+Gp*(rc-re)-dJ*dq
	matAdd(&tauNC, &sim->dyn.h, matMul3(&Tmp21_2, &sim->dyn.Mq, &sim->kine.Jinv, &Tmp21));	// tauNC = h+Mq*J^{-1}*(ddrc+Gv*(drc-dre)+Gp*(rc-re)-dJ*dq)
	matSignInv(matMul(&tauIN, &sim->kine.Jt, &sim->var.F));		// tauIN = -J^T*F
	matAdd(tau, &tauNC, &tauIN);		// tau = tauNC+tauIN
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則（畳み込み表現）
// dK = 0, dC = 0, Mは時変対応
// 現在はK, Cが対角行列の時のみ対応
// F = M*ddr+dM*dr+∫_0^t{exp(-(t-τ)*K*C^{-1})*K*dr}dτ
////////////////////////////////////////////////////////
int ctrlMaxwellConv(cFinger *sim, Matrix *tau)
{
	static Matrix	Tmp21(2,1), Tmp22(2,2), Tmp21_2(2,1);
	static Matrix	tauNC(2,1), tauVE(2,1), tauIN(2,1), E(2,2);
	static Matrix	Integ(2,1);
	static Matrix	re(2,1), dre(2,1);	// 手先位置速度変位，目標位置速度変位
	static Matrix	dre_prev(2,1);	// 
	static Matrix	Exp(2,2), Inc(2,1), Expt(2,2);	// 

	auto entity = EntityManager::get();
	if (entity->step == 0) {
		// 初期設定
		matMulScl(&Tmp22, -SYSTEM_CYCLE_TIME, matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv));		//  = -Δt*K*C^{-1}
		double	GpVal[] = {exp(Tmp22.el[0][0]), exp(Tmp22.el[1][1])};	// e^{-Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
//		matSetValDiag(&Exp, GpVal);		// Exp = e^{-Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
		Exp.setDiag(GpVal);		// Exp = e^{-Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
		sim->armCalcImpPeriod();		// 周期計算
	}
#if 1
	// inertia shapingなしの場合
	sim->armWithoutInertiaShaping();		// 慣性行列Mの計算
#endif
	// 前処理
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matSub(&dre, &sim->var.dr, &sim->var_init.dr);		// 手先速度変位
	matSub(&dre_prev, &sim->var_prev.dr, &sim->var_init.dr);		// 手先速度変位(1フレーム前)
#if 0
	if(sim->step > 0){
		matMulScl(&Inc, SYSTEM_CYCLE_TIME/2.0, matAdd(&Tmp21, matMul(&Tmp21, &sim->imp.K, &dre), matMul3(&Tmp21_2, &Exp, &sim->imp.K, &dre_prev)));		// Inc = Δt/2*(K*dr+Exp*K*dr_prev)
		matAdd(&Integ, matMul(&Tmp21, &Exp, &Integ), &Inc);		// Integ = Exp*Integ+Inc
//		if(sim->step%100 ==0)matPrint(&Integ);
//		if(sim->step < 10)matPrint(&Integ);
		if(sim->step < 10)matPrint(&dre);
	}
#elif 0
	simpson(sim, &Integ, SYSTEM_CYCLE_TIME);		// Integ= ∫exp{-(t-τ)K*C^{-1}}*K*dre dτ
#else
	simpson2(sim, &Integ, SYSTEM_CYCLE_TIME);		// Integ= K*C^{-1}*∫exp{-(t-τ)K*C^{-1}}*K*re dτ
#endif
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &sim->kine.Jinv, &sim->imp.Minv);	// E = Mq*J^{-1}*M^{-1}
//	matSub(&tauNC, &sim->dyn.h, matMul4(&Tmp21, &sim->dyn.Mq, &sim->kine.Jinv, &sim->kine.dJ, &sim->var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq（Mが定数のとき）
	matAdd3(&tauNC, &sim->dyn.h, matSignInv(matMul4(&Tmp21, &sim->dyn.Mq, &sim->kine.Jinv, &sim->kine.dJ, &sim->var.dq)), matSignInv(matMul3(&Tmp21_2, &E, &sim->imp.dM, &dre)));	// tauNC = h-Mq*J^{-1}*dJ*dq-E*dM*dr
#if 0		// simpsonに対応
	matSignInv(matMul(&tauVE, &E, &Integ));	// tauVE = -E*Integ
#elif 1		// simpson2に対応
	matMul(&tauVE, &E, matSub(&Tmp21, &Integ, matMul(&Tmp21_2, &sim->imp.K, &re)));	// tauVE = E*(Integ-K*re)
#else		// simpsonに対応．初期値F0ありのバージョンだが変化なし？
	matMulScl(&Tmp22, -SYSTEM_CYCLE_TIME*sim->step, matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv));		//  = -t*K*C^{-1}
	double	GpVal[] = {exp(Tmp22.el[0][0]), exp(Tmp22.el[1][1])};	// e^{-t*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
	matSetValDiag(&Expt, GpVal);		// Expt = e^{-t*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
	matMul(&tauVE, &E, matSub(&Tmp21, matMul(&Tmp21_2, &Expt, &sim->var_init.F), &Integ));	// tauVE = E*(Expt*F0-Integ)
#endif
	matMul(&tauIN, matSub(&Tmp22, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matAdd3(tau, &tauNC, &tauVE, &tauIN);		// tau = tauNC+tauVE+tauIN
	return	0;
}

#if 0	// ctrlMaxwellConv()内でarmWithoutInertiaShaping()を実行すればctrlMaxwellConvWithoutInertiaShaping()と同一
////////////////////////////////////////////////////////
// Maxwell制御則（畳み込み表現）
// dK = 0, dC = 0, Mは時変対応
// 現在はK, Cが対角行列の時のみ対応
// F = M*ddr+dM*dr+∫_0^t{exp(-(t-τ)*K*C^{-1})*K*dr}dτ
////////////////////////////////////////////////////////
int ctrlMaxwellConvWithoutInertiaShaping(SIM *sim, Matrix *tau)
{
	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp21_2(2, 1);
	static Matrix	tauNC(2, 1), tauVE(2, 1), tauIN(2, 1), E(2, 2);
	static Matrix	Integ(2, 1);
	static Matrix	re(2, 1), dre(2, 1);	// 手先位置速度変位，目標位置速度変位
	static Matrix	dre_prev(2, 1);	// 
	static Matrix	Exp(2, 2), Inc(2, 1), Expt(2, 2);	// 
	if (sim->step == 0) {
		// 初期設定
		matMulScl(&Tmp22, -SYSTEM_CYCLE_TIME, matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv));		//  = -Δt*K*C^{-1}
		double	GpVal[] = { exp(Tmp22.el[0][0]), exp(Tmp22.el[1][1]) };	// e^{-Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
//		matSetValDiag(&Exp, GpVal);		// Exp = e^{-Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
		Exp.setDiag(GpVal);		// Exp = e^{-Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
		matPrint(&Exp);
		sim->armCalcImpPeriod();		// 周期計算
	}
	// 前処理
	sim->armWithoutInertiaShaping();		// 慣性行列Mの計算
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matSub(&dre, &sim->var.dr, &sim->var_init.dr);		// 手先速度変位
	matSub(&dre_prev, &sim->var_prev.dr, &sim->var_init.dr);		// 手先速度変位(1フレーム前)
#if 0
	if (sim->step > 0) {
		matMulScl(&Inc, SYSTEM_CYCLE_TIME / 2.0, matAdd(&Tmp21, matMul(&Tmp21, &sim->imp.K, &dre), matMul3(&Tmp21_2, &Exp, &sim->imp.K, &dre_prev)));		// Inc = Δt/2*(K*dr+Exp*K*dr_prev)
		matAdd(&Integ, matMul(&Tmp21, &Exp, &Integ), &Inc);		// Integ = Exp*Integ+Inc
																//		if(sim->step%100 ==0)matPrint(&Integ);
																//		if(sim->step < 10)matPrint(&Integ);
		if (sim->step < 10)matPrint(&dre);
	}
#elif 0
	simpson(sim, &Integ, SYSTEM_CYCLE_TIME);		// Integ= ∫exp{-(t-τ)K*C^{-1}}*K*dre dt
#else
	simpson2(sim, &Integ, SYSTEM_CYCLE_TIME);		// Integ= K*C^{-1}*∫exp{-(t-τ)K*C^{-1}}*K*re dt
#endif
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &sim->kine.Jinv, &sim->imp.Minv);	// E = Mq*J^{-1}*M^{-1}
//	matSub(&tauNC, &sim->dyn.h, matMul4(&Tmp21, &sim->dyn.Mq, &sim->kine.Jinv, &sim->kine.dJ, &sim->var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd3(&tauNC, &sim->dyn.h, matSignInv(matMul4(&Tmp21, &sim->dyn.Mq, &sim->kine.Jinv, &sim->kine.dJ, &sim->var.dq)), matSignInv(matMul3(&Tmp21_2, &E, &sim->imp.dM, &dre)));	// tauNC = h-Mq*J^{-1}*dJ*dq-E*dM*dr
#if 0		// simpsonに対応
	matSignInv(matMul(&tauVE, &E, &Integ));	// tauVE = -E*Integ
#elif 1		// simpson2に対応
	matMul(&tauVE, &E, matSub(&Tmp21, &Integ, matMul(&Tmp21_2, &sim->imp.K, &re)));	// tauVE = E*(Integ-K*re)
#else		// simpsonに対応．初期値F0ありのバージョンだが変化なし？
	matMulScl(&Tmp22, -SYSTEM_CYCLE_TIME * sim->step, matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv));		//  = -t*K*C^{-1}
	double	GpVal[] = { exp(Tmp22.el[0][0]), exp(Tmp22.el[1][1]) };	// e^{-t*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
	matSetValDiag(&Expt, GpVal);		// Expt = e^{-t*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
	matMul(&tauVE, &E, matSub(&Tmp21, matMul(&Tmp21_2, &Expt, &sim->var_init.F), &Integ));	// tauVE = E*(Expt*F0-Integ)
#endif
	matMul(&tauIN, matSub(&Tmp22, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matAdd3(tau, &tauNC, &tauVE, &tauIN);		// tau = tauNC+tauVE+tauIN
//	matPrint(&tauIN);		// 0であればOK
	matPrint(&sim->imp.M);		// 
	return	0;
}
#endif

#if 1
// 速度変数の積分計算
int simpson(cFinger *sim, Matrix *Integ, double dt)
{
	static Matrix	K1(2,1), K2(2,1), K3(2,1), K(2,1);
	static Matrix	Tmp21(2,1), Tmp22(2,2);
	static Matrix	dre(2,1), dre_prev(2,1), dre_prev2(2,1);	// 手先位置速度変位，目標位置速度変位
	static Matrix	Exp(2,2), Exp2(2,2);	//
	auto entity = EntityManager::get();
	if (entity->step == 0) {
		matMulScl(&Tmp22, -dt, matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv));		//  = -Δt*K*C^{-1}
		double	GpVal[] = {exp(Tmp22.el[0][0]), exp(Tmp22.el[1][1])};	// e^{-Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
		matSetValDiag(&Exp, GpVal);		// Exp = e^{-Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
		matMulScl(&Tmp22, -2.0*dt, matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv));		//  = -2Δt*K*C^{-1}
		GpVal[0] = exp(Tmp22.el[0][0]); GpVal[1] = exp(Tmp22.el[1][1]);	// e^{-2Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
		matSetValDiag(&Exp2, GpVal);		// Exp = e^{-2Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
	}
	if(entity->step % 2 == 0){
		matSub(&dre, &sim->var.dr, &sim->var_init.dr);		// 手先速度変位
		matSub(&dre_prev, &sim->var_prev.dr, &sim->var_init.dr);		// 手先速度変位(1フレーム前)
		matSub(&dre_prev2, &sim->var_prev2.dr, &sim->var_init.dr);		// 手先速度変位(2フレーム前)
		// K
		matMul(&K1, &sim->imp.K, &dre);		// K1 = K*dre
		matMul3(&K2, &Exp, &sim->imp.K, &dre_prev);		// K2 = Exp*K*dre_prev
		matMul3(&K3, &Exp2, &sim->imp.K, &dre_prev2);		// K3 = Exp2*K*dre_prev2
		matMulScl(&K, dt/3.0, matAdd3(&Tmp21, &K1, matMulScl(&Tmp21, 4.0, &K2), &K3));		// K = (K1+4*K2+K3)*2dt/6 = (K1+4*K2+K3)*dt/3
		matAdd(Integ, matMul(&Tmp21, &Exp2, Integ), &K);		// Integ = Exp2*Integ+K
	}
	return	0;
}

// 位置変数の積分計算
int simpson2(cFinger *sim, Matrix *Integ, double dt)
{
	static Matrix	K1(2, 1), K2(2, 1), K3(2, 1), K(2, 1);
	static Matrix	Tmp21(2, 1), Tmp22(2, 2);
	static Matrix	re(2, 1), re_prev(2, 1), re_prev2(2, 1);	// 手先位置速度変位，目標位置速度変位
	static Matrix	Exp(2, 2), Exp2(2, 2);	//
	auto entity = EntityManager::get();
	if (entity->step == 0) {
		matMulScl(&Tmp22, -dt, matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv));		//  = -Δt*K*C^{-1}
		double	GpVal[] = { exp(Tmp22.el[0][0]), exp(Tmp22.el[1][1]) };	// e^{-Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
		matSetValDiag(&Exp, GpVal);		// Exp = e^{-Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
		matMulScl(&Tmp22, -2.0*dt, matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv));		//  = -2Δt*K*C^{-1}
		GpVal[0] = exp(Tmp22.el[0][0]); GpVal[1] = exp(Tmp22.el[1][1]);	// e^{-2Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
		matSetValDiag(&Exp2, GpVal);		// Exp = e^{-2Δt*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
	}
	if (entity->step % 2 == 0) {
		matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
		matSub(&re_prev, &sim->var_prev.r, &sim->var_init.r);		// 手先速度変位(1フレーム前)
		matSub(&re_prev2, &sim->var_prev2.r, &sim->var_init.r);		// 手先速度変位(2フレーム前)
		// K
		matMul4(&K1, &sim->imp.K, &sim->imp.Cinv, &sim->imp.K, &re);		// K1 = K*C^{-1}*K*re
		matMul5(&K2, &sim->imp.K, &sim->imp.Cinv, &Exp, &sim->imp.K, &re_prev);		// K2 = K*C^{-1}*Exp*K*re_prev
		matMul5(&K3, &sim->imp.K, &sim->imp.Cinv, &Exp2, &sim->imp.K, &re_prev2);		// K3 = K*C^{-1}*Exp2*Kdre_prev2
		matMulScl(&K, dt / 3.0, matAdd3(&Tmp21, &K1, matMulScl(&Tmp21, 4.0, &K2), &K3));		// K = (K1+4*K2+K3)*2dt/6 = (K1+4*K2+K3)*dt/3
		matAdd(Integ, matMul(&Tmp21, &Exp2, Integ), &K);		// Integ = Exp2*Integ+K
	}
	return	0;
}
#endif

////////////////////////////////////////////////////////
// Maxwell制御則（畳み込み表現）
// dK = 0, dC = 0, Mは時変対応
// M*ddr + w = F
// w =∫_0^t{exp(-(t-τ)*K*C^{-1})*K*dr}dτ
// y = [x; dx; w], dy = A*y + B*F
// A = [0, I, 0; 0, -M^{-1}*dM, -M^{-1}; 0, K, -K*C{-1}],  B = [0; M^{-1}; 0]
// A = [0, I, 0; 0, 0, -M^{-1}; 0, K, -K*C{-1}],  B = [0; M^{-1}; 0]	// Mが定数の場合
////////////////////////////////////////////////////////
int ctrlMaxwellConvInnerLoop(cFinger *sim, Matrix *tau)
{
	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp21_2(2, 1);
	static Matrix	tauNC(2, 1), tauIN(2, 1);
	static Matrix	re(2, 1), dre(2, 1), rc, drc, ddrc;	// 手先位置速度変位，目標位置速度変位
	static Matrix	A(6,6), B(6, 2), X(6, 1), U(2, 1), Xnext(6, 1), dX(6, 1);	// 状態空間
	constexpr double GAIN_INNERLOOP = 20;		// 
	double	GpVal[] = { GAIN_INNERLOOP*GAIN_INNERLOOP / 4.0, GAIN_INNERLOOP*GAIN_INNERLOOP / 4.0 };	// 臨界減衰
	double	GvVal[] = { GAIN_INNERLOOP, GAIN_INNERLOOP };

	auto entity = EntityManager::get();
	if (entity->step == 0) {
		// 初期設定
		matShareInit(&rc, 2, 1); matShareInit(&drc, 2, 1); matShareBlock(&rc, &X, 0, 0); matShareBlock(&drc, &X, 2, 0);	// X = [rc; drc; w]
		matShareInit(&ddrc, 2, 1); matShareBlock(&ddrc, &dX, 2, 0);	// dX = [drc; ddrc; dw]
		matSetValDiag(&sim->imp.Gp, GpVal); matSetValDiag(&sim->imp.Gv, GvVal);	// ゲイン設定
		matAssign(&A, matUnit(&Tmp22), 0, 2);		// I
		sim->armCalcImpPeriod();		// 周期計算
	}
#if 1
	// inertia shapingなしの場合
	sim->armWithoutInertiaShaping();		// 慣性行列Mの計算
#endif
	// 前処理
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matSub(&dre, &sim->var.dr, &sim->var_init.dr);		// 手先速度変位
	// インナーループ目標値計算
	matAssign(&A, matSignInv(matMul(&Tmp22, &sim->imp.Minv, &sim->imp.dM)), 2, 2);		// -M^{-1}*dM
	matAssign(&A, matMulScl(&Tmp22, -1.0, &sim->imp.Minv), 2, 4);		// -M^{-1}
	matAssign(&A, &sim->imp.K, 4, 2);		// K
	matAssign(&A, matSignInv(matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv)), 4, 4);		// -K*C^{-1}
	matAssign(&B, &sim->imp.Minv, 2, 0);		// M^{-1}
	matCopy(&X, &Xnext);	// 1サイクル前の計算値を初期値として代入
	rk4Acc(sim, &Xnext, &dX, &X, &A, &B, &sim->var.F, SIM_CYCLE_TIME);		// Xを次のサイクルの値で更新
	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_pos[crd] = rc.el[crd][0];		// 変数保存
	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_vel[crd] = drc.el[crd][0];		// 変数保存
//	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_vel[crd] = X.el[crd+4][0];		// wの値を保存
//	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_acc[crd] = ddrc.el[crd][0];		// 変数保存
	// 制御則
	matAdd3(&Tmp21, &ddrc, matMulSub(&Tmp21, &sim->imp.Gv, &drc, &dre), matMulSub(&Tmp21_2, &sim->imp.Gp, &rc, &re));		// ddrc+Gv*(drc-dre)+Gp*(rc-re)
	matSub(&Tmp21, &Tmp21, matMul(&Tmp21_2, &sim->kine.dJ, &sim->var.dq));		// ddrc+Gv*(drc-dre)+Gp*(rc-re)-dJ*dq
	matAdd(&tauNC, &sim->dyn.h, matMul3(&Tmp21_2, &sim->dyn.Mq, &sim->kine.Jinv, &Tmp21));	// tauNC = h+Mq*J^{-1}*(ddrc+Gv*(drc-dre)+Gp*(rc-re)-dJ*dq)
	matSignInv(matMul(&tauIN, &sim->kine.Jt, &sim->var.F));		// tauIN = -J^T*F
	matAdd(tau, &tauNC, &tauIN);		// tau = tauNC+tauIN
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則（畳み込み表現）
// dK = 0, dC = 0, Mは時変対応
// F = M*ddx + dM*dx + w (w=∫_0^t{exp(-(t-τ)*K*C^{-1})*K*dx}dτ)
// dw = -K*C^{-1}*w+K*dx をルンゲクッタで計算
////////////////////////////////////////////////////////
int ctrlMaxwellConvRK(cFinger *sim, Matrix *tau)
{
	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp21_2(2, 1);
	static Matrix	tauNC(2, 1), tauVE(2, 1), tauIN(2, 1), E(2, 2);
	static Matrix	re(2, 1), dre(2, 1);	// 手先位置速度変位，目標位置速度変位
	static Matrix	A(2, 2), W(2, 1), U(2, 1), Wnext(2, 1), dW(2, 1);	// 状態空間(B=Kなので未定義)
	constexpr double GAIN_INNERLOOP = 20;		// 
	double	GpVal[] = { GAIN_INNERLOOP*GAIN_INNERLOOP / 4.0, GAIN_INNERLOOP*GAIN_INNERLOOP / 4.0 };	// 臨界減衰
	double	GvVal[] = { GAIN_INNERLOOP, GAIN_INNERLOOP };


	auto entity = EntityManager::get();
	if (entity->step == 0) {

		// 初期設定
		matSetValDiag(&sim->imp.Gp, GpVal); matSetValDiag(&sim->imp.Gv, GvVal);	// ゲイン設定
		sim->armCalcImpPeriod();		// 周期計算
	}
	// 前処理
#if 1
	// inertia shapingなしの場合
	sim->armWithoutInertiaShaping();		// 慣性行列Mの計算
#endif
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matSub(&dre, &sim->var.dr, &sim->var_init.dr);		// 手先速度変位
	// サブインナーループ目標値計算
	matSignInv(matMul(&A, &sim->imp.K, &sim->imp.Cinv));		// A = -K*C^{-1}
	matCopy(&W, &Wnext);	// 1サイクル前の計算値を初期値として代入
	rk4Acc(sim, &Wnext, &dW, &W, &A, &sim->imp.K, &dre, SIM_CYCLE_TIME);		// Xを次のサイクルの値で更新
//	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_pos[crd] = W.el[crd][0];		// 変数保存
	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_vel[crd] = W.el[crd][0];		// 変数保存
//	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_acc[crd] = ddrc.el[crd][0];		// 変数保存
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &sim->kine.Jinv, &sim->imp.Minv);	// E = Mq*J^{-1}*M^{-1}
//	matSub(&tauNC, &sim->dyn.h, matMul4(&Tmp21, &sim->dyn.Mq, &sim->kine.Jinv, &sim->kine.dJ, &sim->var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd3(&tauNC, &sim->dyn.h, matSignInv(matMul4(&Tmp21, &sim->dyn.Mq, &sim->kine.Jinv, &sim->kine.dJ, &sim->var.dq)), matSignInv(matMul3(&Tmp21_2, &E, &sim->imp.dM, &dre)));	// tauNC = h-Mq*J^{-1}*dJ*dq-E*dM*dr
#if 1		// simpsonに対応
	matSignInv(matMul(&tauVE, &E, &W));	// tauVE = -E*w
#elif 1		// simpson2に対応
	matMul(&tauVE, &E, matSub(&Tmp21, &W, matMul(&Tmp21_2, &sim->imp.K, &re)));	// tauVE = E*(w-K*re)
#else		// simpsonに対応．初期値F0ありのバージョンだが変化なし？
	matMulScl(&Tmp22, -SYSTEM_CYCLE_TIME * sim->step, matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv));		//  = -t*K*C^{-1}
	double	GpVal[] = { exp(Tmp22.el[0][0]), exp(Tmp22.el[1][1]) };	// e^{-t*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
	matSetValDiag(&Expt, GpVal);		// Expt = e^{-t*K*C^{-1}} ただし，現在はK,Cが対角行列の時のみ対応
	matMul(&tauVE, &E, matSub(&Tmp21, matMul(&Tmp21_2, &Expt, &sim->var_init.F), &Integ));	// tauVE = E*(Expt*F0-Integ)
#endif
	matMul(&tauIN, matSub(&Tmp22, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matAdd3(tau, &tauNC, &tauVE, &tauIN);		// tau = tauNC+tauVE+tauIN
//	matPrint(&tauIN);		// 0であればOK
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則（畳み込み表現）
// dK = 0, dC = 0, Mは時変対応
// M*ddx + K*x - w = F (w=K*C^{-1}∫_0^t{exp(-(t-τ)*K*C^{-1})*K*x}dτ)
// dw = -K*C^{-1}*w+K*C^{-1}*K*x をルンゲクッタで計算
////////////////////////////////////////////////////////
int ctrlMaxwellConvRK2(cFinger *sim, Matrix *tau)
{
	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp21_2(2, 1);
	static Matrix	tauNC(2, 1), tauVE(2, 1), tauIN(2, 1), E(2, 2);
	static Matrix	re(2, 1), dre(2, 1);	// 手先位置速度変位，目標位置速度変位
	static Matrix	A(2, 2), B(2,2), W(2, 1), U(2, 1), Wnext(2, 1), dW(2, 1);	// 状態空間
	constexpr double GAIN_INNERLOOP = 20;		// 
	double	GpVal[] = { GAIN_INNERLOOP*GAIN_INNERLOOP / 4.0, GAIN_INNERLOOP*GAIN_INNERLOOP / 4.0 };	// 臨界減衰
	double	GvVal[] = { GAIN_INNERLOOP, GAIN_INNERLOOP };


	auto entity = EntityManager::get();
	if (entity->step == 0) {

		// 初期設定
		matSetValDiag(&sim->imp.Gp, GpVal); matSetValDiag(&sim->imp.Gv, GvVal);	// ゲイン設定
		sim->armCalcImpPeriod();		// 周期計算
	}
#if 1
	// inertia shapingなしの場合
	sim->armWithoutInertiaShaping();		// 慣性行列Mの計算
#endif
	// 前処理
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matSub(&dre, &sim->var.dr, &sim->var_init.dr);		// 手先速度変位
	// サブインナーループ目標値計算
	matSignInv(matMul(&A, &sim->imp.K, &sim->imp.Cinv));		// A = -K*C^{-1}
	matMul3(&B, &sim->imp.K, &sim->imp.Cinv, &sim->imp.K);		// B = K*C^{-1}*K
	matCopy(&W, &Wnext);	// 1サイクル前の計算値を初期値として代入
	rk4Acc(sim, &Wnext, &dW, &W, &A, &B, &re, SIM_CYCLE_TIME);		// Xを次のサイクルの値で更新
//	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_pos[crd] = W.el[crd][0];		// 変数保存
	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_vel[crd] = W.el[crd][0];		// 変数保存
//	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_acc[crd] = ddrc.el[crd][0];		// 変数保存
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &sim->kine.Jinv, &sim->imp.Minv);	// E = Mq*J^{-1}*M^{-1}
	matSub(&tauNC, &sim->dyn.h, matMul4(&Tmp21, &sim->dyn.Mq, &sim->kine.Jinv, &sim->kine.dJ, &sim->var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matMul(&tauVE, &E, matSub(&Tmp21, &W, matMul(&Tmp21_2, &sim->imp.K, &re)));	// tauVE = E*(w-K*re)
	matMul(&tauIN, matSub(&Tmp22, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matAdd3(tau, &tauNC, &tauVE, &tauIN);		// tau = tauNC+tauVE+tauIN
	return	0;
}

// まだ作成中
int ctrlMaxwellConvObserver(cFinger *sim, Matrix *tau)
{
	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp21_2(2, 1);
	static Matrix	tauNC(2, 1), tauIN(2, 1);
	static Matrix	re(2, 1), dre(2, 1), rc, drc, ddrc;	// 手先位置速度変位，目標位置速度変位
	static Matrix	A(6,6), B(6, 2), X(6, 1), U(2, 1), Xnext(6, 1), dX(6, 1);	// 状態空間
//#define	GAIN_INNERLOOP	20		// 
	constexpr double GAIN_INNERLOOP = 20;		// 
	double	GpVal[] = { GAIN_INNERLOOP*GAIN_INNERLOOP / 4.0, GAIN_INNERLOOP*GAIN_INNERLOOP / 4.0 };	// 臨界減衰
	double	GvVal[] = { GAIN_INNERLOOP, GAIN_INNERLOOP };

	auto entity = EntityManager::get();
	if (entity->step == 0) {
		// 初期設定
		matShareInit(&rc, 2, 1); matShareInit(&drc, 2, 1); matShareBlock(&rc, &X, 0, 0); matShareBlock(&drc, &X, 2, 0);	// X = [rc; drc; w]
		matShareInit(&ddrc, 2, 1); matShareBlock(&ddrc, &dX, 2, 0);	// dX = [drc; ddrc; dw]
		matSetValDiag(&sim->imp.Gp, GpVal); matSetValDiag(&sim->imp.Gv, GvVal);	// ゲイン設定
		matAssign(&A, matUnit(&Tmp22), 0, 2);		// I
		sim->armCalcImpPeriod();		// 周期計算
	}
	// 前処理
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matSub(&dre, &sim->var.dr, &sim->var_init.dr);		// 手先速度変位
	// インナーループ目標値計算
	matAssign(&A, matMulScl(&Tmp22, -1.0, &sim->imp.Minv), 2, 4);		// -M^{-1}
	matAssign(&A, &sim->imp.K, 4, 2);		// K
	matAssign(&A, matSignInv(matMul(&Tmp22, &sim->imp.K, &sim->imp.Cinv)), 4, 4);		// -K*C^{-1}
	matAssign(&B, &sim->imp.Minv, 2, 0);		// M^{-1}
	matCopy(&X, &Xnext);	// 1サイクル前の計算値を初期値として代入
	rk4Acc(sim, &Xnext, &dX, &X, &A, &B, &sim->var.F, SIM_CYCLE_TIME);		// Xを次のサイクルの値で更新
	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_pos[crd] = rc.el[crd][0];		// 変数保存
	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_vel[crd] = drc.el[crd][0];		// 変数保存
//	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_vel[crd] = X.el[crd+4][0];		// wの値を保存
//	for (int crd = 0; crd<DIM2; crd++)	sim->ref_eff_acc[crd] = ddrc.el[crd][0];		// 変数保存
	// 制御則
	matAdd3(&Tmp21, &ddrc, matMulSub(&Tmp21, &sim->imp.Gv, &drc, &dre), matMulSub(&Tmp21_2, &sim->imp.Gp, &rc, &re));		// ddrc+Gv*(drc-dre)+Gp*(rc-re)
	matSub(&Tmp21, &Tmp21, matMul(&Tmp21_2, &sim->kine.dJ, &sim->var.dq));		// ddrc+Gv*(drc-dre)+Gp*(rc-re)-dJ*dq
	matAdd(&tauNC, &sim->dyn.h, matMul3(&Tmp21_2, &sim->dyn.Mq, &sim->kine.Jinv, &Tmp21));	// tauNC = h+Mq*J^{-1}*(ddrc+Gv*(drc-dre)+Gp*(rc-re)-dJ*dq)
	matSignInv(matMul(&tauIN, &sim->kine.Jt, &sim->var.F));		// tauIN = -J^T*F
	matAdd(tau, &tauNC, &tauIN);		// tau = tauNC+tauIN
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則
// M*ddx + K*C^{-1}*M*dx + K*x = F + K*C^{-1}∫Fdt
////////////////////////////////////////////////////////
int cFinger::ctrlMaxwell(Matrix* tau)
{
	
	int	jnt, crd;
	//以下の4つはstaticがついていた
	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp21_2(2, 1);
	static Matrix	tauNC(2, 1), tauVE(2, 1), tauIN(2, 1), tauPL(2, 1), E(2, 2);
	static Matrix	Integ(2, 1);
	static Matrix	re(2, 1), dre(2, 1);	// 手先位置変位，手先速度変位

	auto entity = EntityManager::get();
	if (entity->step == 0){
		armCalcImpPeriod();		// 周期計算
	}
	// 前処理
	matSub(&re, &var.r,&var_init.r);		// 手先位置変位
	matSub(&dre, &var.dr, &var_init.dr);		// 手先速度変位
#if 1
	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &var.F));		// Integ = ∫Fdt
#else
	if(sim->step > 0)	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &sim->var.F));		// Integ = ∫Fdt
#endif
	// 制御則
	matMul3(&E, &dyn.Mq, &kine.Jinv, &imp.Minv);	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, &dyn.h, matMul4(&Tmp21, &dyn.Mq, &kine.Jinv, &kine.dJ, &var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd(&Tmp21, matMul4(&tauVE, &imp.K, &imp.Cinv, &imp.M, &dre), matMul(&Tmp21, &imp.K, &re));		// Kd*Cd^{-1}*Md*dr+Kd*r
	matSignInv(matMul(&tauVE, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*dr+Kd*r}
	matMul(&tauIN, matSub(&Tmp22, &E, &kine.Jt), &var.F);		// tauIN = (E-J^T)F
	matMul4(&tauPL, &E, &imp.K, &imp.Cinv, &Integ);		// tauPL = E*Kd*Cd^{-1}∫Fdt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);
	// デバッグ
	matPrint(&tauIN);		// Inertia Shaping無しの場合は0になればOK
//	matPrint(&sim->imp.M);	matPrint(&sim->imp.C);	matPrint(&sim->imp.K);
	return	0;
}

int cFinger::ctrlMaxwell2(Matrix* tau)
{

	int	jnt, crd;
	//以下の4つはstaticがついていた
	static Matrix	Tmp21(2, 1), Tmp22(2, 2), Tmp21_2(2, 1);
	static Matrix	tauNC(2, 1), tauVE(2, 1), tauIN(2, 1), tauPL(2, 1), E(2, 2);
	static Matrix	Integ(2, 1);
	static Matrix	re(2, 1), dre(2, 1);	// 手先位置変位，手先速度変位

	auto entity = EntityManager::get();
	if (entity->step == 0) {
		armCalcImpPeriod();		// 周期計算
	}
	// 前処理
	matSub(&re, &var.r, &var_init.r);		// 手先位置変位
	matSub(&dre, &var.dr, &var_init.dr);		// 手先速度変位
#if 1
	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &var.F));		// Integ = ∫Fdt
#else
	if (sim->step > 0)	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &sim->var.F));		// Integ = ∫Fdt
#endif
	// 制御則
	matMul3(&E, &dyn.Mq, &kine.Jinv, &imp.Minv);	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, &dyn.h, matMul4(&Tmp21, &dyn.Mq, &kine.Jinv, &kine.dJ, &var.dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd(&Tmp21, matMul4(&tauVE, &imp.K, &imp.Cinv, &imp.M, &dre), matMul(&Tmp21, &imp.K, &re));		// Kd*Cd^{-1}*Md*dr+Kd*r
	matSignInv(matMul(&tauVE, &E, &Tmp21));	// tauVE = -E{Kd*Cd^{-1}*Md*dr+Kd*r}
	matMul(&tauIN, matSub(&Tmp22, &E, &kine.Jt), &var.F);		// tauIN = (E-J^T)F
	matMul4(&tauPL, &E, &imp.K, &imp.Cinv, &Integ);		// tauPL = E*Kd*Cd^{-1}∫Fdt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);
	// デバッグ
	matPrint(&tauIN);		// Inertia Shaping無しの場合は0になればOK
//	matPrint(&sim->imp.M);	matPrint(&sim->imp.C);	matPrint(&sim->imp.K);
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則
// dK = 0, dC = 0, Mは時変対応
// M*ddx + (K*C^{-1}*M+dM)*dx + K*x = F + K*C^{-1}∫Fdt
////////////////////////////////////////////////////////
int ctrlMaxwellWithoutInertiaShaping(cFinger *sim, Matrix *tau)
{
	int	jnt, crd;
	static Matrix	Tmp21(2,1), Tmp22(2,2), Tmp21_2(2,1);
	static Matrix	tauNC(2,1), tauVE(2, 1), tauIN(2, 1), tauPL(2, 1), E(2, 2);
	static Matrix	Integ(2, 1);
	static Matrix	re(2, 1), dre(2, 1);	// 手先位置変位，手先速度変位

	auto entity = EntityManager::get();
	if (entity->step == 0) {
		sim->armCalcImpPeriod();		// 周期計算
	}

	// 前処理
	sim->armWithoutInertiaShaping();		// 慣性行列Mの計算
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matSub(&dre, &sim->var.dr, &sim->var_init.dr);		// 手先速度変位
	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &sim->var.F));		// Integ = ∫Fdt	長方形近似
	// 制御則
	matMul3(&E, &sim->dyn.Mq, &sim->kine.Jinv, &sim->imp.Minv);	// E = Mq*J^{-1}*Md^{-1}
	matAdd3(&tauNC, &sim->dyn.h, matSignInv(matMul4(&Tmp21, &sim->dyn.Mq, &sim->kine.Jinv, &sim->kine.dJ, &sim->var.dq)), matSignInv(matMul3(&Tmp21_2, &E, &sim->imp.dM, &dre)));	// tauNC = h-Mq*J^{-1}*dJ*dq-E*dM*dr
	matAdd(&Tmp21, matMul4(&tauVE, &sim->imp.K, &sim->imp.Cinv, &sim->imp.M, &dre), matMul(&Tmp21, &sim->imp.K, &re));		// K*C^{-1}*M*dr+K*r
	matSignInv(matMul(&tauVE, &E, &Tmp21));	// tauVE = -E{K*C^{-1}*M*dr+K*r}
	matMul(&tauIN, matSub(&Tmp22, &E, &sim->kine.Jt), &sim->var.F);		// tauIN = (E-J^T)F
	matMul4(&tauPL, &E, &sim->imp.K, &sim->imp.Cinv, &Integ);		// tauPL = E*K*C^{-1}∫Fdt
	matAdd4(tau, &tauNC, &tauVE, &tauIN, &tauPL);
	// デバッグ
//	if(sim->step % 10 == 1)	matPrint(&tauIN);		// Inertia Shaping無しの場合は0になればOK
//	matPrint(&tauIN);		// Inertia Shaping無しの場合は0になればOK
	//	matPrint(&sim->imp.M);	matPrint(&sim->imp.C);	matPrint(&sim->imp.K);
	return	0;
}

////////////////////////////////////////////////////////
// Voigt制御則
////////////////////////////////////////////////////////
int ctrlVoigt(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Md, const Matrix *Cd, const Matrix *Kd)
{
	auto _this = EntityManager::get()->getFinger();
	int	jnt, crd;
	static Matrix	Jinv, Jt, Tmp21, Tmp22;
	static Matrix	tauNC, tauVE, tauIN, E;

	auto entity = EntityManager::get();
	// 初期化
	if(entity->step == 0){
		matInit(&Jinv,2,2);	matInit(&Jt,2,2);
		matInit(&Tmp21,2,1); matInit(&Tmp22,2,2);
		matInit(&tauNC,2,1); matInit(&tauVE,2,1); matInit(&tauIN,2,1); matInit(&E,2,2);
		for(crd=0;crd<2;crd++){
			if(_this->imp.C.el[crd][crd]*_this->imp.C.el[crd][crd] < 4*_this->imp.M.el[crd][crd]*_this->imp.K.el[crd][crd] ){
				_this->imp.T[crd] = 2*PI/sqrt(_this->imp.K.el[crd][crd]/_this->imp.M.el[crd][crd]-_this->imp.C.el[crd][crd]*_this->imp.C.el[crd][crd]/(4*_this->imp.M.el[crd][crd]*_this->imp.M.el[crd][crd]));
				printf("T[%d]=%f\n", crd, _this->imp.T[crd]);
			}
		}
	}
	// 制御則
	matTrans(&Jt, J);	matInv(&Jinv, NULL, J);	// J^T, J^{-1}
	matMul3(&E, Mq, &Jinv, matInv(&Tmp22, NULL, Md));	// E = Mq*J^{-1}*Md^{-1}
	matSub(&tauNC, h, matMul4(&Tmp21, Mq, &Jinv, dJ, dq));	// tauNC = h-Mq*J^{-1}*dJ*dq
	matAdd(&Tmp21, matMul(&tauVE, Cd, dre), matMul(&Tmp21, Kd, re));
	matMulScl(&tauVE, -1, matMul(&Tmp21, &E, &Tmp21));	// tauVE = -E(Cd*dr+Kd*r)
	matMul(&tauIN, matSub(&Tmp22, &E, &Jt), F);		// tauIN = (E-J^T)F
	matAdd3(tau, &tauNC, &tauVE, &tauIN);
	return	0;
}
