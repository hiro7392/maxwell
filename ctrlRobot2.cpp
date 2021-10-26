#include "simMain.h"
#include "ctrlRobot.h"
using Eigen::MatrixXd;
//using namespace Eigen;

#include <time.h>
// clock()関数の分解能は10[ms]程度
//#define DEBAG_DISPLAY

// シミュレーション変数
extern	SIM sim;

////////////////////////////////////////////////////////
// Maxwell+位置制御インナーループ制御則(Eigen)
////////////////////////////////////////////////////////
// dX = AX + U;
void RungeKutta(MatrixXd &X, MatrixXd A, MatrixXd U, double dt) {
	MatrixXd k1 = A*X + U;
    MatrixXd k2 = A*(X + 0.5*k1*dt) + U;
    MatrixXd k3 = A*(X + 0.5*k2*dt) + U;
    MatrixXd k4 = A*(X + k3*dt) + U;
    MatrixXd k = (k1 + 2.0*k2 + 2.0*k3 + k4)*dt / 6.0;
    X = X + k;
}

// まだ途中
int ctrlMaxwellInLoopE(cFinger *sim, Matrix *tau)
{
	// Inner Loop
	MatrixXd	X(4,1), U(4,1);		// dX = AX+U
	MatrixXd	A = MatrixXd::Zero(4,4);		// 
	// 定義
	static MatrixXd	Mq = MatrixXd::Zero(2,2);		// 
	static MatrixXd	J = MatrixXd::Zero(2,2);		// 
	static MatrixXd	dJ = MatrixXd::Zero(2,2);		// 
	static MatrixXd	dq = MatrixXd::Zero(2,1);		// 
	static MatrixXd	M = MatrixXd::Zero(2,2);		// 
	static MatrixXd	C = MatrixXd::Zero(2,2);		// 
	static MatrixXd	K = MatrixXd::Zero(2,2);		// 
	static MatrixXd	F = MatrixXd::Zero(2,1);		// 
	static MatrixXd	r0 = MatrixXd::Zero(2,1);		// 
	static MatrixXd	r = MatrixXd::Zero(2,1);		// 
	static MatrixXd	dr = MatrixXd::Zero(2,1);		// 
	static MatrixXd	Fint = MatrixXd::Zero(2,1);		// 
	static MatrixXd	h_ = MatrixXd::Zero(2,1);		// 
	static MatrixXd	tau_NC = MatrixXd::Zero(2,1);		// 
	static MatrixXd	tau_VE = MatrixXd::Zero(2,1);		// 
	static MatrixXd	tau_IN = MatrixXd::Zero(2,1);		// 
	static MatrixXd	tau_PL = MatrixXd::Zero(2,1);		// 
	static MatrixXd	tau_ = MatrixXd::Zero(2,1);		// 
	static MatrixXd	E_ = MatrixXd::Zero(2,2);		// 
	// 代入
	for(int i=0; i<2; i++){
		for(int j=0; j<2; j++){
			M(i,j) = sim->imp.M.el[i][j];
			C(i,j) = sim->imp.C.el[i][j];
			K(i,j) = sim->imp.K.el[i][j];
			Mq(i,j) = sim->dyn.Mq.el[i][j];
			J(i,j) = sim->kine.J.el[i][j];
			dJ(i,j) = sim->kine.dJ.el[i][j];
		}
		F(i,0) = sim->var.F.el[i][0];
		dr(i,0) = sim->var.dr.el[i][0];		// 手先速度変位（初速度が0の場合のみ適用）
		dq(i,0) = sim->var.dq.el[i][0];		// 関節速度
		r(i,0) = sim->var.r.el[i][0]-sim->var_init.r.el[i][0];		// 手先位置変位
		h_(i,0) = sim->dyn.h.el[i][0];
	}
	Fint = Fint + F*SIM_CYCLE_TIME;
	E_ = Mq*J.inverse()*M.inverse();
	tau_NC = h_-Mq*J.inverse()*dJ*dq;
	tau_VE = -E_*(K*C.inverse()*M*dr+K*r);
	tau_IN = (E_-J.transpose())*F;
	tau_PL = E_*K*C.inverse()*Fint;
	tau_ = tau_NC+tau_VE+tau_IN+tau_PL;
#if 1
	// 計算
	//	A.block(0,0,2,2) = MatrixXd::Zero(2,2);
	A.block(0,2,2,2) = MatrixXd::Identity(2,2);		// (0,2)要素から2x2の単位行列
//    A(1, 0) = -k / m;    A(1, 1) = -c / m;
//    B(0, 0) = 0;    B(1, 0) = 1 / m;
	RungeKutta(X, A, U, SIM_CYCLE_TIME);
#endif
	// 代入
	for(int i=0; i<2; i++)	tau->el[i][0] = tau_(i,0);
	// デバッグ
//	matPrint(&tauIN);		// Inertia Shaping無しの場合は0になればOK
//	matPrint(&sim->imp.M);	matPrint(&sim->imp.C);	matPrint(&sim->imp.K);
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則(Eigen)
////////////////////////////////////////////////////////
int ctrlMaxwellE(cFinger *sim, Matrix *tau)
{
	// 定義
	static MatrixXd	Mq = MatrixXd::Zero(2,2);		// 
	static MatrixXd	J = MatrixXd::Zero(2,2);		// 
	static MatrixXd	dJ = MatrixXd::Zero(2,2);		// 
	static MatrixXd	dq = MatrixXd::Zero(2,1);		// 
	static MatrixXd	M = MatrixXd::Zero(2,2);		// 
	static MatrixXd	C = MatrixXd::Zero(2,2);		// 
	static MatrixXd	K = MatrixXd::Zero(2,2);		// 
	static MatrixXd	F = MatrixXd::Zero(2,1);		// 
	static MatrixXd	r0 = MatrixXd::Zero(2,1);		// 
	static MatrixXd	r = MatrixXd::Zero(2,1);		// 
	static MatrixXd	dr = MatrixXd::Zero(2,1);		// 
	static MatrixXd	Fint = MatrixXd::Zero(2,1);		// 
	static MatrixXd	h_ = MatrixXd::Zero(2,1);		// 
	static MatrixXd	tau_NC = MatrixXd::Zero(2,1);		// 
	static MatrixXd	tau_VE = MatrixXd::Zero(2,1);		// 
	static MatrixXd	tau_IN = MatrixXd::Zero(2,1);		// 
	static MatrixXd	tau_PL = MatrixXd::Zero(2,1);		// 
	static MatrixXd	tau_ = MatrixXd::Zero(2,1);		// 
	static MatrixXd	E_ = MatrixXd::Zero(2,2);		// 
	// 代入
	for(int i=0; i<2; i++){
		for(int j=0; j<2; j++){
			M(i,j) = sim->imp.M.el[i][j];
			C(i,j) = sim->imp.C.el[i][j];
			K(i,j) = sim->imp.K.el[i][j];
			Mq(i,j) = sim->dyn.Mq.el[i][j];
			J(i,j) = sim->kine.J.el[i][j];
			dJ(i,j) = sim->kine.dJ.el[i][j];
		}
		F(i,0) = sim->var.F.el[i][0];
		dr(i,0) = sim->var.dr.el[i][0];		// 手先速度変位（初速度が0の場合のみ適用）
		dq(i,0) = sim->var.dq.el[i][0];		// 関節速度
		r(i,0) = sim->var.r.el[i][0]-sim->var_init.r.el[i][0];		// 手先位置変位
		h_(i,0) = sim->dyn.h.el[i][0];
	}
	// 時間計測スタート
//	clock_t start = clock();
	// 計算
	Fint = Fint + F*SIM_CYCLE_TIME;
	E_ = Mq*J.inverse()*M.inverse();
	tau_NC = h_-Mq*J.inverse()*dJ*dq;
	tau_VE = -E_*(K*C.inverse()*M*dr+K*r);
	tau_IN = (E_-J.transpose())*F;
	tau_PL = E_*K*C.inverse()*Fint;
	tau_ = tau_NC+tau_VE+tau_IN+tau_PL;
	// 時間計測
//	clock_t end = clock();
#ifdef DEBAG_DISPLAY
	const double time = static_cast<double>(end - start) / CLOCKS_PER_SEC * 1000.0;
	printf("time %lf[ms]\n", time);
#endif
	// 代入
	for(int i=0; i<2; i++)	tau->el[i][0] = tau_(i,0);
	return	0;
}

////////////////////////////////////////////////////////
// Maxwell制御則
// 目標値を Mddr = F-K(r-p), K(r-p) = Cdp から計算
////////////////////////////////////////////////////////
// うまくいっていない．デバッグ中．pの目標値ならば計算可能だが，実際のpの値が取得できない（対応点がない）ので無理か？
int ctrlMaxwellImplicit(cFinger *sim, Matrix *tau)
{
	int	jnt, crd;
	static Matrix	Tmp21, Tmp22, Tmp21_2;
	static Matrix	tauNC, tauVE, tauIN, tauPL, E;
	static Matrix	Integ;
	static Matrix	re, dre, rc, drc, ddr;	// 手先位置速度変位，目標位置速度変位
	static Matrix	A, B, X, U, Xnext, dX;	// 状態空間
	auto entity = EntityManager::get();
	if (entity->step == 0) {
		// 行列初期化
		matInit(&Tmp21, 2, 1); matInit(&Tmp22, 2, 2); matInit(&Tmp21_2, 2, 1);
		matInit(&tauNC, 2, 1); matInit(&tauVE, 2, 1); matInit(&tauIN, 2, 1); matInit(&tauPL, 2, 1); matInit(&E, 2, 2);
		//		matInit(&Integ, 2, 1);
		matInit(&A, 6, 6); matInit(&B, 6, 2); matInit(&X, 6, 1); matInit(&U, 2, 1); matInit(&Xnext, 6, 1); matInit(&dX, 6, 1);
		// 初期設定
		matShareInit(&re, 2, 1); matShareInit(&dre, 2, 1); matShareBlock(&re, &X, 0, 0); matShareBlock(&dre, &X, 2, 0);	// X = [r; dr; p]
																														//		matShareInit(&rc, 2, 1); matShareInit(&drc, 2, 1); 
		matShareInit(&ddr, 2, 1); matShareBlock(&ddr, &dX, 2, 0);	// dX = [dr; ddr; dp]
																	//matShareBlock(&rc, &X, 0, 0); matShareBlock(&drc, &X, 2, 0);
		matAssign(&A, matUnit(&Tmp22), 0, 2);		// I
		matAssign(&A, matSignInv(matMul(&Tmp22, &sim->imp.Minv, &sim->imp.K)), 2, 0);		// -M^{-1}*K
		matAssign(&A, matMul(&Tmp22, &sim->imp.Minv, &sim->imp.K), 2, 4);		// M^{-1}*K
		matAssign(&A, matMul(&Tmp22, &sim->imp.Cinv, &sim->imp.K), 4, 0);		// C^{-1}*K
		matAssign(&A, matSignInv(matMul(&Tmp22, &sim->imp.Cinv, &sim->imp.K)), 4, 4);		// -C^{-1}*K
		matAssign(&B, &sim->imp.Minv, 2, 0);		// M^{-1}
													// 周期計算
		for (crd = 0; crd<2; crd++) {
			if (sim->imp.C.el[crd][crd] * sim->imp.C.el[crd][crd] > sim->imp.M.el[crd][crd] * sim->imp.K.el[crd][crd] / 4) {
				sim->imp.T[crd] = 2 * PI / sqrt(sim->imp.K.el[crd][crd] / sim->imp.M.el[crd][crd] - sim->imp.K.el[crd][crd] * sim->imp.K.el[crd][crd] / (4 * sim->imp.C.el[crd][crd] * sim->imp.C.el[crd][crd]));
				printf("T[%d]=%f\n", crd, sim->imp.T[crd]);
			}
		}
	}
	// 前処理
	matSub(&re, &sim->var.r, &sim->var_init.r);		// 手先位置変位
	matCopy(&dre, &sim->var.dr);	// 手先速度変位（初期速度が0の場合のみ適用）
									//	matAdd(&Integ, &Integ, matMulScl(&Tmp21, SIM_CYCLE_TIME, &sim->var.F));		// Integ = ∫Fdt
									// インナーループ目標値計算
									//	matCopy(&X, &Xnext);	// 1サイクル前の計算値Xnextを初期値Xとして代入
									//	matAdd(&U, &sim->var.F, matMul3(&Tmp22, &sim->imp.K, &sim->imp.Cinv, &Integ));		// U = F+K*C^{-1}*∫Fdt
									//	rk4Acc(sim, &Xnext, &dX, &X, &A, &B, &sim->var.F, SIM_CYCLE_TIME);
	rk4Acc(sim, &X, &dX, &X, &A, &B, &sim->var.F, SIM_CYCLE_TIME);		// Xを次のサイクルの値で更新
																		//	for (crd = 0; crd<DIM2; crd++)	sim->ref_eff_pos[crd] = rc.el[crd][0];		// 変数保存
																		//	for (crd = 0; crd<DIM2; crd++)	sim->ref_eff_vel[crd] = drc.el[crd][0];		// 変数保存
																		// 制御則
																		//	matMul3(&E, &sim->dyn.Mq, &sim->kine.Jinv, &sim->imp.Minv);	// E = Mq*J^{-1}*M^{-1}
																		//	matAdd(&Tmp21, matMulSub(&Tmp21, &sim->imp.Gv, &drc, &dre), matMulSub(&Tmp21_2, &sim->imp.Gp, &rc, &re));		// Gv*(drc-dre)+Gp*(rc-re)
	matSub(&Tmp21, &ddr, matMul(&Tmp21_2, &sim->kine.dJ, &sim->var.dq));		// ddr-dJ*dq
	matAdd(&tauNC, &sim->dyn.h, matMul3(&Tmp21_2, &sim->dyn.Mq, &sim->kine.Jinv, &Tmp21));	// tauNC = h+Mq*J^{-1}*(ddr-dJ*dq)
																							//	matAdd(&Tmp21, matMul4(&tauVE, &sim->imp.K, &sim->imp.Cinv, &sim->imp.M, &drc), matMul(&Tmp21, &sim->imp.K, &rc));		// K*C^{-1}*M*drc+K*rc ← 通常のMaxwellと違い目標値を代入
																							//	matSignInv(matMul(&tauVE, &E, &Tmp21));	// tauVE = -E{K*C^{-1}*M*drc+K*rc}
	matSignInv(matMul(&tauIN, &sim->kine.Jt, &sim->var.F));		// tauIN = -J^T*F
																//	matMul4(&tauPL, &E, &sim->imp.K, &sim->imp.Cinv, &Integ);		// tauPL = E*K*C^{-1}*∫Fdt
	matAdd(tau, &tauNC, &tauIN);		// tau = tauNC+tauIN
	return	0;
}
