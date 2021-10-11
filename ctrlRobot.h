#ifndef _INC_CTRLROBOT
#define _INC_CTRLROBOT

#include "simMain.h"

#define IMP_SWITCH_STEP		5000

////////////////////////////////////////////////////////
// プロトタイプ
////////////////////////////////////////////////////////
int ctrlMaxwellVar(SIM *sim, Matrix *tau);
int ctrlMaxwellInnerLoop(SIM *sim, Matrix *tau);
int ctrlMaxwell(SIM *sim, Matrix *tau);
int ctrlVoigt(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlHybrid(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Fint, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlSLS(SIM *sim, Matrix *tau);

int rk4(SIM *sim, Matrix *Xnext, Matrix *X, Matrix *A, Matrix *B, Matrix *U, double dt);
int rk4Acc(SIM *sim, Matrix *Xnext, Matrix *dX, Matrix *X, Matrix *A, Matrix *B, Matrix *U, double dt);
int ctrlMaxwellImplicit(SIM *sim, Matrix *tau);
int ctrlMaxwellInnerLoopImplicit(SIM *sim, Matrix *tau);
int ctrlMaxwellConv(SIM *sim, Matrix *tau);
int simpson(SIM *sim, Matrix *Integ, double dt);
int simpson2(SIM *sim, Matrix *Integ, double dt);
int ctrlMaxwellInnerLoopJntSpace(SIM *sim, Matrix *tau);
int ctrlMaxwellWithoutInertiaShaping(SIM *sim, Matrix *tau);
int ctrlMaxwellConvInnerLoop(SIM *sim, Matrix *tau);
int ctrlMaxwellConvRK(SIM *sim, Matrix *tau);
int ctrlMaxwellConvRK2(SIM *sim, Matrix *tau);

// Eigenライブラリを用いた行列表記
int ctrlMaxwellE(SIM *sim, Matrix *tau);
int ctrlMaxwellInLoopE(SIM *sim, Matrix *tau);

#endif
