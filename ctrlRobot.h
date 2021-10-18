#ifndef _INC_CTRLROBOT
#define _INC_CTRLROBOT

#include "simMain.h"

#define IMP_SWITCH_STEP		5000

////////////////////////////////////////////////////////
// プロトタイプ
////////////////////////////////////////////////////////
int ctrlMaxwellVar(cFinger *sim, Matrix *tau);
int ctrlMaxwellInnerLoop(cFinger *sim, Matrix *tau);
int ctrlMaxwell(cFinger *sim, Matrix *tau);
int ctrlVoigt(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlHybrid(Matrix *tau, const Matrix *Mq, const Matrix *h, const Matrix *J, const Matrix *dJ, const Matrix *q, const Matrix *dq, const Matrix *re, const Matrix *dre, const Matrix *F, const Matrix *Fint, const Matrix *Md, const Matrix *Cd, const Matrix *Kd);
int ctrlSLS(cFinger *sim, Matrix *tau);

int rk4(cFinger *sim, Matrix *Xnext, Matrix *X, Matrix *A, Matrix *B, Matrix *U, double dt);
int rk4Acc(cFinger *sim, Matrix *Xnext, Matrix *dX, Matrix *X, Matrix *A, Matrix *B, Matrix *U, double dt);
int ctrlMaxwellImplicit(cFinger *sim, Matrix *tau);
int ctrlMaxwellInnerLoopImplicit(cFinger *sim, Matrix *tau);
int ctrlMaxwellConv(cFinger *sim, Matrix *tau);
int simpson(cFinger *sim, Matrix *Integ, double dt);
int simpson2(cFinger *sim, Matrix *Integ, double dt);
int ctrlMaxwellInnerLoopJntSpace(cFinger *sim, Matrix *tau);
int ctrlMaxwellWithoutInertiaShaping(cFinger *sim, Matrix *tau);
int ctrlMaxwellConvInnerLoop(cFinger *sim, Matrix *tau);
int ctrlMaxwellConvRK(cFinger *sim, Matrix *tau);
int ctrlMaxwellConvRK2(cFinger *sim, Matrix *tau);

// Eigenライブラリを用いた行列表記
int ctrlMaxwellE(cFinger *sim, Matrix *tau);
int ctrlMaxwellInLoopE(cFinger *sim, Matrix *tau);

#endif
