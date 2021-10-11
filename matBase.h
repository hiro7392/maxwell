#ifndef _INC_MATBASE
#define _INC_MATBASE

#include <stdio.h>

typedef	double	SCALAR;
#define MAT_EPS		(1e-6)		// 許容誤差
///////////////////////////
// state定数
///////////////////////////
#define MAT_STATE_UNINIT	0		// 初期化されていない
#define MAT_STATE_NORMAL	1		// 通常の行列
#define MAT_STATE_ERROR		2		// 
#define MAT_STATE_SHARE		3		// 共有行列

///////////////////////////
// エラー定数
///////////////////////////
#define MAT_ERROR_MEMORY	0			// メモリ確保に失敗
#define MAT_ERROR_UNMATCH_SIZE	1		// 行列サイズが違うため計算不能
#define MAT_ERROR_NONEXISTENT	2		// 計算結果が存在しない
#define MAT_ERROR_ARGUMENT	3		// 引数が間違っている

///////////////////////////
// デバッグ定数
///////////////////////////
#define MAT_CHECK_SIZE	1

///////////////////////////
// 行列構造体
///////////////////////////
#if 0
typedef struct{
	int	row, col;   // 行数，列数
	int	state;		// 状態
	SCALAR	**el;	// 要素へのポインタ
}Matrix;
#elif 0
typedef struct matrix{
	int	row, col;   // 行数，列数
	int	state;		// 状態
	SCALAR	**el;	// 要素へのポインタ
	matrix(): row(0), col(0), state(MAT_STATE_UNINIT), el(NULL) {}		// デフォルトコンストラクタ
}Matrix;
#else
struct Matrix{
	int	row, col;   // 行数，列数
	int	state;		// 状態
	SCALAR	**el;	// 要素へのポインタ
	Matrix(): row(0), col(0), state(MAT_STATE_UNINIT), el(NULL) {}		// デフォルトコンストラクタ
	Matrix(int Row, int Col){
			// 引数チェック
		if(Row > 0 && Col > 0){ row = Row; col = Col; }
//		else{ matErr(A, MAT_ERROR_MEMORY); return NULL; }
		// 行メモリ確保
		el = (SCALAR **)malloc(sizeof(SCALAR *) * row);
//		if(el == NULL){ matErr(A, MAT_ERROR_MEMORY); return NULL; }
		// 列メモリ確保
		for(int _row=0; _row<row; _row++){
			el[_row] = (SCALAR *)malloc(sizeof(SCALAR) * col);
			if(el[_row] == NULL){
				while(_row > 0)	free(el[--_row]);
				free(el); //matErr(A, MAT_ERROR_MEMORY); return NULL;
			}
		}
		state = MAT_STATE_NORMAL;
		// 各要素を0で初期化
		for(int _row=0; _row<row; _row++) for(int _col=0; _col<col; _col++)  el[_row][_col] = 0.0;
	}
	// matFreeを使っているmat関数があるためエラーが起きる
	/*
	~matrix(){
		if(state = MAT_STATE_NORMAL){ for(int _row=0; _row<row; _row++) free(el[row]); free(el);
		}else if(state = MAT_STATE_SHARE) free(el);
	}*/
	Matrix *setDiag(double *element)
	{
		int  sqr, diag;
		if (row > col)	sqr = col;	else	sqr = row;	// 行と列の小さい方を選択
		for (diag = 0; diag<sqr; diag++) el[diag][diag] = element[diag];
		return	this;
	}
};
#endif
#if 0
Matrix::Matrix(): row(0), col(0), state(MAT_STATE_UNINIT), el(NULL) {}		// デフォルトコンストラクタ
Matrix::Matrix(int Row, int Col){
	// 引数チェック
	if(Row > 0 && Col > 0){ row = Row; col = Col; }
//		else{ matErr(A, MAT_ERROR_MEMORY); return NULL; }
	// 行メモリ確保
	el = (SCALAR **)malloc(sizeof(SCALAR *) * row);
//		if(el == NULL){ matErr(A, MAT_ERROR_MEMORY); return NULL; }
	// 列メモリ確保
	for(int _row=0; _row<row; _row++){
		el[_row] = (SCALAR *)malloc(sizeof(SCALAR) * col);
		if(el[_row] == NULL){
			while(_row > 0)	free(el[--_row]);
			free(el); //matErr(A, MAT_ERROR_MEMORY); return NULL;
		}
	}
	state = MAT_STATE_NORMAL;
	// 各要素を0で初期化
	for(int _row=0; _row<row; _row++) for(int _col=0; _col<col; _col++)  el[_row][_col] = 0.0;
}
#endif

///////////////////////////
// 行列基本操作
///////////////////////////
Matrix *matInit(Matrix *A, int row, int col);
int matFree(Matrix *mat);
Matrix *matSetVal(Matrix *mat, double *element);
Matrix *matSetValDiag(Matrix *A, double *element);
Matrix *matRand(Matrix *A);
int matPrint(const Matrix *A);
Matrix *matCopy(Matrix *B, const Matrix *A);
Matrix *matAssign(Matrix *B, const Matrix *A, int start_row, int start_col);
Matrix *matCut(Matrix *B, const Matrix *A, int start_row, int start_col);
Matrix *matReplacRow(Matrix *B, const Matrix *A, int row1, int row2);
//Matrix *matCutRow(Matrix *B, const Matrix *A, int row);
int matErr(Matrix *A, int err_no);
Matrix *matReset(Matrix *A, int row, int col);

Matrix *matShareInit(Matrix *A, int row, int col);
Matrix *matShareBlock(Matrix *B, const Matrix *A, int start_row, int start_col);
Matrix *matShareResize(Matrix *A, int row, int col);
int matShareFree(Matrix *A);

///////////////////////////
// 行列基本演算
///////////////////////////
Matrix *matZero(Matrix *A);
Matrix *matUnit(Matrix *A);
Matrix *matAdd(Matrix *C, const Matrix *A, const Matrix *B);
Matrix *matAdd3(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C);
Matrix *matAdd4(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C, const Matrix *D);
Matrix *matSub(Matrix *C, const Matrix *A, const Matrix *B);
Matrix *matMul(Matrix *C, const Matrix *A, const Matrix *B);
Matrix *matMul3(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C);
Matrix *matMul4(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C, const Matrix *D);
Matrix *matMul5(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C, const Matrix *D, const Matrix *E);
Matrix *matMulAdd(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C);
Matrix *matMulSub(Matrix *S, const Matrix *A, const Matrix *B, const Matrix *C);
Matrix *matTrans(Matrix *B, const Matrix *A);
double matTrace(const Matrix *A);
Matrix *matMulScl(Matrix *B, double k, const Matrix *A);
Matrix *matMulSclAdd(Matrix *S, double a, const Matrix *A, double b, const Matrix *B);
Matrix *matMulSclAdd3(Matrix *S, double a, const Matrix *A, double b, const Matrix *B, double c, const Matrix *C);
Matrix *matMulSclAdd4(Matrix *S, double a, const Matrix *A, double b, const Matrix *B, double c, const Matrix *C, double d, const Matrix *D);
Matrix *matSignInv(Matrix *A);
double matInnerProd(const Matrix *A, const Matrix *B);
double matQuadForm(Matrix *vec1, Matrix *mat, Matrix *vec2);

///////////////////////////////////////////////////
// 大文字には行列，小文字にはベクトルを代入
///////////////////////////////////////////////////
int matRank(const Matrix *A);
double matDet(const Matrix *A);
Matrix *matInv(Matrix *B, double *det, const Matrix *A);
int matLUD(Matrix *L, Matrix *U, const Matrix *A);
int matQRD(Matrix *Q, Matrix *R, const Matrix *A);
Matrix *matFwdSubsti(Matrix *c, const Matrix *L, const Matrix *b);
Matrix *matBackSubsti(Matrix *x, const Matrix *U, const Matrix *c);

Matrix *matLinEqLUD(Matrix *x, const Matrix *A, const Matrix *b);
Matrix *matDiv(Matrix *X, const Matrix *A, const Matrix *B);

// 作成中
Matrix *matEgVal(Matrix *x, const Matrix *A);
Matrix *matHouseholder(Matrix *H, const Matrix *x);
Matrix *matHessenberg(Matrix *H, const Matrix *A);
int matQRDHessen(Matrix *Q, Matrix *R, const Matrix *A);
Matrix *matEigen(Matrix *S, Matrix *x, const Matrix *A);
Matrix *matEigenInvPower(Matrix *x, const Matrix *A);

#endif
