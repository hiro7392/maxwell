#ifndef _INC_MATBASE
#define _INC_MATBASE

#include <stdio.h>

typedef	double	SCALAR;
#define MAT_EPS		(1e-6)		// ���e�덷
///////////////////////////
// state�萔
///////////////////////////
#define MAT_STATE_UNINIT	0		// ����������Ă��Ȃ�
#define MAT_STATE_NORMAL	1		// �ʏ�̍s��
#define MAT_STATE_ERROR		2		// 
#define MAT_STATE_SHARE		3		// ���L�s��

///////////////////////////
// �G���[�萔
///////////////////////////
#define MAT_ERROR_MEMORY	0			// �������m�ۂɎ��s
#define MAT_ERROR_UNMATCH_SIZE	1		// �s��T�C�Y���Ⴄ���ߌv�Z�s�\
#define MAT_ERROR_NONEXISTENT	2		// �v�Z���ʂ����݂��Ȃ�
#define MAT_ERROR_ARGUMENT	3		// �������Ԉ���Ă���

///////////////////////////
// �f�o�b�O�萔
///////////////////////////
#define MAT_CHECK_SIZE	1

///////////////////////////
// �s��\����
///////////////////////////
#if 0
typedef struct{
	int	row, col;   // �s���C��
	int	state;		// ���
	SCALAR	**el;	// �v�f�ւ̃|�C���^
}Matrix;
#elif 0
typedef struct matrix{
	int	row, col;   // �s���C��
	int	state;		// ���
	SCALAR	**el;	// �v�f�ւ̃|�C���^
	matrix(): row(0), col(0), state(MAT_STATE_UNINIT), el(NULL) {}		// �f�t�H���g�R���X�g���N�^
}Matrix;
#else
struct Matrix{
	int	row, col;   // �s���C��
	int	state;		// ���
	SCALAR	**el;	// �v�f�ւ̃|�C���^
	Matrix(): row(0), col(0), state(MAT_STATE_UNINIT), el(NULL) {}		// �f�t�H���g�R���X�g���N�^
	Matrix(int Row, int Col){
			// �����`�F�b�N
		if(Row > 0 && Col > 0){ row = Row; col = Col; }
//		else{ matErr(A, MAT_ERROR_MEMORY); return NULL; }
		// �s�������m��
		el = (SCALAR **)malloc(sizeof(SCALAR *) * row);
//		if(el == NULL){ matErr(A, MAT_ERROR_MEMORY); return NULL; }
		// �񃁃����m��
		for(int _row=0; _row<row; _row++){
			el[_row] = (SCALAR *)malloc(sizeof(SCALAR) * col);
			if(el[_row] == NULL){
				while(_row > 0)	free(el[--_row]);
				free(el); //matErr(A, MAT_ERROR_MEMORY); return NULL;
			}
		}
		state = MAT_STATE_NORMAL;
		// �e�v�f��0�ŏ�����
		for(int _row=0; _row<row; _row++) for(int _col=0; _col<col; _col++)  el[_row][_col] = 0.0;
	}
	// matFree���g���Ă���mat�֐������邽�߃G���[���N����
	/*
	~matrix(){
		if(state = MAT_STATE_NORMAL){ for(int _row=0; _row<row; _row++) free(el[row]); free(el);
		}else if(state = MAT_STATE_SHARE) free(el);
	}*/
	Matrix *setDiag(double *element)
	{
		int  sqr, diag;
		if (row > col)	sqr = col;	else	sqr = row;	// �s�Ɨ�̏���������I��
		for (diag = 0; diag<sqr; diag++) el[diag][diag] = element[diag];
		return	this;
	}
};
#endif
#if 0
Matrix::Matrix(): row(0), col(0), state(MAT_STATE_UNINIT), el(NULL) {}		// �f�t�H���g�R���X�g���N�^
Matrix::Matrix(int Row, int Col){
	// �����`�F�b�N
	if(Row > 0 && Col > 0){ row = Row; col = Col; }
//		else{ matErr(A, MAT_ERROR_MEMORY); return NULL; }
	// �s�������m��
	el = (SCALAR **)malloc(sizeof(SCALAR *) * row);
//		if(el == NULL){ matErr(A, MAT_ERROR_MEMORY); return NULL; }
	// �񃁃����m��
	for(int _row=0; _row<row; _row++){
		el[_row] = (SCALAR *)malloc(sizeof(SCALAR) * col);
		if(el[_row] == NULL){
			while(_row > 0)	free(el[--_row]);
			free(el); //matErr(A, MAT_ERROR_MEMORY); return NULL;
		}
	}
	state = MAT_STATE_NORMAL;
	// �e�v�f��0�ŏ�����
	for(int _row=0; _row<row; _row++) for(int _col=0; _col<col; _col++)  el[_row][_col] = 0.0;
}
#endif

///////////////////////////
// �s���{����
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
// �s���{���Z
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
// �啶���ɂ͍s��C�������ɂ̓x�N�g������
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

// �쐬��
Matrix *matEgVal(Matrix *x, const Matrix *A);
Matrix *matHouseholder(Matrix *H, const Matrix *x);
Matrix *matHessenberg(Matrix *H, const Matrix *A);
int matQRDHessen(Matrix *Q, Matrix *R, const Matrix *A);
Matrix *matEigen(Matrix *S, Matrix *x, const Matrix *A);
Matrix *matEigenInvPower(Matrix *x, const Matrix *A);

#endif
