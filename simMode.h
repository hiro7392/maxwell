#ifndef _INC_SIMMODE
#define _INC_SIMMODE

// シミュレーションモードの選択
#define	SIM_OBJ_IMPACT	0		// 対象衝突
#define	SIM_ADD_EXT_FORCE	1		// 定常外力
// 対象モードの選択
#define	SIM_OBJ_CASE1	0		//
#define	SIM_OBJ_CASE2	0		//
#define	SIM_OBJ_CASE3	1		//
#define	SIM_OBJ_INIT_ABS_VEL	(1.0)	// 対象の初期速度（移動方向をx軸に設定）

// 制御系の選択
#define	SIM_CTRL_MODE_HYBRID	0		// 弾塑性ハイブリッド制御
#define	SIM_CTRL_MODE_MAXWELL	1		// 塑性変形制御
#define	SIM_CTRL_MODE_VOIGT		0		// 弾性変形制御

// パラメータ誤差ありのモード選択
//#define	SIM_CTRL_MODE_PARA_ERR		// 誤差を追加

// 対象
#define	OBJ_RADIUS	(0.10)

#endif
