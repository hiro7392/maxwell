#include "simMain.h"
#include "setRobot.h"
#include "setEnv.h"
#include"calculateParameter.h"
// シミュレーション変数
//extern	SIM sim;

////////////////////////////////////////////////////////
// 衝突検出計算コールバック関数
// 摩擦係数(contact[cnt].surface.mu):この値は並進摩擦であり，回転摩擦は自分で設定する必要がある．無限大(dInfinity)では，衝突の際に対象orアームが動かないため過大な力が発生して，変な挙動を発生することがある．
// つまり，滑りが発生している時の摩擦値．円柱では接触線方向の運動時．
////////////////////////////////////////////////////////
void ODE::nearCallback(void *data, dGeomID o1, dGeomID o2)
{
	//return;
	static const int N = 10;     // 接触点数
	int	flag_ground, flag_sensor,flagFingerCylinder, flagFingerTopCapsule;	// 衝突検出用フラグ
	int	flag_ground2, flag_sensor2, flagFingerCylinder2, flagFingerTopCapsule2;	// 衝突検出用フラグ
	dContact contact[N];
	dBodyID b1, b2;
	dJointID c;
	auto _this = EntityManager::get();

	//for (int idx = 0; idx < ARM_NUM; idx++) {
	for (int idx = 0; idx < 1; idx++) {
//		MyObject *sensor = &_this->sys.finger[idx].sensor;
		auto sensor = EntityManager::get()->getFinger()->getParts()[3];
		auto sensor2 = EntityManager::get()->getFinger2()->getParts()[3];
		auto fingerCylinder = EntityManager::get()->getFinger()->getParts()[2];
		auto fingerCylinder2 = EntityManager::get()->getFinger2()->getParts()[2];
		
		cPartsCapsule& fingerTopCapsule =(EntityManager::get()->getFinger()->fingerTopCapsule);
		cPartsCapsule& fingerTopCapsule2 = (EntityManager::get()->getFinger2()->fingerTopCapsule);

		// 地面との衝突検出
		flag_ground = ((o1 == _this->ground) || (o2 == _this->ground));
		// アーム手先との衝突検出
		flag_sensor = ((o1 == sensor->getGeom()) || (o2 == sensor->getGeom()));
		flag_sensor2 = ((o1 == sensor->getGeom()) || (o2 == sensor->getGeom()));
		//	指先の衝突判定
		flagFingerTopCapsule=((o1 == fingerTopCapsule.getGeom()) || (o2 == fingerTopCapsule.getGeom()));
		flagFingerTopCapsule2 = ((o1 == fingerTopCapsule2.getGeom()) || (o2 == fingerTopCapsule2.getGeom()));



		if (o1 == fingerCylinder->getGeom() || o2 == fingerCylinder->getGeom())return;
		if (o1 == sensor->getGeom() || o2 == sensor->getGeom())return;
		if (o1 == fingerCylinder2->getGeom() || o2 == fingerCylinder2->getGeom())return;
		if (o1 == sensor2->getGeom() || o2 == sensor2->getGeom())return;

		//	指先カプセルとアームの第二関節円柱が接触していても判定はないことにする
		if (((o1 == fingerCylinder->getGeom()) && (o2 == fingerTopCapsule.getGeom())) ||
		((o2 == fingerCylinder->getGeom()) && (o1 == fingerTopCapsule.getGeom())))return;

		if (((o1 == fingerCylinder2->getGeom()) && (o2 == fingerTopCapsule2.getGeom())) ||
			((o2 == fingerCylinder2->getGeom()) && (o1 == fingerTopCapsule2.getGeom())))return;


		//センサと指先カプセルが接触していても判定しない
		if (((o1 == sensor->getGeom()) && (o2 == fingerTopCapsule.getGeom())) ||
			((o2 == sensor->getGeom()) && (o1 == fingerTopCapsule.getGeom())))return;

		if (((o1 == sensor2->getGeom()) && (o2 == fingerTopCapsule2.getGeom())) ||
			((o2 == sensor2->getGeom()) && (o1 == fingerTopCapsule2.getGeom())))return;


		// 2つのボディがジョイントで結合されていたら衝突検出しない
		b1 = dGeomGetBody(o1);	b2 = dGeomGetBody(o2);
		if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
		// 衝突設定
		int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

		//地面と接触していたとき
		if (flag_ground) {
			for (int cnt = 0; cnt < n; cnt++) {
				contact[cnt].surface.mode = dContactBounce | dContactSoftERP | dContactSoftCFM;
				contact[cnt].surface.soft_erp = 0.2;   // 接触点のERP
				contact[cnt].surface.soft_cfm = 0.001; // 接触点のCFM
				contact[cnt].surface.mu = 0.9; // 摩擦係数　もともと0.5
				c = dJointCreateContact(_this->world, _this->contactgroup, &contact[cnt]);
				dJointAttach(c, dGeomGetBody(contact[cnt].geom.g1), dGeomGetBody(contact[cnt].geom.g2));
				// 転がり摩擦
				//#define	COEF_FRIC_ROLL	0.006
#define	COEF_FRIC_ROLL	0.0006
				rolling_function(_this, o1, COEF_FRIC_ROLL / n, contact + cnt);
				rolling_function(_this, o2, COEF_FRIC_ROLL / n, contact + cnt);
			}
			//		if(n!=0)( o1, COEF_FRIC_ROLL, contact );
		}
		//センサーor指の第二関節が何かと接触していた時
		if (/*flag_sensor||/*flagFingerCylinder ||*/  flagFingerTopCapsule || flagFingerTopCapsule2 ) {
			for (int cnt = 0; cnt < n; cnt++) {
				//指先と指の腹の接触判定をなくす
				if(((contact[cnt].geom.g1 == fingerCylinder->getGeom()) && (contact[cnt].geom.g2 == fingerTopCapsule.getGeom())) ||
					((contact[cnt].geom.g2 == fingerCylinder->getGeom()) && (contact[cnt].geom.g1 == fingerTopCapsule.getGeom())))return;
				if (((contact[cnt].geom.g1 == fingerCylinder2->getGeom()) && (contact[cnt].geom.g2 == fingerTopCapsule2.getGeom())) ||
					((contact[cnt].geom.g2 == fingerCylinder2->getGeom()) && (contact[cnt].geom.g1 == fingerTopCapsule2.getGeom())))return;


				//センサーと先端カプセル
				if (((contact[cnt].geom.g1 == sensor->getGeom()) && (contact[cnt].geom.g2 == fingerTopCapsule.getGeom())) ||
					((contact[cnt].geom.g2 == sensor->getGeom()) && (contact[cnt].geom.g1 == fingerTopCapsule.getGeom())))return;
				if (((contact[cnt].geom.g1 == sensor2->getGeom()) && (contact[cnt].geom.g2 == fingerTopCapsule2.getGeom())) ||
					((contact[cnt].geom.g2 == sensor2->getGeom()) && (contact[cnt].geom.g1 == fingerTopCapsule2.getGeom())))return;

				//把持物体と指先
				contact[cnt].surface.mode = dContactBounce | dContactSoftERP | dContactSoftCFM;
				//			contact[cnt].surface.soft_erp   = 0.45;   // 接触点のERP
				//			contact[cnt].surface.soft_cfm   = 0.005; // 接触点のCFM
#if 1
				contact[cnt].surface.soft_erp = 0.2;   // 接触点のERP
				contact[cnt].surface.soft_cfm = 0.005; // 接触点のCFM
#else
				contact[cnt].surface.soft_erp = 0.2;   // 接触点のERP
				contact[cnt].surface.soft_cfm = 0.0005; // 接触点のCFM
#endif
				contact[cnt].surface.mu = 2.0;			// 摩擦係数	xy平面上での実験時は0.5
				contact[cnt].surface.bounce = 0.5;		// 反発係数 xy平面上での実験時は0.5
				c = dJointCreateContact(_this->world, _this->contactgroup, &contact[cnt]);
				dJointAttach(c, dGeomGetBody(contact[cnt].geom.g1), dGeomGetBody(contact[cnt].geom.g2));
			}

			if (flagFingerTopCapsule != 0)	_this->getFinger()->state_contact = 1;
			if (flagFingerTopCapsule2 != 0)	_this->getFinger2()->state_contact = 1;

		}
	}
}

////////////////////////////////////////////////////////
// 初期化
////////////////////////////////////////////////////////
// パラメータ誤差を追加
// 物理シミュレーションとは異なるパラメータを代入してロバスト性を検証

int cFinger::ctrlInitErr()

{
#if 0
	this->dyn.m[ARM_M1] += 0.3;	this->dyn.m[ARM_M2] += 0.2;		// 質量誤差は数百g程度
	this->dyn.V[ARM_M1] *= 2.0;	this->dyn.V[ARM_M2] *= 2.5;		// 粘性摩擦の誤差は数倍
#endif
	return	0;
}

////////////////////////////////////////////////////////
// 前処理(転置行列，逆行列等)
// 実行前に以下の行列を設定する
// 入力：sim.imp.M, sim.imp.C, sim.imp.K
// 出力：sim.imp.Minv, sim.imp.Cinv, sim.imp.Kinv
////////////////////////////////////////////////////////

int cFinger::ctrlPreProcessing()
{
	// 前処理
//	matTrans(&this->kine.Jt, &this->kine.J);		// Jt
//	matInv(&this->kine.Jinv, NULL, &this->kine.J);		// Jinv（正則の場合のみ対応）
	// インピーダンス設定
//	setInherentInertia(sim);
	// インピーダンス逆行列
	matInv(&this->imp.Minv, NULL, &this->imp.M);
	matInv(&this->imp.Cinv, NULL, &this->imp.C);
	matInv(&this->imp.Kinv, NULL, &this->imp.K);
	return	0;
}

////////////////////////////////////////////////////////
// 慣性インピーダンスに固有慣性を設定(Inertia Shaping なし)
// 出力：this->imp.M, this->imp.dM
////////////////////////////////////////////////////////

int cFinger::armWithoutInertiaShaping()
{
	static Matrix	Jinvt(DIM2,ARM_JNT), d_Jinv(ARM_JNT,DIM2), d_Jinvt(DIM2,ARM_JNT);		// J^{-T}, d[J^{-1}], d[J^{-1}]^{T}
	static Matrix	Tmp22(DIM2,DIM2), Tmp22_2(DIM2,DIM2), Tmp22_3(DIM2,DIM2);
	// 前処理
	matTrans(&Jinvt, &this->kine.Jinv);		// J^{-T}
	matMulScl(&d_Jinv, -1, matMul3(&Tmp22, &this->kine.Jinv, &this->kine.dJ, &this->kine.Jinv));		// d[J^{-1}] = -J^{-1}*dJ*J^{-1}
	matTrans(&d_Jinvt, &d_Jinv);		// d[J^{-1}]^T
	// 慣性設定
	matMul3(&this->imp.M, &Jinvt, &this->dyn.Mq, &this->kine.Jinv);	// M = J^{-T}*Mq*J^{-1}
	matAdd3(&this->imp.dM, matMul3(&Tmp22, &d_Jinvt, &this->dyn.Mq, &this->kine.Jinv), matMul3(&Tmp22_2, &Jinvt, &this->dyn.dMq, &this->kine.Jinv), matMul3(&Tmp22_3, &Jinvt, &this->dyn.Mq, &d_Jinv));		// dM = d[J^{-1}]^{T}*Mq*J^{-1}+J^{-T}*Mq*J^{-1}+J^{-T}*Mq*J^{-1}
	matInv(&this->imp.Minv, NULL, &this->imp.M);		// 	M^{-1}
	// デバッグ
	//	matPrint(&this->imp.M);
	return	0;
}

////////////////////////////////////////////////////////
// ダイナミクス設定
// 入力：関節変数
// 出力：ヤコビアン，慣性項，遠心・コリオリ力項
////////////////////////////////////////////////////////
int cFinger::armDynPara()
{
	int	jnt;
	//static double	m1, m2, l1, l2, lg1, lg2, I1, I2;
	double	m1, m2, l1, l2, lg1, lg2, I1, I2;

	double	C1, C2, S1, S2, C12, S12;
	auto entity = EntityManager::get();
	// パラメータ設定
	//if(entity->step == 0){
		m1 = this->dyn.m[ARM_M1]; m2 = this->dyn.m[ARM_M2];
		l1 = this->kine.l[ARM_M1]; l2 = this->kine.l[ARM_M2];
		lg1 = this->kine.lg[ARM_M1]; lg2 = this->kine.lg[ARM_M2];
		I1 = (this->kine.r[ARM_M1]*this->kine.r[ARM_M1]/4+l1*l1/12)*m1;
		I2 = (this->kine.r[ARM_M2]*this->kine.r[ARM_M2]/4+l2*l2/12)*m2;
	//}
	// 三角関数
	C1 = cos(this->var.q.el[0][0]); C2 = cos(this->var.q.el[1][0]); S1 = sin(this->var.q.el[0][0]); S2 = sin(this->var.q.el[1][0]);
	C12 = cos(this->var.q.el[0][0]+this->var.q.el[1][0]); S12 = sin(this->var.q.el[0][0]+this->var.q.el[1][0]);
	// 慣性行列
	this->dyn.Mq.el[0][0] = m1*lg1*lg1+I1+m2*(l1*l1+lg2*lg2+2*l1*lg2*C2)+I2;
	this->dyn.Mq.el[0][1] = this->dyn.Mq.el[1][0] = m2*(lg2*lg2+l1*lg2*C2)+I2;
	this->dyn.Mq.el[1][1] = m2*lg2*lg2+I2;
	// 遠心・コリオリ力
	this->dyn.h.el[0][0] = -m2*l1*lg2*S2*(this->var.dq.el[1][0]*this->var.dq.el[1][0]+2*this->var.dq.el[0][0]*this->var.dq.el[1][0]);
	this->dyn.h.el[1][0] = m2*l1*lg2*S2*this->var.dq.el[0][0]*this->var.dq.el[0][0];
	// 関節粘性摩擦力をhに追加
	//for(jnt=0;jnt<ARM_JNT;jnt++)	this->dyn.h.el[jnt][0] += this->dyn.V[jnt] * this->jnt_vel[jnt];	//オリジナル
	for (jnt = 0; jnt < ARM_JNT; jnt++)	this->dyn.h.el[jnt][0] -= this->dyn.V[jnt] * this->jnt_vel[jnt];

	// ヤコビアン
	this->kine.J.el[0][0] = -(l1*S1+l2*S12);	this->kine.J.el[0][1] = -l2*S12;
	this->kine.J.el[1][0] = l1*C1+l2*C12;	this->kine.J.el[1][1] = l2*C12;
	// 慣性行列微分
	this->dyn.dMq.el[0][0] = -2*m2*l1*lg2*S2*this->var.dq.el[1][0];
	this->dyn.dMq.el[0][1] = this->dyn.dMq.el[1][0] = -m2*l1*lg2*S2*this->var.dq.el[1][0];
	this->dyn.dMq.el[1][1] = 0.0;
	// ヤコビアン微分
	this->kine.dJ.el[0][0] = -this->eff_vel[CRD_Y]; this->kine.dJ.el[0][1] = -l2*C12*(this->var.dq.el[0][0]+this->var.dq.el[1][0]);
	this->kine.dJ.el[1][0] = this->eff_vel[CRD_X]; this->kine.dJ.el[1][1] = -l2*S12*(this->var.dq.el[0][0]+this->var.dq.el[1][0]);
	// ヤコビアン転置，ヤコビアン逆行列
	matTrans(&this->kine.Jt, &this->kine.J);		// J^{T}
	matInv(&this->kine.Jinv, NULL, &this->kine.J);		// J^{-1}（正則の場合のみ対応）
	
	//　重力項の計算
	//  計算した重力項をhに足し合わせる
	// 
	// 旋回関節の動的パラメータ
	const dReal* senkai_base_pos = dBodyGetPosition(this->senkai_base.getBody());

	//	旋回リンクの重心位置
	const dReal* senkai_link_pos = dBodyGetPosition(this->senkai_link.getBody());

	//	リンク1の重心位置
	const dReal* link1_pos = dBodyGetPosition(this->link1.getBody());
	//	リンク2の重心位置
	const dReal* link2_pos = dBodyGetPosition(this->link2.getBody());
	//	カプセルの重心位置
	const dReal* capsule_pos = dBodyGetPosition(this->fingerTopCapsule.getBody());
	//	センサの重心位置
	const dReal* sensor_pos = dBodyGetPosition(this->sensor.getBody());

	//	旋回関節角に関する慣性モーメント求める

	//	旋回関節角からの距離を計算する
	static double link1_dist, link2_dist,capsule_dist,sensor_dist;
	link1_dist =	getDistPlain(senkai_base_pos[0], link1_pos[0], senkai_base_pos[1], link1_pos[1]);
	link2_dist =	getDistPlain(senkai_base_pos[0], link2_pos[0], senkai_base_pos[1], link2_pos[1]);
	capsule_dist =	getDistPlain(senkai_base_pos[0], capsule_pos[0], senkai_base_pos[1], capsule_pos[1]);
	sensor_dist =	getDistPlain(senkai_base_pos[0], sensor_pos[0], senkai_base_pos[1], sensor_pos[1]);

	// 各リンクの接点からの距離×質量
	static double senkai_link_Ix,base_Ix,link1_Ix, link2_Ix, capsule_Ix, sensor_Ix;
	senkai_link_Ix	= senkai_link.getMass() * SENKAI_LINK_LEN / 2.0;
	base_Ix		=	base.getMass() * SENKAI_LINK_LEN;
	link1_Ix	=	link1_dist * link1.getMass();
	link2_Ix	=	link2_dist * link2.getMass();
	capsule_Ix	=	capsule_dist * fingerTopCapsule.getMass();
	sensor_Ix	=	sensor_dist * sensor.getMass();
	//	各関節について　(zy平面における距離r^2)*質量m を足し合わせる
	this->rotImp.Iq = senkai_link_Ix + base_Ix + link1_Ix + link2_Ix + capsule_Ix + sensor_Ix;
	printf("rotImp.Iq =%lf\n ",this->rotImp.Iq);

	//	同次行列の計算
	setTransMatrix();

	//	各リンクの質量中心を設定
	setMassCenterPosition(imp.s1, link1_pos[0], link1_pos[1], link1_pos[2]);
	setMassCenterPosition(imp.s2, link2_pos[0], link2_pos[1], link2_pos[2]);
	setMassCenterPosition(imp.ss, senkai_link_pos[0], senkai_link_pos[1], senkai_link_pos[2]);

	//MatPrintDebug4x1(imp.s1, "s1");
	//MatPrintDebug4x1(imp.s2, "s2");
	//MatPrintDebug4x1(imp.ss, "ss");

	//	各関節について重力項を計算
	calculateGravity();
	printf("fingerID = %d\n", fingerID);
	MatPrintDebug3x1(imp.G, "G");
	//	重力項をhに足す
	dyn.h.el[0][0] += imp.G.el[0][0];
	dyn.h.el[1][0] += imp.G.el[0][1];
	rotImp.h += imp.G.el[0][2];
	return	0;
}



//
int cFinger::senkaiDynPara()
{
	// 関節粘性摩擦力をhに追加
	//for (jnt = 0; jnt < ARM_JNT; jnt++)	this->dyn.h.el[jnt][0] += this->dyn.V[jnt] * this->jnt_vel[jnt];
	
	return 0;
}

////////////////////////////////////////////////////////
// ヤコビアン計算(ヤコビアン, 転置ヤコビアン, 逆ヤコビアン, 微分ヤコビアン)
// 入力：var, sim(リンク長はsimに設定済み)
// 出力：kine
////////////////////////////////////////////////////////
int cFinger::armJacob(Kinematics *kine, Variable *var)
{
	static double	l1, l2;
	double	C1, C2, S1, S2, C12, S12;

	auto entity = EntityManager::get();
	// パラメータ設定
	if (entity->step == 0) { l1 = this->kine.l[ARM_M1]; l2 = this->kine.l[ARM_M2]; }
	// 三角関数
	C1 = cos(var->q.el[0][0]); C2 = cos(var->q.el[1][0]); S1 = sin(var->q.el[0][0]); S2 = sin(var->q.el[1][0]);
	C12 = cos(var->q.el[0][0] + var->q.el[1][0]); S12 = sin(var->q.el[0][0] + var->q.el[1][0]);
	// ヤコビアン
	kine->J.el[0][0] = -(l1*S1 + l2 * S12);	kine->J.el[0][1] = -l2 * S12;
	kine->J.el[1][0] = l1 * C1 + l2 * C12;	kine->J.el[1][1] = l2 * C12;
	// ヤコビアン転置，ヤコビアン逆行列
	matTrans(&kine->Jt, &kine->J);		// J^{T}
	matInv(&kine->Jinv, NULL, &kine->J);		// J^{-1}（正則の場合のみ対応
#if 1
	matMul(&var->dq, &kine->Jinv, &var->dr);		// dq = J^{-1}*dr
	// ヤコビアン微分
	kine->dJ.el[0][0] = -(kine->J.el[1][0]*var->dq.el[0][0]+kine->J.el[1][1]*var->dq.el[1][0]); kine->dJ.el[0][1] = -l2 * C12*(var->dq.el[0][0] + var->dq.el[1][0]);
	kine->dJ.el[1][0] = kine->J.el[0][0]*var->dq.el[0][0]+kine->J.el[0][1]*var->dq.el[1][0]; kine->dJ.el[1][1] = -l2 * S12*(var->dq.el[0][0] + var->dq.el[1][0]);
#endif
	return	0;
}

////////////////////////////////////////////////////////
// 逆運動学計算(手先変数⇒関節変数)
// 入力：var, sim(リンク長はsimに設定済み)
// 出力：kine
////////////////////////////////////////////////////////
int cFinger::armInvKine(Kinematics *kine, Variable *var)
{
	static double	l1, l2;
	auto entity = EntityManager::get();
	// パラメータ設定
	if (entity->step == 0) { l1 = this->kine.l[ARM_M1]; l2 = this->kine.l[ARM_M2]; }
	// 関節角
#if 0
	pos[0] = refPos->el[0][0] + initPos.el[0][0];
	pos[1] = refPos->el[1][0] + initPos.el[1][0]; 
	d1 = (pos[0]*pos[0]+pos[1]*pos[1]-lfs[0]*lfs[0]-lfs[1]*lfs[1])/(2*lfs[0]*lfs[1]);
	d2 = sqrt(1-d1*d1);
	jnt_ang[0] = atan2(pos[1], pos[0])-atan2(lfs[1]*d2, lfs[0]+lfs[1]*d1);
	jnt_ang[1] = atan2(d2, d1);
	//2軸はatan2で-PIからPIの範囲に収まるため
	if (jnt_ang[0] < -PI)	jnt_ang[0] += 2 * PI;
	else if (jnt_ang[0] > PI)	jnt_ang[0] -= 2 * PI;
#else
	double	d1 = (var->r.el[CRD_X][0]*var->r.el[CRD_X][0]+var->r.el[CRD_Y][0]*var->r.el[CRD_Y][0]-l1*l1-l2*l2)/(2*l1*l2);
	double	d2 = sqrt(1-d1*d1);
	var->q.el[ARM_M1][0] = atan2(var->r.el[CRD_Y][0], var->r.el[CRD_X][0])-atan2(l2*d2, l1+l2*d1);
	var->q.el[ARM_M2][0] = atan2(d2, d1);
#if 0
	// 2軸はatan2で-PIからPIの範囲に収まるため
	if (var->q.el[ARM_M1][0] < -PI)	var->q.el[ARM_M1][0] += 2 * PI;
	else if (var->q.el[ARM_M1][0] > PI)	var->q.el[ARM_M1][0] -= 2 * PI;
#else
	// 1軸は0から2*PIの範囲に収める
	if (var->q.el[ARM_M1][0] < 0)	var->q.el[ARM_M1][0] += 2 * PI;
	else if (var->q.el[ARM_M1][0] > 2*PI)	var->q.el[ARM_M1][0] -= 2 * PI;
#endif
#endif
	// 関節速度
#if 1
	//auto _this = EntityManager::get()->getFinger();
	armJacob(kine, var);		// 後で順番を要検討
#endif
//	matMul(&var->dq, &kine->Jinv, &var->dr);		// dq = J^{-1}*dr
	return	0;
}

////////////////////////////////////////////////////////
// 行列初期化
// 初期化が不要な構造体にはNULLを代入
////////////////////////////////////////////////////////
int cFinger::armInitMat(Variable *var, Kinematics *kine, Dynamics *dyn, Impedance *imp)
{
	if(var != NULL){
		matInit(&var->q,ARM_JNT,1);	matInit(&var->dq,ARM_JNT,1); matInit(&var->ddq,ARM_JNT,1);
		matInit(&var->r,DIM2,1); matInit(&var->dr,DIM2,1); matInit(&var->ddr,DIM2,1);
		matInit(&var->F,DIM2,1);
	}
	if(kine != NULL){	// 運動学
		matInit(&kine->J,ARM_JNT,ARM_JNT); matInit(&kine->dJ,ARM_JNT,ARM_JNT);
		matInit(&kine->Jt,ARM_JNT,ARM_JNT); matInit(&kine->Jinv,ARM_JNT,ARM_JNT);
	}
	if(dyn != NULL){	// 動力学
		matInit(&dyn->Mq,ARM_JNT,ARM_JNT); matInit(&dyn->h,ARM_JNT,1);
		matInit(&dyn->dMq,ARM_JNT,ARM_JNT);
	}
	if(imp != NULL){	// インピーダンス
		matInit(&imp->M,DIM2,DIM2); matInit(&imp->C,DIM2,DIM2); matInit(&imp->K,DIM2,DIM2);
		matInit(&imp->Minv,DIM2,DIM2); matInit(&imp->Cinv,DIM2,DIM2); matInit(&imp->Kinv,DIM2,DIM2);
		matInit(&imp->Gp,DIM2,DIM2); matInit(&imp->Gv,DIM2,DIM2);
		matInit(&imp->dM,DIM2,DIM2); matInit(&imp->dC,DIM2,DIM2); matInit(&imp->dK,DIM2,DIM2);
		matInit(&imp->K0,DIM2,DIM2);
	}
	return 0;
}

#if 0
int SIM::armInitMatVar(Variable *var)
{
	matInit(&var->q,ARM_JNT,1);	matInit(&var->dq,ARM_JNT,1); matInit(&var->ddq,ARM_JNT,1);
	matInit(&var->F,DIM2,1);
	matInit(&var->r,DIM2,1); matInit(&var->dr,DIM2,1); matInit(&var->ddr,DIM2,1);
	return 0;
}

int SIM::armInitMatKine(Kinematics *kine)
{
	matInit(&kine->J,ARM_JNT,ARM_JNT); matInit(&kine->dJ,ARM_JNT,ARM_JNT);
	matInit(&kine->Jt,ARM_JNT,ARM_JNT); matInit(&kine->Jinv,ARM_JNT,ARM_JNT);
	return 0;
}
#endif

// 塑性変形制御の振動周期計算（インピーダンス制御とは異なるので注意）
int cFinger::armCalcImpPeriod()
{
	for (int crd = 0; crd<2; crd++){
		if (this->imp.C.el[crd][crd] * this->imp.C.el[crd][crd] > this->imp.M.el[crd][crd] * this->imp.K.el[crd][crd] / 4){
			this->imp.T[crd] = 2 * PI / sqrt(this->imp.K.el[crd][crd] / this->imp.M.el[crd][crd] - this->imp.K.el[crd][crd] * this->imp.K.el[crd][crd] / (4 * this->imp.C.el[crd][crd] * this->imp.C.el[crd][crd]));
			printf("T[%d]=%f\n", crd, this->imp.T[crd]);
		}
	}
	return 0;
}
