#include "texturepath.h"
#include "simMain.h"
#include "simMode.h"
#include "setEnv.h"
#include "makeRobot.h"
#include "command.h"
#include "matBase.h"
#include "ctrlRobot.h"
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>

////////////////////////////////////////////////////////
// 転がり摩擦
////////////////////////////////////////////////////////
void rolling_function(SIM *sim, dGeomID o, dReal coef, dContact *c )//床摩擦関数
{
//	MyObject *obj = &sim->sys.obj;
	auto obj = EntityManager::get()->getObj();
	//	if (o && dGeomGetClass(o) == dSphereClass) {
	//	if (o && dGeomGetClass(o) == dCylinderClass) {
	if (o == obj->getGeom()) {
		dBodyID b = dGeomGetBody(o);
		if (!b) return;
		dMass m;
		dBodyGetMass( b, &m );
		dReal* normal = c->geom.normal; // 垂直抗力ベクトル
		dReal w = m.mass*(normal[2]*9.8); // 質量, (memo:角度差cosΘ = (normal[0]*0.0 + normal[1]*0.0 + normal[2]*1.0))
		//dReal r = dGeomSphereGetRadius( o ); // 半径
		dReal r = OBJ_RADIUS; // 半径
		dReal F = coef * (w / r ); // 転がり摩擦(力)
		dReal T = F * r; // 転がり摩擦のトルク?
		//const dReal* av = dBodyGetAngularVel(b);

		const dReal* av = dBodyGetAngularVel(obj->getBody());
		dReal a_speed = sqrt(av[0]*av[0] + av[1]*av[1] + av[2]*av[2]); // 回転スピード
//if(sim.step == 800) printf("%f %f %f %f\n", c->geom.normal[0], c->geom.normal[1], c->geom.normal[2], a_speed);
		if (a_speed > 1.0e-5) {
			dReal n_av[3] = { av[0]/a_speed, av[1]/a_speed, av[2]/a_speed }; // 回転方向の正規化
			dBodyAddTorque( b, -n_av[0]*T, -n_av[1]*T, -n_av[2]*T ); // 転がり摩擦をトルクとしてあたえる
		}else {
			dBodySetAngularVel( b, 0.0f, 0.0f, 0.0f ); // 停止
		}
	}
}

////////////////////////////////////////////////////////
// 距離関数
// 対象(円柱)と手先の最近傍点の距離を計算
////////////////////////////////////////////////////////
int cFinger::calcDist()
{
	double	tmp[DIM2];
	double	C12, S12;
	double	R[12];
	const dReal	*p_R = R;
	double	norX, norY;
//	MyObject *obj = &sim->sys.obj;
	auto obj = EntityManager::get()->getObj();
	// 三角関数
	C12 = cos(jnt_pos[ARM_M1]+jnt_pos[ARM_M2]); S12 = sin(jnt_pos[ARM_M1]+jnt_pos[ARM_M2]);
	// 対象回転軸(p_R[2],p_R[6],p_R[10])のz成分を除去してxy方向で正規化
	p_R = dBodyGetRotation(obj->getBody());
	norX = p_R[2]/sqrt(p_R[2]*p_R[2]+p_R[6]*p_R[6]);	
	norY = p_R[6]/sqrt(p_R[2]*p_R[2]+p_R[6]*p_R[6]);
	// 手先に一番近い点の計算
	if(C12*p_R[2]+S12*p_R[6] > 0){
//		printf("%f %f \n", p_R[2], p_R[6]);
		tmp[CRD_X] = obj_pos[CRD_X] - obj->getl()/2.0*norX + obj->getr()*norY;
		tmp[CRD_Y] = obj_pos[CRD_Y] - obj->getl() /2.0*norY -obj->getr()*norX;
//		printf("%f %f \n", tmp[0], tmp[1]);
	}else{
		tmp[CRD_X] = obj_pos[CRD_X] + obj->getl() /2.0*norX +obj->getr()*norY;
		tmp[CRD_Y] = obj_pos[CRD_Y] + obj->getl() /2.0*norY -obj->getr()*norX;
	}
	// 距離計算
	dist = fabs(C12*(tmp[CRD_X]-eff_pos[CRD_X])+S12*(tmp[CRD_Y]-eff_pos[CRD_Y]));
	return	0;
}

////////////////////////////////////////////////////////
// 外力設定
////////////////////////////////////////////////////////
void cFinger::addExtForce(){
	auto sim = EntityManager::get();
	double ext_force[DIM3];
	double	time;
	time = sim->step*SIM_CYCLE_TIME;
	// 外力を設定
#if 0
	ext_force[CRD_X] = 2.0;//0.2;
	ext_force[CRD_Y] = 2.0;//0.2;
	ext_force[CRD_Z] = 0.0;
#elif 0
	if(sim->step <= 2000){
		ext_force[CRD_X] = 2.0-time;
		ext_force[CRD_Y] = 2.0-time;
		ext_force[CRD_Z] = 2.0-time;
	}else{
		ext_force[CRD_X] = 0.0;
		ext_force[CRD_Y] = 0.0;
		ext_force[CRD_Z] = 0.0;
	}
#else
	//2000ステップまでは外力を加える
	if(sim->step <= 1000){

		if (fingerID == 1) {
			ext_force[CRD_X] = 0.0;
			ext_force[CRD_Y] = 0.0;
			ext_force[CRD_Z] = 0.0;
		}
		else if (fingerID == 2) {
			ext_force[CRD_X] = 0.0;
			ext_force[CRD_Y] = 0.0;
			ext_force[CRD_Z] = 0.0;
		}
	}else{
		ext_force[CRD_X] = 0.0;
		ext_force[CRD_Y] = 0.0;
		ext_force[CRD_Z] = 0.0;
	}
#endif

	// 手先リンク表面の中心に外力入力q
//	MyObject *sensor = &sim->sys.finger[ARM_N1].sensor;
//	dBodyAddForceAtPos(sensor->body, ext_force[CRD_X], ext_force[CRD_Y], ext_force[CRD_Z], sim->eff_pos[CRD_X], sim->eff_pos[CRD_Y], sim->eff_pos[CRD_Z]);
	
	if (fingerID == 1) {
		auto sensor = sim->getFinger()->getParts()[3];
		cPartsCapsule& fingerTopCapsule = sim->getFinger()->fingerTopCapsule;
		//指先のカプセルの内部に外力を加えるとき
		//dBodyAddForceAtPos(fingerTopCapsule.getBody(), ext_force[CRD_X], ext_force[CRD_Y], ext_force[CRD_Z], eff_pos[CRD_X], eff_pos[CRD_Y],eff_pos[CRD_Z]);
		
		//指先先端に取り付けた円柱に力を加えるとき
		dBodyAddForceAtPos(forceContactPoint.getBody(), ext_force[CRD_X], ext_force[CRD_Y], ext_force[CRD_Z], eff_pos[CRD_X], eff_pos[CRD_Y], eff_pos[CRD_Z]);

	}
	else {
		auto sensor = sim->getFinger2()->getParts()[3];
		cPartsCapsule& fingerTopCapsule = sim->getFinger2()->fingerTopCapsule;
		//指先のカプセルの内部に外力を加えるとき
		//dBodyAddForceAtPos(fingerTopCapsule.getBody(), ext_force[CRD_X], ext_force[CRD_Y], ext_force[CRD_Z], eff_pos[CRD_X],eff_pos[CRD_Y],eff_pos[CRD_Z]);
		
		//指先先端に取り付けた円柱に力を加えるとき
		dBodyAddForceAtPos(forceContactPoint.getBody(), ext_force[CRD_X], ext_force[CRD_Y], ext_force[CRD_Z], eff_pos[CRD_X], eff_pos[CRD_Y], eff_pos[CRD_Z]);

	}
#if 1
	printf("Finger %d extForce(fx,fy) = (%lf,%lf) \n",fingerID, ext_force[CRD_X], ext_force[CRD_Y]);
#endif

}

void cFinger::printInfo() {

	std::cout << "finger " << fingerID << std::endl;
	for (int i = 0; i < 2; i++)std::cout << "init_jnt_pos" << init_jnt_pos[i]<<std::endl;
	for(int i=0;i<2;i++)std::cout <<"jnt_force  "<<i<<" : "<< jnt_force[i] << std::endl;

	for (int i = 0; i < ARM_JNT; i++)std::cout << "ref_jnt_vel  " << i << " : " << ref_jnt_vel[i] << std::endl;
	for (int i = 0; i < ARM_JNT; i++)std::cout << "ref_jnt_pos  " << i << " : " << ref_jnt_pos[i] << std::endl;

	for (int i = 0; i < ARM_JNT; i++)std::cout << "eff_pos  " << i << " : " << eff_pos[i] << std::endl;
	for (int i = 0; i < ARM_JNT; i++)std::cout << "eff_vel  " << i << " : " <<eff_vel[i] << std::endl;

	for (int i = 0; i < ARM_JNT; i++)std::cout << "jnt_force  " << i << " : " << jnt_force[i] << std::endl;

	for (int i = 0; i < ARM_JNT; i++)std::cout << "obj_pos  " << i << " : " << obj_pos[i] << std::endl;

	std::cout << "kine " << kine.J.el[0][0] << std::endl;
	std::cout << "this kine " << this->kine.J.el[0][0] << std::endl;
	std::cout << std::endl;

}

////////////////////////////////////////////////////////
// ODEでは関節摩擦を手動で設定
// dJointAddHingeTorque()は上書きではなくインクリメントされることに注意
////////////////////////////////////////////////////////
void cFinger::setJntFric(){
	//auto finger = EntityManager::get()->getFinger();
	dReal	jnt_fric[ARM_JNT] = { ARM_JNT1_VISCOUS, ARM_JNT2_VISCOUS };
//	jnt_fric[ARM_M1] = ARM_JNT1_VISCOUS; jnt_fric[ARM_M1] = ARM_JNT2_VISCOUS;
	// 粘性摩擦
	for(int jnt=0;jnt<ARM_JNT;jnt++){
//		dBodyAddForce(arm[jnt].body, -sim.dyn.V[jnt] * sim.jnt_vel[jnt], 0, 0);	// 直動関節
//		dJointAddHingeTorque(r_joint[jnt], -sim->dyn.V[jnt] * sim->jnt_vel[jnt]);	// 回転関節
		dJointAddHingeTorque(r_joint[jnt], -jnt_fric[jnt] * jnt_vel[jnt]);	// 回転関節
	}
}
