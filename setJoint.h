#pragma once
////////////////////////////////////////////////////////
// 実体定義
////////////////////////////////////////////////////////
cParts::cParts(dReal m) : m(m) {
	this->body = dBodyCreate(EntityManager::get()->getWorld()); dMassSetZero(&mass);
	//this->geom = dCreateBox(EntityManager::get()->getSpace(), 1.0, 1.0, 1.0);
}


cParts::cParts(dReal m, Vec3 init_pos) : cParts(m) {
	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
}

cPartsBox::cPartsBox(dReal m, Vec3 l) : cParts(m), sides{ l.x, l.y, l.z } {
	dMassSetBoxTotal(&this->mass, this->m, this->sides[CRD_X], this->sides[CRD_Y], this->sides[CRD_Z]);
	dBodySetMass(this->body, &mass);
	this->geom = dCreateBox(EntityManager::get()->getSpace(), this->sides[CRD_X], this->sides[CRD_Y], this->sides[CRD_Z]);
	dGeomSetBody(this->geom, this->body);
}

cPartsBox::cPartsBox(dReal m, Vec3 init_pos, Vec3 l) : cPartsBox(m, l) {
	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
	//this->geom = dCreateBox(EntityManager::get()->getSpace(), init_pos.x, init_pos.y, init_pos.z);
}

cPartsCylinder::cPartsCylinder(dReal m, dReal l, dReal r) : cParts(m), l(l), r(r) {

	dMassSetCylinderTotal(&this->mass, this->m, DIR_LONG_AXIS_Z, this->r, this->l);
	dBodySetMass(this->body, &mass);
	//ジオメトリの生成
	this->geom = dCreateCylinder(EntityManager::get()->getSpace(), this->r, this->l);
	dGeomSetBody(this->geom, this->body);
}

cPartsCapsule::cPartsCapsule(dReal m, dReal l, dReal r) : cParts(m), l(l), r(r) {

	//	ここからカプセル関連
	dMassSetZero(&mass);

	//	Cylinderの部分とほぼ同じ
	//this->body = dBodyCreate(EntityManager::get()->getWorld());
	dMassSetCapsuleTotal(&this->mass, this->m, DIR_LONG_AXIS_Z, this->r, this->l);
	dBodySetMass(this->body, &mass);

	//	ジオメトリの生成
	this->geom = dCreateCapsule(EntityManager::get()->getSpace(), this->r, this->l);
	dGeomCapsuleSetParams(this->geom, this->r, this->l);
	//ジオメトリとボディの対応付け
	dGeomSetBody(this->geom, this->body);

}

cPartsCylinder::cPartsCylinder(dReal m, Vec3 init_pos, dReal l, dReal r) : cPartsCylinder(m, l, r) {		// デフォルトコンストラクタ
	dBodySetPosition(this->body, init_pos.x, init_pos.y, init_pos.z);
}

//各関節を結合
//HingeJoint:=稼働する関節
void cFinger::setJoint() {

	auto sim = EntityManager::get();
	// ヒンジジョイント1
	r_joint[ARM_M1] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M1], finger[1]->getBody(), finger[0]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M1], x0, y0, 0.4 / 2);
	dJointSetHingeAxis(r_joint[ARM_M1], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);

	// ヒンジジョイント2
	r_joint[ARM_M2] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M2], finger[2]->getBody(), finger[1]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M2], x0 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]), y0 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]), 0.4 / 2);
	dJointSetHingeAxis(r_joint[ARM_M2], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M2], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M2], dParamHiStop, M_PI);

	// 固定ジョイント
	f2_joint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(f2_joint, finger[3]->getBody(), finger[2]->getBody());
	dJointSetFixed(f2_joint);

	// 固定ジョイント	センサと指先の円柱	
	sensor2FingerTop = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(sensor2FingerTop, fingerTopCapsule.getBody(), finger[3]->getBody());
	dJointSetFixed(sensor2FingerTop);

	// センサ設定（力とトルクの取得に必要）
	dJointSetFeedback(sensor2FingerTop, &force);


	//	指旋回関節の土台
	senkai_base_joint = dJointCreateFixed(sim->getWorld(), 0);
	dJointAttach(senkai_base_joint, senkai_base.getBody(), 0);
	dJointSetFixed(senkai_base_joint);

	//	駆動しているトルクを測定
	dJointSetFeedback(senkai_base_joint, &senkai_force);

	// ヒンジジョイント	指旋回関節
	senkai_link_joint = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(senkai_link_joint, senkai_link.getBody(), senkai_base.getBody());
	dJointSetHingeAnchor(senkai_link_joint, senkai_base_x0, senkai_base_y0, senkai_base_z0);
	dJointSetHingeAxis(senkai_link_joint, 1, 0, 0);
	dJointSetHingeParam(senkai_link_joint, dParamLoStop, -M_PI / 2.0);
	dJointSetHingeParam(senkai_link_joint, dParamHiStop, M_PI / 2.0);

	// 固定ジョイント 指旋回関節のリンク->指根本関節
	senkai_link2finger_joint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(senkai_link2finger_joint, finger[0]->getBody(), senkai_link.getBody());
	dJointSetFixed(senkai_link2finger_joint);
#if useForceContactPoint
	//指先カプセルとセンサを接続	//固定ジョイント
	FingerTop2ForcePoint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(FingerTop2ForcePoint, forceContactPoint.getBody(), fingerTopCapsule.getBody());
	dJointSetFixed(FingerTop2ForcePoint);

	dJointSetFeedback(FingerTop2ForcePoint, &fingerTop2ForcePoint_joint);
#endif
	// センサ設定（力とトルクのに必要）
	//dJointSetFeedback(f2_joint, &force);


}
//把持物体の初期位置
dReal capX = -1.2, capY = -0.60, capZ = 0.3;//0.3(二次元) z=1.5(3次元)
//把持物体の大きさ
const dReal plateX = 1.5, plateY = 0.50, plateZ = 0.4;//z=1.2
// const dReal plateX = 1.5, plateY = 0.50, plateZ = 0.4;


//二本目の指の初期位置設定
void cFinger::setJoint2() {

	//実際に把持する用のプレートの生成
#if usePlateToGrasp
	plateToGrasp.body = dBodyCreate(EntityManager::get()->getWorld());
	dMassSetBox(&mass, DENSITY, plateX, plateY, plateZ);

	//Bodyで位置と質量の設定
	dMass massPlate;
	dReal newMass = 0.5;
	dMassSetZero(&massPlate);
	dMassSetBoxTotal(&massPlate, newMass, plateX, plateY, plateZ);
	dBodySetPosition(plateToGrasp.body, capX, capY, capZ);	//位置 //x=-1.1
	//ジオメトリの生成
	auto geomBodyPlate = dCreateBox(EntityManager::get()->getSpace(), plateX, plateY, plateZ);
	//	動力学Bodyと衝突計算ジオメトリの対応
	dGeomSetBody(geomBodyPlate, plateToGrasp.body);
#endif
	auto sim = EntityManager::get();

	// ヒンジジョイント1
	r_joint[ARM_M1] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M1], finger[1]->getBody(), finger[0]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M1], x1, y1, 0.4 / 2);
	dJointSetHingeAxis(r_joint[ARM_M1], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);

	// ヒンジジョイント2
	r_joint[ARM_M2] = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(r_joint[ARM_M2], finger[2]->getBody(), finger[1]->getBody());
	dJointSetHingeAnchor(r_joint[ARM_M2], x1 + ARM_LINK1_LEN * cos(jnt_pos[ARM_M1]), y1 + ARM_LINK1_LEN * sin(jnt_pos[ARM_M1]), 0.4 / 2);
	dJointSetHingeAxis(r_joint[ARM_M2], 0, 0, 1);
	dJointSetHingeParam(r_joint[ARM_M1], dParamLoStop, -M_PI);
	dJointSetHingeParam(r_joint[ARM_M1], dParamHiStop, M_PI);

	// 固定ジョイント
	f2_joint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(f2_joint, finger[3]->getBody(), finger[2]->getBody());
	dJointSetFixed(f2_joint);

	// 固定ジョイント	センサと指先の円柱	
	sensor2FingerTop = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(sensor2FingerTop, fingerTopCapsule.getBody(), finger[3]->getBody());
	dJointSetFixed(sensor2FingerTop);

	// センサ設定（力とトルクの取得に必要）
	dJointSetFeedback(sensor2FingerTop, &force);

	//	指旋回関節の土台
	senkai_base_joint = dJointCreateFixed(sim->getWorld(), 0);
	dJointAttach(senkai_base_joint, senkai_base.getBody(), 0);
	dJointSetFixed(senkai_base_joint);

	// 駆動しているトルクを測定
	dJointSetFeedback(senkai_base_joint, &senkai_force);

	// ヒンジジョイント	指旋回関節
	senkai_link_joint = dJointCreateHinge(sim->getWorld(), 0);
	dJointAttach(senkai_link_joint, senkai_link.getBody(), senkai_base.getBody());
	dJointSetHingeAnchor(senkai_link_joint, senkai_base_x1, senkai_base_y1, senkai_base_z1);
	dJointSetHingeAxis(senkai_link_joint, 1, 0, 0);
	dJointSetHingeParam(senkai_link_joint, dParamLoStop, -M_PI / 2.0);
	dJointSetHingeParam(senkai_link_joint, dParamHiStop, M_PI / 2.0);

	// 固定ジョイント 指旋回関節のリンク->指根本関節
	senkai_link2finger_joint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(senkai_link2finger_joint, finger[0]->getBody(), senkai_link.getBody());
	dJointSetFixed(senkai_link2finger_joint);

#if useForceContactPoint
	//指先カプセルとセンサを接続	//固定ジョイント
	FingerTop2ForcePoint = dJointCreateFixed(sim->getWorld(), 0);  // 固定ジョイント
	dJointAttach(FingerTop2ForcePoint, forceContactPoint.getBody(), fingerTopCapsule.getBody());
	dJointSetFixed(FingerTop2ForcePoint);

	dJointSetFeedback(FingerTop2ForcePoint, &fingerTop2ForcePoint_joint);
#endif

	// センサ設定（力とトルクの取得に必要）
	//dJointSetFeedback(f2_joint, &force);

}