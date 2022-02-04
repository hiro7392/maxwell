#ifndef _INC_MAKEROBOT
#define _INC_MAKEROBOT

#include "simMain.h"

//int createRobot(SIM *sim);
int drawRobot(); 
//int drawCoM();
int drawExtForce();
int drawExtForce2();
//int destroyRobot();
int createObject(SIM *sim);
int drawObject(SIM *sim); 
int destroyObject(SIM *sim);
void drawForceCylinder(cParts* sensor, dJointFeedback* p_force);
void drawForceFingerTop(cPartsCylinder& sensor, dJointFeedback* p_force);
void drawArrow(dVector3	p_s, dVector3 p_e, dVector3 ext_f);
#endif
