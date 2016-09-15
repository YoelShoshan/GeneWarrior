#include "stdafx.h"
// Based on Python Code made by Matt Heinzen
#include "Ragdoll.h"
#include <assert.h>


extern FILE* g_pLog;

// rotation directions are named by the third (z-axis) row of the 3x3 matrix,
//   because ODE capsules are oriented along the z-axis
#define rightRot Mat3x3(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0)
#define leftRot	 Mat3x3(0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0)
#define upRot	 Mat3x3(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0)
#define downRot	 Mat3x3(1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0)
#define bkwdRot	 Mat3x3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

// axes used to determine constrained joint rotations
#define rightAxis	Vec3d (1.0, 0.0, 0.0)
#define leftAxis	Vec3d (-1.0, 0.0, 0.0)
#define upAxis		Vec3d (0.0, 1.0, 0.0) // changed
#define downAxis	Vec3d (0.0, -1.0, 0.0)
#define bkwdAxis	Vec3d (0.0, 0.0, 1.0)
#define fwdAxis		Vec3d (0.0, 0.0, -1.0)

#define UPPER_ARM_LEN  0.30
#define FORE_ARM_LEN  0.25
#define HAND_LEN  0.13 // wrist to mid-fingers only
#define FOOT_LEN  0.18 // ankles to base of ball of foot only
#define HEEL_LEN  0.05

#define BROW_H  1.68
#define MOUTH_H  1.53
#define NECK_H  1.50
#define SHOULDER_H  1.37
#define CHEST_H  1.35
#define HIP_H  0.86
#define KNEE_H  0.48
#define ANKLE_H  0.08

#define SHOULDER_W 0.41
#define CHEST_W 0.36 // actually wider, but we want narrower than shoulders (esp. with large radius)
#define LEG_W  0.28 // between middles of upper legs
#define PELVIS_W  0.25 // actually wider, but we want smaller than hip width

#define VEC(x,y,z) Vec3d(x,z,y)
//#define VEC(x,y,z) Vec3d(x,y,z)

#define R_SHOULDER_POS  VEC(-SHOULDER_W * 0.5, SHOULDER_H,0.0 )
#define L_SHOULDER_POS  VEC(SHOULDER_W * 0.5, SHOULDER_H,0.0 )
#define R_ELBOW_POS  sub3(R_SHOULDER_POS, VEC(UPPER_ARM_LEN, 0.0, 0.0))
#define L_ELBOW_POS  add3(L_SHOULDER_POS, VEC(UPPER_ARM_LEN, 0.0, 0.0))
#define R_WRIST_POS  sub3(R_ELBOW_POS, VEC(FORE_ARM_LEN, 0.0, 0.0))
#define L_WRIST_POS  add3(L_ELBOW_POS, VEC(FORE_ARM_LEN, 0.0, 0.0))
#define R_FINGERS_POS  sub3(R_WRIST_POS, VEC(HAND_LEN, 0.0, 0.0))
#define L_FINGERS_POS  add3(L_WRIST_POS, VEC(HAND_LEN, 0.0, 0.0))

#define R_HIP_POS  VEC(-LEG_W * 0.5, HIP_H,0.0 )
#define L_HIP_POS  VEC(LEG_W * 0.5,HIP_H,0.0  )
#define R_KNEE_POS  VEC(-LEG_W * 0.5, KNEE_H,0.0 ) 
#define L_KNEE_POS  VEC(LEG_W * 0.5, KNEE_H,0.0 )
#define R_ANKLE_POS  VEC(-LEG_W * 0.5, ANKLE_H,0.0 )
#define L_ANKLE_POS  VEC(LEG_W * 0.5, ANKLE_H,0.0 )
#define R_HEEL_POS  sub3(R_ANKLE_POS, VEC(0.0,0.0,HEEL_LEN  ))
#define L_HEEL_POS  sub3(L_ANKLE_POS, VEC(0.0,0.0,HEEL_LEN  ))
#define R_TOES_POS  add3(R_ANKLE_POS, VEC(0.0,0.0,FOOT_LEN ))
#define L_TOES_POS  add3(L_ANKLE_POS, VEC(0.0,0.0,FOOT_LEN ))

//added
#define BACK_POS  VEC(0,CHEST_H,0)


double Ragdoll::s_amotors_max_forces[MAX_MOTORS][3];

bool Ragdoll::s_bStaticsInited = false;

//int Ragdoll::m_iLinked[9] = { -1,-1,1,-1,3,-1,5,-1,7};
int Ragdoll::m_iLinked[9] = { -1,-1,-1,-1,-1,-1,-1,-1,-1};

double Ragdoll::GetAMotorMaxForce(int iMotorCount, int iAxis)
{
	if (!s_bStaticsInited)
		__asm int 3;

	return s_amotors_max_forces[iMotorCount][iAxis]*0.6;
}



void Ragdoll::InitStatics()
{
	//midspine
	s_amotors_max_forces[0][0] = 0.0;
	s_amotors_max_forces[0][1] = 100.0;
	s_amotors_max_forces[0][2] = 0.0;
	

	//hips
	s_amotors_max_forces[1][0] = s_amotors_max_forces[1][1] = 100.0;
	s_amotors_max_forces[1][2] = 0.0;
	s_amotors_max_forces[2][0] = s_amotors_max_forces[2][1] = 100.0;
	s_amotors_max_forces[2][2] = 0.0;

	//knees
	s_amotors_max_forces[3][0] = 80.0;
	s_amotors_max_forces[3][1] = 80.0;
	s_amotors_max_forces[3][2] = 0.0;
		
	s_amotors_max_forces[4][0] = 80.0;
	s_amotors_max_forces[4][1] = 80.0;
	s_amotors_max_forces[4][2] = 0.0;
	

	//ankles
	s_amotors_max_forces[5][0] = 20.0;
	s_amotors_max_forces[5][1] = 20.0;
	s_amotors_max_forces[5][2] = 0.0;
 
	s_amotors_max_forces[6][0] = 20.0;
	s_amotors_max_forces[6][1] = 20.0;
	s_amotors_max_forces[6][2] = 0.0; //remember that i moved all 1.0 into 0.0

	//shoulders
	s_amotors_max_forces[7][0] = s_amotors_max_forces[7][1] = 20.0; 
	s_amotors_max_forces[7][2] = 10.0;
	s_amotors_max_forces[8][0] = s_amotors_max_forces[8][1] = 20.0;
	s_amotors_max_forces[8][2] = 10.0; 
	
	for (int i=0;i<9;i++)
	{ 
		for (int j=0;j<3;j++)
		{
			s_amotors_max_forces[i][j] /= 7.5;
		}
	}
	

	s_bStaticsInited = true;
}

Vec3d Ragdoll::getBodyPosition(unsigned int iBodyIndex)
{
	// head pos
	Vec3d pos;
	//memcpy(pos.data,dGeomGetPosition(m_Geomtries[3].geom),sizeof(Vec3d));

	assert(iBodyIndex >= 0);
	assert(iBodyIndex < m_Geomtries.size());

	memcpy(pos.data,dGeomGetPosition(m_Geomtries[iBodyIndex].geom),sizeof(Vec3d));

	return pos;
}

Vec3d Ragdoll::getPosition()
{	
	//Average of all bodies position
	Vec3d pos;
	int iCount=0;
	for (vector<tGeom>::iterator it =  m_Geomtries.begin(); it!=m_Geomtries.end(); ++it)
	{
		memcpy(pos.data,dGeomGetPosition(it->geom),sizeof(Vec3d));
		iCount++;
	}

	div3(pos,1.0 / double(iCount));
	return pos;
}

dJointID Ragdoll::addJoint_Fixed(dBodyID body1, dBodyID body2)
{
	dJointID fixed = dJointCreateFixed (m_World,0);
	dJointAttach (fixed , body1, body2);
	dJointSetFixed (fixed );
	m_Joints[m_uiJoints++] = fixed;	
	return fixed;
}

double Ragdoll::getAMotorVelocityMax(int iMotorIndex,int iAxisIndex)
{
	if ((unsigned int)iMotorIndex >= m_uiJoints_AMotor)
	{
		__asm int 3;
		return 0.0;

	}
	
	if (iAxisIndex == 0)
		return dJointGetAMotorParam(m_Joints_AMotor[iMotorIndex],dParamFMax);


	if (iAxisIndex == 1)
		return dJointGetAMotorParam(m_Joints_AMotor[iMotorIndex],dParamFMax2);

	if (iAxisIndex == 2)
		return dJointGetAMotorParam(m_Joints_AMotor[iMotorIndex],dParamFMax3);


	__asm int 3;
	return 0.0;
}

dJointID Ragdoll::addJoint_AMotor(
		dBodyID body1, dBodyID body2,
		dReal loStop1, dReal hiStop1,
		dReal loStop2, dReal hiStop2,
		dReal loStop3, dReal hiStop3,
		double forceMax = 120.0,
		double forceMax2 = 120.0,
		double forceMax3 = 120.0
		)
{

	//char temp[100];
	//sprintf(temp,"AMotor 0x%8X <-> 0x%8X\n",body1,body2);
	//ODS(temp);

	//forceMax-=1.0;

	dJointID joint = dJointCreateAMotor (m_World,0);
	//debug
	dJointAttach (joint , body1, body2);
	
	dJointSetAMotorNumAxes (joint,3);
    dJointSetAMotorAxis (joint,0,1, 0,0,1);
	dJointSetAMotorAxis (joint,2,2, 0,1,0);
    dJointSetAMotorMode (joint,dAMotorEuler);

	dJointSetAMotorParam(joint,dParamFMax,forceMax);
	dJointSetAMotorParam(joint,dParamFMax2,forceMax2);
	dJointSetAMotorParam(joint,dParamFMax3,forceMax3);

	//stops
	dJointSetAMotorParam(joint, dParamLoStop, loStop1);
	dJointSetAMotorParam(joint, dParamHiStop, hiStop1 );

	dJointSetAMotorParam(joint, dParamLoStop2, loStop2);
	dJointSetAMotorParam(joint, dParamHiStop2, hiStop2);

	dJointSetAMotorParam(joint, dParamLoStop3, loStop3);
	dJointSetAMotorParam(joint, dParamHiStop3, hiStop3);

	/*dJointSetAMotorParam(joint, dParamLoStop, -PI/2.0);
	dJointSetAMotorParam(joint, dParamHiStop, PI/2.0 );

	dJointSetAMotorParam(joint, dParamLoStop2, -PI/2.0);
	dJointSetAMotorParam(joint, dParamHiStop2, PI/2.0);

	dJointSetAMotorParam(joint, dParamLoStop3, -PI/2.0);
	dJointSetAMotorParam(joint, dParamHiStop3, PI/2.0);*/

	m_Joints_AMotor[m_uiJoints_AMotor] = joint;	

	dJointSetFeedback(m_Joints_AMotor[m_uiJoints_AMotor],&m_Joints_Feedback_Info_AMotor[m_uiJoints_AMotor]);

	m_uiJoints_AMotor++;

	return joint;
}

 dHingeJoint* Ragdoll::addJoint_Hinge(dBodyID body1,dBodyID body2, Vec3d& anchor, Vec3d& axis, double loStop = dInfinity,
       double  hiStop = dInfinity)
 {

    Vec3d anc = add3(anchor, m_Offset);
	
    dHingeJoint* joint = new dHingeJoint(m_World);
    joint->attach(body1, body2);
    joint->setAnchor(anc.data);
    joint->setAxis(axis.data);
    joint->setParam(dParamLoStop, loStop);
    joint->setParam(dParamHiStop, hiStop);

	m_Joints_Hinge[m_uiJoints_Hinge] = joint;

	dJointSetFeedback(m_Joints_Hinge[m_uiJoints_Hinge]->id(),&m_Joints_Feedback_Info_Hinge[m_uiJoints_Hinge]);

	m_uiJoints_Hinge++;
	return joint;

/*	dJointID joint = dJointCreateHinge(m_World,0);
	dJointAttach(joint,body1,body2);
	//dJointSetHingeAnchor(joint,anchor.x,anchor.y,anchor.z);
	//dJointSetHingeAxis(joint,axis.x,axis.y,axis.z);
	//dJointSetHingeParam(joint,dParamLoStop,loStop);
	//dJointSetHingeParam(joint,dParamHiStop,hiStop); 
	//dJointSetHingeAnchorDelta()
	
	m_Joints[m_uiJoints] = joint;

	return joint;*/
 }


dBallJoint* Ragdoll::addJoint_Ball(
								   dBodyID body1, 
								   dBodyID body2, 
								   Vec3d& anchor
								   )
{
	anchor = add3(anchor, m_Offset);

	// create the joint
	dBallJoint* joint = new dBallJoint(m_World);
	joint->attach(body1, body2);
	joint->setAnchor(anchor.data);

	m_Joints_Ball[m_uiJoints_Ball] = joint;

	
	dJointSetFeedback(m_Joints_Ball[m_uiJoints_Ball]->id(),&m_Joints_Feedback_Info_Ball[m_uiJoints_Ball]);

	//after discovering Angular Motor (AMotor) in ODE, i found it better. 
	//keeping the next code commented just for future reference.
	
	// store the base orientation of the joint in the local coordinate system
	//   of the primary body (because baseAxis and baseTwistUp may not be
	//   orthogonal, the nearest vector to baseTwistUp but orthogonal to
	//   baseAxis is calculated and stored with the joint)

/*	tBallJointPrivateData* priv = new tBallJointPrivateData();
	Mat3x3 r;
	memcpy(r.data,dBodyGetRotation(body1),sizeof(Mat3x3));
	priv->baseAxis = getBodyRelVec(r, baseAxis);
	Vec3d tempTwistUp = getBodyRelVec(r, baseTwistUp);
	Vec3d baseSide = norm3(cross(tempTwistUp, priv->baseAxis));
	priv->baseTwistUp = norm3(cross(priv->baseAxis, baseSide));


	memcpy(r.data,dBodyGetRotation(body2),sizeof(Mat3x3));
	// store the base twist up vector (original version) in the local
	//   coordinate system of the secondary body
	priv->baseTwistUp2 = getBodyRelVec(r, baseTwistUp);

	// store joint rotation limits and resistive force factors
	priv->flexLimit = flexLimit;
	priv->twistLimit = twistLimit;
	priv->flexForce = flexForce;
	priv->twistForce = twistForce;

	joint->setData(priv);*/
	
	m_uiJoints_Ball++;
	return joint;
}

dUniversalJoint* Ragdoll::addJoint_Universal(
								   dBodyID body1, dBodyID body2, 
								   Vec3d& anchor, Vec3d& axis1, Vec3d& axis2,
								   double loStop1 = -dInfinity, double hiStop1 = dInfinity,
								   double loStop2 = -dInfinity, double hiStop2 = dInfinity)
{
	Vec3d anc = add3(anchor, m_Offset);

	dUniversalJoint* joint = new dUniversalJoint(m_World);
	joint->attach(body1, body2);
	joint->setAnchor(anc.data);
	joint->setAxis1(axis1.data);
	joint->setAxis2(axis2.data);
	joint->setParam(dParamLoStop, loStop1);
	joint->setParam(dParamHiStop, hiStop1);
	joint->setParam(dParamLoStop2, loStop2);
	joint->setParam(dParamHiStop2, hiStop2);

	/*joint.style = "univ"
	self.joints.append(joint)*/

	return joint;
}

//0 midspine_motor
//1 right hip
//2 left hip
//3 right knee
//4 left knee
//5 right ankle
//6 left ankle
//7 right shoulder
//8 left shoulder

void Ragdoll::GetMotorName(unsigned int motor, char* pFillMe, int iFillMeSize)
{
	if (motor > 8 || iFillMeSize < 20)
	{
		sprintf(pFillMe,"error");
		return;
	}

	switch(motor)
	{
		//0 midspine_motor
		//1 right hip
		//2 left hip
		//3 right knee
		//4 left knee
		//5 right ankle
		//6 left ankle
		//7 right shoulder
		//8 left shoulder
		case 0:
		sprintf(pFillMe,"mid_spine");
		break;

		case 1:
		sprintf(pFillMe,"right_hip");
		break;

		case 2:
		sprintf(pFillMe,"left_hip");
		break;

		case 3:
		sprintf(pFillMe,"right_knee");
		break;

		case 4:
		sprintf(pFillMe,"left_knee");
		break;

		case 5:
		sprintf(pFillMe,"right_ankle");
		break;

		case 6:
		sprintf(pFillMe,"left_ankle");
		break;

		case 7:
		sprintf(pFillMe,"right_shoulder");
		break;

		case 8:
		sprintf(pFillMe,"left_shoulder");
		break;

		default:
		DebugBreak(); // should never get here.
		break;
	};


}

Ragdoll::Ragdoll(dWorldID world,dSpaceID space, double density, Vec3d& offset,Renderer* pRenderer)
{
	memset(m_Joints,0,sizeof(m_Joints));
	m_World = world;
	m_Space = space;
	m_Density = density;
	m_TotalMass = 0.0;
	m_Offset = offset;
	m_uiJoints = 0;
	m_uiJoints_Hinge = 0;
	m_uiJoints_Ball = 0;
	m_uiJoints_AMotor = 0;
	m_uiActiveMotor = 0;

	m_pRenderer = pRenderer;

	dBodyID chest = addBody_Capsule(
		VEC(-CHEST_W * 0.5, CHEST_H,0.0),
		VEC(CHEST_W * 0.5, CHEST_H,0.0), 
		0.10);

	//char temp[100];
	//sprintf(temp,"chest = 0x%8X\n",chest);
	//ODS(temp);

	dBodyID belly = addBody_Capsule(
		VEC(0.0, CHEST_H - 0.1, 0.0),
		VEC(0.0, HIP_H + 0.1, 0.0), 
		0.11);

	//sprintf(temp,"belly = 0x%8X\n",belly);
	//ODS(temp);

	dBallJoint* midspine = addJoint_Ball(chest,belly,BACK_POS);
	
	dJointID midspine_motor = addJoint_AMotor(chest,belly,
		0.0,0.0,
		0.0,GW_PI*0.2,
		0.0,0.0,
		s_amotors_max_forces[m_uiJoints_AMotor][0],s_amotors_max_forces[m_uiJoints_AMotor][1],s_amotors_max_forces[m_uiJoints_AMotor][2]
	);

	dBodyID pelvis = addBody_Capsule(
		VEC(-PELVIS_W * 0.5,HIP_H,0.0),
		VEC(PELVIS_W * 0.5, HIP_H,0.0), 
		0.125);

	//sprintf(temp,"pelvis = 0x%8X\n",pelvis);
	//ODS(temp);

	dJointID lowspine = addJoint_Fixed(belly,pelvis);
	
	dBodyID head = addBody_Capsule(
		VEC(0.0, BROW_H,0.0 ), 
		VEC(0.0, MOUTH_H,0.0 ), 
		0.11);

	//sprintf(temp,"head = 0x%8X\n",head);
	//ODS(temp);

	dJointID neck = addJoint_Fixed(chest,head);
	
	dBodyID rightUpperLeg = addBody_Capsule(
		R_HIP_POS, 
		R_KNEE_POS, 
		0.11);


	dBallJoint* rightHip = addJoint_Ball(
		pelvis,rightUpperLeg,R_HIP_POS);//recheck

	dJointID rightHip_motor = addJoint_AMotor(pelvis,rightUpperLeg,
		-GW_PI*0.15,GW_PI*0.15,
		-GW_PI*0.3,GW_PI*0.3,
		0.0,0.0,
		s_amotors_max_forces[m_uiJoints_AMotor][0],s_amotors_max_forces[m_uiJoints_AMotor][1],s_amotors_max_forces[m_uiJoints_AMotor][2]
		);

	dBodyID leftUpperLeg = addBody_Capsule(
		L_HIP_POS, 
		L_KNEE_POS, 
		0.11);

	//sprintf(temp,"leftUpperLeg = 0x%8X\n",leftUpperLeg);
	//ODS(temp);

	//dJointID leftHip = addJoint_Fixed(pelvis,leftUpperLeg);

	dBallJoint* leftHip = addJoint_Ball(
		pelvis,leftUpperLeg,L_HIP_POS);//recheck

	dJointID leftHip_motor = addJoint_AMotor(pelvis,leftUpperLeg,
		-GW_PI*0.15,GW_PI*0.15,
		-GW_PI*0.3,GW_PI*0.3,
		0.0,0.0,
		s_amotors_max_forces[m_uiJoints_AMotor][0],s_amotors_max_forces[m_uiJoints_AMotor][1],s_amotors_max_forces[m_uiJoints_AMotor][2]
		);


	dBodyID rightLowerLeg = addBody_Capsule(
		R_KNEE_POS, 
		R_ANKLE_POS, 
		0.09);

	//sprintf(temp,"rightLowerLeg = 0x%8X\n",rightLowerLeg);
	//ODS(temp);

	dBallJoint* rightKnee = addJoint_Ball(rightUpperLeg, rightLowerLeg, R_KNEE_POS );
	
	dJointID rightKnee_motor = addJoint_AMotor(rightUpperLeg,rightLowerLeg,
		-GW_PI*0.05,GW_PI*0.05,
		-GW_PI*0.4,0.0,
		0.0,0.0,
		s_amotors_max_forces[m_uiJoints_AMotor][0],s_amotors_max_forces[m_uiJoints_AMotor][1],s_amotors_max_forces[m_uiJoints_AMotor][2]
		);
	
	dBodyID leftLowerLeg = addBody_Capsule(
		L_KNEE_POS, 
		L_ANKLE_POS, 
		0.09);

	//sprintf(temp,"leftLowerLeg = 0x%8X\n",leftLowerLeg);
	//ODS(temp);

	dBallJoint* leftKnee = addJoint_Ball(leftUpperLeg, leftLowerLeg, L_KNEE_POS );

	dJointID leftKnee_motor = addJoint_AMotor(leftUpperLeg,leftLowerLeg,
		-GW_PI*0.05,GW_PI*0.05,
		-GW_PI*0.4,0.0,
		0.0,0.0,
		s_amotors_max_forces[m_uiJoints_AMotor][0],s_amotors_max_forces[m_uiJoints_AMotor][1],s_amotors_max_forces[m_uiJoints_AMotor][2]
		);

	double dBoxSize = 0.18;
	double dBoxSize2 = 0.12;
	double dBoxSize3 = 0.35;

	dBodyID rightFoot = addBody_Box(
		mul3(add3(add3(R_TOES_POS,R_HEEL_POS),VEC(0.0,0.0,-0.2)),0.5),
		VEC(dBoxSize,dBoxSize2,dBoxSize3));

	//sprintf(temp,"rightFoot = 0x%8X\n",rightFoot);
	//ODS(temp);

	dBallJoint* rightAnkle = addJoint_Ball(rightLowerLeg, rightFoot, R_ANKLE_POS);
	dJointID rightAnkle_motor = addJoint_AMotor(rightLowerLeg,rightFoot,
		-GW_PI*0.02,GW_PI*0.02,
		-GW_PI*0.11,GW_PI*0.11,
		0.0,0.0,
		s_amotors_max_forces[m_uiJoints_AMotor][0],s_amotors_max_forces[m_uiJoints_AMotor][1],s_amotors_max_forces[m_uiJoints_AMotor][2]
		);

	dBodyID leftFoot = addBody_Box(
		//L_TOES_POS,
		mul3(add3(add3(L_TOES_POS,L_HEEL_POS),VEC(0.0,0.0,-0.2)),0.5),
		VEC(dBoxSize,dBoxSize2,dBoxSize3));

	//sprintf(temp,"leftFoot = 0x%8X\n",leftFoot);
	//ODS(temp);

	dBallJoint* leftAnkle = addJoint_Ball(leftLowerLeg, leftFoot, L_ANKLE_POS);
	dJointID leftAnkle_motor = addJoint_AMotor(leftLowerLeg,leftFoot,
		-GW_PI*0.02,GW_PI*0.02,
		-GW_PI*0.11,GW_PI*0.11,
		0.0,0.0,
		s_amotors_max_forces[m_uiJoints_AMotor][0],s_amotors_max_forces[m_uiJoints_AMotor][1],s_amotors_max_forces[m_uiJoints_AMotor][2]
		);

	dBodyID rightUpperArm = addBody_Capsule(
		R_SHOULDER_POS, 
		R_ELBOW_POS, 
		0.06);

	//sprintf(temp,"rightUpperArm = 0x%8X\n",rightUpperArm);
	//ODS(temp);

	dBallJoint* rightShoulder = addJoint_Ball(
		chest, rightUpperArm,R_SHOULDER_POS);

	dJointID rightShoulder_motor = addJoint_AMotor(chest,rightUpperArm,
		-GW_PI*0.15,GW_PI*0.15,
		-GW_PI*0.15,GW_PI*0.15,
		-GW_PI*0.5,GW_PI*0.5,
		s_amotors_max_forces[m_uiJoints_AMotor][0],s_amotors_max_forces[m_uiJoints_AMotor][1],s_amotors_max_forces[m_uiJoints_AMotor][2]
		);

	dBodyID leftUpperArm = addBody_Capsule(
		L_SHOULDER_POS, 
		L_ELBOW_POS, 
		0.06);

	//sprintf(temp,"leftUpperArm = 0x%8X\n",leftUpperArm);
	//ODS(temp);

	dBallJoint* leftShoulder = addJoint_Ball(
		chest, leftUpperArm,L_SHOULDER_POS);

	dJointID leftShoulder_motor = addJoint_AMotor(chest,leftUpperArm,
		-GW_PI*0.15,GW_PI*0.15,
		-GW_PI*0.15,GW_PI*0.15,
		-GW_PI*0.5,GW_PI*0.5,
		s_amotors_max_forces[m_uiJoints_AMotor][0],s_amotors_max_forces[m_uiJoints_AMotor][1],s_amotors_max_forces[m_uiJoints_AMotor][2]
		);

	dBodyID rightForeArm = addBody_Capsule(
		R_ELBOW_POS, 
		R_WRIST_POS, 
		0.06);

	//sprintf(temp,"rightForeArm = 0x%8X\n",rightForeArm);
	//ODS(temp);
	
	dHingeJoint* rightElbow = addJoint_Hinge(rightUpperArm, rightForeArm,R_ELBOW_POS, downAxis, 0.0, 0.6 * GW_PI);
		
	dBodyID leftForeArm = addBody_Capsule(
		L_ELBOW_POS, 
		L_WRIST_POS, 
		0.06);

	//sprintf(temp,"leftForeArm = 0x%8X\n",leftForeArm);
	//ODS(temp);

	dHingeJoint* leftElbow = addJoint_Hinge(leftUpperArm, leftForeArm,L_ELBOW_POS, upAxis, 0.0, 0.6 * GW_PI);
	
	
	dBodyID rightHand = addBody_Capsule(
		R_WRIST_POS, 
		R_FINGERS_POS, 
		0.06);

	//sprintf(temp,"rightHand = 0x%8X\n",rightHand);
	//ODS(temp);

	dJointID rightWrist = addJoint_Fixed(rightForeArm,rightHand);      

	dBodyID leftHand = addBody_Capsule(
		L_WRIST_POS, 
		L_FINGERS_POS, 
		0.06);

	//sprintf(temp,"leftHand = 0x%8X\n",leftHand);
	//ODS(temp);

	dJointID leftWrist = addJoint_Fixed(leftForeArm,leftHand);

	assert(MOTORS_NUM>=m_uiJoints_AMotor);

	//UpdateCurrentAngularMotors();
}

void Ragdoll::FillBodyState(IN tBodyState& bodyState)
{
	// update angular motors angles
	for (unsigned int i=0;i<m_uiJoints_AMotor;i++)
	{
		for (int a=0;a<AXIS_NUM;a++)
		{
			bodyState.dAngularMotorsAngles[i][a] = dJointGetAMotorAngle(m_Joints_AMotor[i],a);
		}
	}

	int iBodiesNum = m_Bodies.size();
	
	for (int i=0;i<iBodiesNum;i++)
	{
		bodyState.dBodiesLinearVel[i]	= *dBodyGetLinearVel (m_Bodies[i]);
		bodyState.dBodiesAngularVel[i]	= *dBodyGetAngularVel (m_Bodies[i]);
	}
	

}

/*void Ragdoll::UpdateCurrentAngularMotors()
{
	for (unsigned int i=0;i<m_uiJoints_AMotor;i++)
	{
		for (int a=0;a<AXIS_NUM;a++)
		{
			m_dCurrentAngularMotorsAngles[i][a] = dJointGetAMotorAngle(m_Joints_AMotor[i],a);
		}

	}
}*/


Ragdoll::~Ragdoll()
{
	for (vector<dBodyID>::iterator it = m_Bodies.begin(); it!= m_Bodies.end(); ++it)
	{
		dBodyDestroy(*it);
	}


	for (vector<tGeom>::iterator it =  m_Geomtries.begin(); it!=m_Geomtries.end(); ++it)
	{
		dGeomDestroy(it->geom);
	}

	for (unsigned int i=0;i<m_uiJoints;i++)
	{
		dJointDestroy(m_Joints[i]);
	}

	for (unsigned int i=0;i<m_uiJoints_Hinge;i++)
	{
		delete m_Joints_Hinge[i];
	}

	for (unsigned int i=0;i<m_uiJoints_Ball;i++)
	{
		delete m_Joints_Ball[i];
	}

	for (unsigned int i=0;i<m_uiJoints_AMotor;i++)
	{
		dJointDestroy(m_Joints_AMotor[i]);
	}
}

void Ragdoll::update()
{

}

void Ragdoll::increaseCurrentActiveMotor()
{
	if (m_uiActiveMotor < m_uiJoints_AMotor-1)
		m_uiActiveMotor++;
	
}

void Ragdoll::decreaseCurrentActiveMotor()
{
	if (m_uiActiveMotor > 0)
		m_uiActiveMotor--;
	
}

unsigned int Ragdoll::getAMotorsNum() const
{
	return m_uiJoints_AMotor;
}

void Ragdoll::getBodyInformation(unsigned int uiBodyIndex,double& linear_vel, double& angular_vel, double& force, double& torque)
{
	linear_vel = *dBodyGetLinearVel(m_Bodies[uiBodyIndex]);
	angular_vel = *dBodyGetAngularVel(m_Bodies[uiBodyIndex]);
	force = *dBodyGetForce(m_Bodies[uiBodyIndex]);
	torque = *dBodyGetTorque(m_Bodies[uiBodyIndex]);
}


double Ragdoll::getAMotorVelocity(unsigned int uiMotorIndex,unsigned int uiAxis)
{	
	switch (uiAxis)
	{
	case 0:
		return dJointGetAMotorParam(m_Joints_AMotor[uiMotorIndex],dParamVel);
		break;
	case 1:
		return dJointGetAMotorParam(m_Joints_AMotor[uiMotorIndex],dParamVel2);
		break;
	case 2:
		return dJointGetAMotorParam(m_Joints_AMotor[uiMotorIndex],dParamVel3);
		break;		
	default:
		assert(0);
		break;
	}

	assert(0);
	return 0.0;
}

void Ragdoll::setAMotorVelocity(unsigned int uiMotorIndex, unsigned int uiAxis,double dVel)
{
	//dJointSetAMotorParam(m_Joints_AMotor[i],dParamVel,dVel);
	//dJointSetAMotorParam(m_Joints_AMotor[i],dParamVel2,dVel);
	//dJointSetAMotorParam(m_Joints_AMotor[i],dParamVel3,dVel);		

	switch (uiAxis)
	{
	case 0:
		dJointSetAMotorParam(m_Joints_AMotor[uiMotorIndex],dParamVel,dVel);
		break;
	case 1:
		dJointSetAMotorParam(m_Joints_AMotor[uiMotorIndex],dParamVel2,dVel);
		break;
	case 2:
		dJointSetAMotorParam(m_Joints_AMotor[uiMotorIndex],dParamVel3,dVel);
		break;		
	default:
		assert(0);
		break;
	}
	

}

void Ragdoll::draw()
{
	const dReal *pos[2],*R[2];
	dBodyID connected_bodies[2];

	map<dBodyID,Vec3d> body_to_force;
	map<dBodyID,Vec3d>::iterator it_body_to_force;

	Vec3d forces[2]; 
	Vec3d torques[2];

	//call forces caused by joints - Balls joints
	for (int i=0;i<m_uiJoints_Ball;i++)
	{
		connected_bodies[0] = dJointGetBody(m_Joints_Ball[i]->id(),0);
		connected_bodies[1] = dJointGetBody(m_Joints_Ball[i]->id(),1);

		memcpy(&forces[0],&m_Joints_Feedback_Info_Ball[i].f1,sizeof(double)*3);
		memcpy(&forces[1],&m_Joints_Feedback_Info_Ball[i].f2,sizeof(double)*3);
		

		for (int b=0;b<2;b++)
		{
			it_body_to_force = body_to_force.find(connected_bodies[b]);

			if (it_body_to_force == body_to_force.end())
			{
				body_to_force[connected_bodies[b]] = Vec3d(forces[b].x,forces[b].y,forces[b].z);
			} else
			{
				Vec3d total_forces = add3(it_body_to_force->second,Vec3d(forces[b].x,forces[b].y,forces[b].z));
				body_to_force[connected_bodies[b]] = total_forces;				
			}			
		}		
	}

	//call forces caused by joints - Hinge joints
	for (int i=0;i<m_uiJoints_Hinge;i++)
	{
		connected_bodies[0] = dJointGetBody(m_Joints_Hinge[i]->id(),0);
		connected_bodies[1] = dJointGetBody(m_Joints_Hinge[i]->id(),1);

		memcpy(&forces[0],&m_Joints_Feedback_Info_Hinge[i].f1,sizeof(double)*3);
		memcpy(&forces[1],&m_Joints_Feedback_Info_Hinge[i].f2,sizeof(double)*3);
		

		for (int b=0;b<2;b++)
		{
			it_body_to_force = body_to_force.find(connected_bodies[b]);

			if (it_body_to_force == body_to_force.end())
			{
				body_to_force[connected_bodies[b]] = Vec3d(forces[b].x,forces[b].y,forces[b].z);
			} else
			{
				Vec3d total_forces = add3(it_body_to_force->second,Vec3d(forces[b].x,forces[b].y,forces[b].z));
				body_to_force[connected_bodies[b]] = total_forces;				
			}			
		}		
	}

	//call forces caused by joints - AMotor joints (Note: doesn't seem to provide values different than ZERO)
	for (int i=0;i<m_uiJoints_AMotor;i++)
	{
		connected_bodies[0] = dJointGetBody(m_Joints_AMotor[i],0);
		connected_bodies[1] = dJointGetBody(m_Joints_AMotor[i],1);

		memcpy(&forces[0],&m_Joints_Feedback_Info_AMotor[i].f1,sizeof(double)*3);
		memcpy(&forces[1],&m_Joints_Feedback_Info_AMotor[i].f2,sizeof(double)*3);
		

		for (int b=0;b<2;b++)
		{
			it_body_to_force = body_to_force.find(connected_bodies[b]);

			if (it_body_to_force == body_to_force.end())
			{
				body_to_force[connected_bodies[b]] = Vec3d(forces[b].x,forces[b].y,forces[b].z);
			} else
			{
				Vec3d total_forces = add3(it_body_to_force->second,Vec3d(forces[b].x,forces[b].y,forces[b].z));
				body_to_force[connected_bodies[b]] = total_forces;				
			}			
		}		
	}

	

	for (vector<tGeom>::iterator it = m_Geomtries.begin(); it != m_Geomtries.end();++it)
	{
		dGeomID geom = it->geom;

		dBodyID body = dGeomGetBody(geom);

		it_body_to_force = body_to_force.find(body);

		if (it_body_to_force != body_to_force.end())
		{
			double dVal = 0.0;
			Vec3d total_forces = it_body_to_force->second;
			dVal = total_forces.x;

			Vec3d adjusted = total_forces;
			if (adjusted.x < 0)
				adjusted.x=-adjusted.x;
			if (adjusted.y < 0)
				adjusted.y=-adjusted.x;
			if (adjusted.z < 0)
				adjusted.z=-adjusted.x;

			double dDivBy = 100.0;
			double dColor = (adjusted.x + adjusted.y + adjusted.z) / dDivBy;

			glColor3d(dColor,0.0,1.0);
		} else
		{
			glColor3d(0.0,1.0,0.0);
		}

		if (it->type == EGeomCapsule)
		{
			m_pRenderer->drawCapsule(geom);
		} else if (it->type == EGeomBox)
		{
			m_pRenderer->drawBox(geom);
		}	
		
		/*pos = dGeomGetPosition(geom);
		R = dGeomGetRotation (geom);

		m_pRenderer->drawSphere(pos,R,0.1);*/
	}
	
	
	double id[16];
	for (int i=0;i<4;i++)
	{
		for (int j=0;j<4;j++)
		{
			if (i==j)
				id[i*4+j] = 1.0;
			else
				id[i*4+j] = 0.0;

		}
	}
			

	Vec3d desired_pos(0.13425869169216720,0.46229024389157436-0.2,0.074724623009372632+0.2);
	//Vec3d desired_pos(0.13425869169216720,0.46229024389157436,0.074724623009372632);
	m_pRenderer->drawSphere(desired_pos.data,&id[0],0.1);



	//m_pRenderer->drawSphere(pos,1.0);

}

dBodyID Ragdoll::addBody_Box(Vec3d& p1,Vec3d& size)
{
	p1 = add3(p1, m_Offset);

	dBodyID body = dBodyCreate(m_World);

	dMass m;
	m.setBox(m_Density,size.x,size.y,size.z);
	dBodySetMass(body,&m);

	dGeomID geom = dCreateBox(m_Space,size.x,size.y,size.z);

	dBodySetPosition(body,p1.x,p1.y,p1.z);

	dGeomSetBody(geom,body);

	m_Bodies.push_back(body);
	
	tGeom g;
	g.geom = geom;
	g.type = EGeomBox;
	m_Geomtries.push_back(g);

	dMass mass;
	dBodyGetMass(body,&mass);
	m_TotalMass += mass.mass;

	return body;
}

dBodyID Ragdoll::addBody_Capsule(Vec3d& p1,Vec3d&  p2,double radius)
{	
	/*double temp = p1.y;
	p1.y = p1.z;
	p1.z = temp;

	temp = p2.y;
	p2.y = p2.z;
	p2.z = temp;*/

    //Adds a capsule body between joint positions p1 and p2 and with given
    //radius to the ragdoll.

    p1 = add3(p1, m_Offset);
    p2 = add3(p2, m_Offset);

    // cylinder length not including endcaps, make capsules overlap by half
    //   radius at joints
    double cyllen = dist3(p1, p2) - radius;

    //body = ode.Body(self.world)
	dBodyID body = dBodyCreate(m_World);

    dMass m;
	//m.setCappedCylinder(self.density, 3, radius, cyllen); <- original
	m.setCapsule(m_Density,3,radius,cyllen);
	//dMassSetCapsuleTotal(&m,0.2,3, radius,cyllen);

	dBodySetMass(body,&m);

    // set parameters for drawing the body
    /*body.shape = "capsule"
    body.length = cyllen
    body.radius = radius*/

    // create a capsule geom for collision detection
    //geom = ode.GeomCCylinder(self.space, radius, cyllen)
	dGeomID geom = dCreateCapsule(m_Space,radius,cyllen);
    
    // define body rotation automatically from body axis
    Vec3d za = norm3(sub3(p2, p1));
	Vec3d xa;

    double ang_with_x = abs(dot3(za, Vec3d(1.0, 0.0, 0.0)));	

	dMatrix3 d_rot;
	
	if (za == Vec3d(0.0,1.0,0.0))
	{
		//dRFromEulerAngles(d_rot,-GW_PI*0.5,0.0,0.0);
		dRFromEulerAngles(d_rot,0.0,GW_PI*1.5,0.0);
		dBodySetRotation(body,d_rot);
	}
	else if (ang_with_x < 0.7) 
	{
		//xa = Vec3d(1.0, 0.0, 0.0);	
		/*dRFromEulerAngles(d_rot,GW_PI*0.5,0.0,0.0);
		dBodySetRotation(body,d_rot);*/
	}
    else 
	{
		//xa = Vec3d(0.0, 1.0, 0.0);
		dRFromEulerAngles(d_rot,0.0,GW_PI*0.5,0.0);
		dBodySetRotation(body,d_rot);
	}

    /*Vec3d ya = cross(za, xa);
    xa = norm3(cross(ya, za));
    ya = cross(za, xa);

	dMatrix3 d_rot;
    Mat3x3 rot(xa(0), ya(0), za(0), xa(1), ya(1), za(1), xa(2), ya(2), za(2));
	//rot = invert3x3(rot);
	memcpy(d_rot,rot.data,sizeof(dMatrix3));
	*/

	Vec3d pos = mul3(add3(p1,p2), 0.5);
	dBodySetPosition(body,pos.x,pos.y,pos.z);

	
	//m_GeomToBody

	dGeomSetBody(geom,body);

	m_Bodies.push_back(body);
	tGeom g;
	g.geom = geom;
	g.type = EGeomCapsule;
	m_Geomtries.push_back(g);
 
	dMass mass;
	dBodyGetMass(body,&mass);
	m_TotalMass += mass.mass;

    return body;
}