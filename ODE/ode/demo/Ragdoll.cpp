// Based on Python Code made by Matt Heinzen
#include "Ragdoll.h"
#include <assert.h>

//#define CHEST_W 0.36
//#define CHEST_H 1.35

#include <drawstuff/drawstuff.h>

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#endif

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
#define upAxis		Vec3d (0.0, 0.0, 1.0) // changed
#define downAxis	Vec3d (0.0, 0.0, -1.0)
#define bkwdAxis	Vec3d (0.0, 1.0, 0.0)
#define fwdAxis		Vec3d (0.0, -1.0, 0.0)

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

#define R_SHOULDER_POS  Vec3d(-SHOULDER_W * 0.5, 0.0, SHOULDER_H)
#define L_SHOULDER_POS  Vec3d(SHOULDER_W * 0.5, 0.0, SHOULDER_H)
#define R_ELBOW_POS  sub3(R_SHOULDER_POS, Vec3d(UPPER_ARM_LEN, 0.0, 0.0))
#define L_ELBOW_POS  add3(L_SHOULDER_POS, Vec3d(UPPER_ARM_LEN, 0.0, 0.0))
#define R_WRIST_POS  sub3(R_ELBOW_POS, Vec3d(FORE_ARM_LEN, 0.0, 0.0))
#define L_WRIST_POS  add3(L_ELBOW_POS, Vec3d(FORE_ARM_LEN, 0.0, 0.0))
#define R_FINGERS_POS  sub3(R_WRIST_POS, Vec3d(HAND_LEN, 0.0, 0.0))
#define L_FINGERS_POS  add3(L_WRIST_POS, Vec3d(HAND_LEN, 0.0, 0.0))

#define R_HIP_POS  Vec3d(-LEG_W * 0.5, 0.0, HIP_H)
#define L_HIP_POS  Vec3d(LEG_W * 0.5,0.0,  HIP_H)
#define R_KNEE_POS  Vec3d(-LEG_W * 0.5, 0.0, KNEE_H) 
#define L_KNEE_POS  Vec3d(LEG_W * 0.5, 0.0, KNEE_H)
#define R_ANKLE_POS  Vec3d(-LEG_W * 0.5, 0.0, ANKLE_H)
#define L_ANKLE_POS  Vec3d(LEG_W * 0.5, 0.0, ANKLE_H)
#define R_HEEL_POS  sub3(R_ANKLE_POS, Vec3d(0.0,HEEL_LEN, 0.0 ))
#define L_HEEL_POS  sub3(L_ANKLE_POS, Vec3d(0.0,HEEL_LEN ,0.0 ))
#define R_TOES_POS  add3(R_ANKLE_POS, Vec3d(0.0,FOOT_LEN ,0.0 ))
#define L_TOES_POS  add3(L_ANKLE_POS, Vec3d(0.0,FOOT_LEN, 0.0 ))

//added
#define BACK_POS  Vec3d(0,0,CHEST_H)

dJointID Ragdoll::addJoint_Fixed(dBodyID body1, dBodyID body2)
{
	dJointID fixed = dJointCreateFixed (m_World,0);
	dJointAttach (fixed , body1, body2);
	dJointSetFixed (fixed );
	m_Joints[m_uiJoints++] = fixed;	
	return fixed;
}

dJointID Ragdoll::addJoint_AMotor(
		dBodyID body1, dBodyID body2,
		dReal loStop1, dReal hiStop1,
		dReal loStop2, dReal hiStop2,
		dReal loStop3, dReal hiStop3
		)
{

	dJointID joint = dJointCreateAMotor (m_World,0);
	dJointAttach (joint , body1, body2);
	
	dJointSetAMotorNumAxes (joint,3);
    dJointSetAMotorAxis (joint,0,1, 0,0,1);
    //dJointSetAMotorAxis (joint,2,2, 1,0,0);
	dJointSetAMotorAxis (joint,2,2, 0,1,0);
    dJointSetAMotorMode (joint,dAMotorEuler);

	dJointSetAMotorParam(joint,dParamFMax,100.0);
	dJointSetAMotorParam(joint,dParamFMax2,100.0);
	dJointSetAMotorParam(joint,dParamFMax3,100.0);

	//stops
	dJointSetAMotorParam(joint, dParamLoStop, loStop1);
	dJointSetAMotorParam(joint, dParamHiStop, hiStop1 );

	dJointSetAMotorParam(joint, dParamLoStop2, loStop2);
	dJointSetAMotorParam(joint, dParamHiStop2, hiStop2);

	dJointSetAMotorParam(joint, dParamLoStop3, loStop3);
	dJointSetAMotorParam(joint, dParamHiStop3, loStop3);

	/*dJointSetAMotorParam(joint, dParamLoStop, 0.0 );
	dJointSetAMotorParam(joint, dParamHiStop, 0.0 );

	dJointSetAMotorParam(joint, dParamLoStop2, - GW_PI*0.25);
	dJointSetAMotorParam(joint, dParamHiStop2, GW_PI*0.25);

	dJointSetAMotorParam(joint, dParamLoStop3, - GW_PI*0.25);
	dJointSetAMotorParam(joint, dParamHiStop3, GW_PI*0.25);*/

	m_Joints_AMotor[m_uiJoints_AMotor++] = joint;	
	return joint;


	/*joint = dJointCreateAMotor (world,0);
    dJointAttach (joint,body[0],body[1]);

    dJointSetAMotorNumAxes (joint,3);
    dJointSetAMotorAxis (joint,0,1, 0,0,1);
    dJointSetAMotorAxis (joint,2,2, 1,0,0);
    dJointSetAMotorMode (joint,dAMotorEuler);*/

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

	m_Joints_Hinge[m_uiJoints_Hinge++] = joint;
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



Ragdoll::Ragdoll(dWorldID world,dSpaceID space, double density, Vec3d& offset)
{
	memset(m_Joints,0,sizeof(m_Joints));
	m_World = world;
	m_Space = space;
	m_Density = density;
	//joints = []
	m_TotalMass = 0.0;
	m_Offset = offset;
	m_uiJoints = 0;
	m_uiJoints_Hinge = 0;
	m_uiJoints_Ball = 0;
	m_uiJoints_AMotor = 0;

	m_uiActiveMotor = 0;

	dBodyID chest = addBody_Capsule(
		Vec3d(-CHEST_W * 0.5, 0.0,CHEST_H),
		Vec3d(CHEST_W * 0.5, 0.0,CHEST_H), 
		0.13);

	dBodyID belly = addBody_Capsule(
		Vec3d(0.0, 0.0,CHEST_H - 0.1),
		Vec3d(0.0, 0.0,HIP_H + 0.1), 
		0.125);

	//dJointID midspine = addJoint_Fixed(chest,belly);	
	//dHingeJoint* midspine = addJoint_Hinge(chest,belly,BACK_POS, leftAxis, -GW_PI * 0.1, GW_PI * 0.1);
	dBallJoint* midspine = addJoint_Ball(chest,belly,BACK_POS);
	
	//dJointID midspine_motor = addJoint_AMotor(chest,belly,- GW_PI*0.25,GW_PI*0.25,- GW_PI*0.25, GW_PI*0.25,- GW_PI*0.25, GW_PI*0.25);
	dJointID midspine_motor = addJoint_AMotor(chest,belly,0.0,0.0,- GW_PI*0.1,GW_PI*0.1,0.0,0.0);

	dBodyID pelvis = addBody_Capsule(
		Vec3d(-PELVIS_W * 0.5,0.0, HIP_H),
		Vec3d(PELVIS_W * 0.5, 0.0,HIP_H), 
		0.125);

	dJointID lowspine = addJoint_Fixed(belly,pelvis);
	
	dBodyID head = addBody_Capsule(
		Vec3d(0.0, 0.0, BROW_H), 
		Vec3d(0.0, 0.0, MOUTH_H), 
		0.11);

	dJointID neck = addJoint_Fixed(chest,head);
	/*dBallJoint* neck = addJoint_Ball(
		chest, 	head,Vec3d(0.0, 0.0, NECK_H));*/
	
	
	dBodyID rightUpperLeg = addBody_Capsule(
		R_HIP_POS, 
		R_KNEE_POS, 
		0.11);

	//dJointID rightHip = addJoint_Fixed(pelvis,rightUpperLeg);
	/*dUniversalJoint* rightHip = addJoint_Universal(
		pelvis, rightUpperLeg,
		R_HIP_POS, bkwdAxis, rightAxis, -0.1 * GW_PI, 0.3 * GW_PI, -0.15 * GW_PI,
		0.75 * GW_PI);*/
	dBallJoint* rightHip = addJoint_Ball(
		pelvis,rightUpperLeg,R_HIP_POS);//recheck

	dJointID rightHip_motor = addJoint_AMotor(pelvis,rightUpperLeg,
		-GW_PI*0.3,GW_PI*0.3,
		-GW_PI*0.3,GW_PI*0.3,
		0.0,0.0
		);

	dBodyID leftUpperLeg = addBody_Capsule(
		L_HIP_POS, 
		L_KNEE_POS, 
		0.11);

	//dJointID leftHip = addJoint_Fixed(pelvis,leftUpperLeg);

	dBallJoint* leftHip = addJoint_Ball(
		pelvis,leftUpperLeg,L_HIP_POS);//recheck

	dJointID leftHip_motor = addJoint_AMotor(pelvis,leftUpperLeg,
		-GW_PI*0.3,GW_PI*0.3,
		-GW_PI*0.3,GW_PI*0.3,
		0.0,0.0
		);


	dBodyID rightLowerLeg = addBody_Capsule(
		R_KNEE_POS, 
		R_ANKLE_POS, 
		0.09);

	//dJointID rightKnee = addJoint_Fixed(rightUpperLeg,rightLowerLeg);
	dBallJoint* rightKnee = addJoint_Ball(rightUpperLeg, rightLowerLeg, R_KNEE_POS );
	
	dJointID rightKnee_motor = addJoint_AMotor(rightUpperLeg,rightLowerLeg,
		0.0,0.0,
		-GW_PI*0.4,0.0,
		0.0,0.0		
		);
	
	dBodyID leftLowerLeg = addBody_Capsule(
		L_KNEE_POS, 
		L_ANKLE_POS, 
		0.09);

	//dJointID leftKnee = addJoint_Fixed(leftUpperLeg,leftLowerLeg);
	dBallJoint* leftKnee = addJoint_Ball(leftUpperLeg, leftLowerLeg, L_KNEE_POS );

	dJointID leftKnee_motor = addJoint_AMotor(leftUpperLeg,leftLowerLeg,
		0.0,0.0,
		-GW_PI*0.4,0.0,
		0.0,0.0
		);

	/*dBodyID rightFoot = addBody_Capsule(
		R_HEEL_POS, 
		R_TOES_POS, 
		0.09);*/

	dBodyID rightFoot = addBody_Box(
		//R_TOES_POS,
		mul3(add3(add3(R_TOES_POS,R_HEEL_POS),Vec3d(0.0,-0.3,0.0)),0.5),
		Vec3d(0.2,0.3,0.2));

	dJointID rightAnkle = addJoint_Fixed(rightLowerLeg,rightFoot);
	//dHingeJoint* rightAnkle = addJoint_Hinge(rightLowerLeg,rightFoot, R_ANKLE_POS, rightAxis, -0.1 * GW_PI, 0.05 * GW_PI);

	/*dBodyID leftFoot = addBody_Capsule(
		L_HEEL_POS, 
		L_TOES_POS, 
		0.09);*/

	dBodyID leftFoot = addBody_Box(
		//L_TOES_POS,
		mul3(add3(add3(L_TOES_POS,L_HEEL_POS),Vec3d(0.0,-0.3,0.0)),0.5),
		Vec3d(0.2,0.3,0.2));

	dJointID leftAnkle = addJoint_Fixed(leftLowerLeg,leftFoot);
	//dHingeJoint* leftAnkle = addJoint_Hinge(leftLowerLeg,leftFoot, L_ANKLE_POS, leftAxis, -0.1 * GW_PI, 0.05 * GW_PI);	

	dBodyID rightUpperArm = addBody_Capsule(
		R_SHOULDER_POS, 
		R_ELBOW_POS, 
		0.08);

	//dJointID rightShoulder = addJoint_Fixed(chest,rightUpperArm);

	dBallJoint* rightShoulder = addJoint_Ball(
		chest, rightUpperArm,R_SHOULDER_POS);

	/*dJointID rightShoulder_motor = addJoint_AMotor(chest,rightUpperArm,
		-GW_PI*0.15,GW_PI*0.15,
		-GW_PI*0.15,GW_PI*0.15,
		0.0,0.0
		);*/

	dBodyID leftUpperArm = addBody_Capsule(
		L_SHOULDER_POS, 
		L_ELBOW_POS, 
		0.08);

	//dJointID leftShoulder = addJoint_Fixed(chest,leftUpperArm);

	dBallJoint* leftShoulder = addJoint_Ball(
		chest, leftUpperArm,L_SHOULDER_POS);

	/*dJointID leftShoulder_motor = addJoint_AMotor(chest,leftUpperArm,
		-GW_PI*0.15,GW_PI*0.15,
		-GW_PI*0.15,GW_PI*0.15,
		0.0,0.0
		);*/

	dBodyID rightForeArm = addBody_Capsule(
		R_ELBOW_POS, 
		R_WRIST_POS, 
		0.075);
	
	//dJointID rightElbow = addJoint_Fixed(rightUpperArm,rightForeArm);
	dHingeJoint* rightElbow = addJoint_Hinge(rightUpperArm, rightForeArm,R_ELBOW_POS, downAxis, 0.0, 0.6 * GW_PI);
	
		
	dBodyID leftForeArm = addBody_Capsule(
		L_ELBOW_POS, 
		L_WRIST_POS, 
		0.075);

	//dJointID lefttElbow = addJoint_Fixed(leftUpperArm,leftForeArm);
	dHingeJoint* leftElbow = addJoint_Hinge(leftUpperArm, leftForeArm,L_ELBOW_POS, upAxis, 0.0, 0.6 * GW_PI);
	
	
	dBodyID rightHand = addBody_Capsule(
		R_WRIST_POS, 
		R_FINGERS_POS, 
		0.075);

	dJointID rightWrist = addJoint_Fixed(rightForeArm,rightHand);
	//dHingeJoint* rightWrist = addJoint_Hinge(rightForeArm,rightHand,R_WRIST_POS, fwdAxis, -0.1 * GW_PI, 0.2 * GW_PI);
        

	dBodyID leftHand = addBody_Capsule(
		L_WRIST_POS, 
		L_FINGERS_POS, 
		0.075);

	dJointID leftWrist = addJoint_Fixed(leftForeArm,leftHand);
	//dHingeJoint* leftWrist = addJoint_Hinge(leftForeArm,leftHand,L_WRIST_POS, bkwdAxis, -0.1 * GW_PI, 0.2 * GW_PI);
	
}

Ragdoll::~Ragdoll()
{

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

void Ragdoll::setMotorVelocity(unsigned int uiAxis,double dVel)
{
	//dJointSetAMotorParam(m_Joints_AMotor[i],dParamVel,dVel);
	//dJointSetAMotorParam(m_Joints_AMotor[i],dParamVel2,dVel);
	//dJointSetAMotorParam(m_Joints_AMotor[i],dParamVel3,dVel);		

	switch (uiAxis)
	{
	case 0:
		dJointSetAMotorParam(m_Joints_AMotor[m_uiActiveMotor],dParamVel,dVel);
		break;
	case 1:
		dJointSetAMotorParam(m_Joints_AMotor[m_uiActiveMotor],dParamVel2,dVel);
		break;
	case 2:
		dJointSetAMotorParam(m_Joints_AMotor[m_uiActiveMotor],dParamVel3,dVel);
		break;
		assert(0);
	default:
		break;
	}
	

}

void Ragdoll::draw()
{
	const dReal *pos,*R;
	dReal radius,length;

	for (vector<tGeom>::iterator it = m_Geomtries.begin(); it != m_Geomtries.end();++it)
	{
		dGeomID geom = it->geom;
		pos = dGeomGetPosition(geom);
		R = dGeomGetRotation (geom);

		switch (it->type)
		{
		case EGeomBox:
			{
				dVector3 sides;
				dGeomBoxGetLengths(it->geom,sides);
				dsDrawBox(pos,R,sides);
			}
			break;
		case EGeomCapsule:
			{
				dGeomCapsuleGetParams(geom,&radius,&length);
				dsDrawCapsule(pos,R,length,radius);
			}
			break;
		default:
			assert(0);
			break;
		}				
	}
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