// Based on Python Code made by Matt Heinzen
#ifndef GW_RAGDOLL_H
#define GW_RAGDOLL_H

#include <ode/ode.h>
#include "gwMath.h"
#include <vector>

using namespace std;

enum EGeometryType
{
	EGeomBox = 0,
	EGeomCapsule,
	EGeomMax
};

struct tGeom
{
	dGeomID geom;
	EGeometryType type;

};

struct tBallJointPrivateData
{
	Vec3d baseAxis;
	Vec3d baseTwistUp;
	Vec3d baseTwistUp2;
	double flexLimit;
	double twistLimit;
	double flexForce;
	double twistForce;
};

class Ragdoll
{
public:
	Ragdoll(dWorldID world, dSpaceID space, double density, Vec3d& offset);
	~Ragdoll();

	void update();
	void draw();
	
	vector<dBodyID> m_Bodies;
	vector<tGeom> m_Geomtries;

	void setMotorVelocity(unsigned int uiAxis,double dVel);

	//debugging
	void increaseCurrentActiveMotor();
	void decreaseCurrentActiveMotor();
	
private:

	

	dBodyID addBody_Box(Vec3d& p1,Vec3d& size);
	dBodyID addBody_Capsule(Vec3d& p1,Vec3d&  p2,double radius);

	dJointID	addJoint_Fixed(dBodyID body1, dBodyID body2);
	dHingeJoint*	addJoint_Hinge(dBodyID body1,dBodyID body2, Vec3d& anchor, Vec3d& axis, double loStop,double  hiStop);
	dBallJoint*	addJoint_Ball( 
		dBodyID body1, 
		dBodyID body2, 
		Vec3d& anchor
		);

	dUniversalJoint* addJoint_Universal(
		dBodyID body1, dBodyID body2, 
		Vec3d& anchor, Vec3d& axis1, Vec3d& axis2,
		double loStop1, double hiStop1,
		double loStop2, double hiStop2);

	dJointID addJoint_AMotor(
		dBodyID body1, dBodyID body2,
		dReal loStop1, dReal hiStop1,
		dReal loStop2, dReal hiStop2,
		dReal loStop3, dReal hiStop3
		);


		//def addBallJoint(self, body1, body2, anchor, baseAxis, baseTwistUp,

	
	dWorldID m_World;
	dSpaceID m_Space;
	Vec3d m_Offset;
	double m_Density;
	double m_TotalMass;

	static const unsigned int MAX_JOINTS_NUM = 100;
	unsigned int m_uiJoints;
	dJointID m_Joints[MAX_JOINTS_NUM];

	unsigned int m_uiJoints_Hinge;
	dHingeJoint* m_Joints_Hinge[MAX_JOINTS_NUM];

	unsigned int m_uiJoints_Ball;
	dBallJoint* m_Joints_Ball[MAX_JOINTS_NUM];

	unsigned int m_uiJoints_AMotor;
	dJointID m_Joints_AMotor[MAX_JOINTS_NUM];

	//debugging
	unsigned int m_uiActiveMotor;
};


#endif