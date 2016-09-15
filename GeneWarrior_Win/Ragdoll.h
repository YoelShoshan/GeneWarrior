// Based on Python Code made by Matt Heinzen
#ifndef GW_RAGDOLL_H
#define GW_RAGDOLL_H

#include <ode/ode.h>
#include "gwMath.h"
#include "Renderer.h"
#include <vector>
#include "Defines.h"
#include <map>

using namespace std;




//motors

//0 midspine_motor
//1 right hip
//2 left hip
//3 right knee
//4 left knee
//5 right ankle
//6 left ankle
//7 right shoulder
//8 left shoulder


//bodies


//0 chest
//1 belly
//2 pelvis
//3 head
//4 rightUpperLeg
//5 leftUpperLeg
//6 rightLowerLeg
//7 leftLowerLeg
//8 rightFoot
//9 leftFoot
//10 rightUpperArm
//11 leftUpperArm
//12 rightForeArm
//13 leftForeArm
//14 rightHand
//15 leftHand



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
	Ragdoll(dWorldID world, dSpaceID space, double density, Vec3d& offset,Renderer* pRenderer);
	~Ragdoll();

	// in: motor index [0,8]
	// out: fills 
	static void GetMotorName(unsigned int motor, char* pFillMe, int iFillMeSize);

	void update();
	void draw();
	
	vector<dBodyID> m_Bodies;
	vector<tGeom> m_Geomtries;

	void getBodyInformation(unsigned int uiBodyIndex,double& linear_vel, double& angular_vel, double& force, double& torque);

	void setAMotorVelocity(unsigned int uiMotorIndex,unsigned int uiAxis,double dVel);

	double getAMotorVelocity(unsigned int uiMotorIndex,unsigned int uiAxis);

	unsigned int getAMotorsNum() const;

	//debugging
	void increaseCurrentActiveMotor();
	void decreaseCurrentActiveMotor();

	Vec3d getBodyPosition(unsigned int iBodyIndex); //get specific body position
	
	Vec3d getPosition(); //average of all bodies position
	

	double getAMotorVelocityMax(int iMotorIndex,int iAxisIndex);

	static void InitStatics();

	

	static double GetAMotorMaxForce(int iMotorCount, int iAxis);

	void FillBodyState(IN tBodyState& bodyState);
	
	
	/*struct tMotorLink
	{
		unsigned int source;
		unsigned int linked;
	};

	tMotorLink m_Linked_motors[4];*/

	//TODO: refactor

	static const unsigned int MAX_JOINTS_NUM = 30;

	// -1 means not linked
	static int m_iLinked[ANGULAR_MOTORS_NUM];

	/*void UpdateCurrentAngularMotors();
	double m_dCurrentAngularMotorsAngles[ANGULAR_MOTORS_NUM][AXIS_NUM]; // 3 axis
	*/

private:

	static double s_amotors_max_forces[MAX_MOTORS][AXIS_NUM];
	static bool s_bStaticsInited;
	
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
		dReal loStop3, dReal hiStop3,
		double forceMax,
		double forceMax2,
		double forceMax3
		);


	//map<dGeomID, dBodyID> m_GeomToBody;

	
	dWorldID m_World;
	dSpaceID m_Space;
	Vec3d m_Offset;
	double m_Density;
	double m_TotalMass;

	
	unsigned int m_uiJoints;
	dJointID m_Joints[MAX_JOINTS_NUM];

	unsigned int m_uiJoints_Hinge;
	dHingeJoint* m_Joints_Hinge[MAX_JOINTS_NUM];

	unsigned int m_uiJoints_Ball;
	dBallJoint* m_Joints_Ball[MAX_JOINTS_NUM];

	unsigned int m_uiJoints_AMotor;
	dJointID m_Joints_AMotor[MAX_JOINTS_NUM];
	
	dJointFeedback m_Joints_Feedback_Info_Hinge[MAX_JOINTS_NUM];
	dJointFeedback m_Joints_Feedback_Info_Ball[MAX_JOINTS_NUM];
	dJointFeedback m_Joints_Feedback_Info_AMotor[MAX_JOINTS_NUM];
	

	//rendering
	Renderer* m_pRenderer;

	//debugging
	unsigned int m_uiActiveMotor;
};


#endif