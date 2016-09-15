#define MAX_MOTORS 100
#define ANGULAR_MOTORS_NUM 9
#define RAGDOLL_BODIES_NUM 16
#define AXIS_NUM 3


enum ESensorType
{
	S_NOT_INITIALIZED=0,
	S_TIME,
	S_ANGULAR_MOTOR_ANGULAR
};

struct tBodyState
{
	/*static const UINT  BODY_PARTS_NUM = 15;
	tBodyPartTransform parts[BODY_PARTS_NUM];*/
	double dAngularMotorsAngles[ANGULAR_MOTORS_NUM][AXIS_NUM];

	double dBodiesLinearVel[RAGDOLL_BODIES_NUM];
	double dBodiesAngularVel[RAGDOLL_BODIES_NUM];

};