#include "glut.h"
#define PI 3.1415265359
#define PIdiv180 3.1415265359/180.0

#include "gwMath.h"

/////////////////////////////////
//Note: All angles in degrees  //
/////////////////////////////////

class CCamera
{
private:
	Vec3d Position;
	Vec3d ViewDir;
	Vec3d StrafeDir;
	GLdouble RotatedX, RotatedY, RotatedZ;	

	void RecalcViewDir();
	void RecalcStrafeDir();

	Vec3d RotateVec(Vec3d v);

public:
	CCamera();				//inits the values (Position: (0|0|0) Target: (0|0|-1) )
	void Render ( void );	//executes some glRotates and a glTranslate command
							//Note: You should call glLoadIdentity before using Render
	void Move ( Vec3d Direction );
	void RotateX ( GLdouble Angle );
	void RotateY ( GLdouble Angle );
	void RotateZ ( GLdouble Angle );
	void RotateXYZ ( Vec3d Angles );
	void MoveForwards ( GLdouble Distance );
	void StrafeRight ( GLdouble Distance );

	Vec3d GetPosition();
	Vec3d GetViewDir();
};


Vec3d AddF3dVectors ( Vec3d * u, Vec3d * v);
void AddF3dVectorToVector ( Vec3d * Dst, Vec3d * V2);
