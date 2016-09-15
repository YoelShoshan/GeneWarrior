#include "stdafx.h"
#include "camera.h"
#include "math.h"
#include <iostream>
#include "windows.h"

void AddF3dVectorToVector ( Vec3d * Dst, Vec3d * V2)
{
	Dst->x += V2->x;
	Dst->y += V2->y;
	Dst->z += V2->z;
}


/***************************************************************************************/

CCamera::CCamera()
{
	//Init with standard OGL values:
	//Position = Vec3d (0.0,0.0,0.0);
	Position = Vec3d (-4.5,0.0,1.0);
	ViewDir = Vec3d(0.0,0.0,-1.0);
	StrafeDir = Vec3d(-1.0,0.0,0.0);
	//ViewDirChanged = false;
	//Only to be sure:
	RotatedX = RotatedY = RotatedZ = 0.0;
	//RotatedX = 180.0;


}

Vec3d CCamera::RotateVec(Vec3d v)
{
	Vec3d res = v;

	Mat3x3 r;

	r = calcRotMatrix(Vec3d(0.0,0.0,1.0),DEG2RAD(-90));
	res = rotate3(r,res);
	res = norm3(res);

	r = calcRotMatrix(Vec3d(0.0,1.0,0.0),DEG2RAD(-90));
	res = rotate3(r,res);
	res = norm3(res);

	r = calcRotMatrix(Vec3d(0.0,1.0,0.0),DEG2RAD(-RotatedX));
	res = rotate3(r,res);
	res = norm3(res);

	r = calcRotMatrix(Vec3d(0.0,0.0,1.0),DEG2RAD(-RotatedY));
	res = rotate3(r,res);
	res = norm3(res);

	return res;
}

void CCamera::RecalcViewDir( void )
{
	ViewDir = Vec3d(0.0,0.0,-1.0);
	ViewDir = RotateVec(ViewDir);	
}


void CCamera::RecalcStrafeDir()
{
	StrafeDir = Vec3d(-1.0,0.0,0.0);
	StrafeDir = RotateVec(StrafeDir);	
}

void CCamera::Move (Vec3d Direction)
{
	AddF3dVectorToVector(&Position, &Direction );
}

Vec3d CCamera::GetPosition()
{
	return Position;
}

void CCamera::RotateY (GLdouble Angle)
{
	RotatedY += Angle;
	//ViewDirChanged = true;
}

void CCamera::RotateX (GLdouble Angle)
{
	RotatedX += Angle;
	//ViewDirChanged = true;
}

Vec3d CCamera::GetViewDir()
{
	RecalcViewDir();
	return ViewDir;
}

void CCamera::Render( void )
{
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();
	glRotatef (90, 0,0,1);
	glRotatef (90, 0,1,0);

	glRotatef(RotatedX , 0.0, 1.0, 0.0);
	glRotatef(RotatedY , 0.0, 0.0, 1.0);
	glTranslatef( -Position.x, -Position.y, -Position.z );
}

void CCamera::MoveForwards( GLdouble Distance )
{
	//if (ViewDirChanged) 
		RecalcViewDir();
	Vec3d MoveVector = mul3(ViewDir,Distance);
	AddF3dVectorToVector(&Position, &MoveVector );
	if (Position.z < 0.5)
		Position.z = 0.5;
}

void CCamera::StrafeRight ( GLdouble Distance )
{
	//if (ViewDirChanged) 
		RecalcStrafeDir();

	Vec3d MoveVector = mul3(StrafeDir,-Distance);
	AddF3dVectorToVector(&Position, &MoveVector );
	
	/*Vec3d MoveVector;
	MoveVector.z = -ViewDir.x * -Distance;
	MoveVector.y = 0.0;
	MoveVector.x = ViewDir.z * -Distance;
	AddF3dVectorToVector(&Position, &MoveVector );*/

}