#ifndef GW_RENDERER_H
#define GW_RENDERER_H

#pragma once 

#include "gwMath.h"
#include <ode/ode.h>
#include "Camera.h"
#include "Texture_GL.h"

class Renderer
{
public:
	Renderer(int iWidth, int iheight);
	~Renderer();

	void prepareGL();
	
	void onResize(int iWidth, int iheight);

	void drawCapsule(dGeomID geom);
	void drawCapsule(const double* pos,const double* rot, double length, double radius);
		
	void drawBox(const double pos[3], const double R[12], const double sides[3]);
	void drawBox(dGeomID geom);

	void drawSphere(const double* pos,const double* rot, double radius);

	void drawFloor();

	void strafeLeft();
	void strafeRight();
	void moveForward();
	void moveBackwards();

	void rotateX(int iDeltaX);
	void rotateY(int iDeltaY);
	void rotateZ(int iDeltaZ);

	Vec3d GetCameraPosition();
	Vec3d GetCameraViewDir();

private:

	int m_iWidth;
	int m_iHeight;

	/*Vec3d rotateDirectionWithAngles(Vec3d& dir, double angX, double angY, double angZ);

	void calculateLookAt();
	void calculateLeft();
	void calculateUp();*/

	void setTransformD (const double pos[3], const double R[12]);

	static const unsigned int CAPSULE_SLICES = 16;
	static const unsigned int CAPSULE_STACKS = 12;

	/*Vec3d m_Pos;	
	Vec3d m_LookDir;
	Vec3d m_Left;
	Vec3d m_Up;
	
	double m_dAngX,m_dAngY,m_dAngZ;*/

	double m_dSpeed;
	double m_dRotateSpeed;

	CCamera m_Camera;
	
	CTexture_GL m_FloorTexture;

	int m_iDrawCount;
};



#endif 
