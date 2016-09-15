#include "stdafx.h"
#include "Renderer.h"
#include "glut.h"
#include "math.h"



Renderer::Renderer(int iWidth, int iHeight)
{

	m_iWidth = iWidth;
	m_iHeight = iHeight;

	/*m_Pos = Vec3d(2.53, 0.71, 3.57);
	m_LookDir = Vec3d(0.0,0.0,-1.0);

	m_dAngX = -47.44;
	m_dAngY = -66.64;
	m_dAngZ = 0.0;//100.25;*/
			
	m_dSpeed = 0.1;
	m_dRotateSpeed = 0.1;

	//absolute path - need to change (did that becuase open dialog probably change the "current path")
	m_FloorTexture.Load("blocks18cgeomtrn2.tga",true,false,false);
	//m_FloorTexture.Load("D:/my_svn_zone/GeneWarrior/GeneWarrior_Win/GeneWarrior_Win/blocks18cgeomtrn2.tga",true,false,false);
	
	m_iDrawCount = 0;
}

void  Renderer::onResize(int iWidth, int iHeight)
{
	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (iHeight == 0)
		iHeight = 1;

	float ratio =  iWidth * 1.0 / iHeight;

	// Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);

	// Reset Matrix
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, iWidth, iHeight);

	// Set the correct perspective.
	gluPerspective(45.0f, ratio, 0.1f, 100.0f);

	// Get Back to the Modelview
	glMatrixMode(GL_MODELVIEW);

	m_iWidth = iWidth;
	m_iHeight = iHeight;
}



void Renderer::prepareGL()
{
	//glClearColor(0.8, 0.8, 0.9, 0.0);
	
	glClearColor(0.3, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	glShadeModel(GL_SMOOTH);
	glDisable(GL_CULL_FACE);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective (45.0, float(m_iWidth) / float(m_iHeight), 0.2, 200.0);

	glViewport(0, 0, m_iWidth, m_iHeight);

	glMatrixMode(GL_MODELVIEW);

	GLfloat  params_pos []= {0,0,1,0};
	GLfloat  params_dif []= {1,1,1,1};
	GLfloat  params_spec []= {1,1,1,1};

	glLightfv(GL_LIGHT0,GL_POSITION,params_pos);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,params_dif);
	glLightfv(GL_LIGHT0,GL_SPECULAR,params_spec);
	glEnable(GL_LIGHT0);

	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.8, 0.8, 0.8);

	/*gluLookAt(
		1.5, 4.0, 3.0, 
		0.5, 1.0, 0.0, 
		0.0, 1.0, 0.0);*/

	glLoadIdentity();

	/*gluLookAt(
		1.5, 4.0, 3.0, 
		0.5, 1.0, 0.0, 
		0.0, 1.0, 0.0);*/


	/*gluLookAt(
		m_Pos.x, m_Pos.y, m_Pos.z,
		m_Pos.x+m_LookDir.x, m_Pos.y+m_LookDir.y,m_Pos.z+m_LookDir.z,
		//0.0,1.0,0.0);
		m_Up.x,m_Up.y,m_Up.z);*/

	/*glRotatef(-m_dAngX , 1.0, 0.0, 0.0);
	glRotatef(-m_dAngY , 0.0, 1.0, 0.0);
	glRotatef(-m_dAngZ , 0.0, 0.0, 1.0);
	glTranslatef( -m_Pos.x, -m_Pos.y, -m_Pos.z );*/

	m_Camera.Render();

	//m_dAngZ+=0.1;

	m_iDrawCount = 0;
}

Vec3d Renderer::GetCameraPosition()
{
	return m_Camera.GetPosition();
}

Vec3d Renderer::GetCameraViewDir()
{
	return m_Camera.GetViewDir();
}

void Renderer::rotateX(int iDeltaX)
{
	//m_dAngX+= double(iDeltaX)*m_dRotateSpeed;
	m_Camera.RotateX(double(iDeltaX)*m_dRotateSpeed);
}
void Renderer::rotateY(int iDeltaY)
{
	//m_dAngY+= double(iDeltaY)*m_dRotateSpeed;
	m_Camera.RotateY(double(iDeltaY)*m_dRotateSpeed);
}
void Renderer::rotateZ(int iDeltaZ)
{
	//m_dAngZ+= double(iDeltaZ)*m_dRotateSpeed;
}

void Renderer::strafeLeft()
{
	//m_Pos = sub3(m_Pos,mul3(m_Left,m_dSpeed));	
	m_Camera.StrafeRight(-m_dSpeed);
}
void Renderer::strafeRight()
{
	//m_Pos = add3(m_Pos,mul3(m_Left,m_dSpeed));	
	m_Camera.StrafeRight(m_dSpeed);
}
void Renderer::moveForward()
{
	//m_Pos = add3(m_Pos,mul3(m_LookDir,m_dSpeed));
	m_Camera.MoveForwards(m_dSpeed);
}
void Renderer::moveBackwards()
{
	//m_Pos = sub3(m_Pos,mul3(m_LookDir,m_dSpeed));
	m_Camera.MoveForwards(-m_dSpeed);
}

Renderer::~Renderer()
{

}

void Renderer::setTransformD (const double pos[3], const double R[12])
{
  GLdouble matrix[16];
  matrix[0]=R[0];
  matrix[1]=R[4];
  matrix[2]=R[8];
  matrix[3]=0;
  matrix[4]=R[1];
  matrix[5]=R[5];
  matrix[6]=R[9];
  matrix[7]=0;
  matrix[8]=R[2];
  matrix[9]=R[6];
  matrix[10]=R[10];
  matrix[11]=0;
  matrix[12]=pos[0];
  matrix[13]=pos[1];
  matrix[14]=pos[2];
  matrix[15]=1;
  glPushMatrix();
  glMultMatrixd (matrix);
}

void Renderer::drawFloor()
{
	const float gsize = 100.0f;
	const float offset = 0; // -0.001f; ... polygon offsetting doesn't work well

	float ground_scale = 1.0;
    float ground_ofsx = 0.5;
	float ground_ofsy = 0.5;

	glEnable( GL_TEXTURE_2D );
	m_FloorTexture.Bind(0);

	glColor3f(1.0,1.0,1.0);

	glBegin (GL_QUADS);
	glNormal3f (0,0,1);
	glTexCoord2f (-gsize*ground_scale + ground_ofsx,
		-gsize*ground_scale + ground_ofsy);
	//glColor3f(1.0,0.0,0.0);
	glVertex3f (-gsize,-gsize,offset);
	glTexCoord2f (gsize*ground_scale + ground_ofsx,
	-gsize*ground_scale + ground_ofsy);
	//glColor3f(0.0,1.0,0.0);
	glVertex3f (gsize,-gsize,offset);
	glTexCoord2f (gsize*ground_scale + ground_ofsx,
		gsize*ground_scale + ground_ofsy);
	//glColor3f(0.0,0.0,1.0);
	glVertex3f (gsize,gsize,offset);
	glTexCoord2f (-gsize*ground_scale + ground_ofsx,
		gsize*ground_scale + ground_ofsy); 
	//glColor3f(1.0,1.0,1.0);
	glVertex3f (-gsize,gsize,offset);

	glEnd();

	glDisable( GL_TEXTURE_2D );
}

void Renderer::drawBox(dGeomID geom)
{
	const dReal *pos,*R;
	
	pos = dGeomGetPosition(geom);
	R = dGeomGetRotation (geom);
	dVector3 sides;
	dGeomBoxGetLengths (geom,sides);
	drawBox(pos,R,sides);
}

void Renderer::drawBox(const double pos[3], const double R[12], const double sides[3])
{
  setTransformD (pos,R);

  float lx = sides[0]*0.5f;
  float ly = sides[1]*0.5f;
  float lz = sides[2]*0.5f;

  // sides
  glBegin (GL_TRIANGLE_STRIP);
  glNormal3f (-1,0,0);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,-ly,lz);
  glVertex3f (-lx,ly,-lz);
  glVertex3f (-lx,ly,lz);
  glNormal3f (0,1,0);
  glVertex3f (lx,ly,-lz);
  glVertex3f (lx,ly,lz);
  glNormal3f (1,0,0);
  glVertex3f (lx,-ly,-lz);
  glVertex3f (lx,-ly,lz);
  glNormal3f (0,-1,0);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,-ly,lz);
  glEnd();

  // top face
  glBegin (GL_TRIANGLE_FAN);
  glNormal3f (0,0,1);
  glVertex3f (-lx,-ly,lz);
  glVertex3f (lx,-ly,lz);
  glVertex3f (lx,ly,lz);
  glVertex3f (-lx,ly,lz);
  glEnd();

  // bottom face
  glBegin (GL_TRIANGLE_FAN);
  glNormal3f (0,0,-1);
  glVertex3f (-lx,-ly,-lz);
  glVertex3f (-lx,ly,-lz);
  glVertex3f (lx,ly,-lz);
  glVertex3f (lx,-ly,-lz);
  glEnd();
  
  glPopMatrix();

}

void Renderer::drawCapsule(dGeomID geom)
{
	const dReal *pos,*R;
	dReal radius,length;
	
	pos = dGeomGetPosition(geom);
	R = dGeomGetRotation (geom);
	dGeomCapsuleGetParams(geom,&radius,&length);
	drawCapsule(pos,R,length,radius);
}

void Renderer::drawSphere(const double* pos,const double* rot, double radius)
{
	/*glPushMatrix();
	glLoadIdentity();
	glTranslated(pos[0],pos[1],pos[2]);*/
	
	m_iDrawCount++;
	m_iDrawCount%=4;

	switch (m_iDrawCount)
	{
	case 0:
		glColor3d(1.0,0.0,0.0);
		break;
	case 1:
		glColor3d(0.0,1.0,0.0);
		break;
	case 2:
		glColor3d(0.0,0.0,1.0);
		break;
	case 3:
		glColor3d(1.0,0.0,1.0);
		break;
	};

	setTransformD(pos,rot);

	//glDepthFunc(GL_ALWAYS);
	glutSolidSphere(radius, 16, 16);
	//glDepthFunc(GL_LEQUAL);

	glPopMatrix();
}

void Renderer::drawCapsule(const double* pos,const double* rot, double length, double radius)
{
	
/*	// Reset transformations
	glLoadIdentity();
	// Set the camera
	gluLookAt(	0.0f, 0.0f, 10.0f,
			0.0f, 0.0f,  0.0f,
			0.0f, 1.0f,  0.0f);

	//glRotatef(180.0, 0.0f, 1.0f, 0.0f);*/

	static float z = -4.0;
	z-=0.1f;
	//glTranslated(0.0,0.0,-3.1);

	//glutSolidSphere(10.0,10,10);
	
	

    //Draw an ODE body.

	//Vec3d zero(0.0,0.0,0.0);

    /*Mat4x4 oglMat = makeOpenGLMatrix(rot, pos);	
    glPushMatrix();
	glMultMatrixd(oglMat.data);*/

	setTransformD(pos,rot);

    double cylHalfHeight = length / 2.0;
    glBegin(GL_QUAD_STRIP);
	double angle,ca,sa;
	for ( int i=0;i<CAPSULE_SLICES + 1;i++)
	{
		angle = (double)i / double(CAPSULE_SLICES) * 2.0 * GW_PI;
		ca = cos(angle);
		sa = sin(angle);
		glNormal3f(ca, sa, 0);
		glVertex3f(radius * ca, radius * sa, cylHalfHeight);
		glVertex3f(radius * ca, radius * sa, -cylHalfHeight);
	}
    glEnd();
    glTranslated(0, 0, cylHalfHeight);
    glutSolidSphere(radius, CAPSULE_SLICES, CAPSULE_STACKS);
    glTranslated(0, 0, -2.0 * cylHalfHeight);
    glutSolidSphere(radius, CAPSULE_SLICES, CAPSULE_STACKS);
    glPopMatrix();
	
}



