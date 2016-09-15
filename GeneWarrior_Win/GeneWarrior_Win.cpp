// GeneWarrior_Win.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
/*#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>*/

#include <assert.h>
#include "GeneWarrior_Win.h"

#include <ode/ode.h>
#include "Ragdoll.h"
//#include "..\ZedTracker\ZedTracker.h" 


#include "window3d.h"
#include <windows.h>

//#include <GL/glut.h>
#include "Renderer.h"

#include "Brain.h"
#include "RandomSinusBrain.h"
#include "GeneticAlgo_Brain.h"
#include "NeuralNetwork_Brain.h"

#include "..\sfmt\sfmt.h"

FILE* g_pLog = NULL;

double g_dFrequency = 0.0;
LARGE_INTEGER g_PreviousTime;

//Vec3d vRagdollStartPos = Vec3d(0.0,0.5,0.0); // original

Vec3d vRagdollStartPos = Vec3d(0.0,0.5,0.0); // original

//Vec3d vRagdollStartPos = Vec3d(0.0,0.5,0.6);

Brain* g_pBrain = NULL;

#include "Input.h"
CInput g_Input;

CWindow3D g_Window;
HDC g_hDC;

bool g_bEvolved = false;
bool g_bPlayMode = false;

bool g_bPlayMode_Started = false;

char g_pcPlayFileName[1024]; 


#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
#ifdef _DEBUG
#define new DEBUG_NEW
#endif

Renderer* g_pRenderer = NULL;

#define SFMT_FLOAT (((float)gen_rand32()/(float)MAXUINT))
#define SFMT_IN_RANGE(start,end) ((SFMT_FLOAT*((end)-(start)))+(start))
#define SFMT_IN_RANGE_UINT(start,end) (((gen_rand32())%((end)-(start)))+start)

//typedef void dMessageFunction (int errnum, const char *msg, va_list ap);

void ErrorCb(int errnum, const char *msg, va_list ap)
{
	printf("Error: %s\n",msg);
}

void DebugCb(int errnum, const char *msg, va_list ap)
{
	printf("Debug: %s\n",msg);
}

void MessageCb(int errnum, const char *msg, va_list ap)
{
	printf("Message: %s\n",msg);
}

Ragdoll* g_pRagdoll = NULL;
//Ragdoll* g_pRagdoll_2 = NULL;

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

//#include "icosahedron_geom.h"

// select correct drawing functions

// some constants

//#define NUM 100			// max number of objects

#define MAX_CONTACTS 8          // maximum number of contact points per body

//#define DENSITY (5.0)		// density of all objects
/*#define GPB 3			// maximum number of geometries per body
#define MAX_FEEDBACKNUM 20
#define GRAVITY         REAL(0.5)
#define USE_GEOM_OFFSET 1*/

// dynamics and collision objects

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom;		// geometries representing this body
};

int num=0;		// number of objects in simulation
int nextobj=0;		// next object to recycle if num==NUM
dWorldID g_world = 0;
dSpaceID space = 0;
MyObject box;
dJointGroupID contactgroup = 0;
int show_aabb = 0;	// show geom AABBs?
int show_contacts = 0;	// show contact points?
int write_world = 0;
int show_body = 0;



float angle = 0.0f;




void processInput()
{
	////////////////////////////
	// Exit
	////////////////////////////

	if (g_Input.KeyPressed(DIK_Q))
	  exit(0);

	////////////////////////////
	// Brain Evolving
	////////////////////////////

	if (g_Input.KeyPressed(DIK_C))
	  g_bEvolved = false;
	

	////////////////////////////
	// Camera movement
	////////////////////////////

	int x = (int) (g_Input.GetRelativeX(false));
	int y = (int) (g_Input.GetRelativeY(false));

	g_pRenderer->rotateX(-y);
	g_pRenderer->rotateY(x);

	if (g_Input.KeyPressed(DIK_W))
	  g_pRenderer->moveForward();
	if (g_Input.KeyPressed(DIK_S))
	  g_pRenderer->moveBackwards();

	if (g_Input.KeyPressed(DIK_A))
	  g_pRenderer->strafeLeft();
	if (g_Input.KeyPressed(DIK_D))
	  g_pRenderer->strafeRight();

	/////////////////////////////
	// Ragdoll control
	/////////////////////////////

	if (g_Input.keyPressed_Once(DIK_LBRACKET))		
	{
		g_pRagdoll->decreaseCurrentActiveMotor();
	}

	if (g_Input.keyPressed_Once(DIK_RBRACKET))		
	{
		g_pRagdoll->increaseCurrentActiveMotor();
	}

	double dVel = 100.0;
	int iCurrMotor = 6;

	/*if (g_Input.KeyPressed(DIK_0))
	  g_pRagdoll->setAMotorVelocity(iCurrMotor,0,dVel);
	if (g_Input.KeyPressed(DIK_1))
	  g_pRagdoll->setAMotorVelocity(iCurrMotor,0,-dVel);

	if (g_Input.KeyPressed(DIK_2))
	  g_pRagdoll->setAMotorVelocity(iCurrMotor,1,dVel);
	if (g_Input.KeyPressed(DIK_3))
	  g_pRagdoll->setAMotorVelocity(iCurrMotor,1,-dVel);

	if (g_Input.KeyPressed(DIK_4))
	  g_pRagdoll->setAMotorVelocity(iCurrMotor,2,dVel);
	if (g_Input.KeyPressed(DIK_5))
	  g_pRagdoll->setAMotorVelocity(iCurrMotor,2,-dVel);*/

}

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
	dBodyID body1 = dGeomGetBody(o1);
	dBodyID body2 = dGeomGetBody(o2);
	
	if (body1 && body2)
	{
		if (dAreConnected(body1, body2))
			return;
	}
	
	dContact contact[MAX_CONTACTS];
	int contacts = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact));
	if (contacts > 0) 
	{
		for (int i=0; i<contacts; i++) 
		{
			/*contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
			dContactSoftERP | dContactSoftCFM | dContactApprox1;
			contact[i].surface.mu = dInfinity;
			contact[i].surface.slip1 = 0.1;
			contact[i].surface.slip2 = 0.1;
			contact[i].surface.soft_erp = 0.5;
			contact[i].surface.soft_cfm = 0.3;
			dJointID c = dJointCreateContact (g_world,contactgroup,&contact[i]);
			dJointAttach (c,
			dGeomGetBody(contact[i].geom.g1),
			dGeomGetBody(contact[i].geom.g2));*/

			contact[i].surface.mode = 0;
			contact[i].surface.mu = dInfinity;//5000.0;//dInfinity;//500.0;
			
			dJointID c = dJointCreateContact (g_world,contactgroup,&contact[i]);
			dJointAttach (
				c,
				dGeomGetBody(contact[i].geom.g1),
				dGeomGetBody(contact[i].geom.g2));

		}
	}
	
    /*// create contact joints
    world, contactgroup = args
    for c in contacts:
        c.setBounce(0.2)
        c.setMu(500) # 0-5 = very slippery, 50-500 = normal, 5000 = very sticky
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())*/

	
}

void simulationStep(double dStep)
{
	
	dSpaceCollide (space,0,&nearCallback);
	{
		//g_pRagdoll->update();
		//g_pRagdoll_2->update();
		dWorldStep(g_world,dStep);
	}
	
	// remove all contact joints
	dJointGroupEmpty (contactgroup);

	/*if (!dBodyIsEnabled(box.body)) 
	{
		//dsSetColor (1,0.0,0);
	}
	else 
	{
		//dsSetColor (1,1,0);
	}*/	

	//glutPostRedisplay();
	
}

void renderScene(void) 
{
	g_pRenderer->prepareGL();

	g_pRenderer->drawFloor();

	assert(g_pRagdoll);
	g_pRagdoll->draw();
	
	/*assert(g_pRagdoll_2);
	g_pRagdoll_2->draw();*/


	g_pRenderer->drawBox(box.geom);

	//glutSwapBuffers();

	SwapBuffers (g_hDC);
}

//#define ODS(buff,

void ApplyBodyMovement(tMovement& movement, bool bDebug)
{
	//debug
	//return;

	assert(g_pRagdoll); 
	char temp[200];

	int iUseIndex=-1;

	double dVel,dVelMax;
	double dMargin = 20.0;

	bool bPrintDebug = bDebug;

	for (int i=0;i<g_pRagdoll->getAMotorsNum();i++)
	{
		Vec3d ang_vel;
		//bool bUseSymmetricSide = false;

		if (bPrintDebug)
		{
			sprintf(temp,"Motor %d - ",i);
			ODS(temp);
		}

		if (i>8)
			continue;		 //??

		//debug - only knee
		/*if (i!=6 && i!=7)
			continue;*/
		
		iUseIndex = i;

		/*if (Ragdoll::m_iLinked[i]!=-1)
		{
			bUseSymmetricSide = true;
		}*/
		

		/*if (bUseSymmetricSide)
			dFlipFactor = -1.f;*/

		//sprintf(temp,"%d using %d:\n",i,iUseIndex);
		//ODS(temp);
		for (int a=0;a<3;a++) //per axis
		{
			dVel = movement.muscle_commands[iUseIndex].axis[a];

			//debug
			//dVel = 10.0;

			//sprintf(temp,"%.3f -> ",dVel);
			//ODS(temp);

			dVelMax = g_pRagdoll->getAMotorVelocityMax(iUseIndex,a);	

			// HACK HACK HACK
			dVelMax*=0.75;
			
			if (dVel > 0.0)
			{
				if (dVel > dVelMax)
				{
					dVel = dVelMax - dMargin;
					if (dVel < 0.0)
					{
						dVel = 0.0;
					}
				} 
			} else if (dVel < 0.0)
			{
				if (dVel < -dVelMax)
				{
					dVel = -dVelMax + dMargin;
					if (dVel > 0.0)
					{
						dVel = 0.0;
					}
				}

			}
						
			ang_vel.data[a] = dVel;
		}

		//if (bPrintDebug)



		/*// print axis limits
		if (bPrintDebug)
		{
			for (int axis=0;axis<3;axis++)
			{
				sprintf(temp,"(%.3f / %.3f ) ",ang_vel.data[axis],dVelMax);
				ODS(temp);
			}
		}*/

		// print final angular velocities 
		if (bPrintDebug)
		{
			sprintf(temp,"%.3f\t%.3f\t%.3f\t",ang_vel.data[0],ang_vel.data[1],ang_vel.data[2]);
			ODS(temp);
		}

		for (int a=0;a<3;a++)
		{			
			g_pRagdoll->setAMotorVelocity(i,a,ang_vel.data[a]);
		}
							
	}

	if (bPrintDebug)
	{
		ODS("\n");
	}

}


void InitObjects()
{
//	size_t i;
//	int j,k;
	
	///////////////////////////////////////////////////////////////////////////
	///// Init Box
	int k;
	dReal sides[3];
	dMass m;
	box.body = dBodyCreate (g_world);
	/*for (k=0; k<3; k++) 
		sides[k] = dRandReal()*0.5+0.1;*/

	sides[0] = sides[1] = sides[2] = 0.25;

	dMatrix3 R;

	dBodySetPosition (box.body,
		dRandReal()*2-1,dRandReal()*2-1,dRandReal()+2);
		dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
		dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);

	dBodySetPosition (box.body,2.0,0.0,0.5);
	//dBodySetPosition (box.body,0.0,0.2,0.5); //under ragdoll

	dBodySetRotation (box.body,R);
	//dBodySetData (box.body,(void*) i);

	dMassSetBox (&m,100,sides[0],sides[1],sides[2]);
	box.geom = dCreateBox (space,sides[0],sides[1],sides[2]);

	if (box.geom) 
		dGeomSetBody(box.geom,box.body);

	dBodySetMass (box.body,&m);

	///////////////////////////////////////////////////////////////////////////

	g_pRagdoll = new Ragdoll(g_world, space,/*400*/200, vRagdollStartPos,g_pRenderer);
 
	//g_pRagdoll_2 = new Ragdoll(g_world, space,500, Vec3d(0.0,-1.5,1.0),g_pRenderer);

	//g_pRagdoll = new Ragdoll(g_world, space,500, Vec3d(0.0,0.5,-1.0),g_pRenderer);	
}


void DestroyWorld()
{

	//Note!
	//i'm probably leaking the box now.

	if (g_pRagdoll)
	{
		delete g_pRagdoll;
		g_pRagdoll = NULL;
	}

	if (contactgroup)
		dJointGroupDestroy (contactgroup);

	if (space)
		dSpaceDestroy (space);

	if (g_world)
		dWorldDestroy (g_world);
}

void CreateWorld(int iSeed)
{
	dRandSetSeed(iSeed);
	srand(iSeed);

	/*char message[100];
	sprintf(message,"CreateWorld: srand(%d)\n",iSeed);
	ODS(message);*/

	g_world = dWorldCreate();
	space = dHashSpaceCreate (0);
	contactgroup = dJointGroupCreate (0);
	dWorldSetGravity (g_world,0,0,-9.8);
	//dWorldSetGravity (g_world,0,0,-8.5);
	//dWorldSetGravity (g_world,0,0,-3.5);

	//dWorldSetGravity (g_world,0,0,0.0);
	//dWorldSetGravity (g_world,0,0,-0.01);
	dWorldSetCFM (g_world,1e-4);


	dWorldSetERP (g_world,0.1);
	//dWorldSetAutoDisableFlag (g_world,1);
	dWorldSetAutoDisableFlag (g_world,0);

	#if 1

	dWorldSetAutoDisableAverageSamplesCount( g_world, 10 );

	#endif

	dWorldSetLinearDamping(g_world, 0.00001);
	dWorldSetAngularDamping(g_world, 0.005);
	dWorldSetMaxAngularSpeed(g_world, 200);

	dWorldSetContactMaxCorrectingVel (g_world,0.1);
	dWorldSetContactSurfaceLayer (g_world,0.001);
	dCreatePlane (space,0,0,1,0);
	memset (&box,0,sizeof(box));

	InitObjects();
}



//#define EVALUATION_TIME_DISPLAY 8.0 //evaluation time when displaying
#define EVALUATION_TIME_DISPLAY 100.0 //evaluation time when displaying
#define EVALUATION_PHYSICS_TIME_STEP (1.0/60.0)


void Throw_Box()
{
	Vec3d camPos = g_pRenderer->GetCameraPosition();
	//dBodySetPosition (box.body,2.0,0.0,0.5);
	dBodySetPosition (box.body,camPos.x,camPos.y,camPos.z);
	
	Vec3d camViewDir = g_pRenderer->GetCameraViewDir();

	Vec3d throw_force = mul3(norm3(camViewDir),6.0);

	dBodySetAngularVel(box.body,0.0,0.0,0.0);
	dBodySetLinearVel(box.body,throw_force.x,throw_force.y,throw_force.z);
	

}

void PushRagdoll_RandomDirection()
{
	double dRange = 1000.0;
		
	static bool bInitedThrowForces = false;
	static Vec3d dirs[10]; 
	static Vec3d forces[10]; 
	int iDirectionsNum = sizeof(dirs) / sizeof(Vec3d);

	if (!bInitedThrowForces)
	{		
		/*for (int i=0;i<iDirectionsNum;i++)
		{
			dirs[i].x = SFMT_IN_RANGE(-1.0,1.0);
			dirs[i].y = SFMT_IN_RANGE(-1.0,1.0);
			dirs[i].z = SFMT_IN_RANGE(0.0,1.0);

			forces[i] = mul3(norm3(dirs[i]),dRange);
		}*/

		//

		int n=0;

		dirs[n].x = 0.75;
		dirs[n].y = 0.25;
		dirs[n].z = 0.0;
		forces[n] = mul3(norm3(dirs[n]),dRange);
		n++;

		dirs[n].x = -0.75;
		dirs[n].y = -0.25;
		dirs[n].z = 0.75;
		forces[n] = mul3(norm3(dirs[n]),dRange);
		n++;

		//

		dirs[n].x = -1.0;
		dirs[n].y = -1.0;
		dirs[n].z = 0.5;
		forces[n] = mul3(norm3(dirs[n]),dRange);
		n++;

		dirs[n].x = -1.0;
		dirs[n].y = 1.0;
		dirs[n].z = 0.5;
		forces[n] = mul3(norm3(dirs[n]),dRange);
		n++;

		dirs[n].x = 1.0;
		dirs[n].y = -1.0;
		dirs[n].z = 0.5;
		forces[n] = mul3(norm3(dirs[n]),dRange);
		n++;

		dirs[n].x = 1.0;
		dirs[n].y = 1.0;
		dirs[n].z = 0.5;
		forces[n] = mul3(norm3(dirs[n]),dRange);
		n++;

		//

		dirs[n].x = -1.0;
		dirs[n].y = -1.0;
		dirs[n].z = 0.75;
		forces[n] = mul3(norm3(dirs[n]),dRange);
		n++;

		dirs[n].x = -1.0;
		dirs[n].y = 1.0;
		dirs[n].z = 0.75;
		forces[n] = mul3(norm3(dirs[n]),dRange);
		n++;

		dirs[n].x = 1.0;
		dirs[n].y = -1.0;
		dirs[n].z = 0.75;
		forces[n] = mul3(norm3(dirs[n]),dRange);
		n++;

		dirs[n].x = 1.0;
		dirs[n].y = 1.0;
		dirs[n].z = 0.75;
		forces[n] = mul3(norm3(dirs[n]),dRange);
		n++;

		


		bInitedThrowForces = true;
	}

	static int iCurrentDirNum = 0;

	//for (int i=0;i<15;i++)
	//int i=3;//head
	int i=0;//chest
	{
		dBodyAddForce(  g_pRagdoll->m_Bodies[i], forces[iCurrentDirNum].x,forces[iCurrentDirNum].y,forces[iCurrentDirNum].z);
	}


	iCurrentDirNum++;
	iCurrentDirNum%= iDirectionsNum;

}

void PushRagdoll(double dTime)
{
	//debug
	//return;

	if (dTime > 0.0)
		return;

	PushRagdoll_RandomDirection();
	
	return;


	// apply some force on the body (for stabilization evoution)

	/*
	Vec3d camPos = g_pRenderer->GetCameraPosition();
	dBodySetPosition (box.body,camPos.x,camPos.y,camPos.z);
	
	Vec3d camViewDir = g_pRenderer->GetCameraViewDir();

	Vec3d throw_force = mul3(norm3(camViewDir),6.0);

	dBodySetAngularVel(box.body,0.0,0.0,0.0);
	dBodySetLinearVel(box.body,throw_force.x,throw_force.y,throw_force.z);
	*/

	Vec3d start_linear_vel;
	Vec3d start_pos;

	start_pos.x = 0.0;
	start_pos.y = 0.0;
	start_pos.z = 0.0;
	
	static int count=0;

	double dVelFactor = 6.0;

	if (count%5==0)
	{
		start_linear_vel.x=-0.45008747785689074;
		start_linear_vel.y=5.8494088037490881;
		start_linear_vel.z=1.2577113774229303;
	} else if (count%5==1) 
	{
		start_linear_vel.x=-0.26626189779225984;
		start_linear_vel.y=5.6460699073301690;
		start_linear_vel.z=2.0127094185015291;
	} else if (count%5==2)
	{
		start_linear_vel.x=1.4134028715105131 ;
		start_linear_vel.y=5.5450003715065428 ;
		start_linear_vel.z=1.8042347970256387 ;		
	} else if (count%5==3)
	{
		start_linear_vel.x=-1.0416807585198591 ;
		start_linear_vel.y=5.5661719072328539 ;
		start_linear_vel.z=1.9830863562793390 ;		
	}  else if (count%5==4)
	{
		start_linear_vel.x=0.0;
		start_linear_vel.y=0.0;
		start_linear_vel.z=0.0;	
	}


	Vec3d camPos;/* = g_pRenderer->GetCameraPosition();*/
	camPos.x = 0.17435700862111869;
	camPos.y = -2.0091483996258321;
	camPos.z = 1.4470431457771087;
	dBodySetPosition (box.body,camPos.x,camPos.y,camPos.z);
	dBodySetAngularVel(box.body,0.0,0.0,0.0);
	dBodySetLinearVel(box.body,start_linear_vel.x,start_linear_vel.y,start_linear_vel.z);

	count++;
	count%=4;
}

void evaluateSimulation_Iteration(bool bVisible, double dDeltaTime, double dTime, int iStepCount, void* pPrivateData)
{
	if (bVisible)
		{
			g_Input.Update();
			processInput();	
		}					

		//////////////////////////////////
		// AI
		tBodyState currBodyState;
		//GetCurrentBodyPartsState(currBodyState);

		tMovement requestedMovement;

		/*if (bPrintDebug)
		{
			sprintf(temp,"<< Step %d >> \n",iStepCount);
			ODS(temp);
		}*/

		//if (iStepCount%5 == 0)
		{
			bool bDebug = false;
			//if (iStepCount == 115)
				//bDebug = true;


			/*
			int size = sizeof(g_pRagdoll->m_dCurrentAngularMotorsAngles);
			memcpy(&currBodyState.dAngularMotorsAngles, &g_pRagdoll->m_dCurrentAngularMotorsAngles,size);*/

			g_pRagdoll->FillBodyState(currBodyState);

			g_pBrain->DecideMovement(dTime,currBodyState,pPrivateData,requestedMovement);

			// [Debug]
			/*requestedMovement.Clear();
			if (g_Input.KeyPressed(DIK_6))
			{
				for (int i=0;i<3;i++)
				{
					requestedMovement.muscle_commands[2].axis[i] = 200.f;
				}
			}*/
			// [/Debug]

			ApplyBodyMovement(requestedMovement,bDebug); 
		}

		/*double dStep = 1.0/60.0;
		if (g_Input.KeyPressed(DIK_0))
			dStep = dStep*0.1;*/

		if (_CrtCheckMemory() == FALSE)
			DebugBreak();
		simulationStep(EVALUATION_PHYSICS_TIME_STEP);
		if (_CrtCheckMemory() == FALSE)
			DebugBreak();

		//if (iStepCount >= 115 && iStepCount <= 120)

		
		

		/*if (bPrintDebug)
		{
			unsigned int iBodiesCount = g_pRagdoll->m_Bodies.size();

			double linear_vel,angular_vel,force,torque;
			for (unsigned int b=0;b<iBodiesCount;b++)
			{								
				g_pRagdoll->getBodyInformation(b,linear_vel,angular_vel,force,torque);;

				sprintf(temp,"Body %d) linear_vel=%.2f, angular_vel=%.2f, force=%.2f, torque=%.2f\n",
					b,linear_vel,angular_vel,force,torque);
				ODS(temp);
			}
			
			ODS("----------------------------\n");			
		}*/

		//g_pRagdoll->UpdateCurrentAngularMotors();

		/*ragdoll_head_pos = g_pRagdoll->getBodyPosition(3); // head
		ragdoll_chest_pos = g_pRagdoll->getBodyPosition(0); // chest
		ragdoll_avg_pos = g_pRagdoll->getPosition(); */

		if (bVisible)
		{
			/*char temp[200];
			sprintf(temp,"Ragdoll [head %.3f,%.3f,%.3f] [chest %.3f,%.3f,%.3f] [avg %.3f,%.3f,%.3f]    Simulation Time:%.3f",
				ragdoll_head_pos.x,ragdoll_head_pos.z,ragdoll_head_pos.y,
				ragdoll_chest_pos.x,ragdoll_chest_pos.z,ragdoll_chest_pos.y,
				ragdoll_avg_pos.x,ragdoll_avg_pos.z,ragdoll_avg_pos.y,
				dTime);
			g_Window.SetWindowText(temp);*/
		}

		if (bVisible)
		{
			renderScene();
		}

}


void evaluateSimulation_CreateWorld(int iSeed)
{
	DestroyWorld();
	CreateWorld(iSeed);
}



//Falling allowed time starts with the entire simulation and slowly moves towards 0.0

//(When it's in 0.0 it means that falling is not allowed)
// ALLOWED_FALL_PART describes the speed that it will come from 1.0 to 0.0
// basically it's number of generation to go from 1.0 to 0.0
//#define FALLING_STOP_GENERATIONS 40000.0

// in each generation, how much time to reduce from maximum_falling_time
//#define COOLING_SPEED 0.0000000001
//#define COOLING_SPEED (ORGANISM_EVALUATION_SIMULATION_TIME / 1000.0)

#define WARMING_SPEED 0.000001
#define FALLING_CHECK_START_TIME 1.5


//TODO - continue lowering only after a capable organism was found!

//score

double evaluateSimulation_walk(int iGenerationNum,bool bVisible, int iSeed, double dSimulationTime,void* pPrivateData)
{
	double dTime = 0.0;
	double dMaxHeight = -1000.0;
	double dMaxForward = 0.0;
	Vec3d ragdoll_head_pos;
	Vec3d ragdoll_chest_pos;
	Vec3d ragdoll_avg_pos;
	
	char temp[100];

	evaluateSimulation_CreateWorld(iSeed);
	

	bool bFell = false;
	double dFallingTime = dSimulationTime;
	//double dForwardWhenFell = 10000.0;

	int iStepCount = 0;

	bool bPrintDebug = false;

	/*double dFallingAllowedT = (((double)iGenerationNum)/FALLING_STOP_GENERATIONS);
	double dFallingAllowedTime = dSimulationTime;

	if ((double) iGenerationNum < FALLING_STOP_GENERATIONS)
	{
		dFallingAllowedTime = dSimulationTime * (1.0-dFallingAllowedT);
	}*/

	/*double dFallingAllowedTime = dSimulationTime*1.2; 
	dFallingAllowedTime-= COOLING_SPEED*iGenerationNum;*/

	//double dFallingAllowedTime = WARMING_SPEED*iGenerationNum+FALLING_CHECK_START_TIME; 

	double dFallingStartCheckingTime = 0.0;

	if (dFallingStartCheckingTime < 0.0)
		dFallingStartCheckingTime = 0.0;

	while (dTime <  dSimulationTime)
	{
		evaluateSimulation_Iteration(bVisible, EVALUATION_PHYSICS_TIME_STEP, dTime,iStepCount,pPrivateData);		

		ragdoll_head_pos = g_pRagdoll->getBodyPosition(3);
		ragdoll_avg_pos = g_pRagdoll->getPosition();



		if (ragdoll_avg_pos.z > dMaxHeight)
		{			
			dMaxHeight = ragdoll_avg_pos.z;			
			//sprintf(temp,"max avg height=%.3f\n",dMaxHeight);
			//ODS(temp);
		}

		/*if (ragdoll_avg_pos.z > 1.9) // a workaround to prevent physics-fail (big bounce from the floor)
		{
			return 0.0;
		}*/

		if (ragdoll_head_pos.y < dMaxForward)
		{			
			dMaxForward = ragdoll_head_pos.y;			
		}

		// start pos is 1.6
		//if (ragdollPos.z < 0.5 || ragdollPos.z > 1.7)

		//only progress is success in current stage


		if ( (!bFell) && (dTime>= dFallingStartCheckingTime) &&			 
			(
			(ragdoll_head_pos.z < 1.3) //fell
			||
			(ragdoll_head_pos.z > 2.0) //jumped too high - might be a physics break
			// (Maybe i should check distance between limbs? big distance means physics system break)
			)


			 )
		{

			if (!bFell)
			{
				dFallingTime = dTime;
				//dForwardWhenFell = ragdoll_head_pos.y;
				bFell = true;
				break;
			}


			// fall means fail for now.
			//return 0.0;
		}

		/*if (bVisible)
		{
			char temp[100];
			//sprintf(temp,"Height: Max=%.3f Current=%.2f time=%.2f",dMaxHeight,ragdollPos.z,dTime);
			sprintf(temp,"Walk Distance: Max=%.3f Current=%.3f time=%.3f",-dMaxForward,ragdollPos.y,dTime);
			g_Window.SetWindowText(temp);
		}*/

		dTime+=EVALUATION_PHYSICS_TIME_STEP;
		iStepCount++;
	}

	///////////////////////////////////////////////////////////////
	// walk forward
	if (dMaxForward >= 0.0)
		return 0.0;

	double result = -dMaxForward;
	///////////////////////////////////////////////////////////////

	// stand still
	//double result = dFallingTime;


	/*
	// WALK FORWARD
	if (bFell)
	{
		dMaxForward = dForwardWhenFell;
	}

	if (dMaxForward>=0.0)
		return 0.0;

	//double result = dMaxHeight; // jump
	double result = dMaxForward*dMaxForward;
	*/

	//if (result>1.0)
		//result = 1000.0*result;

	/*if (bFell)
	{
		//double dPunish = dFallingTime / dSimulationTime;
		//result*= dPunish*dPunish*dPunish;
		result = 0.0;
	}*/

	return result;
}

double evaluateSimulation_jump(int iGenerationNum,bool bVisible, int iSeed, double dSimulationTime,void* pPrivateData)
{
	double dTime = 0.0;
	double dMaxHeight = -1000.0;
	double dMaxForward = 0.0;
	Vec3d ragdoll_head_pos;
	Vec3d ragdoll_chest_pos;
	Vec3d ragdoll_avg_pos;
	
	char temp[100];

	evaluateSimulation_CreateWorld(iSeed);

	bool bFell = false;
	double dFallingTime = dSimulationTime;

	int iStepCount = 0;

	bool bPrintDebug = false;

	while (dTime <  dSimulationTime)
	{
		evaluateSimulation_Iteration(bVisible, EVALUATION_PHYSICS_TIME_STEP, dTime,iStepCount,pPrivateData);

		ragdoll_head_pos = g_pRagdoll->getBodyPosition(3); // head
		ragdoll_chest_pos = g_pRagdoll->getBodyPosition(0); // chest
		ragdoll_avg_pos = g_pRagdoll->getPosition(); 


		/*if (ragdoll_avg_pos.z > dMaxHeight)
		{			
			dMaxHeight = ragdoll_avg_pos.z;			
		}*/

		if (ragdoll_head_pos.z > dMaxHeight)
		{			
			dMaxHeight = ragdoll_head_pos.z;			
		}

		if (ragdoll_head_pos.y < dMaxForward)
		{			
			dMaxForward = ragdoll_head_pos.y;			
		}

		if (ragdoll_head_pos.z < 0.4) //fell
		{
			break;
		}

		dTime+=EVALUATION_PHYSICS_TIME_STEP;
		iStepCount++;
	}
	
	double result = dMaxHeight;
	return result;
}

double evaluateSimulation_stabilize(int iGenerationNum,bool bVisible, int iSeed, double dSimulationTime,void* pPrivateData)
{
	double dTime = 0.0;
	double dMaxHeight = -1000.0;
	double dMaxForward = 0.0;
	Vec3d ragdoll_head_pos;
	Vec3d ragdoll_chest_pos;
	Vec3d ragdoll_avg_pos;
	
	char temp[100];

	evaluateSimulation_CreateWorld(iSeed);
	

	bool bFell = false;
	double dFallingTime = dSimulationTime;

	int iStepCount = 0;

	bool bPrintDebug = false;

	while (dTime <  dSimulationTime) 
	{		
		PushRagdoll(dTime);

		evaluateSimulation_Iteration(bVisible, EVALUATION_PHYSICS_TIME_STEP, dTime,iStepCount,pPrivateData);		

		ragdoll_head_pos = g_pRagdoll->getBodyPosition(3); // head
		ragdoll_chest_pos = g_pRagdoll->getBodyPosition(0); // chest
		ragdoll_avg_pos = g_pRagdoll->getPosition(); 

		if (ragdoll_avg_pos.z > dMaxHeight)
		{			
			dMaxHeight = ragdoll_avg_pos.z;			
		}

		if (ragdoll_head_pos.y < dMaxForward)
		{			
			dMaxForward = ragdoll_head_pos.y;			
		}

		if (ragdoll_chest_pos.z < 0.8) //fell
		{
			break;
		}

		dTime+=EVALUATION_PHYSICS_TIME_STEP;
		iStepCount++;
	}
	
	/*double result = dMaxHeight;
	return result;*/

	return dTime;
}

double evaluateSimulation_raise_leg(int iGenerationNum,bool bVisible, int iSeed, double dSimulationTime,void* pPrivateData)
{
	double dTime = 0.0;
	double dMaxHeight = -1000.0;
	double dMaxForward = 0.0;
	
	char temp[100];

	evaluateSimulation_CreateWorld(iSeed);
	
	bool bFell = false;
	double dFallingTime = dSimulationTime;

	int iStepCount = 0;

	bool bPrintDebug = false;

	Vec3d desired_avg_pos;
	Vec3d curr_left_foot_pos, curr_avg_pos;
	double dDistance_foot,dDistance_avg;

	desired_avg_pos = g_pRagdoll->getPosition();

	Vec3d desired_left_foot_pos(0.13425869169216720,0.46229024389157436-0.2,0.074724623009372632+0.2);

	double dScore = 0.0;

	while (dTime <  dSimulationTime) 
	{		
		evaluateSimulation_Iteration(bVisible, EVALUATION_PHYSICS_TIME_STEP, dTime,iStepCount,pPrivateData);		

		curr_left_foot_pos = g_pRagdoll->getBodyPosition(9); // left foot		
		dDistance_foot = len3(sub3(curr_left_foot_pos,desired_left_foot_pos));		
		dScore -= dDistance_foot;

		curr_avg_pos = g_pRagdoll->getPosition();
		dDistance_avg = len3(sub3(curr_avg_pos,desired_avg_pos));
		dScore -= dDistance_avg*dDistance_avg;


		dTime+=EVALUATION_PHYSICS_TIME_STEP;
		iStepCount++;
	}
	
	/*double result = dMaxHeight;
	return result;*/

	return dScore;
}

//double(*g_evalFuncPtr)(int,bool,int,double,void*) = &evaluateSimulation_walk;
double(*g_evalFuncPtr)(int,bool,int,double,void*) = &evaluateSimulation_jump;
//double(*g_evalFuncPtr)(int,bool,int,double,void*) = &evaluateSimulation_stabilize;
//double(*g_evalFuncPtr)(int,bool,int,double,void*) = &evaluateSimulation_raise_leg;


// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
  static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
  //dsSetViewpoint (xyz,hpr);
  printf ("To drop another object, press:\n");
  printf ("   b for box.\n"); 
}


char locase (char c)
{
  if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
  else return c;
}


// called when a key pressed

static void command (int cmd)
{
  cmd = locase (cmd);

 

}





// draw a geom

/*void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
{
  int i;
	
  if (!g) return;
  if (!pos) pos = dGeomGetPosition (g);
  if (!R) R = dGeomGetRotation (g);

  int type = dGeomGetClass (g);
  if (type == dBoxClass) 
	{
		dVector3 sides;
		dGeomBoxGetLengths (g,sides);
		dsDrawBox (pos,R,sides);
	}
  
  if (show_body) {
    dBodyID body = dGeomGetBody(g);
    if (body) {
      const dReal *bodypos = dBodyGetPosition (body); 
      const dReal *bodyr = dBodyGetRotation (body); 
      dReal bodySides[3] = { 0.1, 0.1, 0.1 };
      dsSetColorAlpha(0,1,0,1);
      dsDrawBox(bodypos,bodyr,bodySides); 
    }
  }
  if (show_aabb) 
	{
		// draw the bounding box for this geom
		dReal aabb[6];
		dGeomGetAABB (g,aabb);
		dVector3 bbpos;
		for (i=0; i<3; i++) 
			{
				bbpos[i] = 0.5*(aabb[i*2] + aabb[i*2+1]);
			}
		dVector3 bbsides;
		for (i=0; i<3; i++) 
			{
				bbsides[i] = aabb[i*2+1] - aabb[i*2];
			}
		dMatrix3 RI;
		dRSetIdentity (RI);
		dsSetColorAlpha (1,0,0,0.5);
		dsDrawBox (bbpos,RI,bbsides);
	}
}*/

bool Init_GL(int iResX, int iResY, HWND hWnd)
{

	DWORD windowStyle = WS_OVERLAPPEDWINDOW;							// Define Our Window Style
	DWORD windowExtendedStyle = WS_EX_APPWINDOW;						// Define The Window's Extended Style

	PIXELFORMATDESCRIPTOR pfd =											// pfd Tells Windows How We Want Things To Be
	{
		sizeof (PIXELFORMATDESCRIPTOR),									// Size Of This Pixel Format Descriptor
		1,																// Version Number
		PFD_DRAW_TO_WINDOW |											// Format Must Support Window
		PFD_SUPPORT_OPENGL |											// Format Must Support OpenGL
		PFD_DOUBLEBUFFER,												// Must Support Double Buffering
		PFD_TYPE_RGBA,													// Request An RGBA Format
		32,			                        							// Select Our Color Depth
		0, 0, 0, 0, 0, 0,												// Color Bits Ignored
		0,																// No Alpha Buffer
		0,																// Shift Bit Ignored
		0,																// No Accumulation Buffer
		0, 0, 0, 0,														// Accumulation Bits Ignored
		32,																// 32Bit Z-Buffer (Depth Buffer)  
		0,																// No Stencil Buffer
		0,																// No Auxiliary Buffer
		PFD_MAIN_PLANE,													// Main Drawing Layer
		0,																// Reserved
		0, 0, 0															// Layer Masks Ignored
	};

	RECT windowRect = {0, 0, iResX, iResY};	// Define Our Window Coordinates

	GLuint PixelFormat;													// Will Hold The Selected Pixel Format

	g_hDC = GetDC (hWnd);									// Grab A Device Context For This Window
	if (!g_hDC)												// Did We Get A Device Context?
	{
		// Failed
		DestroyWindow (hWnd);									// Destroy The Window
		hWnd = 0;												// Zero The Window Handle
		return FALSE;													// Return False
	}

	PixelFormat = ChoosePixelFormat (g_hDC, &pfd);				// Find A Compatible Pixel Format
	if (!PixelFormat)												// Did We Find A Compatible Format?
	{
		// Failed
		ReleaseDC (hWnd, g_hDC);							// Release Our Device Context
		g_hDC = 0;												// Zero The Device Context
		DestroyWindow (hWnd);									// Destroy The Window
		hWnd = 0;												// Zero The Window Handle
		return FALSE;													// Return False
	}

	if (SetPixelFormat (g_hDC, PixelFormat, &pfd) == FALSE)	
	{
		// Failed
		ReleaseDC (hWnd, g_hDC);							
		g_hDC = 0;											
		DestroyWindow (hWnd);								
		hWnd = 0;											
		return FALSE;										
	}

	HGLRC hRC = wglCreateContext (g_hDC);						
	if (!hRC)											
	{
		// Failed
		ReleaseDC (hWnd, g_hDC);							
		g_hDC = 0;											
		DestroyWindow (hWnd);								
		hWnd = 0;											
		return FALSE;										
	}

	// Make The Rendering Context Our Current Rendering Context
	if (wglMakeCurrent (g_hDC, hRC) == FALSE)
	{
		// Failed
		wglDeleteContext (hRC);							
		hRC = 0;											
		ReleaseDC (hWnd, g_hDC);							
		g_hDC = 0;												
		DestroyWindow (hWnd);									
		hWnd = 0;												
		return FALSE;											
	}

	return TRUE;
}

#include <Commdlg.h>


#define MAX_LOADSTRING 100

DWORD WINAPI evolve_thread_main_func(LPVOID lpThreadParameter)
{
	if (!lpThreadParameter)
		return 1;

	Brain* pBrain = (Brain*) lpThreadParameter;

	pBrain->Evolve();

	return 0;
}


// Global Variables:
HINSTANCE hInst;								// current instance
TCHAR szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR szWindowClass[MAX_LOADSTRING];			// the main window class name

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY _tWinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPTSTR    lpCmdLine,
                     int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

 	// TODO: Place code here.
	MSG msg;
	HACCEL hAccelTable;

	g_pcPlayFileName[0] = NULL;

	//_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );

	//_CrtDumpMemoryLeaks();
	
	//pCat = (char*) test_new(1000,__FILE__, __LINE__);

	char file_name[512];
	if (strstr(lpCmdLine,"play"))
	{
		sprintf(file_name,"./zzzz_play_Log_%d.txt",GetCurrentProcessId());
	} else
	{
		sprintf(file_name,"C:/GeneWarrior_WinnerSessions/zzzzLog_%d.txt",GetCurrentProcessId());
	}
	g_pLog = fopen(file_name,"w");
	if (!g_pLog)
	{
		ALLOC_LOG;
		sprintf(log,"Error! failed creating log file at: [%s]\n",file_name);
		OutputDebugStringA(log);
		MessageBoxA(0,log,0,0);	
	}

	if (strstr(lpCmdLine,"play"))
	{
		OPENFILENAME ofn;       // common dialog box structure
		char szFile[260];       // buffer for file name
		HANDLE hf;              // file handle

		// Initialize OPENFILENAME
		ZeroMemory(&ofn, sizeof(ofn));
		ofn.lStructSize = sizeof(ofn);
		ofn.hwndOwner = NULL;
		ofn.lpstrFile = szFile;
		// Set lpstrFile[0] to '\0' so that GetOpenFileName does not 
		// use the contents of szFile to initialize itself.
		ofn.lpstrFile[0] = '\0';
		ofn.nMaxFile = sizeof(szFile);
		ofn.lpstrFilter = "Organism Files\0*.org";
		ofn.nFilterIndex = 1;
		ofn.lpstrFileTitle = NULL;
		ofn.nMaxFileTitle = 0;
		ofn.lpstrInitialDir = NULL;
		ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

		char current_dir[4096];
		GetCurrentDirectory(4096,current_dir);		
		bool bRet = GetOpenFileName(&ofn);
		SetCurrentDirectory(current_dir);

		if (bRet)
		{
			strcpy(g_pcPlayFileName,ofn.lpstrFile);
			g_bPlayMode = true;
		} else
		{
			exit(0);
		}
	}

	// Initialize global strings
	LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadString(hInstance, IDC_GENEWARRIOR_WIN, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	// Perform application initialization:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}

	// create world
	dInitODE2(0);//

	dSetErrorHandler(&ErrorCb);
	dSetDebugHandler(&DebugCb);
	dSetMessageHandler(&MessageCb);

	Ragdoll::InitStatics();

	//g_pBrain = new RandomSinusBrain();
	g_pBrain = new GeneticAlgo_Brain();
	//g_pBrain = new NeuralNetwork_Brain();
	
	g_pBrain->SetEvaulationFunction(g_evalFuncPtr);
	
	//CreateWorld();

	// Init window

	tWinProperties windowProps;
	windowProps.iXPos = 0;
	windowProps.iYPos = 0;


	windowProps.iClientWidth  = 800;
	windowProps.iClientHeight = 600;

	if (g_bPlayMode)
	{
		windowProps.pCaption = new char[50];
		//sprintf(windowProps.pCaption,"Crimson Engine");
		sprintf_s(windowProps.pCaption,50,"Crimson Engine");

		windowProps.pClassName = new char[50];
		sprintf_s(windowProps.pClassName,50,"Crimson Engine");
		g_Window.Create(hInstance, windowProps );
		g_Window.SetWindowText("Crimson Engine");
	

		g_Input.Init((unsigned long)windowProps.iClientWidth,(unsigned long)windowProps.iClientHeight,g_Window.GetHWND());
		g_Input.SetCursorPosition((unsigned long)windowProps.iClientWidth / 2,(unsigned long)windowProps.iClientHeight / 2);

		Init_GL(800,600,g_Window.GetHWND());

		g_pRenderer = new Renderer(800,600);

	}

	
	
	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_GENEWARRIOR_WIN));

	/*// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		idleFunc();
		renderScene();
	}*/
			

	LARGE_INTEGER freq;
	
	if (!QueryPerformanceFrequency(&freq))
		assert(0);

	g_dFrequency = double(freq.QuadPart);
	if (!QueryPerformanceCounter(&g_PreviousTime))
		assert(0);


	char message[100];
	double dTime = 0.0;
	int iStepCount = 0;
	Vec3d ragdoll_head_pos,ragdoll_chest_pos,ragdoll_avg_pos;
	double dMaxHeight = -9999.0;

	bool bIsMessagePumpActive = TRUE;								// Set isMessagePumpActive To TRUE
	while (bIsMessagePumpActive == TRUE)						// While The Message Pump Is Active
	{
		// Success Creating Window.  Check For Window Messages
		if (PeekMessage (&msg, NULL, 0, 0, PM_REMOVE) != 0)
		{
			// Check For WM_QUIT Message
			if (msg.message != WM_QUIT)						// Is The Message A WM_QUIT Message?
			{
				DispatchMessage (&msg);						// If Not, Dispatch The Message
			}
			else											// Otherwise (If Message Is WM_QUIT)
			{
				bIsMessagePumpActive = FALSE;				// Terminate The Message Pump
			}
		}
		else												// If There Are No Messages
		{									
			if (!g_bEvolved && !g_bPlayMode)
			{
				/*HANDLE thread = CreateThread(0,1024*1024*50,evolve_thread_main_func,g_pBrain,0,0);
				if (!thread)
				{
					MessageBoxA(0,"Error at creating evolve thread!",0,0);
				}

				WaitForSingleObject(thread,INFINITE);				*/

				g_pBrain->Evolve();
				g_bEvolved = true;
				break;
			} else
			{
				if (g_bPlayMode && !g_bPlayMode_Started) 
				{
					g_pBrain->LoadFromFile(g_pcPlayFileName);

					evaluateSimulation_CreateWorld(g_pBrain->GetRandomSeed());

					g_bPlayMode_Started = true;

					g_pBrain->PrintWinnerDebugInfo();
				}

				//evaluateSimulation(true,g_pBrain->GetRandomSeed(),EVALUATION_TIME_DISPLAY,NULL);

				//EVALUATION_PHYSICS_TIME_STEP contains how much a single step means in the physics test (in seconds)

				LARGE_INTEGER current_time;
				
				if (!QueryPerformanceCounter(&current_time))
					assert(0);

				double dDelta = ((double)(current_time.QuadPart - g_PreviousTime.QuadPart)) / g_dFrequency; //ms
				//dDelta *= 1000;
				double dDesiredDelta = EVALUATION_PHYSICS_TIME_STEP * 1000; //ms

				double dDiff = dDesiredDelta - dDelta;

				if (dDiff > 0.0)
				{
					Sleep((int)dDiff);
				}

				g_PreviousTime = current_time;

				if (g_Input.KeyPressed(DIK_0))
					Sleep(200);

				if (g_Input.KeyPressed(DIK_9))
					Sleep(100);

				if (g_Input.KeyPressed(DIK_8))
					Sleep(50);

				static bool bPushPressed = false;
				if (g_Input.KeyPressed(DIK_T))
				{
					if (!bPushPressed)
					{
						PushRagdoll_RandomDirection();
					}
					bPushPressed = true;
				} else
				{
					bPushPressed = false;
				}
					
				//PushRagdoll(dTime);
				evaluateSimulation_Iteration(true,EVALUATION_PHYSICS_TIME_STEP,dTime,iStepCount,NULL);

				

				dTime+=EVALUATION_PHYSICS_TIME_STEP;
				iStepCount++;

				static bool bRestartPressed = false;
				bool bRequestRestart = false;

				if (g_Input.KeyPressed(DIK_R))
				{
					if (!bRestartPressed)
					{
						bRequestRestart = true;
						bRestartPressed = true;
					}
				} else
				{
					bRestartPressed = false;
				}

				if (dTime > EVALUATION_TIME_DISPLAY || bRequestRestart)
				{
					dTime = 0.0;
					iStepCount = 0;
					evaluateSimulation_CreateWorld(g_pBrain->GetRandomSeed());
				}

				static bool bThrowBoxTriggered=false;
				if (g_Input.KeyPressed(DIK_SPACE))
				{
					if (!bThrowBoxTriggered)
					{
						Throw_Box();

						bThrowBoxTriggered = true;
					}
				} else
				{
					bThrowBoxTriggered = false;
				}
			}
			
		}
	}	

	DestroyWorld();
	
	dCloseODE();

	if (g_pBrain)
	{
		delete g_pBrain;
		g_pBrain = NULL;
	}

	return (int) msg.wParam;
}



//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
//  COMMENTS:
//
//    This function and its usage are only necessary if you want this code
//    to be compatible with Win32 systems prior to the 'RegisterClassEx'
//    function that was added to Windows 95. It is important to call this function
//    so that the application will get 'well formed' small icons associated
//    with it.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= WndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_GENEWARRIOR_WIN));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_GENEWARRIOR_WIN);
	wcex.lpszClassName	= szWindowClass;
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

	return RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   HWND hWnd;

   hInst = hInstance; // Store instance handle in our global variable

   hWnd = CreateWindow(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);

   if (!hWnd)
   {
      return FALSE;
   }

   //ShowWindow(hWnd, nCmdShow); //yoel
   UpdateWindow(hWnd);

   return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND	- process the application menu
//  WM_PAINT	- Paint the main window
//  WM_DESTROY	- post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	PAINTSTRUCT ps;
	HDC hdc;

	switch (message)
	{
	case WM_COMMAND:
		wmId    = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;
	case WM_PAINT:
		hdc = BeginPaint(hWnd, &ps);
		// TODO: Add any drawing code here...
		EndPaint(hWnd, &ps);
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	UNREFERENCED_PARAMETER(lParam);
	switch (message)
	{
	case WM_INITDIALOG:
		return (INT_PTR)TRUE;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}
