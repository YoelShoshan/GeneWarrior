#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "Ragdoll.h"
#include <assert.h>

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

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#include "icosahedron_geom.h"

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawConvex dsDrawConvexD
#endif


// some constants

#define NUM 100			// max number of objects
#define DENSITY (5.0)		// density of all objects
#define GPB 3			// maximum number of geometries per body
#define MAX_CONTACTS 8          // maximum number of contact points per body
#define MAX_FEEDBACKNUM 20
#define GRAVITY         REAL(0.5)
#define USE_GEOM_OFFSET 1

// dynamics and collision objects

struct MyObject {
  dBodyID body;			// the body
  dGeomID geom;		// geometries representing this body
};

static int num=0;		// number of objects in simulation
static int nextobj=0;		// next object to recycle if num==NUM
static dWorldID g_world;
static dSpaceID space;
static MyObject box;
static dJointGroupID contactgroup;
static int show_aabb = 0;	// show geom AABBs?
static int show_contacts = 0;	// show contact points?
static int write_world = 0;
static int show_body = 0;


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
			contact[i].surface.mu = 500.0;
			
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


// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread(dAllocateMaskAll);

  static float xyz[3] = {2.1640f,-1.3079f,1.7600f};
  static float hpr[3] = {125.5000f,-17.0000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
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

  if (cmd == '[')
	  g_pRagdoll->decreaseCurrentActiveMotor();

  if (cmd == ']')
	  g_pRagdoll->increaseCurrentActiveMotor();

  if (cmd =='0')
	  g_pRagdoll->setMotorVelocity(0,20.0);
  if (cmd =='1')
	  g_pRagdoll->setMotorVelocity(0,-20.0);

  if (cmd =='2')
	  g_pRagdoll->setMotorVelocity(1,20.0);
  if (cmd =='3')
	  g_pRagdoll->setMotorVelocity(1,-20.0);

  if (cmd =='4')
	  g_pRagdoll->setMotorVelocity(2,20.0);
  if (cmd =='5')
	  g_pRagdoll->setMotorVelocity(2,-20.0);

}


void InitObjects()
{
//	size_t i;
//	int j,k;
	int k;
	dReal sides[3];
	dMass m;
	box.body = dBodyCreate (g_world);
	for (k=0; k<3; k++) 
		sides[k] = dRandReal()*0.5+0.1;

	dMatrix3 R;

	/*dBodySetPosition (box.body,
		dRandReal()*2-1,dRandReal()*2-1,dRandReal()+2);
		dRFromAxisAndAngle (R,dRandReal()*2.0-1.0,dRandReal()*2.0-1.0,
		dRandReal()*2.0-1.0,dRandReal()*10.0-5.0);*/

	//dBodySetPosition (box.body,2.0,0.0,0.5);
	dBodySetPosition (box.body,0.0,0.2,0.5); //under ragdoll

	dBodySetRotation (box.body,R);
	//dBodySetData (box.body,(void*) i);

	dMassSetBox (&m,DENSITY,sides[0],sides[1],sides[2]);
	box.geom = dCreateBox (space,sides[0],sides[1],sides[2]);

	if (box.geom) 
		dGeomSetBody(box.geom,box.body);

	dBodySetMass (box.body,&m);

	g_pRagdoll = new Ragdoll(g_world, space,500, Vec3d(0.0,0.5,1.0));

}


// draw a geom

void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb)
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
}


// simulation loop

static void simLoop (int pause)
{
	dsSetColor (0,0,2);
	dSpaceCollide (space,0,&nearCallback);
	if (!pause) 
		//g_pRagdoll->applyTorque();
		g_pRagdoll->update();
		//dWorldQuickStep(g_world,0.02);
		dWorldStep(g_world,1.0/60.0);
		//dWorldStep(g_world,1.0/60000.0);
	
	// remove all contact joints
	dJointGroupEmpty (contactgroup);

	dsSetColor (1,1,0);
	dsSetTexture (DS_WOOD);
	if (! dBodyIsEnabled(box.body)) 
	{
		dsSetColor (1,0.0,0);
	}
	else 
	{
		dsSetColor (1,1,0);
	}

	assert(g_pRagdoll);
	g_pRagdoll->draw();

	drawGeom (box.geom,0,0,show_aabb);	

	
}


int main (int argc, char **argv)
{
  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = 0;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  // create world
  dInitODE2(0);

  dSetErrorHandler(&ErrorCb);
  dSetDebugHandler(&DebugCb);
  dSetMessageHandler(&MessageCb);

  g_world = dWorldCreate();
  space = dHashSpaceCreate (0);
  contactgroup = dJointGroupCreate (0);
  //dWorldSetGravity (g_world,0,0,-9.8);
  //dWorldSetGravity (g_world,0,0,-6.8);
  dWorldSetGravity (g_world,0,0,-1.01);
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

  // run simulation
  dsSimulationLoop (argc,argv,800,600,&fn);

  dJointGroupDestroy (contactgroup);
  dSpaceDestroy (space);
  dWorldDestroy (g_world);
  dCloseODE();
  return 0;
}
