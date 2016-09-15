#ifndef YOEL_BASE_BRAIN_H
#define YOEL_BASE_BRAIN_H

#pragma once

#include "gwMath.h"
#include "Defines.h"


#define AXIS_NUM 3


/*enum EBodyParts
{
	EBP_PELVIS	= 0,
	EBP_BACK,
	EBP_HEAD,
	EBP_LEFT_LEG_1,
	EBP_LEFT_LEG_2,
	EBP_RIGHT_LEG_1,
	EBP_RIGHT_LEG_2,
	EBP_LEFT_FOOT,
	EBP_RIGHT_FOOT,
	EBP_LEFT_HAND_1,
	EBP_LEFT_HAND_2,
	EBP_RIGHT_HAND_1,
	EBP_RIGHT_HAND_2,
	EBP_LEFT_PALM,
	EBP_RIGHT_PALM,
	EBP_NUM
};*/

#define X 0
#define Y 2
#define Z 1

///////////////////////////////////////////////////////////////
// INPUT rigid bodies rotational values - kept in WORLD SPACE

struct tBodyPartTransform
{	
	Vec3d		rot; 
	Vec3d		pos;
};



/////////////////////////////////////
// OUTPUT movement command - out in 

struct tMuscleCommand
{
	static const UINT MUSCLE_MOVE_AXIS_NUM = AXIS_NUM;

	void Add(tMuscleCommand& other)
	{
		for (int i=0;i<MUSCLE_MOVE_AXIS_NUM;i++)
		{
			axis[i] += other.axis[i];
		}
	}

	double axis[MUSCLE_MOVE_AXIS_NUM];
};

struct tMovement 
{
	void Clear()
	{
		for (int i=0;i<ANGULAR_MOTORS_NUM;i++)
		{
			for (int a=0;a<tMuscleCommand::MUSCLE_MOVE_AXIS_NUM;a++)
			{
				muscle_commands[i].axis[a] = 0.f;
			}
		}
	}

	void Add(tMovement& other)
	{
		for (int i=0;i<ANGULAR_MOTORS_NUM;i++)
		{
			muscle_commands[i].Add(other.muscle_commands[i]);
		}
	}
	tMuscleCommand muscle_commands[ANGULAR_MOTORS_NUM];
};

/////////////////////////////////////



// base class for the brain
class Brain
{
public:
	Brain() {m_evalFuncPtr=NULL;}
	virtual ~Brain() {};

	virtual void SetEvaulationFunction(double(*evalFuncPtr)(int,bool,int,double,void*)) {m_evalFuncPtr = evalFuncPtr;};

	virtual void DecideMovement(IN float time, IN tBodyState& current,IN void* pPrivateData,OUT tMovement& movement ) = NULL;

	virtual void Evolve() {};

	virtual int GetRandomSeed() = NULL; //for display mode

	virtual bool LoadFromFile(const char* pcFileName) = NULL;

	virtual void PrintWinnerDebugInfo() = NULL;
	
protected:

	double(*m_evalFuncPtr)(int,bool,int,double,void*);
	
};




#endif