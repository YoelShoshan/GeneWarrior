#ifndef YOEL_RANDOM_SINUS_BRAIN_H
#define YOEL_RANDOM_SINUS_BRAIN_H

#pragma once

#include "Brain.h"
#include "GMath.h"
#include "stdio.h"
#include "time.h"
#include "..\sfmt\sfmt.h"
#include "Ragdoll.h"

#define MAX_WAVES_LAYERS 100

/*#define RANDOM_FLOAT (((float)rand()/(float)MAX))
#define RAND_IN_RANGE(start,end) ((RANDOM_FLOAT*((end)-(start)))+(start))
#define SFMT_IN_RANGE_INT(start,end) (((rand())%((end)-(start)))+start)*/

#define SFMT_FLOAT (((float)gen_rand32()/(float)MAXUINT))
#define SFMT_IN_RANGE(start,end) ((SFMT_FLOAT*((end)-(start)))+(start))
#define SFMT_IN_RANGE_UINT(start,end) (((gen_rand32())%((end)-(start)))+start)


struct SensorBasedWave
{
	SensorBasedWave(): sensor_type(S_TIME), sensor_index(0)
	{
	}

	~SensorBasedWave()
	{
	}

	ESensorType sensor_type;
	unsigned int sensor_index; //index into this specific sensor type
	CWave wave;
};

struct RS_Layer
{
	//CWave waves[MOTORS_NUM][tMuscleCommand::MUSCLE_MOVE_AXIS_NUM];
	SensorBasedWave waves[MOTORS_NUM][tMuscleCommand::MUSCLE_MOVE_AXIS_NUM];

};

struct RS_Organism
{
	void Init()
	{
		m_uiActiveWavesLayers = 0;
		for (int x=0;x<MAX_WAVES_LAYERS;x++)
		{
			for (int i=0;i<MOTORS_NUM;i++)
			{
				for (int a=0;a<tMuscleCommand::MUSCLE_MOVE_AXIS_NUM;a++)
				{
					layers[x].waves[i][a].wave.SetParams(WF_SQUARE,0.f,0.f,0.f,0.f);
				}
			}
		}

		for (int i=0;i<MOTORS_NUM;i++)
		{
			m_iShifts[i] = SFMT_IN_RANGE_UINT(-15,15);//0;
		}

		srand ( time(NULL) );
		m_iRandomSeed =  GetTickCount();
		m_dScore = 0.0;
		m_uiAge = 0;
		m_uiDeathAge = 0;

		AssignID();		

		m_uiCalcluatedScoreForID = (unsigned int)-1;
	}

	void AssignID()
	{
		static unsigned int S_NextID = 0;

		m_uiID = S_NextID;
		S_NextID++;
	}

	void Print()
	{
		
	}


	void SaveToFile_Text(const char* pFileName);


	void SaveToFile(const char* pFileName);

	bool LoadFromFile(const char* pFileName);

	unsigned int m_uiActiveWavesLayers;

	RS_Layer layers[MAX_WAVES_LAYERS];

	//CWave waves[MAX_WAVES_LAYERS][MOTORS_NUM][tMuscleCommand::MUSCLE_MOVE_AXIS_NUM];

	int m_iRandomSeed;
	double	m_dScore;
	unsigned int m_uiCalcluatedScoreForID;
	unsigned int m_uiAge;
	unsigned int m_uiDeathAge;
	unsigned int m_uiID;

	//double m_dShifts[MOTORS_NUM];
	int	m_iShifts[MOTORS_NUM];
};



class RandomSinusBrain: public Brain
{
public:
	RandomSinusBrain();
	~RandomSinusBrain();

	void DecideMovement(IN float time, IN tBodyState& current,IN void* pPrivateData,OUT tMovement& movement );

	void Evolve();

	int GetRandomSeed();

	bool LoadFromFile(const char* pcFileName) {return true;};

	void PrintWinnerDebugInfo() {};

protected:

	RS_Organism m_BestOrganism;
	RS_Organism m_CurrentOrganism;
	double m_dBestScore;
};


#endif