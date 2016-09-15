#include "stdafx.h"
#include "RandomSinusBrain.h"
#include "stdio.h"
#include "input.h"
#include <time.h>
#include <assert.h>
#include "Ragdoll.h"

#include <Commdlg.h>

extern FILE* g_pLog;

extern CInput g_Input;
extern Ragdoll* g_pRagdoll;


#define EVALUATION_TIME_EVOLVE 9.0 //evaluation time when evolving


void RS_Organism::SaveToFile_Text(const char* pFileName)
{
	FILE* f = fopen(pFileName,"wb");

	if (!f)
	{
		ALLOC_LOG;
		sprintf(log,"Error: SaveToFile_Text - failed to open file for writing: [%s]\n",pFileName);
		OutputDebugStringA(log);
		MessageBoxA(0,log,0,0);	
		return;
	}
	//fwrite(this,sizeof(RS_Organism),1,f);		

	char motor_name[30];
	//char body_name[30];
	char function_name[30];

	//char temp[200];
	fprintf(f,"Random_seed@%d@",m_iRandomSeed);				
	fprintf(f,"Layers_num@%u@\n", m_uiActiveWavesLayers);
	for (int i=0;i<MOTORS_NUM;i++)
	{

		Ragdoll::GetMotorName(i,motor_name,30);

		fprintf(f,"@Muscle@%d@%s@\n",i,motor_name);

		for (unsigned int x=0;x<m_uiActiveWavesLayers;x++)
		{
			fprintf(f,"\t@SensorBasedWave@%d@:\n",x);	
		
			for (int a=0;a<tMuscleCommand::MUSCLE_MOVE_AXIS_NUM;a++)
			{

				/*ESensorType sensor_type;
				unsigned int sensor_index; //index into this specific sensor type
				CWave wave;*/

				CWave::GetWaveTypeName(layers[x].waves[i][a].wave.m_uiWaveType,function_name,30);


				fprintf(f,"\t\t@Axis@%d@sensor_type@%u@sensor_ind@%u@%u@%s@%.4f@%.4f@%.4f@%.4f\n",
					a,
					layers[x].waves[i][a].sensor_type,
					layers[x].waves[i][a].sensor_index,
					layers[x].waves[i][a].wave.m_uiWaveType,
					function_name,
					layers[x].waves[i][a].wave.m_fBase,
					layers[x].waves[i][a].wave.m_fAmplitude,
					layers[x].waves[i][a].wave.m_fPhase,
					layers[x].waves[i][a].wave.m_fFreq);
			}
		}
	}
	
	fprintf(f,"Shifts:\t");

	for (int i=0;i<MOTORS_NUM;i++)
	{
		fprintf(f,"%d\t",m_iShifts[i]);
	}
	fprintf(f,"\n");

	fclose(f);
}

void RS_Organism::SaveToFile(const char* pFileName)
{
	FILE* f = fopen(pFileName,"wb");
	if (!f)
	{
		ALLOC_LOG;
		sprintf(log,"Error: SaveToFile - failed creating file for writing: [%s]\n",pFileName);
		OutputDebugStringA(log);
		MessageBoxA(0,log,0,0);	
		return;
	}
	fwrite(this,sizeof(RS_Organism),1,f);
	fclose(f);
}

bool RS_Organism::LoadFromFile(const char* pFileName)
{
	FILE* f = fopen(pFileName,"rb");

	if (!f)
	{
		ALLOC_LOG;
		sprintf(log,"Error: LoadFromFile: failed creating file for reading: [%s]\n",pFileName);
		OutputDebugStringA(log);
		MessageBoxA(0,log,0,0);	
		return false;
	}

	fread(this,sizeof(RS_Organism),1,f);
	fclose(f);

	return true;
}

//RS_Organism

RandomSinusBrain::RandomSinusBrain()
{
	m_BestOrganism.Init();
	m_CurrentOrganism.Init();
	//m_dBestScore = -9999.0;
	m_dBestScore = 0.0;

	srand ( time(NULL) );
}

RandomSinusBrain::~RandomSinusBrain()
{

}

#define MAX_FORCE 350.0

#define RSB_GET_RAND_RANGE ((SFMT_FLOAT*MAX_FORCE)-(MAX_FORCE*0.5))



#define EVAL_TIMES 1

//#define SFMT_FLOAT (((float)rand()/(float)RAND_MAX))



//change into making sure that this is deterministic !!!

int RandomSinusBrain::GetRandomSeed()
{
	return m_CurrentOrganism.m_iRandomSeed;
}


void RandomSinusBrain::Evolve()
{
	char temp[100];
	int iGenerationsNum = 99999;
	double dScore = -9999.0;
	double dVelMax;

	//for (int i=0;i<iGenerationsNum;i++)
	int i=0;
	while(1)
	{

		g_Input.Update();

		

		if (g_Input.KeyPressed(DIK_L)) // debuggin !! - restore !!
		{
			OPENFILENAME ofn;       // common dialog box structure
			char szFile[260];       // buffer for file name
			//HANDLE hf;              // file handle

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

			BOOL bRet = GetOpenFileName(&ofn);

			m_BestOrganism.LoadFromFile(ofn.lpstrFile);
			ODS("Loaded winner from winner.org\n");
		}

		if (g_Input.KeyPressed(DIK_S))
		{
			m_BestOrganism.SaveToFile("./winner.org");
			ODS("saved winner into winner.org\n");
		}


		if (g_Input.KeyPressed(DIK_RETURN)) // RESTORE !!!
		{
			memcpy(&m_CurrentOrganism,&m_BestOrganism,sizeof(RS_Organism));
			return;
		}

		//m_CurrentOrganism.Init();
		memcpy(&m_CurrentOrganism,&m_BestOrganism,sizeof(RS_Organism));

		///////// add changed to the organism:
		// 1. Adding another random layer
		// 2. Changing the random seed
		// accumulate on top of last best organism
		
		m_CurrentOrganism.m_iRandomSeed =  GetTickCount();

		unsigned int uiBestOrganismNumLayers = m_BestOrganism.m_uiActiveWavesLayers;

		m_CurrentOrganism.m_uiActiveWavesLayers = uiBestOrganismNumLayers+1;

		//add texture to the floor

		//char debug[100];

		double dPhaseShift = SFMT_IN_RANGE(0.0,3.1415926535);

		for (int j=0;j<MOTORS_NUM;j++)
		{

			if (Ragdoll::m_iLinked[j]!=-1) //if linked
			{
				int iSource = Ragdoll::m_iLinked[j];
				for (int a=0;a<AXIS_NUM;a++)
				{
					/*sprintf(debug,"m_uiActiveWavesLayers=%d,j=%d,a=%d,iSource=%d\n",
						m_CurrentOrganism.m_uiActiveWavesLayers,
						j,
						a,
						iSource);
					ODS(debug);*/

					m_CurrentOrganism.layers[m_CurrentOrganism.m_uiActiveWavesLayers-1].waves[j][a].wave.SetParams(
						m_CurrentOrganism.layers[m_CurrentOrganism.m_uiActiveWavesLayers-1].waves[iSource][a].wave);

					m_CurrentOrganism.layers[m_CurrentOrganism.m_uiActiveWavesLayers-1].waves[j][a].wave.m_fPhase+=
						dPhaseShift;
						//SFMT_IN_RANGE(0.0,3.1415926535)

				}


				continue;
			}

			for (int a=0;a<AXIS_NUM;a++) // per axis
			{
				dVelMax = Ragdoll::GetAMotorMaxForce(j,a);

				//dVelMax = 150.0;

				m_CurrentOrganism.layers[m_CurrentOrganism.m_uiActiveWavesLayers-1].waves[j][a].wave.SetParams(
					(rand()%6)+1,  //wavetype
					//SFMT_IN_RANGE(-dVelMax, dVelMax), //base
					(SFMT_FLOAT*2.f)-1.0,
					SFMT_IN_RANGE(-dVelMax, dVelMax), //amplitude
					SFMT_FLOAT, // phase
					SFMT_FLOAT); // freq
			}	

		}

		dScore = 0.0;

		double scores[EVAL_TIMES];

		double min = 0.0;
	
		for (int i=0;i<EVAL_TIMES;i++)
		{
			scores[i] = m_evalFuncPtr(0,false,m_CurrentOrganism.m_iRandomSeed,EVALUATION_TIME_EVOLVE,NULL);

			if (i>0)
			{
				if (scores[i]!=scores[i-1])
				{
					//assert(0); // not deterministic !!
					__asm int 3; // not deterministic !!
				}

			}
			/*dScore+= scores[i];
			int k=0;
			k++;*/

			if (scores[i] < min)
			{
				min = scores[i];
			}
		}

		dScore = min;

		//dScore /= double(EVAL_TIMES);


		//dScore = m_evalFuncPtr(false);

		if (dScore > m_dBestScore) // if we found a better organism
		{
			sprintf(temp,"Generation %d Winner! %.4f > %.4f\n",i,dScore,m_dBestScore); // <-- this seems weird - using the i variable here.
			ODS(temp);
			memcpy(&m_BestOrganism,&m_CurrentOrganism,sizeof(RS_Organism));
			m_dBestScore = dScore;
			sprintf(temp,"New best score = %.3f\n",m_dBestScore);
			ODS(temp);

			ODS("Best organism description:\n");
			ODS("--------------------------\n");
			m_BestOrganism.Print();
			ODS("--------------------------\n");

			m_BestOrganism.SaveToFile("./curr_winner.org");

		} else
		{
			sprintf(temp,"Generation %d score %.4f (Winner is %.4f)\n",i,dScore,m_dBestScore);
			ODS(temp);
		}
		
		/*if (dScore > 1.40)
	{	
			memcpy(&m_CurrentOrganism,&m_BestOrganism,sizeof(RS_Organism));
			return;
		}*/

		i++;
	}

	memcpy(&m_CurrentOrganism,&m_BestOrganism,sizeof(RS_Organism));

}

void RandomSinusBrain::DecideMovement(IN float time, IN tBodyState& current,IN void* pPrivateData,OUT tMovement& movement )
{
	movement.Clear();

	for (int i=0;i<MOTORS_NUM;i++)
	{			
		//init to zero before accumulating begins
		for (int a=0;a<tMuscleCommand::MUSCLE_MOVE_AXIS_NUM;a++)
		{
			movement.muscle_commands[i].axis[a] = 0.0;
		}

		//accumulate the layers
		for (unsigned int x=0;x<m_CurrentOrganism.m_uiActiveWavesLayers;x++)
		{
			for (int a=0;a<tMuscleCommand::MUSCLE_MOVE_AXIS_NUM;a++)
			{			
				movement.muscle_commands[i].axis[a] += m_CurrentOrganism.layers[x].waves[i][a].wave.Compute(time);			
			}
		}
	}
}
