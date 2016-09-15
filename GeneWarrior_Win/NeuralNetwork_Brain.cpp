#include "stdafx.h"
#include "NeuralNetwork_Brain.h"
#include "..\sfmt\sfmt.h"
#include <algorithm>
#include "math.h"
#include <assert.h>
#include <vector>
#include <algorithm>

using namespace std;

#define SFMT_FLOAT (((float)gen_rand32()/(float)MAXUINT))
#define SFMT_IN_RANGE(start,end) ((SFMT_FLOAT*((end)-(start)))+(start))
#define SFMT_IN_RANGE_UINT(start,end) (((gen_rand32())%((end)-(start)))+start)

extern FILE* g_pLog;

NeuralNetwork_Brain::NeuralNetwork_Brain()
{
	m_iGenerationNum = 0;
	m_dBestScore = -9999999.0;
	//m_dSimulationTime = 5.0;
	m_dSimulationTime = 8.0;

	init_gen_rand(GetTickCount());	

	//create_new_org
	//RandomizeNeuralNetwork(&m_CurrentNN);
	//memcpy(&m_BestNN,&m_CurrentNN,sizeof(NeuralNetwork));

	m_iPopulationSize = 0;

	m_pCurrent_Generation = m_Population_1;
	m_pNext_Generation = m_Population_2;
}

NeuralNetwork_Brain::~NeuralNetwork_Brain()
{
	
}

void NeuralNetwork_Brain::SetNeutralNeuralNetwork(NeuralNetwork* pNN)
{
	pNN->Init(
			(ANGULAR_MOTORS_NUM*AXIS_NUM)+2,//5, //time+sine+bias
			//RAGDOLL_BODIES_NUM+1,
			//2,
			ANGULAR_MOTORS_NUM*tMuscleCommand::MUSCLE_MOVE_AXIS_NUM,
			1,
			//2,
			15,
			false);		


}

//create_new_org
void NeuralNetwork_Brain::RandomizeNeuralNetwork(NeuralNetwork* pNN)
{
	assert(pNN);

	double dScore = -9999999.0;

	int iTriesCount = 0;

	//while (dScore <= 2.7)
	//while (dScore <= 0.8)
	while (dScore <= m_dBasicScore)
	//while (dScore <= 0.8)
	{
		if (iTriesCount%100==0)
		{
			ODS(".");
		}

		//pNN->Init(1+ANGULAR_MOTORS_NUM*AXIS_NUM, //time + all motors angles
		pNN->Init(
			(ANGULAR_MOTORS_NUM*AXIS_NUM)+2,//5, //time+sine+bias
			//RAGDOLL_BODIES_NUM+1,
			//2,
			ANGULAR_MOTORS_NUM*tMuscleCommand::MUSCLE_MOVE_AXIS_NUM,
			1,
			//2,
			15,
			true);		
		
		EvaluateOrganismScore(pNN);
		dScore = pNN->m_dScore;

		iTriesCount++;
	}
	char message[120];
	sprintf(message,"Created random organism - %.3f (ID=%d)\n",pNN->m_dScore,pNN->m_uiID);
	ODS(message);
	OutputDebugStringA(message);

	pNN->m_Genome.FromOrganism(pNN);	
}

void NeuralNetwork_Brain::DecideMovement(IN float time, IN tBodyState& current,IN void* pPrivateData,OUT tMovement& movement )
{
	double inputs[100/*+ANGULAR_MOTORS_NUM*AXIS_NUM*/]; //time+sine+bias
	double outputs[ANGULAR_MOTORS_NUM*tMuscleCommand::MUSCLE_MOVE_AXIS_NUM];

	NeuralNetwork* pNN = NULL;
	
	if (pPrivateData)
	{
		pNN = (NeuralNetwork*)pPrivateData;		
	} else
	{
		pNN = &m_BestNN;
	}

	/////////////////////////////////////////////////////////////////////
	//prepare inputs
	
	/*inputs[0] = sin(time*5.0);
	inputs[1] = sin(time*6.0);
	inputs[2] = sin(time*7.0);
	inputs[3] = sin(time*8.0);
	inputs[4] = -1.0; //for bias*/
	
	// angular motor angles
	int iInNum = 0;
	int m=0;
	for (m=0;m<ANGULAR_MOTORS_NUM;m++)
	{
		for (int a=0;a<AXIS_NUM;a++)
		{
			//inputs[iInNum] = current.dAngularMotorsAngles[m][a];

			inputs[iInNum] = current.dAngularMotorsAngles[m][a] / 10.0;

			iInNum++;
		}
	}

	inputs[(ANGULAR_MOTORS_NUM*AXIS_NUM)] = sin(time*8.0);
	inputs[(ANGULAR_MOTORS_NUM*AXIS_NUM)+1] = -1.0; //for bias
	/////////////////////////////////////////////////////////////////////

	// bodies linear velocity
	/*for (int i=0;i<RAGDOLL_BODIES_NUM;i++)
	{
		inputs[i] = current.dBodiesLinearVel[i];
	}
	inputs[RAGDOLL_BODIES_NUM] = -1.0; // bias*/

	/*double dAverageVel = 0.0;
	for (int i=0;i<RAGDOLL_BODIES_NUM;i++)
	{
		dAverageVel += current.dBodiesLinearVel[i];
	}
	dAverageVel/= (double)RAGDOLL_BODIES_NUM;

	inputs[0] = dAverageVel;
	inputs[1] = -1.0;*/


	pNN->Process(
		/*5*///(ANGULAR_MOTORS_NUM*AXIS_NUM)+1,
		//RAGDOLL_BODIES_NUM + 1,
		//2,
		(ANGULAR_MOTORS_NUM*AXIS_NUM)+2,
		inputs,
		ANGULAR_MOTORS_NUM*tMuscleCommand::MUSCLE_MOVE_AXIS_NUM,
		outputs);

	
	/////////////////////////////////////////////////////////////////////
	//fill outputs
	int iOutNum = 0;
	for (int m=0;m<ANGULAR_MOTORS_NUM;m++)
	{
		for (int a=0;a<tMuscleCommand::MUSCLE_MOVE_AXIS_NUM;a++)
		{
			movement.muscle_commands[m].axis[a] = outputs[iOutNum];
			iOutNum++;
		}
	}
	/////////////////////////////////////////////////////////////////////

	

}

void NeuralNetwork_Brain::Evolve()
{	

	// first of all, find what is the score when the brain doesn't signal anything.
	// This is our base score, and when we create completely random organisms, we demand to be slightly higher than this value.

	SetNeutralNeuralNetwork(&m_pCurrent_Generation[0]);
	EvaluateOrganismScore(&m_pCurrent_Generation[0]);
	m_dBasicScore = m_pCurrent_Generation[0].m_dScore;
	OUTPUT("Neutral organism got score=%f\n",m_dBasicScore);

	while (1)
	{
		/*m_CurrentNN.Init(1+ANGULAR_MOTORS_NUM*AXIS_NUM, //time + all motors angles
			ANGULAR_MOTORS_NUM*tMuscleCommand::MUSCLE_MOVE_AXIS_NUM,
			1,
			15);

		EvaluateOrganismScore(&m_CurrentNN);
		m_CurrentNN.m_Genome.FromOrganism(&m_CurrentNN);

		double dScore = m_CurrentNN.m_dScore;
		CheckForWinner(&m_CurrentNN);*/

		// evluate the score of each element in current generation (if wasn't evaluated yet)
		EvaluateCurrentGeneration();

		// sort current generation
		SortOrganisms();

		// create the next generation by mating and mutating
		CreateNextGeneration();

		// flip between current and previous generation
		Flip();
		
		m_iGenerationNum++;
	}

}

int NeuralNetwork_Brain::GetRandomSeed()
{
	return m_CurrentNN.m_iRandomSeed;
}

bool NeuralNetwork_Brain::LoadFromFile(const char* pcFileName)
{
	m_BestNN.LoadFromFile(pcFileName);
	//OUTPUT("Loaded winner from [%s]\n", pcFileName);

	return true;
}

void NeuralNetwork_Brain::EvaluateOrganismScore(NeuralNetwork* pOrg)
{
	/*
	// average of few scores	
	int iScoresNum = 10;
	vector<double> scores;
	double dScore;
	

	if (pOrg->m_uiCalcluatedScoreForID != pOrg->m_uiID)
	{
		for (int i=0;i<iScoresNum;i++)
		{
			dScore = m_evalFuncPtr(m_iGenerationNum,false,pOrg->m_iRandomSeed,m_dSimulationTime,pOrg);
			scores.push_back(dScore);
		}

		assert(iScoresNum == scores.size());

		sort(scores.begin(),scores.end()); 

		if (iScoresNum%2 == 0)
		{
			int iFirst = (iScoresNum/2)-1;
			int iSecond = iFirst+1;

			pOrg->m_dScore = (scores[iFirst] + scores[iSecond]) *0.5;			
		} else
		{
			int iMiddle = iScoresNum/2;

			pOrg->m_dScore = scores[iMiddle];
		}

		
		// simple average
		//pOrg->m_dScore = 0.0;
		//for (int i=0;i<iScoresNum;i++)
		//{
		//	pOrg->m_dScore+= dScores[i];
		//}
		//pOrg->m_dScore /= (double) iScoresNum;
		

		// find median


		pOrg->m_uiCalcluatedScoreForID = pOrg->m_uiID;
	}
	*/
	
	// single test
	double dScore = m_evalFuncPtr(m_iGenerationNum,false,pOrg->m_iRandomSeed,m_dSimulationTime,pOrg);
	pOrg->m_dScore = dScore;
	pOrg->m_uiCalcluatedScoreForID = pOrg->m_uiID;
}

bool NeuralNetwork_Brain::CheckForWinner(NeuralNetwork* pOrg)
{
	if (pOrg->m_dScore > m_dBestScore)
	{

		pOrg->m_uiDeathAge *= 2;


		memcpy(&m_BestNN, pOrg, sizeof(NeuralNetwork));
		m_dBestScore = pOrg->m_dScore;

		char temp[256];
		sprintf(temp,"New Winner! [%.3f]\n",m_dBestScore);
		ODS(temp);

		ODS("*************************************************************************\n");
		//m_BestNN.Print();
		ODS("*************************************************************************\n");

		static int winner_num = 0;
		char file_name[512];
		sprintf(file_name,"C:/GeneWarrior_WinnerSessions/session_winner_PID_%d_%04d_%.3f.org",GetCurrentProcessId(),winner_num,m_dBestScore);
		m_BestNN.SaveToFile(file_name);

		/*sprintf(file_name,"C:/GeneWarrior_WinnerSessions/session_winner_PID_%d_%04d_%.3f.org.txt",GetCurrentProcessId(),winner_num,m_dBestScore);
		m_BestNN.SaveToFile_Text(file_name);*/
		
		winner_num++;
		
		return true;
	}

	return false;
}

bool NeuralNetwork_Brain::EvaluateCurrentGeneration()
{
	NeuralNetwork* pOrg = NULL;
	for (int i=0;i<m_iPopulationSize;i++)
	{
		if (0) //(g_Input.KeyPressed(DIK_RETURN)) // RESTORE !!!
		{
			return false;
		}

		pOrg = &m_pCurrent_Generation[i];
		
		if (pOrg->m_uiCalcluatedScoreForID != pOrg->m_uiID)
		{
			//MessageBoxA(0,"EvaluateCurrentGeneration - shouldn't reach here!",0,0);
			pOrg->m_dScore = m_evalFuncPtr(m_iGenerationNum,false,pOrg->m_iRandomSeed,m_dSimulationTime,pOrg);
			pOrg->m_uiCalcluatedScoreForID = pOrg->m_uiID;
		}	else
		{
			int temp = 0;
			temp++;
		}

		CheckForWinner(pOrg);

		pOrg->m_uiAge++;

	}

	return true;
}

bool CompareOrganismsFunc(NeuralNetwork* a, NeuralNetwork* b)
{
	return (a->m_dScore > b->m_dScore);
}

void NeuralNetwork_Brain::SortOrganisms()
{
	char message[100];

	//
	bool bPrintedDied=false;

	m_SortedOrganisms.clear();
	for (int i=0;i<m_iPopulationSize;i++)
	{
		if ( (m_pCurrent_Generation[i].m_uiAge < m_pCurrent_Generation[i].m_uiDeathAge)	)
		{
			m_SortedOrganisms.push_back(&m_pCurrent_Generation[i]);
		} else
		{
			if (!bPrintedDied)
			{
				ODS("Died: ");
				bPrintedDied = true;
			}

			sprintf(message,"%d ",m_pCurrent_Generation[i].m_uiID);
			ODS(message);			
		}
	}

	ODS("\n");

	sort(m_SortedOrganisms.begin(),m_SortedOrganisms.end(),CompareOrganismsFunc);
}

void NeuralNetwork_Brain::CreateNextGeneration()
{
	//random for now
	int iParent_1 = -1;
	int iParent_2 = -1;

	//double dTemp;


	char message[100];
	sprintf(message,"Best: [%d] ",m_SortedOrganisms.size());
	ODS(message);
	//debug print the best organisms in each generation
	//for (int i=0;i<POPULATION_SIZE;i++)
	for (std::vector<NeuralNetwork*>::iterator it = m_SortedOrganisms.begin(); it!=m_SortedOrganisms.end();++it)
	{
		sprintf(message,"%.3f, ",(*it)->m_dScore);
		ODS(message);		
	}
	ODS("\n");		

	//half of the generation continues

	int iSortedAndAlive = m_SortedOrganisms.size();

	m_iPopulationSize = iSortedAndAlive;

	sprintf(message,"Lived: [%d]", iSortedAndAlive);
	ODS(message);
	for (int i=0;i<iSortedAndAlive;i++)
	{
		memcpy(&m_pNext_Generation[i],m_SortedOrganisms[i],sizeof(NeuralNetwork));

		sprintf(message,"(%d Age:%d/%d Score:%.3f) ",m_SortedOrganisms[i]->m_uiID,  m_SortedOrganisms[i]->m_uiAge,m_SortedOrganisms[i]->m_uiDeathAge,m_SortedOrganisms[i]->m_dScore);
		ODS(message);		
	}

	ODS("\n");

	// and the other half is result of mating
	
	if (iSortedAndAlive>0)
	{				
		int iPlaceForChildren = POPULATION_MAX-m_iPopulationSize;
		int iChildrenInsertStartPoint = m_iPopulationSize;

		if (iPlaceForChildren > 0)
		{
			sprintf(message,"Newborn: [%d] ",iPlaceForChildren);
			ODS(message);
			
			for (int i=0;i<iPlaceForChildren;i++)
			{
				MatePair(iSortedAndAlive,iChildrenInsertStartPoint,i);					
				m_iPopulationSize++;
			}
		}

		ODS("\n");
	}	

	int iFreePlaces = POPULATION_MAX-m_iPopulationSize;
	sprintf(message,"Free Places:[%d] ",iFreePlaces);
	ODS(message);

	for (int i=0;i<iFreePlaces;i++)
	{
		//create_new_org
		RandomizeNeuralNetwork(&m_pNext_Generation[i]);
		m_iPopulationSize++;
	}

	ODS("\n");
}

void NeuralNetwork_Brain::MatePair(int iSortedAndAlive,int iChildrenInsertStartPoint, int iChildNum)
{
	char message[100];
	
	int iChildIndex = iChildrenInsertStartPoint+iChildNum;

	if (iChildIndex < 0 ||
		iChildIndex > 49)
		DebugBreak();

	NeuralNetwork* pChild = &m_pNext_Generation[iChildIndex];

	bool bContinue = false;
	
	double dTemp;
	int iParent_1, iParent_2;
	do 
	{
		UINT new_blood = SFMT_IN_RANGE_UINT(0,100);
		if (new_blood==0)
		{
			//pChild
			//CreateRandomOrganism_NonNegative(pChild); //here
			//CreateRandomOrganism(pChild);

			RandomizeNeuralNetwork(pChild);

			bContinue = false;

			iParent_1 = -1;
			iParent_2 = -1;

		} else
		{
			dTemp = (double)SFMT_IN_RANGE_UINT(0,iSortedAndAlive*iSortedAndAlive);
			dTemp = sqrt(dTemp);
			iParent_1 = (int) dTemp;
			iParent_1 = iSortedAndAlive-iParent_1;
			if (iParent_1<0)
				iParent_1 = 0;
			if (iParent_1>iSortedAndAlive-1)
				iParent_1 = iSortedAndAlive-1;

			dTemp = (double)SFMT_IN_RANGE_UINT(0,iSortedAndAlive*iSortedAndAlive);
			dTemp = sqrt(dTemp);
			iParent_2 = (int) dTemp;
			iParent_2 = iSortedAndAlive-iParent_2;
			if (iParent_2<0)
				iParent_2 = 0;
			if (iParent_2>iSortedAndAlive-1)
				iParent_2 = iSortedAndAlive-1;

			/*sprintf(message,"trying %d+%d ",iParent_1,iParent_2);
			ODS(message);

			sprintf(message,"(id= %d+%d ",m_SortedOrganisms[iParent_1]->m_uiID,m_SortedOrganisms[iParent_2]->m_uiID);
			ODS(message);
			
			sprintf(message,"<size_bef=%d>",m_SortedOrganisms.size());
			ODS(message);*/

			//Mate( m_SortedOrganisms[iParent_1],m_SortedOrganisms[iParent_2],pChild);		

			NeuralNetwork* pPar1 = m_SortedOrganisms[iParent_1];
			NeuralNetwork* pPar2 = m_SortedOrganisms[iParent_2];

 
			//create_new_org
			if (gen_rand32()%2)
			{
				pPar1->m_Genome.CrossOverWith(&pPar2->m_Genome,&pChild->m_Genome,NULL);
				pChild->m_Genome.Mutate();
				pChild->Revive();
				pChild->m_Genome.ToOrganism(pChild);
				
			} else
			{
				pPar1->m_Genome.CrossOverWith(&pPar2->m_Genome,NULL,&pChild->m_Genome);
				pChild->m_Genome.Mutate();
				pChild->Revive();
				pChild->m_Genome.ToOrganism(pChild);
			}

			if (gen_rand32()%2)
			{
				pChild->m_iRandomSeed = pPar1->m_iRandomSeed;
			} else
			{
				pChild->m_iRandomSeed = pPar2->m_iRandomSeed;
			}

			EvaluateOrganismScore(pChild);
		
			/*sprintf(message,"<size_after=%d>",m_SortedOrganisms.size());
			ODS(message);*/
			
			//don't spawn child if it wasn't better
			// - rethink this.
			
			if ((pChild->m_dScore < pPar1->m_dScore) &&
			(pChild->m_dScore < pPar2->m_dScore)
			)
				bContinue = true;
			else
				bContinue = false;

		}
		
	} while( 
		bContinue 		 		
		);

	if ( (iParent_1 == -1) && 
		(iParent_2 == -1))
	{
		ODS("No parents used.\n");
	} else
	{
		sprintf(message,"%d+%d=%d passed ",m_SortedOrganisms[iParent_1]->m_uiID,m_SortedOrganisms[iParent_2]->m_uiID, pChild->m_uiID);
		ODS(message);
	}

}

void NeuralNetwork_Brain::Flip()
{
	NeuralNetwork* temp = m_pCurrent_Generation;
	m_pCurrent_Generation = m_pNext_Generation;
	m_pNext_Generation = temp;	 	
}