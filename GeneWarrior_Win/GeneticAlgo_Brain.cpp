#include "stdafx.h"
#include "GeneticAlgo_Brain.h"

#include "stdio.h"
#include "input.h"
#include <time.h>
#include <assert.h>
#include "Ragdoll.h"

#include <windows.h>

#include <Commdlg.h>

#include "..\sfmt\sfmt.h"

#include "crtdbg.h"

extern FILE* g_pLog;

extern CInput g_Input;
extern Ragdoll* g_pRagdoll;

//search for "choose sensor"

GeneticAlgo_Brain::GeneticAlgo_Brain()
{
	init_gen_rand(GetTickCount());	

	m_pCurrent_Generation = m_Population_1;
	m_pNext_Generation = m_Population_2;

	m_iPopulationSize = 0;	

	m_dBestScore = 0.0;
	m_uiGenerationNum = 0;
}

GeneticAlgo_Brain::~GeneticAlgo_Brain()
{
	

}

void GeneticAlgo_Brain::DecideMovement(IN float time, IN tBodyState& current,IN void* pPrivateData,OUT tMovement& movement )
{
	RS_Organism* pOrg = NULL;
	
	if (pPrivateData)
	{
		pOrg = (RS_Organism*) pPrivateData;
	}
	else // if pPrivateData is NULL, it means that we're on display mode, so we'll use the winner
	{
		pOrg = &m_BestOrganism;
	}
	
	movement.Clear();

	//double dPhaseShift = 1.0;//RAND_IN_RANGE(0.0,3.1415926535);
	//double dPhaseShift = 0.0;
	double FlipFactors[AXIS_NUM] = {-1.f,1.f,1.f};

	//char temp[100];

	double dBackupPhase = 0.0;
	//double sensor_value = -99999.0;

	for (int i=0;i<MOTORS_NUM;i++)
	{			
		//init to zero before accumulating begins
		for (int a=0;a<tMuscleCommand::MUSCLE_MOVE_AXIS_NUM;a++)
		{
			movement.muscle_commands[i].axis[a] = 0.0;
		}

		/*if (i==1 || i==2)
		{
			sprintf(temp,"\n%d Motor Start\n",i);
			ODS(temp);
		}*/

		//accumulate the layers
		for (unsigned int x=0;x<pOrg->m_uiActiveWavesLayers;x++)
		{
			for (int a=0;a<tMuscleCommand::MUSCLE_MOVE_AXIS_NUM;a++)
			{			
				if (Ragdoll::m_iLinked[i]!=-1)
				{
					//dBackupPhase = pOrg->waves[x][Ragdoll::m_iLinked[i]][a].m_fPhase;

					//pOrg->waves[x][Ragdoll::m_iLinked[i]][a].m_fPhase += pOrg->m_dShifts[i];

					//movement.muscle_commands[i].axis[a] += pOrg->waves[x][Ragdoll::m_iLinked[i]][a].Compute(time)*FlipFactors[a];

					float fShift = -(1.0/60.0)*5.0;
					
					fShift*= (float)pOrg->m_iShifts[i];
										
					float sensor_input = 0.0;

					SensorBasedWave* sensor_based_wave = &pOrg->layers[x].waves[Ragdoll::m_iLinked[i]][a];

					switch (sensor_based_wave->sensor_type)
					{ 
						case S_TIME:
							sensor_input = time;
							break;
						case S_ANGULAR_MOTOR_ANGULAR:
							sensor_input = current.dAngularMotorsAngles[sensor_based_wave->sensor_index][a];
							break;
						default:
							//Shouldn't reach here!
							DebugBreak();
					};

					float vel = sensor_based_wave->wave.Compute(sensor_input+fShift)*FlipFactors[a];

					/*if ((i==1 || i==2) && (a == 0))
					{
						sprintf(temp,"%.3f\t",vel);
						ODS(temp);
					}*/

					movement.muscle_commands[i].axis[a] += vel;

					//pOrg->waves[x][Ragdoll::m_iLinked[i]][a].m_fPhase = dBackupPhase;
				} else
				{

					float sensor_input = 0.0;

					SensorBasedWave* sensor_based_wave = &pOrg->layers[x].waves[i][a];

					switch (sensor_based_wave->sensor_type)
					{ 
						case S_TIME:
							sensor_input = time;
							break;
						case S_ANGULAR_MOTOR_ANGULAR:
							sensor_input = current.dAngularMotorsAngles[sensor_based_wave->sensor_index][a];
							break;
						default:
							//Shouldn't reach here!
							DebugBreak();
					};

					float vel = sensor_based_wave->wave.Compute(sensor_input);

					/*if ((i==1 || i==2) && (a == 0))
					{
						sprintf(temp,"%.3f\t",vel);
						ODS(temp);
					}*/

					movement.muscle_commands[i].axis[a] += vel;	
				}
			}
		}

		/*if (Ragdoll::m_iLinked[i]!=-1)
		{
			for (int a=0;a<tMuscleCommand::MUSCLE_MOVE_AXIS_NUM;a++)
			{
				movement.muscle_commands[i].axis[a] = movement.muscle_commands[Ragdoll::m_iLinked[i]].axis[a]*FlipFactors[a];
			}
		}*/
	}



}


bool GeneticAlgo_Brain::LoadFromFile(const char* pcFileName)
{
	bool bRet = m_BestOrganism.LoadFromFile(pcFileName);
	OUTPUT("Loaded winner from [%s]\n", pcFileName);

	return bRet;
}

void GeneticAlgo_Brain::PrintWinnerDebugInfo()
{
	m_BestOrganism.Print();
}

void GeneticAlgo_Brain::Evolve()
{
	RS_Organism* pOrg = NULL;

	m_uiGenerationNum = 0;
	char message[100];

	bool bContinue;

	while(1)
	{
		sprintf(message,"Generation %d\n",m_uiGenerationNum);
		ODS(message);

		//g_Input.Update();

		//if (g_Input.KeyPressed(DIK_L)) // debuggin !! - restore !!
		if (0)
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

		if (0)//(g_Input.KeyPressed(DIK_S))
		{
			m_BestOrganism.SaveToFile("./winner.org");
			ODS("saved winner into winner.org\n");
		}
		
		bContinue = EvaluateCurrentGeneration();

		if (!bContinue)
			return;

		CreateNextGeneration();

		Flip();
		
		m_uiGenerationNum++;

		//if (0)//(g_Input.KeyPressed(DIK_RETURN)) // RESTORE !!!
		if (g_Input.KeyPressed(DIK_RETURN)) // RESTORE !!!
		{
			return;
		}

	}
}


void GeneticAlgo_Brain::CreateNextGeneration()
{
	DecideAndMatePairs();
}


void GeneticAlgo_Brain::Mate(const RS_Organism* pParent_1, const RS_Organism* pParent_2, RS_Organism* pChild)
{
	const RS_Organism* pMainParent = NULL;
	const RS_Organism* pOtherParent = NULL;

	//ODS("a");

	/*if (pParent_1->m_uiAge > MAX_AGE || pParent_2->m_uiAge)
	{
		CreateRandomOrganism(pChild);
		return;
	}*/


	/*// start with the bigger parent
	if (pParent_1->m_uiActiveWavesLayers > pParent_2->m_uiActiveWavesLayers)
	{
		memcpy(pChild,pParent_1,sizeof(RS_Organism));
		pMainParent = pParent_1;
		pOtherParent = pParent_2;
	} else
	{
		memcpy(pChild,pParent_2,sizeof(RS_Organism));
		pMainParent = pParent_2;
		pOtherParent = pParent_1;
	}*/

	// start with the better score parent
	if (pParent_1->m_dScore > pParent_2->m_dScore)
	{
		memcpy(pChild,pParent_1,sizeof(RS_Organism));
		pMainParent = pParent_1;
		pOtherParent = pParent_2;
	} else
	{
		memcpy(pChild,pParent_2,sizeof(RS_Organism));
		pMainParent = pParent_2;
		pOtherParent = pParent_1;
	}


	pChild->AssignID();
	pChild->m_uiAge = 0;
	pChild->m_uiDeathAge = SFMT_IN_RANGE_UINT(MIN_AGE,MAX_AGE);

	/*if (gen_rand32()%2)
	{
		pChild->m_iRandomSeed = pMainParent->m_iRandomSeed;
	} else
	{
		pChild->m_iRandomSeed = pOtherParent->m_iRandomSeed;
	}*/


	int iRand = gen_rand32()%100;
	int iLayer_Child = -1;
	int iLayer_OtherParent = -1;


	//ODS("b");

	char message[100];

	/*sprintf(message,"Random=%d, parent ID=%d [score=%.3f], Child ID=%d\n ",
		iRand,
		pMainParent->m_uiID,
		pMainParent->m_dScore,
		pChild->m_uiID);
	ODS(message);*/

	if (iRand < 20) //only 
	{
		//ODS("-(VariateRandomLayer) ");
		ODS("1 ");
		VariateRandomLayer(pChild);
	} else if (iRand < 40) 
	{
		//ODS("-(AddRandomLayer) ");
		ODS("2 ");
		AddRandomLayer(pChild);		
	}  else if (iRand < 60) //replace child layer with other_parent layer
	{
		//ODS("-(from other to my layer) ");
		ODS("3 ");
		iLayer_Child = SFMT_IN_RANGE_UINT(0,pChild->m_uiActiveWavesLayers);
		iLayer_OtherParent = SFMT_IN_RANGE_UINT(0,pOtherParent->m_uiActiveWavesLayers);
		memcpy(&pChild->layers[iLayer_Child],&pOtherParent->layers[iLayer_OtherParent],sizeof(RS_Layer));
	} else if (iRand < 80)
	{		
		iLayer_Child = AddLayer(pChild);

		if (iLayer_Child < 0)
		{
			iLayer_Child = SFMT_IN_RANGE_UINT(0,pChild->m_uiActiveWavesLayers);
		}

		iLayer_OtherParent = SFMT_IN_RANGE_UINT(0,pOtherParent->m_uiActiveWavesLayers);

		/*sprintf(message,"((%d,%d))\n",iLayer_Child,iLayer_OtherParent);
		ODS(message);*/

		/*sprintf(message, "m_Population_1=0x%X, m_Population_2=0x%X ",
			m_Population_1, m_Population_2);
		ODS(message);

		sprintf(message, "m_pCurrent_Generation=[0x%X - 0x%X], m_pNext_Generation=[0x%X - 0x%X]\n",
			&m_pCurrent_Generation[0], ((char*)&m_pCurrent_Generation[0])-1+ sizeof(RS_Organism)*POPULATION_MAX,
			&m_pNext_Generation[0],((char*)&m_pNext_Generation[0])-1+ sizeof(RS_Organism)*POPULATION_MAX);
		ODS(message);

		sprintf(message, "pChild=0x%X, pOtherParent=0x%X\n",
			pChild, pOtherParent);
		ODS(message);

		sprintf(message, "&pChild->layers[iLayer_Child]=0x%X, &pOtherParent->layers[iLayer_OtherParent]=0x%X\n",
			&pChild->layers[iLayer_Child], &pOtherParent->layers[iLayer_OtherParent]);
		ODS(message);

		sprintf(message, "sizeof(RS_Layer)=0x%X, sizeof(RS_Organism)=0x%X\n",
			sizeof(RS_Layer),sizeof(RS_Organism));
		ODS(message);*/

		memcpy(&pChild->layers[iLayer_Child],&pOtherParent->layers[iLayer_OtherParent],sizeof(RS_Layer));

	} else //if (iRand < 101)
	{
		ODS("5 ");
		//ODS("-(Tone down) ");
		iLayer_Child = SFMT_IN_RANGE_UINT(0,pChild->m_uiActiveWavesLayers);
		ToneDownLayer(pChild,iLayer_Child);
	}

	//ODS("PI");

	EvaluateOrganismScore(pChild);

	//ODS("CAKE");
	
	sprintf(message,"[score=%.3f] ",
		pChild->m_dScore);
	ODS(message);

}

//bool myfunction (int i,int j) { return (i<j); }

bool CompareOrganismsFunc(RS_Organism* a, RS_Organism* b)
{
	return (a->m_dScore > b->m_dScore);
}

void GeneticAlgo_Brain::SortOrganisms()
{
	char message[100];

	//ODS("Died: ");

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

	//vector<double>::iterator 

	/*for (int i=0;i<POPULATION_SIZE;i++)
	{
		m_SortedOrganisms[i] = i;
	}*/

}

void GeneticAlgo_Brain::MatePair(int iSortedAndAlive,int iChildrenInsertStartPoint, int iChildNum)
{
	char message[100];
	
	int iChildIndex = iChildrenInsertStartPoint+iChildNum;

	if (iChildIndex < 0 ||
		iChildIndex > 49)
		DebugBreak();

	RS_Organism* pChild = &m_pNext_Generation[iChildIndex];

	

	bool bContinue = false;
	
	double dTemp;
	int iParent_1, iParent_2;
	do 
	{
		/*DWORD time = GetTickCount();
		srand(time);
		sprintf(message,"MatePair: srand(%d) test=%d\n", time,rand());
		ODS(message);*/

		UINT new_blood = SFMT_IN_RANGE_UINT(0,100);
		if (new_blood==0)
		{
			//pChild
			CreateRandomOrganism_NonNegative(pChild); //here
			//CreateRandomOrganism(pChild);
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

			Mate( m_SortedOrganisms[iParent_1],m_SortedOrganisms[iParent_2],pChild);		

			/*sprintf(message,"<size_after=%d>",m_SortedOrganisms.size());
			ODS(message);*/

			if ((pChild->m_dScore < m_SortedOrganisms[iParent_1]->m_dScore) &&
			(pChild->m_dScore < m_SortedOrganisms[iParent_2]->m_dScore)
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
		sprintf(message,"%d+%d passed ",m_SortedOrganisms[iParent_1]->m_uiID,m_SortedOrganisms[iParent_2]->m_uiID);
		ODS(message);
	}

}

void GeneticAlgo_Brain::DecideAndMatePairs()
{
	//sort based on results

	SortOrganisms();

/*	if (m_SortedOrganisms.size() == 0 ) // if extinct or start
	{
		ODS("Start or Extinct!!!\n");
		CreateRandomOrganism_NonNegative(&m_pCurrent_Generation[0]);
		SortOrganisms();
	}*/

	//random for now
	int iParent_1 = -1;
	int iParent_2 = -1;

	//double dTemp;


	char message[100];
	sprintf(message,"Best: [%d] ",m_SortedOrganisms.size());
	ODS(message);
	//debug print the best organisms in each generation
	//for (int i=0;i<POPULATION_SIZE;i++)
	for (std::vector<RS_Organism*>::iterator it = m_SortedOrganisms.begin(); it!=m_SortedOrganisms.end();++it)
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
		memcpy(&m_pNext_Generation[i],m_SortedOrganisms[i],sizeof(RS_Organism));

		sprintf(message,"(%d Age:%d/%d Score:%.3f) ",m_SortedOrganisms[i]->m_uiID,  m_SortedOrganisms[i]->m_uiAge,m_SortedOrganisms[i]->m_uiDeathAge,m_SortedOrganisms[i]->m_dScore);
		ODS(message);
		
		//if (m_pNext_Generation[i].m_uiAge > MAX_AGE)
		//{
		//	CreateRandomOrganism(&m_pNext_Generation[i]);
		//	return;
		//}
	}

	ODS("\n");

	// and the other half is result of mating
	
	if (iSortedAndAlive>0)
	{				
		int iPlaceForChildren = POPULATION_MAX-m_iPopulationSize;
		int iChildrenInsertStartPoint = m_iPopulationSize;

		sprintf(message,"Newborn: [%d] ",iPlaceForChildren);
		ODS(message);
		
		for (int i=0;i<iPlaceForChildren;i++)
		{
			MatePair(iSortedAndAlive,iChildrenInsertStartPoint,i);
			m_iPopulationSize++;
		}

		ODS("\n");
	}	

	int iFreePlaces = POPULATION_MAX-m_iPopulationSize;
	sprintf(message,"Free Places:[%d] ",iFreePlaces);
	ODS(message);

/*	int iCreateNew = iFreePlaces;

	if (iCreateNew > 10)
		iCreateNew = 10;*/

	for (int i=0;i<iFreePlaces;i++)
	{
		CreateRandomOrganism_NonNegative(&m_pNext_Generation[i]); //here
		//CreateRandomOrganism(&m_pNext_Generation[i]);
		m_iPopulationSize++;
	}

	ODS("\n");
}

bool GeneticAlgo_Brain::EvaluateCurrentGeneration()
{
	RS_Organism* pOrg = NULL;
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
			pOrg->m_dScore = m_evalFuncPtr((int)m_uiGenerationNum,false,pOrg->m_iRandomSeed,ORGANISM_EVALUATION_SIMULATION_TIME,pOrg);
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


bool GeneticAlgo_Brain::CheckForWinner(RS_Organism* pOrg)
{
	if (pOrg->m_dScore > m_dBestScore)
	{
		memcpy(&m_BestOrganism, pOrg, sizeof(RS_Organism));
		m_dBestScore = pOrg->m_dScore;

		char temp[256];
		sprintf(temp,"New Winner! [%.3f]\n",m_dBestScore);
		ODS(temp);

		ODS("*************************************************************************\n");
		m_BestOrganism.Print();
		ODS("*************************************************************************\n");

		static int winner_num = 0;
		char file_name[512];
		sprintf(file_name,"C:/GeneWarrior_WinnerSessions/session_winner_PID_%d_%04d_%.3f.org",GetCurrentProcessId(),winner_num,m_dBestScore);
		m_BestOrganism.SaveToFile(file_name);

		sprintf(file_name,"C:/GeneWarrior_WinnerSessions/session_winner_PID_%d_%04d_%.3f.org.txt",GetCurrentProcessId(),winner_num,m_dBestScore);
		m_BestOrganism.SaveToFile_Text(file_name);
		

		winner_num++;
		
		return true;
	}

	return false;
}

// note that i'm creating this even for non used (Ragdoll::m_iLinked should be checked, 
// if  there's a value that isn't -1, then i shouldn't calculate as this won't be used anyway

void GeneticAlgo_Brain::RandomizeLayer(RS_Organism* pOrg, int iLayer)
{
	assert((unsigned int) iLayer < pOrg->m_uiActiveWavesLayers);

	double dVelMax;
	unsigned int uiTemp;

	for (int j=0;j<MOTORS_NUM;j++)
	{
		if (Ragdoll::m_iLinked[j]!= -1 )
			continue;

		for (int a=0;a<AXIS_NUM;a++) // per axis
		{
			dVelMax = Ragdoll::GetAMotorMaxForce(j,a);

			pOrg->layers[iLayer].waves[j][a].wave.SetParams(
				//(gen_rand32()%WF_INVERSESAWTOOTH)+1,  //wavetype
				WF_SIN,
				(SFMT_FLOAT*2.f)-1.0,
				SFMT_IN_RANGE(-dVelMax, dVelMax), //amplitude
				SFMT_FLOAT, // phase
				//SFMT_FLOAT
				SFMT_IN_RANGE(0.0,2.0*PI)
				); // freq

			uiTemp = gen_rand32()%2;

			// choose sensor

			if (uiTemp==0)
			{
				pOrg->layers[iLayer].waves[j][a].sensor_type = S_TIME;
			} else
			{
				pOrg->layers[iLayer].waves[j][a].sensor_type = S_ANGULAR_MOTOR_ANGULAR;
				pOrg->layers[iLayer].waves[j][a].sensor_index = gen_rand32()%ANGULAR_MOTORS_NUM;
			}
		}	

	}

}

int GeneticAlgo_Brain::AddLayer(RS_Organism* pOrg)
{
	if (pOrg->m_uiActiveWavesLayers >= MAX_WAVES_LAYERS)
	{
		//no more place for new wave layers, aborting.
		//DebugBreak();
		return -1;
	}
	pOrg->m_uiActiveWavesLayers++;

	return pOrg->m_uiActiveWavesLayers-1;
}

void GeneticAlgo_Brain::ToneDownLayer(RS_Organism* pOrg, int iLayer)
{
	assert((unsigned int)iLayer < pOrg->m_uiActiveWavesLayers);

	for (int j=0;j<MOTORS_NUM;j++)
	{
		for (int a=0;a<AXIS_NUM;a++) // per axis
		{
			pOrg->layers[iLayer].waves[j][a].wave.m_fAmplitude /= SFMT_IN_RANGE(1.01,1.5);
		}
	}
}

void GeneticAlgo_Brain::VariateRandomLayer(RS_Organism* pOrg)
{
	//choose layer
	int iLayer = gen_rand32()%pOrg->m_uiActiveWavesLayers;

	RandomizeLayer(pOrg,iLayer);	

}

void GeneticAlgo_Brain::AddRandomLayer(RS_Organism* pOrg)
{
	if (pOrg->m_uiActiveWavesLayers >= MAX_WAVES_LAYERS)
	{
		//no more place for new wave layers, aborting.
		return;
	}

	pOrg->m_uiActiveWavesLayers++;

	RandomizeLayer(pOrg,pOrg->m_uiActiveWavesLayers-1);
}

void GeneticAlgo_Brain::EvaluateOrganismScore(RS_Organism* pOrg)
{
	if (pOrg->m_uiCalcluatedScoreForID != pOrg->m_uiID)
	{
		pOrg->m_dScore = m_evalFuncPtr(m_uiGenerationNum,false,pOrg->m_iRandomSeed,ORGANISM_EVALUATION_SIMULATION_TIME,pOrg);
		pOrg->m_uiCalcluatedScoreForID = pOrg->m_uiID;
	}
}

void GeneticAlgo_Brain::CreateRandomOrganism_NonNegative(RS_Organism* pOrg)
{
	char message[100];

	double dMax = -9999.0;
	int iTriesCount = 0;

	int iCount = 0;
	do 
	{
		if (iTriesCount%100==0)
		{
			ODS(".");
		}
		CreateRandomOrganism(pOrg);
		EvaluateOrganismScore(pOrg);

		if (pOrg->m_dScore > dMax)
		{
			dMax = pOrg->m_dScore;
			sprintf(message,"%.3f, ",dMax);
			ODS(message);
		}
		iTriesCount++;
		iCount++;

		if (iCount%500 == 0)
		{
			ODS(".");
		}

	//} while (pOrg->m_dScore <= 0.5); 
	} while (pOrg->m_dScore <= NEW_BLOOD_SCORE_THRESHOLD); 
	
	
	sprintf(message,"Created random organism - %.3f (ID=%d)\n",pOrg->m_dScore,pOrg->m_uiID);
	ODS(message);
}

void GeneticAlgo_Brain::CreateRandomOrganism(RS_Organism* pOrg)
{
	pOrg->Init();
	pOrg->m_iRandomSeed = GetTickCount();
	
	//char temp[100];
	
	for (int i=0;i<3;i++)
	{		
		AddRandomLayer(pOrg);	

		/*sprintf(temp,"Debug layer %d \n",i);
		ODS(temp);		
		pOrg->Print();*/
	}

	pOrg->m_uiDeathAge = SFMT_IN_RANGE_UINT(MIN_AGE,MAX_AGE);

	/*for (int i=0;i<MOTORS_NUM;i++)
	{
		if (Ragdoll::m_iLinked[i]!=-1)
		{
			pOrg->m_dShifts[i] = SFMT_IN_RANGE(0.0,3.14);
		}
	}*/
}

void GeneticAlgo_Brain::Flip()
{
	RS_Organism* temp = m_pCurrent_Generation;
	m_pCurrent_Generation = m_pNext_Generation;
	m_pNext_Generation = temp;	 	 
}

int GeneticAlgo_Brain::GetRandomSeed()
{
	return m_BestOrganism.m_iRandomSeed;
}