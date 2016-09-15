#ifndef GENETIC_ALGO_BRAIN_H
#define GENETIC_ALGO_BRAIN_H

#pragma once

#include "Brain.h"
#include "GMath.h"
#include "stdio.h"
#include "time.h"

#include "RandomSinusBrain.h"

#include <vector>
#include <algorithm>

#define ORGANISM_EVALUATION_SIMULATION_TIME 12.0
//#define NEW_BLOOD_SCORE_THRESHOLD 0.01
#define NEW_BLOOD_SCORE_THRESHOLD 0.5

#define MIN_AGE 150
#define MAX_AGE 300

class GeneticAlgo_Brain: public Brain
{
public:
	GeneticAlgo_Brain();
	~GeneticAlgo_Brain();

	void DecideMovement(IN float time, IN tBodyState& current,IN void* pPrivateData,OUT tMovement& movement );

	void Evolve();

	int GetRandomSeed();

	bool LoadFromFile(const char* pcFileName);

	void PrintWinnerDebugInfo();

private:

	void CreateRandomOrganism(RS_Organism* pOrg);

	void CreateRandomOrganism_NonNegative(RS_Organism* pOrg);

	void EvaluateOrganismScore(RS_Organism* pOrg);

	void Flip();

	int AddLayer(RS_Organism* pOrg);

	static const int POPULATION_MAX = 50;

	int m_iPopulationSize;

	unsigned int m_uiGenerationNum;

	bool CheckForWinner(RS_Organism* pOrg);
	

	bool EvaluateCurrentGeneration();

	void CreateNextGeneration();

	void DecideAndMatePairs();

	void MatePair(int iSortedAndAlive, int iChildrenInsertStartPoint,int iChildNum);

	void Mate(const RS_Organism* pParent_1, const RS_Organism* pParent_2, RS_Organism* pChild);

	void AddRandomLayer(RS_Organism* pOrg);

	void RandomizeLayer(RS_Organism* pOrg, int iLayer);

	void VariateRandomLayer(RS_Organism* pOrg);

	void ToneDownLayer(RS_Organism* pOrg, int iLayer);

	void SortOrganisms();

	//bool CompareOrganismsFunc(int a, int b);


	// using two array, and fliping between them using pointer
	// each time one array is considered current generation, and children are created on the other one

	RS_Organism m_Population_1[POPULATION_MAX];
	RS_Organism m_Population_2[POPULATION_MAX];

	RS_Organism* m_pCurrent_Generation;
	RS_Organism* m_pNext_Generation;

	//unsigned int m_MatingPairs[POPULATION_MAX][2];

	//unsigned int m_SortedOrganisms[POPULATION_MAX];
	std::vector<RS_Organism*> m_SortedOrganisms;
	//RS_Organism* m_SortedOrganism[POPULATION_MAX];

	double m_dBestScore;
	RS_Organism m_BestOrganism;
};







#endif