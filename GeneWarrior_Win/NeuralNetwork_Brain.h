#ifndef NEURAL_NETWORK_BRAIN_H
#define NEURAL_NETWORK_BRAIN_H

#include "Brain.h"
#include "NeuralNetwork.h"
#include <vector>

class NeuralNetwork_Brain : public Brain
{
public:

	NeuralNetwork_Brain();
	~NeuralNetwork_Brain();

	void DecideMovement(IN float time, IN tBodyState& current,IN void* pPrivateData,OUT tMovement& movement );
	
	int GetRandomSeed();

	bool LoadFromFile(const char* pcFileName);

	void PrintWinnerDebugInfo() {};

	void Evolve();
	
	void EvaluateOrganismScore(NeuralNetwork* pOrg);
	bool CheckForWinner(NeuralNetwork* pOrg);
	
	bool EvaluateCurrentGeneration();
	void SortOrganisms();
	void CreateNextGeneration();
	void MatePair(int iSortedAndAlive,int iChildrenInsertStartPoint, int iChildNum);
	void Flip();

	void RandomizeNeuralNetwork(NeuralNetwork* pNN);
	void SetNeutralNeuralNetwork(NeuralNetwork* pNN);

	static const int POPULATION_MAX = 50;

private:

	// using two array, and fliping between them using pointer
	// each time one array is considered current generation, and children are created on the other one

	NeuralNetwork m_Population_1[POPULATION_MAX];
	NeuralNetwork m_Population_2[POPULATION_MAX];

	NeuralNetwork* m_pCurrent_Generation;
	NeuralNetwork* m_pNext_Generation;

	std::vector<NeuralNetwork*> m_SortedOrganisms;

	NeuralNetwork m_CurrentNN;
	NeuralNetwork m_BestNN;

	int m_iPopulationSize; //actual population size
	int m_iGenerationNum;

	double m_dBestScore;
	double m_dSimulationTime;
	double m_dBasicScore; // the score when not signaling any actions.
};






#endif