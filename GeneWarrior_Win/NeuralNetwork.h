#ifndef NEURAL_NETWORK_H
#define NEURAL_NETWORK_H

#include "NN_Genome.h"

//////////////////////////////////////////////////////////////////////////
// General notes about randomness
// 1. Every organism (ANN) contains seed that the Physics system used
// 2. For the evolution process (mating, variation, mutating etc.) the SFMT library is used.
// TODO: consider using SFMT for the physics as well (but don't let the physics random seeding 
// interfere with the evolution random seeding.
//////////////////////////////////////////////////////////////////////////

struct Neuron
{
	Neuron(int iInputsNum);
	Neuron();
	void Init(int iInputsNum, bool bRandomize);

	int m_iInputsNum;
	static const int MAX_INPUTS=30;
	double weights[MAX_INPUTS];
};

struct NeuronLayer
{	
	NeuronLayer();
	NeuronLayer(int iNeuronsNum, int iInputsPerNeuron);
	void Init(int iNeuronsNum, int iInputsPerNeuron, bool bRandomize);

	int m_iNeuronsNum;
	static const int MAX_NEURONS=30;
	Neuron neurons[MAX_NEURONS];
};

struct NeuralNetwork
{
public:
	int	m_iInputsNum;
	int	m_iOutputsNum;
	int m_iHiddenLayersNum;
	int m_iHiddenLayerNeuronsNum;
	static const int MAX_LAYERS=2;
	NeuronLayer layers[MAX_LAYERS];	
	inline double	  Sigmoid(double activation, double p);

	void AssignID()
	{
		static unsigned int S_NextID = 0;

		m_uiID = S_NextID;
		S_NextID++;
	}

	void Revive();
	NeuralNetwork();
	void Init(int iInputsNum, int iOutputsNum, int iHiddenLayersNum, int iHiddenLayerNeuronsNum, bool bRandomize);
	void Process(IN int iInputsNum,IN double* pInputs, OUT int iOutputsNum, OUT double* pOutputs);
	void SaveToFile(const char* pFileName);
	bool LoadFromFile(const char* pFileName);
	
	static const int MAGIC_VAL = 0x12345;	
	int m_iMagicVal; //to identify in case of void pointer.
	
	int m_iRandomSeed;
	double	m_dScore;
	unsigned int m_uiCalcluatedScoreForID;
	unsigned int m_uiAge;
	unsigned int m_uiDeathAge;
	unsigned int m_uiID;

	NN_Genome m_Genome;
};






#endif