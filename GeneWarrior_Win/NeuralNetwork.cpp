#include "stdafx.h"
#include "NeuralNetwork.h"
#include <assert.h>
#include "..\sfmt\sfmt.h"
#include "GMath.h"
#include "Defines.h"

//TODO: fix it to return double

#define SFMT_FLOAT (((float)gen_rand32()/(float)MAXUINT))
#define SFMT_IN_RANGE(start,end) ((SFMT_FLOAT*((end)-(start)))+(start))
#define SFMT_IN_RANGE_UINT(start,end) (((gen_rand32())%((end)-(start)))+start)



//the plus one is for the neuron the simulates the bias.
// since it's Sigma(weight * Input) > t
// it can be made
// w1*i2 + ... wN*iN + wN+1*(-1.0) > 0
Neuron::Neuron(int iInputsNum)
{
	Init(iInputsNum,true);
}

void Neuron::Init(int iInputsNum, bool bRandomize)
{
	assert(iInputsNum > 0);
	assert(iInputsNum < MAX_INPUTS);
	
	m_iInputsNum = iInputsNum;
	for (int i=0;i<m_iInputsNum;i++)
	{
		if (bRandomize)
		{
			weights[i] = SFMT_IN_RANGE(-1.0,1.0);
		} else
		{
			weights[i] = 0.0;
		}
	}
}

Neuron::Neuron() : m_iInputsNum(0)
{
	
}

NeuronLayer::NeuronLayer()
{
	m_iNeuronsNum = 0;

}

/*NeuronLayer::NeuronLayer(int iNeuronsNum, int iInputsPerNeuron)
{
	Init(iNeuronsNum,iInputsPerNeuron);	
}*/

void NeuronLayer::Init(int iNeuronsNum, int iInputsPerNeuron, bool bRandomize)
{
	assert(iNeuronsNum > 0);
	assert(iNeuronsNum < MAX_NEURONS);
	m_iNeuronsNum = iNeuronsNum;
	
	for (int i=0;i<m_iNeuronsNum;i++)
	{
		neurons[i].Init(iInputsPerNeuron,bRandomize);
	}
}

NeuralNetwork::NeuralNetwork() : m_iInputsNum(0), m_iOutputsNum(0), m_iHiddenLayersNum(0), m_iHiddenLayerNeuronsNum(0), m_iMagicVal(MAGIC_VAL)
{

}

void NeuralNetwork::Revive()
{
	AssignID();
	m_uiAge = 0;
	m_uiDeathAge = SFMT_IN_RANGE_UINT(150,300);
	m_uiCalcluatedScoreForID = (unsigned int)-1;
}


void NeuralNetwork::Init(int iInputsNum, int iOutputsNum, int iHiddenLayersNum, int iHiddenLayerNeuronsNum,bool bRandomize)  
{
	m_iInputsNum = iInputsNum;
	m_iOutputsNum = iOutputsNum;
	m_iHiddenLayersNum = iHiddenLayersNum;
	m_iHiddenLayerNeuronsNum = iHiddenLayerNeuronsNum;

	assert(m_iInputsNum>0);
	assert(m_iOutputsNum>0);
	assert(m_iHiddenLayersNum>0);
	assert(m_iHiddenLayersNum <= MAX_LAYERS-1);
	assert(m_iHiddenLayerNeuronsNum>0);

	Revive();

	m_iRandomSeed =  GetTickCount();
	
	//create the first hidden layer, which is fed from the starting input
	layers[0].Init(m_iHiddenLayerNeuronsNum,m_iInputsNum,bRandomize);

	//for any further hidden layer, the input is the previous hidden layer
	for (int i=0;i<m_iHiddenLayersNum-1;i++)
	{
		layers[i+1].Init(m_iHiddenLayerNeuronsNum,m_iHiddenLayerNeuronsNum,bRandomize);
	}

	//finally, create the output layer
	layers[m_iHiddenLayersNum].Init(m_iOutputsNum,m_iHiddenLayerNeuronsNum,bRandomize);
}

 double NeuralNetwork::Sigmoid(double activation, double p)
 {
	 return ( 1 / ( 1 + exp(-activation / p)));
 }

///////////////////////////////////////////////////////////
// note: always remember to pass an extra input for bias !
 //////////////////////////////////////////////////////////

 void NeuralNetwork::Process(IN int iInputsNum,IN  double* pInputs, OUT int iOutputsNum, OUT double* pOutputs)
 {
	 assert(iInputsNum == m_iInputsNum);
	 assert(pInputs);
	 assert(iOutputsNum == m_iOutputsNum);
	 assert(pOutputs);
	 
	 //fixme: 500 as max value here
	 double neuron_layer_output_vals[200];

	 int iCurrInputsNum;
	 double sigma;

	 double* pOrigInputs = pInputs;

	 double dBias = -1.0;
	 double dActivationResponse = 1.0;

	 // for each layer
	 for (int l=0;l<m_iHiddenLayersNum+1;l++)
	 {

		 //for each neuron 
		 for (int n=0;n<layers[l].m_iNeuronsNum;n++)
		 {
			 iCurrInputsNum = layers[l].neurons[n].m_iInputsNum;

			 sigma = 0.0;

			 //for each weight (except from the bias one)
			 for (int w=0;w<iCurrInputsNum-1;w++)
			 {
				 sigma += layers[l].neurons[n].weights[w] * pInputs[w];
			 }

			 //bias add
			 sigma += layers[l].neurons[n].weights[iCurrInputsNum-1] * dBias;

			 neuron_layer_output_vals[n] = Sigmoid(sigma,dActivationResponse);
		 }

		 //current output is input for the next layer
		 pInputs = &neuron_layer_output_vals[0];
	 }

	 memcpy(pOutputs,pInputs,sizeof(double)*iOutputsNum);

	 //increase output range

	 for (int i=0;i<iOutputsNum;i++)
	 {
		 //[0,1] -> [-0.5,0.5] -> [-X,X]
		 //pOutputs[i] = (pOutputs[i] - 0.5) * 15.0;

		 pOutputs[i] = (pOutputs[i] - 0.5) * 5.0;
	 }
	
	 
 }


void NeuralNetwork::SaveToFile(const char* pFileName)
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
	fwrite(this,sizeof(NeuralNetwork),1,f);
	fclose(f);
}

bool NeuralNetwork::LoadFromFile(const char* pFileName)
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

	fread(this,sizeof(NeuralNetwork),1,f);
	fclose(f);

	return true;
}