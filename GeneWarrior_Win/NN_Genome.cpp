#include "stdafx.h"
#include "NN_Genome.h"
#include "NeuralNetwork.h"
#include <assert.h>
#include "..\sfmt\sfmt.h"

#define SFMT_FLOAT (((float)gen_rand32()/(float)MAXUINT))
#define SFMT_IN_RANGE(start,end) ((SFMT_FLOAT*((end)-(start)))+(start))
#define SFMT_IN_RANGE_UINT(start,end) (((gen_rand32())%((end)-(start)))+start)

NN_Genome::NN_Genome()
{
	m_iActiveGenomeCells = 0;
}

NN_Genome::~NN_Genome()
{

}

// read data from an organism
void NN_Genome::FromOrganism(const void* pOrg)
{
	assert(pOrg);
	NeuralNetwork* pNN = (NeuralNetwork*) pOrg;
	assert(pNN->m_iMagicVal == NeuralNetwork::MAGIC_VAL);

	m_iActiveGenomeCells = 0;
	
	for (int l=0; l<pNN->m_iHiddenLayersNum+1; l++)
	{
		for (int n=0;n<pNN->layers[l].m_iNeuronsNum;n++)
		{
			//copy all the weights of this neuron
			memcpy(&m_Genome[m_iActiveGenomeCells],
				pNN->layers[l].neurons[n].weights,
				sizeof(double)*pNN->layers[l].neurons[n].m_iInputsNum);

			m_iActiveGenomeCells+= pNN->layers[l].neurons[n].m_iInputsNum;
		}
	}

	assert(m_iActiveGenomeCells < MAX_GENOME_CELLS);
}

// fill data into an organism (organism assumed precreated)
void NN_Genome::ToOrganism(void* pOrg) const
{
	assert(pOrg);
	NeuralNetwork* pNN = (NeuralNetwork*) pOrg;
	assert(pNN->m_iMagicVal == NeuralNetwork::MAGIC_VAL);

	int iCurrentPos = 0;

	while (iCurrentPos < m_iActiveGenomeCells)
	{
		for (int l=0; l<pNN->m_iHiddenLayersNum+1; l++)
		{
			for (int n=0;n<pNN->layers[l].m_iNeuronsNum;n++)
			{
				//copy all the weights of this neuron
				memcpy(pNN->layers[l].neurons[n].weights,
					&m_Genome[iCurrentPos],
					sizeof(double)*pNN->layers[l].neurons[n].m_iInputsNum);

				iCurrentPos+= pNN->layers[l].neurons[n].m_iInputsNum;
			}
		}
	}
}

// mutate
void NN_Genome::Mutate()
{
	double dMutatePercent = 0.1; //mutate 0.1 of the genes

	int iMutateNum = (int)  ((double)m_iActiveGenomeCells *dMutatePercent);

	unsigned int uiSelected;

	for (int i=0;i<iMutateNum;i++)
	{
		uiSelected = gen_rand32()%m_iActiveGenomeCells;
		m_Genome[uiSelected] += SFMT_IN_RANGE(-1.0,1.0);
	}
}

//mate
void NN_Genome::CrossOverWith(const Genome* pSoulMate, Genome* pBaby1, Genome* pBaby2)
{
	assert(pSoulMate);

	assert(pBaby1 || pBaby2);

	//assert(pBaby1);
	//assert(pBaby2);

	//int iSplitAt = gen_rand32()%m_iActiveGenomeCells;

	NN_Genome* soulmate = (NN_Genome*)pSoulMate;		

	int iSplitAt = (gen_rand32()%(m_iActiveGenomeCells-2))+1; //[1,m_iActiveGenomeCells-1]
	
	if (pBaby1)
	{
		//baby1
		NN_Genome* baby1 = (NN_Genome*)pBaby1;
		memcpy(baby1->m_Genome, m_Genome, sizeof(double) * iSplitAt);
		memcpy(baby1->m_Genome + iSplitAt, soulmate->m_Genome + iSplitAt, sizeof(double) * (m_iActiveGenomeCells-iSplitAt));
	}

	if (pBaby2)
	{
		//baby2
		NN_Genome* baby2 = (NN_Genome*)pBaby2;
		memcpy(baby2->m_Genome, soulmate->m_Genome, sizeof(double) * iSplitAt);
		memcpy(baby2->m_Genome + iSplitAt, m_Genome + iSplitAt, sizeof(double) * (m_iActiveGenomeCells-iSplitAt));
	}
}
