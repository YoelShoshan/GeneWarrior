#ifndef NN_GENOME_H
#define NN_GENOME_H

#include "Genome.h"

class NN_Genome: public Genome
{
public:
	NN_Genome();
	~NN_Genome();

	// read data from an organism
	void FromOrganism(const void* pOrg);

	// fill data into an organism (organism assumed preallocated)
	void ToOrganism(void* pOrg) const;

	// mutate
	void Mutate();

	//mate
	void CrossOverWith(const Genome* pSoulMate, Genome* pBaby1, Genome* pBaby2);

	int m_iActiveGenomeCells;
	static const int MAX_GENOME_CELLS = 900;
	double m_Genome[MAX_GENOME_CELLS];

};

#endif