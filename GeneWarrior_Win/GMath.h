#ifndef YOEL_GENERAL_MATH_H
#define YOEL_GENERAL_MATH_H

#pragma once

#include <Windows.h>
#include <math.h>


//#define PI (3.1415926535897932384626433832795028841971693993751)
#define PI    (3.14159265358979)
//#define TWOPI (6.283185307179586476925286766558)
#define TWOPI (6.28318530717958)
#define TINY_EPSILON    0.00000000001f
#define EPSILON         0.0078125f
#define GIANT_EPSILON   7.f
#define PIOVER180 (0.0174532925)
#define DEG2RAD( x ) ( (x) * PI / 180.0f )
#define RAD2DEG( x ) ( (180.f) * x /PI   )

//#define RANDOM_NUM        ((float)rand()/(RAND_MAX+1))

bool IsOdd(int iTest);

// Wave Functions

#define WF_NONE            0
#define WF_SIN             1
#define WF_TRIANGLE        2
#define WF_SQUARE          3
#define WF_SAWTOOTH        4
#define WF_INVERSESAWTOOTH 5
#define WF_NOISE           6

class CWave
{
public:
	CWave();
	CWave(UINT uiWaveType,float fBase,float fAmplitude,float fPhase,float fFreq);
	~CWave();

	static void GetWaveTypeName(unsigned int wave_type, char* pFillMe, unsigned int fill_me_size);

	void SetParams(UINT uiWaveType,float fBase,float fAmplitude,float fPhase,float fFreq);
	void SetParams(const CWave& other);

	float ComputeSin(float fTime);
	float ComputeTriangle(float time);
	float ComputeSquare(float time);
	float ComputeSawTooth(float time);
	float ComputeInverseSawTooth(float time);
	float ComputeNoise(float time);

	float Compute(float fTime);

	UINT  m_uiWaveType;
	float m_fBase;
	float m_fAmplitude;
	float m_fPhase;
	float m_fFreq;
};



#endif // YOEL_GENERAL_MATH_H