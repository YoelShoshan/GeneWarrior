#include "stdafx.h"
#include "GMath.h"
#include <stdio.h>


bool IsOdd(int iTest)
{	
	if ( (iTest%2)==1)
		return true;
	else
		return false;
}

CWave::CWave()
{
}

CWave::CWave(UINT uiWaveType,float fBase,float fAmplitude,float fPhase,float fFreq)
{
	SetParams(uiWaveType,fBase,fAmplitude,fPhase,fFreq);
}

void CWave::GetWaveTypeName(unsigned int wave_type, char* pFillMe, unsigned int fill_me_size)
{
	if (wave_type < WF_SIN || wave_type > WF_NOISE|| fill_me_size < 20)
	{
		sprintf(pFillMe,"error");
		return;
	}

	switch(wave_type)
	{

		case WF_SIN:
		sprintf(pFillMe,"SIN");
		break;

		case WF_TRIANGLE:
		sprintf(pFillMe,"TRIANGLE");
		break;

		case WF_SQUARE:
		sprintf(pFillMe,"SQUARE");
		break;

		case WF_SAWTOOTH:
		sprintf(pFillMe,"SAWTOOTH");
		break;

		case WF_INVERSESAWTOOTH:
		sprintf(pFillMe,"INVERSESAWTOOTH");
		break;

		case WF_NOISE:
		sprintf(pFillMe,"NOISE");
		break;

		default:
		DebugBreak(); // should never get here.
		break;
	};
}


CWave::~CWave()
{
	m_uiWaveType  = WF_SIN;
	m_fBase       = 0.5f;
	m_fAmplitude  = 0.f;
	m_fPhase      = 0.f;
	m_fFreq       = 0.f;
	
}

void CWave::SetParams(const CWave& other)
{
	m_uiWaveType  = other.m_uiWaveType;
	m_fBase       = other.m_fBase;
	m_fAmplitude  = other.m_fAmplitude;
	m_fPhase      = other.m_fPhase;
	m_fFreq       = other.m_fFreq;
}

void CWave::SetParams(UINT uiWaveType,float fBase,float fAmplitude,float fPhase,float fFreq)
{
	m_uiWaveType  = uiWaveType;
	m_fBase       = fBase;
	m_fAmplitude  = fAmplitude;
	m_fPhase      = fPhase;
	m_fFreq       = fFreq;
}

float CWave::Compute(float fTime)
{
	//// ONLY FOR DEBUG!!!!!!!!
	//fTime/=1000.f;
	//////////////////////////

	if (m_uiWaveType==WF_SIN)
		return ComputeSin(fTime);
	else
	if (m_uiWaveType==WF_TRIANGLE)
		return ComputeTriangle(fTime);
	else
	if (m_uiWaveType==WF_SQUARE)
		return ComputeSquare(fTime);
	else
	if (m_uiWaveType==WF_SAWTOOTH)
		return ComputeSawTooth(fTime);
	else
	if (m_uiWaveType==WF_INVERSESAWTOOTH)
		return ComputeInverseSawTooth(fTime);
	else
	if (m_uiWaveType==WF_NOISE) //sigh...
		return ComputeNoise(fTime);
	else
		return ComputeSin(fTime);
}

float CWave::ComputeSin(float fTime)
{
	fTime = m_fPhase + m_fFreq*fTime;
	//fTime -= floorf(fTime); //??? interesting - maybe i need to remove this for other sensors ???
	//fTime = sinf (fTime*TWOPI);
	fTime = sinf (fTime);
	fTime = m_fBase + m_fAmplitude*fTime;
	//printf("Time = %.3f\n",fTime);
	return fTime;
}


float CWave::ComputeTriangle(float fTime)
{
	fTime = m_fPhase + m_fFreq*fTime;
	fTime -= floorf(fTime);
	fTime = (fTime < 0.5f)? 2.0f*fTime : 2.0f-2.0f*fTime;
	return m_fBase + m_fAmplitude*fTime;
}
float CWave::ComputeSawTooth(float fTime)
{
	fTime = m_fPhase + m_fFreq*fTime;
	fTime -= floorf(fTime);
	return m_fBase + m_fAmplitude*fTime;
}
float CWave::ComputeInverseSawTooth(float fTime)
{
	fTime = m_fPhase + m_fFreq*fTime;
	fTime -= floorf(fTime);
	fTime=  1.0f-fTime;
	return m_fBase + m_fAmplitude*fTime;
}
float CWave::ComputeSquare(float fTime)
{
	fTime = m_fPhase + m_fFreq*fTime;
	fTime -= floorf(fTime);
	fTime= (fTime < 0.5) ? 1.0f : -1.0f;
	return m_fBase + m_fAmplitude*fTime;
}
float CWave::ComputeNoise(float fTime)
{
	fTime = m_fPhase + m_fFreq*fTime;
	fTime -= floorf(fTime);
	fTime=(float)rand()*(1.0f/(float)RAND_MAX);
	//fTime=(((float)rand()+(float)rand())/2.0f) / (float)RAND_MAX;

	return m_fBase + m_fAmplitude*fTime;
	////return fTime;
}

