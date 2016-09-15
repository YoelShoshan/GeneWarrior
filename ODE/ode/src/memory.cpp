/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>*/

#include <ode/odeconfig.h>
#include "config.h"
#include <ode/memory.h>
#include <ode/error.h>

//#include "..\..\ZedTracker\ZedTracker.h" 

static dAllocFunction *allocfn = 0;
static dReallocFunction *reallocfn = 0;
static dFreeFunction *freefn = 0;

#ifdef __MINGW32__
/* 
   this is a guard against AC_FUNC_MALLOC and AC_FUNC_REALLOC
   which break cross compilation, no issues in native MSYS.
*/
#undef malloc
#undef realloc
#endif

void dSetAllocHandler (dAllocFunction *fn)
{
  allocfn = fn;
}


void dSetReallocHandler (dReallocFunction *fn)
{
  reallocfn = fn;
}


void dSetFreeHandler (dFreeFunction *fn)
{
  freefn = fn;
}


dAllocFunction *dGetAllocHandler()
{
  return allocfn;
}


dReallocFunction *dGetReallocHandler()
{
  return reallocfn;
}


dFreeFunction *dGetFreeHandler()
{
  return freefn;
}

#include <windows.h>
#ifdef _DEBUG
#define DEBUG_MESSAGE_IF_NEEDED static int count=0;\
	if (count%5000 == 0) \
	{OutputDebugStringA("Warning - Using ODE Debug lib!\n");}\
	count++
#else
#define DEBUG_MESSAGE_IF_NEEDED
#endif

void * dAlloc (size_t size)
{
	void* res;

	DEBUG_MESSAGE_IF_NEEDED;
	
	if (allocfn) 
	{
		res = allocfn (size);
		if (!res)
		{
			__asm int 3;
		}

		return res; 
	}
	else 
	{
		if (size>99999 || size <= 0)
		{
			char text[100];
			sprintf(text,"Unreasonable allocation size requested: %d",size);
			MessageBoxA(0,text,0,0);
		}

		res =  malloc (size);
		if (!res)
		{
			char text[100];
			sprintf(text,"Malloc returned NULL for requested size=%d",size);
			MessageBoxA(0,text,0,0);
			__asm int 3;
		}

		return res;
	}

	// orig
	//if (allocfn) return allocfn (size); else return malloc (size);
}


void * dRealloc (void *ptr, size_t oldsize, size_t newsize)
{
  if (reallocfn) return reallocfn (ptr,oldsize,newsize);
  else return realloc (ptr,newsize);
}


void dFree (void *ptr, size_t size)
{
  if (!ptr) return;
  if (freefn) freefn (ptr,size); else free (ptr);
}
