/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2013, Thomas Mörwald
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author thomas.moerwald
 *
 */

#ifndef TIMER_H
#define TIMER_H
#pragma once

#ifdef _WIN32
	#include <windows.h>
#else
	#include <time.h>
	#include <sys/time.h>
#endif

// ***********************************************************************************

namespace Tracking{

class Timer
{
private:
#ifdef WIN32
	LONGLONG m_StartTicks;		// QueryPerformance - Ticks at application start
	LONGLONG m_EndTicks;		// QueryPerformance - Ticks when calling Now()
	LONGLONG m_Frequency;		// QueryPerformance - Fequency
	double fNow;
#else	
	struct timespec AppStart, act, old;
#endif
	double m_fAppTime;			// Time since application started
	double m_fTime;				// Time between two Update calls


public:
	Timer(void);
	~Timer(void);
	
	void	Reset();
	double	Update();
	
	double	GetFrameTime(){ return m_fTime;}
	double	GetApplicationTime(){ return m_fAppTime;}
	
};

}

#endif

