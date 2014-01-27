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
/** @file TrackerAPI.h
 * 
 * public interface for Tracker Library
 * attempts to minimise leakage of #define's and globals
 * 
 * @author	Sebastian Zurek (UoB)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _TRACKER_TRACKERAPI_H_
#define _TRACKER_TRACKERAPI_H_

#include "headers.h"
#include "Tracker.h"
#include "EdgeTracker.h"
#include "TextureTracker.h"
#include "Timer.h"

/* now clean up */

#undef GL_ORTHO
#undef GL_PERSPECTIVE
#undef NONE
#undef BARREL
#undef SOBEL_THRESHOLD
#undef THINNING_THRESHOLD
#undef SPREADING_THRESHOLD
#undef DISTANCE_SCALING
#undef SPREADING_LOOPS
#undef PI
#undef EPSILON
#undef DEG2RAD
#undef RAD2DEG
#undef FN_LEN
#undef myalloc
#undef GAUSS
#undef NORMAL
#undef FTOL
#undef PIOVER180
#undef g_Resources
#undef OFFSETOF
#undef FLOAT_NULL
#undef DISTLEN

#endif /*_TRACKER_TRACKERAPI_H_*/
