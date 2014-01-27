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
 /**
 * @file Particle.h
 * @author Thomas Mörwald
 * @date January 2010
 * @version 0.1
 * @brief Defining a point/particle in the likelihood distribution
 * @namespace Tracker
 */
 
#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include "v4r/TomGine/tgPose.h"
#include "v4r/TomGine/tgMathlib.h"
#include "headers.h"		// stdio.h

namespace Tracking{

/**	@brief class Particle */
class Particle : public TomGine::tgPose
{
public:
	TomGine::vec3 r;				///< rotations
	float z;			///< scaling (zoom)
	
	std::vector<float> userData;
	
	double w;			///< weighted likelihood (sum of w of distribution = 1)
	float ec;			///< confidence level (matching pixels divided by overall pixels)
	float cc;
	float c;
	int em;
	int et;
	int cm;
	int ct;

//	Particle(float val=0.0);
	Particle(	TomGine::vec3 t=TomGine::vec3(), TomGine::vec3 r=TomGine::vec3(),
				float z=0.0f, float w=0.0f, float c=0.0f,
				TomGine::tgQuaternion q=TomGine::tgQuaternion());
	Particle(const Particle& p2);
	Particle(const TomGine::tgPose& p2);
	
	bool operator==(const Particle& p) const { return t==p.t && q==p.q; }
	Particle& operator=(const Particle& p2);
	Particle& operator=(const TomGine::tgPose& p);
	Particle operator*(const float& f) const;
	Particle operator+(const Particle& p) const;
	Particle operator-(const Particle& p) const;
	
	/**	@brief Comparing weighted likelihood of two particles	*/
	inline bool operator<(const Particle& p2) const { return w < p2.w; }
	inline bool operator>(const Particle& p2) const { return w > p2.w; }

};

} // namespace Tracking

#endif
