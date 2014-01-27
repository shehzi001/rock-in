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

#include "myPredictor.h"

using namespace Tracking;


void myPredictor::sampleFromGaussian(Distribution& d, int num_particles, Particle mean, Particle variance, float sigma){
	Particle p;
	Particle epsilon;
	
	for(int i=0; i<num_particles; i++){
		
		p = mean;
		
		epsilon = genNoise(sigma, variance);
		
		p.Translate( m_cam_view.x * p.z, m_cam_view.y * p.z, m_cam_view.z * p.z);
		
		d.push_back(p);
		
		
	}	
}
