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
#ifndef _TRACKING_NOISE_
#define _TRACKING_NOISE_

 
namespace Tracking{

class Noise
{
public:

	//	"Polar" version without trigonometric calls
	float randn_notrig(float mu=0.0f, float sigma=1.0f);

	//	Standard version with trigonometric calls
	float randn_trig(float mu=0.0f, float sigma=1.0f);

	//	"Polar" version without trigonometric calls
	double randn_notrig(double mu=0.0, double sigma=1.0);

	//	Standard version with trigonometric calls
	double randn_trig(double mu=0.0, double sigma=1.0);

};

} 
 
#endif
