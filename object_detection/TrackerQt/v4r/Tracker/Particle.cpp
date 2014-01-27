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
#include "Particle.h"

using namespace Tracking;
using namespace TomGine;

//Particle::Particle(float val){
//	t.x = val;
//	t.y = val;
//	t.z = val;
//	r.x = val;
//	r.y = val;
//	r.z = val;
//
//	z = val;
//
//	w = val;
//	c = val;
//
//	q = tgQuaternion();
//}

Particle::Particle(	vec3 t,
					vec3 r,
					float z,
					float w, float c,
					TomGine::tgQuaternion q)
{
	this->t = t;
	this->r = r;
	this->z = z;
	
	this->w = w;
	this->c = c;
	this->cc = c;
	this->ec = c;
	
	this->q = q;
}

Particle::Particle(const Particle& p2){
	t.x = p2.t.x;
	t.y = p2.t.y;
	t.z = p2.t.z;
	r.x = p2.r.x;
	r.y = p2.r.y;
	r.z = p2.r.z;

	z = p2.z;
	
	userData = p2.userData;
	
	w = p2.w;
	c = p2.c;
	cc = p2.cc;
	ec = p2.ec;
	
	q = p2.q;
}

Particle::Particle(const tgPose& p2){
	t = p2.t;
	r.x = 0.0;
	r.y = 0.0;
	r.z = 0.0;
	
	z = 0.0;	
	
	w = 0.0;
	c = 0.0;
	cc = 0.0;
	ec = 0.0;
	
	q = p2.q;
}

Particle& Particle::operator=(const Particle& p2){
	t.x = p2.t.x;
	t.y = p2.t.y;
	t.z = p2.t.z;
	r.x = p2.r.x;
	r.y = p2.r.y;
	r.z = p2.r.z;
	z = p2.z;
	
	userData = p2.userData;
	
	w = p2.w;
	c = p2.c;
	cc = p2.cc;
	ec = p2.ec;
	
	q = p2.q;
	
	return *this;
}

Particle& Particle::operator=(const tgPose& p2){
	t = p2.t;
	r.x = 0.0;
	r.y = 0.0;
	r.z = 0.0;
	
	z = 0.0;	
	
	w = 0.0;
	c = 0.0;
	cc = c;
	ec = c;
	
	q = p2.q;

	return *this;
}

Particle Particle::operator*(const float &f) const{
	Particle p;
	p.r  = r  * f;
	p.t  = t  * f;
	p.z  = z  * f;
	
	return p;
}

Particle Particle::operator+(const Particle &p) const{
	Particle res;
	res.t = t + p.t;
	res.q = q + p.q;
	res.r = r + p.r;
	res.z = z + p.z;
	res.w = w + p.w;
	res.c = c + p.c;
	return res;
}

Particle Particle::operator-(const Particle &p) const{
	Particle res;
	res.t = t - p.t;
	res.q = q - p.q;
	res.r = r - p.r;
	res.z = z - p.z;
	res.w = w - p.w;
	res.c = c - p.c;
	return res;
}
