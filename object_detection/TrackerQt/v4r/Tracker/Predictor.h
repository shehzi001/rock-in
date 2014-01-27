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
* @file Predictor.h
* @author Thomas Mörwald
* @date January 2009
* @version 0.1
* @brief Prediction for motion of object to track
* @namespace Tracker
*/

#ifndef __PREDICTOR_H__
#define __PREDICTOR_H__

#include "headers.h"
#include "Distribution.h"
#include "v4r/TomGine/tgMathlib.h"
#include "Timer.h"

namespace Tracking{

#define GAUSS  0
#define NORMAL 1

/** @brief class Predictor */
class Predictor
{
protected:
	double m_dTime;
	float m_powTime;
	float m_powTimeSteps;
	float c_pred;
	
	float m_noConvergence;
	float m_keepParticles;

	TomGine::vec3 m_cam_view;
	
	float noise(float sigma, unsigned int type=GAUSS);
	Particle genNoise(float sigma, Particle pConstraint, unsigned int type=GAUSS);

public:
	Predictor();
	
	/** @brief Set vector pointing from camera to object mean, to enable zooming*/
	void setCamViewVector(TomGine::vec3 v){ m_cam_view = v; m_cam_view.normalize(); }
	
	/** @brief Set the number of particles (in percent) which are voting for no convergence (for capturing fast movement)*/
	void setNoConvergence(float v){ if(v>=0.0f && v<=1.0f) m_noConvergence = v; }
	
	/** @brief Set the number of best particles (in percent) which are not moved during resampling. */
	void setKeepParticles(float v){ if(v>=0.0f && v<=1.0f) m_keepParticles = v; }

	/** @brief Adds samples/particles to a distribution d by adding gaussian noise
	*		@param d particle distribution
	*		@param num_particles number of particles of resampled likelihood distribution
	*		@param mean mean particle of distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle)
	*		@param sigma standard variation of sampling (dependent on confidence value -> sigma(c)) */
	virtual void sampleFromGaussian(Distribution& d, int num_particles, Particle mean, Particle variance, float sigma=1.0);
	
	/** @brief Interface for Prediction/Motion systems
	*		@param poseIn actual pose
	*		@param poseOut predicted/moved pose
	*		@param c confidence about prediction/movement ( 1.0 = high confidence; 0.0 = no confidence ) */
	virtual void movePredicted(const TomGine::tgPose& poseIn, TomGine::tgPose& poseOut, float &c);
	
	/**	@brief Resample particles according to current likelihood distribution (move particles)
	*		@param d pointer to distribution
	*		@param num_particles number of particles of resampled likelihood distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle) */
	virtual void resample(Distribution& d, int num_particles, Particle variance, const int &iter, const int &max_iter);
	
	/** @brief Sample new distribution 
	*		@param d pointer to distribution
	*		@param num_particles number of particles to represent likelihood distribution
	*		@param mean mean particle of distribution
	*		@param variance variance of sampling in each degree of freedom (represented as particle) */
	virtual void sample(Distribution& d, int num_particles, Particle mean, Particle variance);
	
	/** @brief Updates time of prediction system for higher order motion model
	*		@param dTime Time since last frame */
	virtual void updateTime(double dTime);
	
};

} /* namespace Tracking */
 
 #endif
