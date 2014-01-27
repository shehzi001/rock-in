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
#ifndef DISTRIBUTION_H
#define DISTRIBUTION_H

#include "Particle.h"
#include "v4r/TomGine/tgQuaternion.h"
#include "v4r/TomGine/tgMathlib.h"
#include "Resources.h"
#include "Timer.h"
#include "headers.h"  // stdio.h, stdlib.h, algorithm, gl.h, vector
#include "v4r/TomGine/tgMathlib.h"

namespace Tracking{

enum Method{
	EDGE=0,
	COLOR=1,
	EDGECOLOR=2,
};

/** @brief typedef ParticleList */
typedef std::vector<Particle> ParticleList;

/**	@brief class Distribution */
class Distribution
{
private:
	
	ParticleList m_particlelist;			///< List of particles forming the likelihood distribution
	std::vector<unsigned int> queryMatches;	///< OpenGL Occlussion Query for counting matching pixels
	std::vector<unsigned int> queryEdges;	///< OpenGL Occlussion Query for counting overall edge pixels
	Particle m_meanParticle;				///< Particle representing weighted mean of particle distribution
	Particle m_maxParticle;					///< Particle with highest weight/confidence of the particle distribution
//	int v_max;								///< Maximum number of  overall edge pixels of current view
	double w_sum;							///< Sum of all weights, before normalisation (afterwards its 1)
	float c_median;
	TomGine::vec3 m_cam_view;

	// Functions
	
	/** @brief Resize Occlusion Queries on GPU */
	void updateQueries();
	void drawParticlesEdges(TrackerModel& model, TomGine::tgShader* shadeCompare, bool showparticles=false);
	void drawParticlesTextured(TrackerModel& model, TomGine::tgShader* shadeCompare, bool showparticles=false);
	void calcEdgeConfidence(int convergence=5);
	void calcColorConfidence(int convergence=5);

public:
	double n_eff;             ///< effective sample size (Doucet et al. 2000)
	double n_eff_max;
	int m_numStaticParticles;				///< number of static particles (particles that remained the same since the last frame)
	int m_numStaticParticlesUnion;
	float m_meanWeightOfStaticParticles;	///< mean weight of static particles, normalized to sum of weight of kept particles
	Particle m_meanKeptParticle;			///< mean particle of set of kept particles
	std::vector<Particle> m_keptParticles;	///< set of kept particles


	/** @brief Calculates the mean of the distribution wrt. weigths. */
	Particle calcWeightedMean(ParticleList &pl);

	/** @brief Calculates the mean of the distribution. */
	Particle calcMean(const ParticleList &pl);

	/** @brief Calculates the median of the distribution wrt. weights. */
	float calcMedianC(const ParticleList &pl);

public:
	~Distribution();

	/**	@brief Gets weighted mean of likelihood distribution */
	Particle  getMean(){ return m_meanParticle; }
	
	/**	@brief Gets particle with max confidence of likelihood distribution */
	Particle getMax(){ return m_maxParticle; }

	/** @brief Gets particle from distribution by id */
	Particle& getParticle(int id){ return (m_particlelist[id]); }
	
	/** @brief Gets particle from distribution by id */
  const Particle& operator[](const int &id){ return m_particlelist[id]; }

	/** @brief Calculates variance confidence level of particle distribution */
	double getVarianceC();
	
	/** @brief Calculates variance of the position of particle distribution */
	double getPositionVariance();
	
	/** @brief Calculates variance of the orientation of particle distribution */
	double getOrientationVariance();

		/** @brief Get number of kept particles that changed since last call of resample. */
	int getNumStaticParticles(){ return m_numStaticParticles; }

	/** @brief Get mean of weights of kept particles */
	float getMeanWeightOfStaticParticles(){ return m_meanWeightOfStaticParticles; }

	Particle getMeanOfKeptParticles(){ return m_meanKeptParticle; }

	/**	@brief Recalculates weights of particlelist to sum up to 1 */
	void normalizeWeights(ParticleList &pl);

	/** @brief Gets median of confidences of particle distribution */
	float getMedianC(){ return c_median; }

	/** @brief Gets max of confidences of particle distribution */
	float getMaxC(){ return m_maxParticle.c; }
	float getMaxCC(){ return m_maxParticle.cc; }
	float getMaxEC(){ return m_maxParticle.ec; }

	/** @brief Gets mean of confidences of particle distribution */
	float getMeanC(){ return m_meanParticle.c; }
	float getMeanCC(){ return m_meanParticle.cc; }
	float getMeanEC(){ return m_meanParticle.ec; }

	float getMeanKeptParticleC(){ return m_meanKeptParticle.c; }
	float getMeanKeptParticleCC(){ return m_meanKeptParticle.cc; }
	float getMeanKeptParticleEC(){ return m_meanKeptParticle.ec; }

	int getUnionKeptParticles(){ return m_numStaticParticlesUnion; }

	/** @brief Get effective sample size */
	float getNeff(){ return n_eff; }

	/** @brief Gets weighted likelihood of a particle by id */
	float getW(int id){ return m_particlelist[id].w; }
	
	/** @brief gets confidence level of a particle by id */
	float getC(int id){ return m_particlelist[id].c; }
	
	/** @brief Number of particles representing the distribution*/
	size_t size(){ return m_particlelist.size(); }
	
	/** @brief Copy ParticleList representing distribution std::vector<Particle> */
	void copy(ParticleList &list){ list = m_particlelist; }
	
	/** @brief Clears all particles from the ParticleList */
	void clear(){ m_particlelist.clear(); }
	
	/** @brief Adds a particle to the ParticleList of the distribution */
	void push_back(Particle p){ m_particlelist.push_back(p); }
	
	/** @brief Functional for confidence value 
	*	@param m Number of matching pixels
	*	@param e Number of edge pixels of model in curren view 
	*	@return Confidence functional */
	float confidenceFunction(const float &m, const float &e, const float& e_max);
	
	/** @brief Update confidence levels and weighted likelihoods of distribution 
	*		@param TrackerModel 3D model to be rendered and compared to camera image
	*		@param TomGine::tgShader comparison shader which draws only pixels matching with the corresponding pixel of the camera image
	*		@param textured flag whether to use textured models or edge models (TextureTracker / EdgeTracker)
	*		@param convergence convergence of weight distribution
	*		@param showparticles makes particle distribution visible on screen*/
	void updateLikelihood(	TrackerModel& model,
							TomGine::tgShader* shEdge, TomGine::tgShader* shColor, Method method,
							bool textured, bool fulltextured, int convergence=5, bool showparticles=false);
	
	/** @brief Update confidence levels using ImageProcessor::Avg functions (fast summations). (under construction) */
	void updateLikelihood(TrackerModel& model, TomGine::tgShader* shadeCompare, TomGine::tgCamera* cam, TomGine::tgTexture2D* tex_edge, int res=256);
	
	void drawCoordinates(float linelength, float linewidth);

	// Measurement
	        static bool
	        sortfunction (Particle p1, Particle p2);

};



} // namespace Tracking


#endif
