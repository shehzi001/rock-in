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
#include "Predictor.h"
#include "Noise.h"

using namespace Tracking;

Predictor::Predictor ()
{
  m_dTime = 0.0;
  c_pred = 0.0;
  m_noConvergence = 0.0f;
  m_keepParticles = 0.0f;
}

// *** private ***
float
Predictor::noise (float sigma, unsigned int type)
{
  float random = 0.0f;

  // Gaussian noise
  if (type == GAUSS)
  {
    for (unsigned i = 0; i < 10; i++)
    {
      random += float (rand ()) / RAND_MAX;
    }
    random = 2.0f * (random / 10.0f - 0.5f);
  }

  // Uniform distributed noise
  if (type == NORMAL)
  {
    random = 2.0f * (float (rand ()) / RAND_MAX - 0.5f);
  }

  // Adjusting to range
  random = random * sigma;

  return random;
}

Particle
Predictor::genNoise (float sigma, Particle pConstraint, unsigned int type)
{
  if (sigma == 0.0f)
    printf ("[Predictor::genNoise] Warning standard deviation sigma is 0.0\n");

  Particle epsilon;
  Noise N;

  epsilon.r.x = N.randn_notrig (0.0f, sigma * pConstraint.r.x);
  epsilon.r.y = N.randn_notrig (0.0f, sigma * pConstraint.r.y);
  epsilon.r.z = N.randn_notrig (0.0f, sigma * pConstraint.r.z);
  epsilon.t.x = N.randn_notrig (0.0f, sigma * pConstraint.t.x);
  epsilon.t.y = N.randn_notrig (0.0f, sigma * pConstraint.t.y);
  epsilon.t.z = N.randn_notrig (0.0f, sigma * pConstraint.t.z);

  epsilon.z = N.randn_notrig (0.0f, sigma * pConstraint.z);

  return epsilon;
}

void
Predictor::sampleFromGaussian (Distribution& d, int num_particles, Particle mean, Particle variance, float sigma)
{
  Particle p;
  Particle epsilon;

  //	d.push_back(mean);
  for (int i = 0; i < num_particles; i++)
  {

    // particle to sample from
    p = mean;

    // move function (gaussian noise)
    epsilon = genNoise (sigma, variance);

    // apply movement
    p.Translate (epsilon.t);
    p.RotateAxis (epsilon.r);
    p.Translate (m_cam_view.x * epsilon.z, m_cam_view.y * epsilon.z, m_cam_view.z * epsilon.z);

    // add particle to distribution
    d.push_back (p);
  }
}

void
Predictor::movePredicted (const TomGine::tgPose& poseIn, TomGine::tgPose& poseOut, float &c)
{
  poseOut = poseIn;
  c = 0.0;
}

// *** public ***

void
Predictor::resample (Distribution& d, int num_particles, Particle variance, const int &iter, const int &max_iter)
{

  if (d.size () <= 0)
  {
    printf ("[Predictor::resample] Warning distribution d is empty.\n");
    return;
  }

  unsigned n, id = 0;
  float sigma = 0.01f;
  float c = 0.0f;
  float w_sum = 0.0f;

  if (num_particles <= 0)
  {
    printf ("[Distribution::resample] Warning number of particles to low (0)\n");
    num_particles = d.size ();
  }
  int remain_particles = num_particles;

  ParticleList pl;
  d.copy (pl);
  d.clear ();

  if (iter == max_iter - 1)
  {
    d.m_numStaticParticles = 0;
    d.m_meanWeightOfStaticParticles = 0.0f;
  }

  // Keep best particles
  std::vector<Particle> keptParticles;
//  unsigned num_keptParticles = unsigned (num_particles * m_keepParticles);
//  for (unsigned i = 0; i < num_keptParticles; i++)
//    d.push_back (pl[i]);

  // Particle voting for no convergence
//  unsigned num_noConvergence = unsigned (num_particles * m_noConvergence);
//  sampleFromGaussian (d, num_noConvergence, pl[0], variance, 1.0f);

//  remain_particles -= (num_keptParticles + num_noConvergence);

  // Particles with motion
  for (id = 0; id < pl.size () && d.size () < num_particles; id++)
  {
    // resampling according to weight
    n = (unsigned)ceil (pl[id].w * num_particles);
    n = std::min<unsigned> (n, unsigned (num_particles - d.size ()));
    c = pl[id].c;

    // Keep best particles
    if (n>1)
    {
      d.push_back(pl[id]);
      n--;
      // Evaluate how many particles remain static since last frame
      if (iter == max_iter - 1)
      {
        if (find (d.m_keptParticles.begin (), d.m_keptParticles.end (), pl[id]) != d.m_keptParticles.end ())
        {
          d.m_meanWeightOfStaticParticles += pl[id].w;
          d.m_numStaticParticles++;
        }
        keptParticles.push_back (pl[id]);
        w_sum += pl[id].w;
      }
    }

    // Standard deviation for normal distribution
    // TODO evaluate optimal power of estimator (accuracy / robustness)
    // 		sigma = (1.0-pow(1.0-pow(1.0-c,2),3));
    sigma = 1.0f - c;

    // crop range of sigma
    if (sigma == 0.0)
      sigma = 0.00001f;
    if (sigma > 1.0)
      sigma = 1.0f;

    // Sample particles from Gaussian distribution
    sampleFromGaussian (d, n, pl[id], variance, sigma);
  }

  // Mean and weighted mean of kept particles
  if (iter == max_iter - 1 && keptParticles.size() > 0)
  {
    d.m_numStaticParticlesUnion = d.m_keptParticles.size() + keptParticles.size() - d.m_numStaticParticles;
    d.m_keptParticles = keptParticles;
    d.m_meanWeightOfStaticParticles = d.m_meanWeightOfStaticParticles / w_sum;
    d.m_meanKeptParticle = d.calcWeightedMean (d.m_keptParticles);
  }
}

void
Predictor::sample (Distribution& d, int num_particles, Particle mean, Particle variance)
{
  d.clear ();
  sampleFromGaussian (d, num_particles, mean, variance);
}

void
Predictor::updateTime (double dTime)
{
  m_dTime = dTime;
}
