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
#include "ModelEntry.h"

using namespace Tracking;
using namespace TomGine;

/** @brief class ModelEntry */
ModelEntry::ModelEntry (float lpf) :
  c_comp (0.7), c_occ_comp (0.5), w_msp (0.5), m_lpf_pose_tx (lpf), m_lpf_pose_ty (lpf), m_lpf_pose_tz (lpf),
      m_lpf_pose_qx (lpf), m_lpf_pose_qy (lpf), m_lpf_pose_qz (lpf), m_lpf_pose_qw (lpf)
{
  predictor = new Predictor ();
  bfc = false;
  lock = false;
  mask_geometry_edges = false;
  mask = 0;
  c_max_runtime = 0.0f;
  ts = ST_OK;
}

ModelEntry::ModelEntry (const TomGine::tgModel& m, float lpf) :
  model (m), c_comp (0.7), c_occ_comp (0.5), w_msp (0.5), m_lpf_pose_tx (lpf), m_lpf_pose_ty (lpf),
      m_lpf_pose_tz (lpf), m_lpf_pose_qx (lpf), m_lpf_pose_qy (lpf), m_lpf_pose_qz (lpf), m_lpf_pose_qw (lpf)
{
  predictor = new Predictor ();
  bfc = false;
  lock = false;
  mask_geometry_edges = false;
  mask = 0;
  c_max_runtime = 0.0f;
  ts = ST_OK;
}

ModelEntry::~ModelEntry ()
{
  delete (predictor);
}

//void ModelEntry::poseDiff(float &t, float &a)
//{
//	vec3 axis0, axis1;
//	float angle0, angle1;
//
//	pose.q.getAxisAngle(axis0, angle0);
//	pose_prev.q.getAxisAngle(axis1, angle1);
//
////	vec3 axis01 = axis0 - axis1;
//	vec3 t01 = pose.t - pose_prev.t;
//
//	a = 0.5f*(1.0-fabs(axis0*axis1)+fabs(angle0-angle1));
//	t = t01.length();
//}
//
//void ModelEntry::speed()
//{
//	static Timer s_timer;
//	float dt = (float)s_timer.Update();
//
//	float t,a;
//	poseDiff(t,a);
//
//	speed_angular = a/dt;
//	speed_translational = t/dt;
//}
//

void
ModelEntry::setLPF(float lpf)
{
  lpf = std::max<float>(0.0,lpf);
  lpf = std::min<float>(1.0,lpf);

  m_lpf_pose_tx = FloatFilter(lpf);
  m_lpf_pose_ty = FloatFilter(lpf);
  m_lpf_pose_tz = FloatFilter(lpf);
  m_lpf_pose_qx = FloatFilter(lpf);
  m_lpf_pose_qy = FloatFilter(lpf);
  m_lpf_pose_qz = FloatFilter(lpf);
  m_lpf_pose_qw = FloatFilter(lpf);

  m_lpf_pose_tx.Set(lpf_pose.t.x);
  m_lpf_pose_ty.Set(lpf_pose.t.y);
  m_lpf_pose_tz.Set(lpf_pose.t.z);

  m_lpf_pose_qx.Set(lpf_pose.q.x);
  m_lpf_pose_qy.Set(lpf_pose.q.y);
  m_lpf_pose_qz.Set(lpf_pose.q.z);
  m_lpf_pose_qw.Set(lpf_pose.q.w);
}

void
ModelEntry::filter_pose ()
{
  lpf_pose.t.x = m_lpf_pose_tx = pose.t.x;
  lpf_pose.t.y = m_lpf_pose_ty = pose.t.y;
  lpf_pose.t.z = m_lpf_pose_tz = pose.t.z;
  lpf_pose.q.x = m_lpf_pose_qx = pose.q.x;
  lpf_pose.q.y = m_lpf_pose_qy = pose.q.y;
  lpf_pose.q.z = m_lpf_pose_qz = pose.q.z;
  lpf_pose.q.w = m_lpf_pose_qw = pose.q.w;
  lpf_pose.q.normalise ();
  pose = lpf_pose;
}

void
ModelEntry::resetPose ()
{
  pose = initial_pose;
  lpf_pose = initial_pose;
  m_lpf_pose_tx.Set (initial_pose.t.x);
  m_lpf_pose_ty.Set (initial_pose.t.y);
  m_lpf_pose_tz.Set (initial_pose.t.z);
  m_lpf_pose_qx.Set (initial_pose.q.x);
  m_lpf_pose_qy.Set (initial_pose.q.y);
  m_lpf_pose_qz.Set (initial_pose.q.z);
  m_lpf_pose_qw.Set (initial_pose.q.w);
}

void
ModelEntry::setInitialPose (const TomGine::tgPose &p)
{
  initial_pose = p;
  initial_pose.c = pose.c;
  initial_pose.w = pose.w;
}

void
ModelEntry::evaluate_signals ()
{
  if (lock)
  {
    ts = ST_LOCKED;
    return;
  }

  c_mkp = distribution.getMeanKeptParticleC ();
  w_msp = distribution.getMeanWeightOfStaticParticles ();

  if (c_mkp > c_max_runtime)
    c_max_runtime = c_mkp;

  c_normalized = c_mkp / c_max_runtime; ///< confidence normalized to maximum confidence
  c_occ_comp = std::max (0.0, c_occ - 0.2 + 0.05 * w_msp); ///< movement compensated occlusion confidence
  c_comp = std::min (1.0, c_normalized + (1.0 - w_msp)); ///< movement compensated normalized confidence
  n_eff = distribution.n_eff;
}

void
ModelEntry::evaluate_tsd ()
{
  // state transitions
  bool occ = (c_occ_comp >= 0.01); // occlusion detection transition
//  bool lo = n_eff > float(this->distribution.size()) / 3.0f;
  //bool lo = ((c_mkp < 0.02) && (w_msp < 0.85)) || ((c_comp < 0.7) && (c_occ_comp < 0.01)); // lost detection transition
  bool lo = n_eff > float(this->distribution.size()) / 2.0f;
  bool rec = (c_comp >= 0.5) && (c_occ_comp < 0.01) && (w_msp > 0.85); // state recovery transition

  // Tracking State Diagram
  switch (ts)
  {
    case ST_OK:
      if (occ)
        ts = ST_OCCLUDED;
      if (lo)
        ts = ST_LOST;
      break;
    case ST_OCCLUDED:
      if (!occ)
        ts = ST_OK;
      if (lo)
        ts = ST_LOST;
      break;
    case ST_LOST:
      if (rec)
        ts = ST_OK;
      break;
    default:
      if (rec)
        ts = ST_OK;
      else
        ts = ST_LOST;
      break;
  }

  //	// Tracking Speed State Diagram
  //	switch(st_movement){
  //	case ST_STILL:
  //		if(w_msp<0.85)	st_movement = ST_SLOW;
  //		break;
  //	case ST_SLOW:
  //		if(w_msp>0.9)	st_movement = ST_STILL;
  //		if(w_msp<0.1)	st_movement = ST_FAST;
  //		break;
  //	case ST_FAST:
  //		if(w_msp>0.2)	st_movement = ST_SLOW;
  //		break;
  //	default:
  //		st_movement = ST_FAST;
  //	}
}