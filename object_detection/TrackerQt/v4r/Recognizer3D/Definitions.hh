/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2013, Johann Prankl
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
 * @author johann.prankl
 *
 */
/**
 * $Id$
 */

#ifndef P_DEFINITIONS_HH
#define P_DEFINITIONS_HH

#pragma once

#include "PNamespace.hh"

//for debugging
//#define DEBUG

#define NUM_THREADS 2



namespace P
{

class Def
{
public:
  static const int DO_MAX_RANSAC_TRIALS;
  static const int DO_MATCHER;     //0 = use match second nearest match, 1=threshold, 2=gpu
  static const float DO_MIN_DESC_DIST;          // descriptor distance for object detection 
  static const double DO_RANSAC_ETA0;         //failure probability
  static const double DO_RANSAC_INL_DIST; //1.;
  static const float DO_CLUSTER_SIGMA_THR;         // cluster threshold for codebook
  static const double DO_TIME_MEAN;
  static const double DISTANCE_RATIO;   // least squares is only accepted if pose does not change very much
};

}
#endif
