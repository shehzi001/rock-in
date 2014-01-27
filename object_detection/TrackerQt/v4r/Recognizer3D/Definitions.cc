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

#include "Definitions.hh"

using namespace P;

const int		Def::DO_MAX_RANSAC_TRIALS = 1000;
const int		Def::DO_MATCHER = 2;     //0 = use match second nearest match, 1=threshold, 2=gpu
const float		Def::DO_MIN_DESC_DIST = 8.;          // descriptor distance for object detection 
const double	Def::DO_RANSAC_ETA0 = 0.01;         //failure probability
const double	Def::DO_RANSAC_INL_DIST = 2; //1.;
const float		Def::DO_CLUSTER_SIGMA_THR=0.2;         // cluster threshold for codebook
const double	Def::DO_TIME_MEAN = 5.;
const double	Def::DISTANCE_RATIO = 1.;   // least squares is only accepted if pose does not change very much

