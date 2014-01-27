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
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KEYPOINT_PAIR_HH
#define P_KEYPOINT_PAIR_HH

#include "Keypoint.hh"

namespace P
{


class KeypointPair
{
public:
  Keypoint *k1;
  Keypoint *k2;

  KeypointPair() : k1(0), k2(0) {};
  KeypointPair(Keypoint *_k1, Keypoint *_k2) : k1(_k1), k2(_k2) {};
  ~KeypointPair(){};
};

}


#endif

