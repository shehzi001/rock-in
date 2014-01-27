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


#include "Recognizer3D/Pose.hh"


namespace P 
{



/***************************** Pose ******************************
 * Constructor/Destructor
 */
Pose::Pose()
{
  t.create(3,1,0.);
  R.identity(3);
}

Pose::Pose(Matrix &RR, Matrix &tt)
{
  t = tt;
  R = RR;
}

Pose::Pose(Matrix &RR, Matrix &tt, Matrix &nn)
{
  t = tt;
  R = RR;
  n = nn;
}

Pose::~Pose()
{
}

/**************************** PRIVATE *************************************/



/**************************** PUBLIC *************************************/

} // --THE END--



