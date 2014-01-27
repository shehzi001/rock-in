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


#include "PoseCv.hh"


namespace P 
{



/***************************** PoseCv ******************************
 * Constructor/Destructor
 */
PoseCv::PoseCv()
{
  R = cvCreateMat( 3, 3, CV_32F );
  t = cvCreateMat( 3, 1, CV_32F );
  n = cvCreateMat( 3, 1, CV_32F );
}

PoseCv::~PoseCv()
{
  cvReleaseMat(&R);
  cvReleaseMat(&t);
  cvReleaseMat(&n);
}

/**************************** PRIVATE *************************************/



/**************************** PUBLIC *************************************/
void InitializePoseCv(PoseCv &pose)
{
  cvmSet(pose.R,0,0,1); cvmSet(pose.R,0,1,0); cvmSet(pose.R,0,2,0);
  cvmSet(pose.R,1,0,0); cvmSet(pose.R,1,1,1); cvmSet(pose.R,1,2,0);
  cvmSet(pose.R,2,0,0); cvmSet(pose.R,2,1,0); cvmSet(pose.R,2,2,1);

  cvmSet(pose.t,0,0,0);
  cvmSet(pose.t,1,0,0);
  cvmSet(pose.t,2,0,0);

  cvmSet(pose.n,0,0,1);
  cvmSet(pose.n,1,0,0);
  cvmSet(pose.n,2,0,0);

   
}

void DeletePoseCv(Array<PoseCv*> &ps)
{
  for (unsigned i=0; i<ps.Size(); i++)
    delete ps[i];
  ps.Clear();
}


} // --THE END--



