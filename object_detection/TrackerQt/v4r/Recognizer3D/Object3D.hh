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

#ifndef P_OBJECT_3D_HH
#define P_OBJECT_3D_HH

#include <opencv/cv.h>
#include <map>
#include "PNamespace.hh"
#include "Array.hh"
#include "PoseCv.hh"
#include "CodebookEntry.hh"
#include "SPolygon.hh"

namespace P
{


class Object3D
{
public:
  unsigned id;
  static unsigned idcnt;
 
  Array<CodebookEntry *> codebook;

  double conf;       //confidence value [0...1]
  double err;        //reprojection error [0...Def::DO_RANSAC_INL_DIST]

  PoseCv pose;  
  SPolygon contour;

  inline unsigned getCodebookSize()
  {
    return codebook.Size();
  }

  Object3D();
  ~Object3D();
};


void DeleteObjects3D(Array<Object3D*> &objects);


/*********************** INLINE METHODES **************************/



}

#endif

