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

#ifndef P_DETECT_GPUSIFT_HH
#define P_DETECT_GPUSIFT_HH

#include <limits.h>
#include <GL/glut.h>
#include "v4rexternal/SiftGPU/src/SiftGPU/SiftGPU.h"

#include "PNamespace.hh"
#include "KeypointDescriptor.hh"
#include "CodebookEntry.hh"
#include "Array.hh"


#ifndef WIN32
#include <dlfcn.h>
#endif


namespace P
{

typedef float SIFTDescriptor[128];


class DetectGPUSIFT
{
private:
  SiftGPU *sift;
  
public:
  DetectGPUSIFT();
  ~DetectGPUSIFT();

  void Operate(IplImage *img, Array<KeypointDescriptor*> &keys);
  //void Match(Array<KeypointDescriptor*> &keys, Array<CodebookEntry *> &cb, int (*matches)[2], int buf_size, int &num); 

  void Draw(IplImage *img, Array<KeypointDescriptor*> &keys);
};


/************************** INLINE METHODES ******************************/



}

#endif

