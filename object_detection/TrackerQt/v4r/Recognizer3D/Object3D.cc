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


#include "Object3D.hh"

namespace P 
{

unsigned Object3D::idcnt=0;


/********************** Object3D ************************
 * Constructor/Destructor
 */
Object3D::Object3D()
 : id(UINT_MAX), conf(0.), err(DBL_MAX)
{
}

Object3D::~Object3D()
{
  DeleteCodebook(codebook);
}




/******************************** PUBLIC **************************/



void DeleteObjects3D(Array<Object3D*> &objects)
{
  for (unsigned i=0; i<objects.Size(); i++)
    delete objects[i];
  objects.Clear();
}



}

