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

#ifndef P_SMATRIX_HH
#define P_SMATRIX_HH

#include "Recognizer3D/PNamespace.hh"
#include "Recognizer3D/Except.hh"

namespace P
{

extern void Mul33(double *m1, double *m2, double *r);
extern void Mul33(double *m, double s, double *r);
extern double Det33(double *m);
extern bool Inv33(double *m, double *r);
extern void Transpose33(double *m, double *r);

}

#include "Recognizer3D/SMatrix.ic"

#endif

