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
 * $Id: Math.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 *
 * Michael Zillich, 2004-3-04
 */

#ifndef P_MATH_HH
#define P_MATH_HH

#include <limits.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "PNamespace.hh"
#include "Except.hh"

#ifdef WIN32
#include <winsock.h>
#else
#include <sys/time.h>
#endif

namespace P
{

const unsigned UNDEF_ID = UINT_MAX;

const unsigned START     = 0;
const unsigned END       = 1;
const unsigned MID       = 2;
const unsigned ONE_THIRD = 3;
const unsigned TWO_THIRD = 4;
const unsigned STOP      = 15;

inline unsigned OtherEnd(unsigned end)
{
  return end ^ 0x1;
}

// same and opposite sense
const unsigned SAME = START;
const unsigned OPP  = END;

const unsigned LEFT  = 0;
const unsigned RIGHT = 1;
const unsigned CAM3D = 2;

// inner and outer ends of junctions
const unsigned INNER = 0;
const unsigned OUTER = 1;

inline unsigned OtherSide(unsigned side)
{
  return side ^ 0x1;
}

inline unsigned Other(unsigned i)
{
  return i ^ 0x1;
}

/// Returns true if the value is near zero (+/- epsilon)
extern bool IsZero(double d);

/// Returns true if the values are equal (+/- epsilon)
extern bool IsEqual(double a, double b);

/// Square of given number
template <class Num>
extern Num Sqr(Num x);

template <class Num>
extern Num Max(Num a, Num b);
template <class Num>
extern Num Min(Num a, Num b);

template <class Num>
extern int Sign(Num x);

template <class T>
inline void Swap(T &a, T &b);

template <class Num>
inline bool Between(Num x, Num l, Num u);
template <class Num>
inline bool BetweenEq(Num x, Num l, Num u);

/// Scale angle to [0..2pi[
extern double ScaleAngle_0_2pi(double a);
/// Scale angle to [-pi..pi[
extern double ScaleAngle_mpi_pi(double a);
/// Scale angle to [0..pi[
extern double ScaleAngle_0_pi(double a);
/// Difference of two angles b - a. The result is scaled to [-pi..pi[
extern double DiffAngle_mpi_pi(double b, double a);
/// Difference of two angles b - a. The result is scaled to [0..2pi[
extern double DiffAngle_0_2pi(double b, double a);
/// Scale an integer angle to [0..8[
extern int ScaleIntAngle_0_8(int a);

extern int timeval_subtract(struct timeval *result, struct timeval *x,
  struct timeval *y);
extern double timespec_diff(struct timespec *x, struct timespec *y);

}

#include "Math.ic"

#endif

