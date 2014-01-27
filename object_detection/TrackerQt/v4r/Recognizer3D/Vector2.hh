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
 * $Id: Vector2.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 *
 * Michael Zillich, 2004-3-04
 */

#ifndef P_VECTOR2_HH
#define P_VECTOR2_HH

#include <iostream>
#include <string>
#include "PNamespace.hh"
#include "Except.hh"

namespace P 
{

/**
 * 2D vector class.
 */
class Vector2
{
public:
  double x;
  double y;

  Vector2();
  Vector2(double xx, double yy);
  void Set(double xi, double yi) {x = xi; y = yi;}
  Vector2& operator+=(const Vector2 &v);
  Vector2& operator-=(const Vector2 &v);
  Vector2& operator*=(double s);
  Vector2& operator/=(double s) throw(Except);
  double NormSquare() const;
  double LengthSquare() const;
  double Norm() const;
  double Length() const;
  void Normalise();
  Vector2 Normal();
  Vector2 NormalClockwise();
  Vector2 NormalAntiClockwise();
};

Vector2 operator-(const Vector2 &v);
bool operator==(const Vector2 &a, const Vector2 &b);
bool operator!=(const Vector2 &a, const Vector2 &b);
Vector2 operator+(const Vector2 &a, const Vector2 &b);
Vector2 operator-(const Vector2 &a, const Vector2 &b);
Vector2 operator*(const double s, const Vector2 &v);
Vector2 operator*(const Vector2 &v, const double s);
Vector2 operator/(const Vector2 &v, const double s) throw(Except);
double PolarAngle(const Vector2 &v);
double Length(const Vector2 &v);
Vector2 Normalise(const Vector2 &v);
double Dot(const Vector2 &a, const Vector2 &b);
double Cross(const Vector2 &a, const Vector2 &b);
bool LeftOf(const Vector2 &a, const Vector2 &b);
bool CounterClockwiseTo(const Vector2 &a, const Vector2 &b);
bool RightOf(const Vector2 &a, const Vector2 &b);
bool ClockwiseTo(const Vector2 &a, const Vector2 &b);
double DistanceSquare(const Vector2 &a, const Vector2 &b);
double Distance(const Vector2 &a, const Vector2 &b);
Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1,
    const Vector2 &p2, const Vector2 &d2) throw(Except);
Vector2 LineIntersection(const Vector2 &p1, const Vector2 &d1,
    const Vector2 &p2, const Vector2 &d2, double *l1, double *l2) throw(Except);
bool LinesIntersecting(const Vector2 &a1, const Vector2 &a2,
    const Vector2 &b1, const Vector2 &b2);
bool LinesIntersecting(const Vector2 &a1, const Vector2 &a2,
    const Vector2 &b1, const Vector2 &b2, Vector2 &isct);
double DistPointToLine(const Vector2 &q, const Vector2 &p, const Vector2 &d);
double AbsDistPointToLine(const Vector2 &q, const Vector2 &p, const Vector2 &d);
Vector2 Rotate(const Vector2 &a, double phi);
Vector2 CircleCenter(const Vector2 &pi, const Vector2 &pj,
   const Vector2 &pk);
Vector2 MidPoint(const Vector2 &a, const Vector2 &b);

ostream& operator<<(ostream &os, const Vector2 &v);
istream& operator>>(istream &is, Vector2 &v) throw(Except);
string&       operator<<(string &s, const Vector2 &v);
const string& operator>>(const string &s, Vector2 &v);

}

#include "Vector2.ic"

#endif

