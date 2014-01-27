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
 * $Id: Vector3.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 */

#ifndef VECTOR3_HH
#define VECTOR3_HH

#include <iostream>
#include "PNamespace.hh"
#include "Except.hh"

namespace P 
{

/**
 *  Vector3 
 */
class Vector3 
{

public:
  double x, y, z;
  Vector3();
  Vector3(double xx, double yy, double zz);
  void Set(double xx, double yy, double zz){ x=xx; y=yy; z=zz;}
  Vector3& operator+=(const Vector3 &v);
  Vector3& operator-=(const Vector3 &v);
  Vector3& operator*=(double s);
  Vector3& operator/=(double s) throw(Except);
  double NormSquare() const;
  double LengthSquare() const;
  double Norm() const;
  double Length() const;
  void Normalise();

};

Vector3 operator-(const Vector3 &v);
bool operator==(const Vector3 &a, const Vector3 &b);
bool operator!=(const Vector3 &a, const Vector3 &b);
Vector3 operator+(const Vector3 &a, const Vector3 &b);
Vector3 operator-(const Vector3 &a, const Vector3 &b);
Vector3 operator*(const double s, const Vector3 &v);
Vector3 operator*(const Vector3 &v, const double s);
Vector3 operator/(const Vector3 &v, const double s) throw(Except);
double Length(const Vector3 &v);
Vector3 Normalise(const Vector3 &v);
double Dot(const Vector3 &a, const Vector3 &b);
Vector3 Cross(const Vector3 &a, const Vector3 &b);
Vector3 MidPoint(const Vector3 &a, const Vector3 &b);
double AngleBetween(const Vector3 &a, const Vector3 &b);
double DistanceSquare(const Vector3 &a, const Vector3 &b);
double Distance(const Vector3 &a, const Vector3 &b);
Vector3 PlaneExp2Normal(const Vector3 &a, const Vector3 &b, const Vector3 &c);

ostream& operator<<(ostream &os, const Vector3 &v);
istream& operator>>(istream &is, Vector3 &v) throw(Except);
string&       operator<<(string &s, const Vector3 &v);
const string& operator>>(const string &s, Vector3 &v);
}

#include "Vector3.ic"

#endif
