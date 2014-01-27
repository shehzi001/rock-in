/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2013, Thomas Mörwald
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
 * @author thomas.moerwald
 *
 */
#ifndef _TRACKING_SMOOTH_FILTER_
#define _TRACKING_SMOOTH_FILTER_

namespace Tracking {

class FloatFilter
{
public:
  FloatFilter(float a = 0.0f)
  {
    this->a = a;
    b = 1.0f - a;
    z = 0.0f;
  }

  inline void Set(const float &z)
  {
    this->z = z;
  }

  operator float() const
  {
    return z;
  }
  FloatFilter& operator=(const float &in)
  {
    if( !isnan(in) )
      z = (in * b) + (z * a);
    return (*this);
  }

private:
  float a, b, z;
};

class FloatFilterRise
{
public:
  FloatFilterRise(float a = 0.0f)
  {
    this->a = a;
    b = 1.0f - a;
    z = 0.0f;
  }

  inline void Set(const float &z)
  {
    this->z = z;
  }

  operator float() const
  {
    return z;
  }
  FloatFilterRise& operator=(const float &in)
  {
    if( !isnan(in) ) {
      if( in >= z )
        z = (in * b) + (z * a);
      else
        z = in;
    }
    return (*this);
  }

private:
  float a, b, z;
};

class FloatFilterFall
{
public:
  FloatFilterFall(float a = 0.0f)
  {
    this->a = a;
    b = 1.0f - a;
    z = 0.0f;
  }

  inline void Set(const float &z)
  {
    this->z = z;
  }

  operator float() const
  {
    return z;
  }
  FloatFilterFall& operator=(const float &in)
  {
    if( !isnan(in) ) {
      if( in <= z )
        z = (in * b) + (z * a);
      else
        z = in;
    }
    return (*this);
  }

private:
  float a, b, z;
};

}

#endif
