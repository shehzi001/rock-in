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

#include "Math.hh"


namespace P 
{


/**
 * Subtract the `struct timeval' values x - y.
 * Return 1 if the difference is negative, otherwise 0.
 */
int timeval_subtract(struct timeval *result, struct timeval *x,
  struct timeval *y)
{
  if(x->tv_usec < y->tv_usec)
  {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if(x->tv_usec - y->tv_usec > 1000000)
  {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;
  return x->tv_sec < y->tv_sec;
}

/**
 * Returns timespecs x - y as double.
 */
double timespec_diff(struct timespec *x, struct timespec *y)
{
  /*timespec res = {x->tv_sec - y->tv_sec, x->tv_nsec - y->tv_nsec};
  if(res.tv_nsec >= 1000000000)
  {
    res.tv_nsec -= 1000000000;
    res.tv_sec++;
  }
  if(res.tv_nsec < 0)
  {
    res.tv_nsec += 1000000000;
    res.tv_sec--;
  }
  return (double)res.tv_sec + 1e-9*(double)res.tv_nsec;*/
  /* TODO: make the above clearer version handle nsec overflows of more than one
   * sec, see below*/
  if(x->tv_nsec < y->tv_nsec)
  {
    int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
    y->tv_nsec -= 1000000000 * nsec;
    y->tv_sec += nsec;
  }
  if(x->tv_nsec - y->tv_nsec > 1000000000)
  {
    int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000;
    y->tv_nsec += 1000000000 * nsec;
    y->tv_sec -= nsec;
  }
  return (double)(x->tv_sec - y->tv_sec) +
    (double)(x->tv_nsec - y->tv_nsec)/1000000000.;
}



}

