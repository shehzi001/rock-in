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

#ifndef P_POSE_CV_HH
#define P_POSE_CV_HH

#include <limits.h>
#include <map>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <iostream>
#include "PNamespace.hh"
#include "Array.hh"


namespace P
{


class PoseCv
{
private:    
  

public:
  CvMat *R;
  CvMat *t;
  CvMat *n;

  PoseCv();
  ~PoseCv();
};

void InitializePoseCv(PoseCv &pose);
void DeletePoseCv(Array<PoseCv*> &ps);

inline void CopyPoseCv(PoseCv &in, PoseCv &out);
inline void Rot2Quat(CvMat *M, double *q);
inline void Rot2Vec3(CvMat *R, double *d);
inline void Quat2Rot(double *q, CvMat *M);
inline void Vec32Rot(double *d, CvMat *R);
inline void InvPoseCv(PoseCv &in, PoseCv &out);



/*********************************** INLINE *********************************/

inline void CopyPoseCv(PoseCv &in, PoseCv &out)
{ 
  cvCopy(in.R, out.R);
  cvCopy(in.t, out.t);
  cvCopy(in.n, out.n);
}

inline void Rot2Quat(CvMat *M, double *q)
{
  float *R = M->data.fl;

  double tmp[4];
  double mag;
  unsigned maxpos;
  tmp[0]=1.0 + R[0] + R[4] + R[8];
  tmp[1]=1.0 + R[0] - R[4] - R[8];
  tmp[2]=1.0 - R[0] + R[4] - R[8];
  tmp[3]=1.0 - R[0] - R[4] + R[8];

  mag=-1.0;
  for(unsigned i=0; i<4; i++){
    if(tmp[i]>mag){
      mag=tmp[i];
      maxpos=i;
    }
  }
  if(maxpos==0){
    q[0]=sqrt(tmp[0])*0.5;
    q[1]=(R[7] - R[5])/(4.0*q[0]);
    q[2]=(R[2] - R[6])/(4.0*q[0]);
    q[3]=(R[3] - R[1])/(4.0*q[0]);
  }
  else if(maxpos==1){
    q[1]=sqrt(tmp[1])*0.5;
    q[0]=(R[7] - R[5])/(4.0*q[1]);
    q[2]=(R[3] + R[1])/(4.0*q[1]);
    q[3]=(R[2] + R[6])/(4.0*q[1]);
  }
  else if(maxpos==2){
    q[2]=sqrt(tmp[2])*0.5;
    q[0]=(R[2] - R[6])/(4.0*q[2]);
    q[1]=(R[3] + R[1])/(4.0*q[2]);
    q[3]=(R[7] + R[5])/(4.0*q[2]);
  }
  else if(maxpos==3){
    q[3]=sqrt(tmp[3])*0.5;
    q[0]=(R[3] - R[1])/(4.0*q[3]);
    q[1]=(R[2] + R[6])/(4.0*q[3]);
    q[2]=(R[7] + R[5])/(4.0*q[3]);
  }
  else
  {
    cout<<"komisch"<<endl;
  }
  // enforce unit length
  mag=q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];

  if(!IsZero(mag-1.))
  {

    mag=1.0/sqrt(mag);
    q[0]*=mag; q[1]*=mag; q[2]*=mag; q[3]*=mag;
  }
}

inline void Rot2Vec3(CvMat *R, double *d)
{
  double t[4];
  Rot2Quat(R,t);

  double mag=sqrt(Sqr(t[0]) + Sqr(t[1]) + Sqr(t[2]) + Sqr(t[3]));
  double sg=(t[0]>=0.0)? 1.0 : -1.0;
  mag=sg/mag;
  d[0] = t[1]*mag;
  d[1] = t[2]*mag;
  d[2] = t[3]*mag;
}

inline void Quat2Rot(double *q, CvMat *M)
{
  float *R = M->data.fl;

  // ensure unit length
  double mag = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  if(!IsZero(mag-1.0))
  {
    mag=1.0/sqrt(mag);
    q[0]*=mag; q[1]*=mag; q[2]*=mag; q[3]*=mag;
  }

  R[0]=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
  R[1]=2*(q[1]*q[2]-q[0]*q[3]);
  R[2]=2*(q[1]*q[3]+q[0]*q[2]);

  R[3]=2*(q[1]*q[2]+q[0]*q[3]);
  R[4]=q[0]*q[0]+q[2]*q[2]-q[1]*q[1]-q[3]*q[3];
  R[5]=2*(q[2]*q[3]-q[0]*q[1]);

  R[6]=2*(q[1]*q[3]-q[0]*q[2]);
  R[7]=2*(q[2]*q[3]+q[0]*q[1]);
  R[8]=q[0]*q[0]+q[3]*q[3]-q[1]*q[1]-q[2]*q[2];
}

inline void Vec32Rot(double *d, CvMat *R)
{
  double q[4];
  q[0] = sqrt(1.0 - Sqr(d[0]) - Sqr(d[1])- Sqr(d[2]));
  q[1] = d[0];
  q[2] = d[1];
  q[3] = d[2];

  Quat2Rot(q,R);  
}

inline void InvPoseCv(PoseCv &in, PoseCv &out)
{
  cvTranspose( in.R, out.R );
  cvGEMM( out.R, in.t, -1, 0, 0, out.t, 0 );
}


} //--THE END--

#endif

