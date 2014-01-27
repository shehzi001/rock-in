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
 * Simple RANSAC based 3d object detector
 *
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_DOBJECT_3D_HH
#define P_DOBJECT_3D_HH

#include <opencv/cv.h>
#include "v4rexternal/SiftGPU/src/SiftGPU/SiftGPU.h"
#include "KeyClusterPair.hh"
#include "Definitions.hh"
#include "Geometry.hh"
#include "Math.hh"
#include "Object3D.hh"
#include "PoseCv.hh"
#include "Vector3.hh"

namespace P
{

class ODetect3D
{
public:
  class Point3D
  {
  public:
    float x;
    float y;
    float z;
  };
  class Point2D
  {
  public:
    float x;
    float y;
  };

private:
  IplImage *dbg;

  CvMat *cameraMatrix;
  CvMat *distCoeffs;

  SiftMatchGPU *matcher;
  int matcherSize;

  Array<KeypointDescriptor*> inlier;   //just a container for drawing inlier...

  void DeletePairs(Array<KeyClusterPair*> &matches);
  void MatchKeypoints2(Array<KeypointDescriptor *> &keys, Array<CodebookEntry *> &cb, 
                       Array<KeyClusterPair*> &matches);
  void MatchKeypoints(Array<KeypointDescriptor *> &keys, Array<CodebookEntry *> &cb, 
                      Array<KeyClusterPair* > &matches);
  void MatchKeypointsGPU(Array<KeypointDescriptor *> &keys, Array<CodebookEntry *> &cb, Array<KeyClusterPair* > &matches);
  void FitModelRANSAC(Array<KeyClusterPair*> &matches, PoseCv &pose, unsigned &numInl);
  void GetRandIdx(unsigned size, unsigned num, P::Array<unsigned> &idx);
  void GetInlier(Array<KeyClusterPair*> &matches, PoseCv &pose, int &inl);
  bool GetBestCorner(PoseCv &pose, KeypointDescriptor *k, CodebookEntry *cbe, Point3D &pos, double &minDist);
  void RefinePoseLS(Array<KeyClusterPair*> &matches, PoseCv &pose, unsigned &inl, double &err);
  void ComputeConfidence(Array<KeypointDescriptor *> &keys, unsigned &numInl, Object3D &object);


  inline void ProjectPoint2Image(double xc, double yc, double zc, CvMat *C, double &xi, double &yi);
  inline void CopyPoseCv(PoseCv &in, PoseCv &out);



public:
  ODetect3D();
  ~ODetect3D();

  bool Detect(Array<KeypointDescriptor *> &keys, Object3D &object);
  void SetCameraParameter(CvMat *C);

  void SetDebugImage(IplImage *img){dbg = img;}

  void DrawInlier(IplImage *img, CvScalar col=CV_RGB(0,255,0));
};

/*********************** INLINE METHODES **************************/
/**
 * ProjectPoint2Image
 * projects a 3d point to the image
 */
inline void ODetect3D::ProjectPoint2Image(double xc, double yc, double zc, CvMat *C, double &xi, double &yi)
{
  xi = C->data.fl[0] * xc/zc + C->data.fl[2];
  yi = C->data.fl[4] * yc/zc + C->data.fl[5];
}

inline void ODetect3D::CopyPoseCv(PoseCv &in, PoseCv &out)
{
  cvCopy(in.R, out.R);
  cvCopy(in.t, out.t);
  cvCopy(in.n, out.n);
}


}

#endif

