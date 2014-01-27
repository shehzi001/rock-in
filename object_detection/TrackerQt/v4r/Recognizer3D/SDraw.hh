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

#ifndef P_SDRAW_HH
#define P_SDRAW_HH

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "PNamespace.hh"
#include "Vector2.hh"
#include "Array.hh"

namespace P
{

class SDraw
{
public:
  static void DrawCross(IplImage *img, double x, double y, 
                  CvScalar c=CV_RGB(0,0,255), int thickness=1);
  static void DrawLine(IplImage *img, double x1, double y1, double x2, double y2, 
                  CvScalar c=CV_RGB(255,255,255), int thickness=1);
  static void DrawCircle(IplImage *img, double x, double y, double r, 
                  CvScalar c=CV_RGB(255,255,255), int thickness=1);
  static void DrawEllipse(IplImage *img, double x, double y, double a, double b, 
                  double angle, CvScalar c=CV_RGB(255,255,255), int thickness=1);
  static void DrawArc(IplImage *img, double x, double y, double r, double start_angle,
                  double angular_span, CvScalar c=CV_RGB(255,0,0), int thickness=1);
  static void DrawTriangle(IplImage *img, double x1, double y1, double x2, double y2, 
                  double x3, double y3, CvScalar c=CV_RGB(0,0,255), int thickness=1);
  static void DrawRectangle(IplImage *img, double x1, double y1, double x2, double y2, 
                  CvScalar c=CV_RGB(0,0,255), int thickness=1);
  static void DrawPoly(IplImage *img, P::Array<Vector2> &vs,
                  CvScalar c=CV_RGB(0,0,255), int thickness=1);
  static void DrawFillPoly(IplImage *img, P::Array<Vector2> &vs,
                  CvScalar c=CV_RGB(0,0,255));
  static void DrawSIFT(IplImage *img, double x, double y, double l, double o, 
                  CvScalar c=CV_RGB(0,0,255), int thickness=1);
  static void WriteText(IplImage *img, const char* text, double x, double y, CvScalar c);
};


}

#endif

