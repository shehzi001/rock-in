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

#include "SDraw.hh"

namespace P 
{

void SDraw::DrawLine(IplImage *img, double x1, double y1, double x2, double y2, CvScalar c, int thickness)
{
  cvLine(img,cvPoint((int)(x1+.5), (int)(y1+.5)), cvPoint((int)(x2+.5), (int)(y2+.5)), c, thickness);
}


void SDraw::DrawCross(IplImage *img, double x, double y, CvScalar c, int thickness)
{
  cvLine(img,cvPoint((int)(x+.5)-2, (int)(y+.5)-2), cvPoint((int)(x+.5)+2, (int)(y+.5)+2),c,thickness);
  cvLine(img,cvPoint((int)(x+.5)-2, (int)(y+.5)+2), cvPoint((int)(x+.5)+2, (int)(y+.5)-2),c,thickness);
}


void SDraw::DrawTriangle(IplImage *img, double x1, double y1, double x2, double y2, double x3, double y3, CvScalar c, int thickness)
{
  cvLine(img,cvPoint((int)(x1+.5), (int)(y1+.5)), cvPoint((int)(x2+.5), (int)(y2+.5)),c, thickness);
  cvLine(img,cvPoint((int)(x2+.5), (int)(y2+.5)), cvPoint((int)(x3+.5), (int)(y3+.5)),c, thickness);
  cvLine(img,cvPoint((int)(x3+.5), (int)(y3+.5)), cvPoint((int)(x1+.5), (int)(y1+.5)),c, thickness);
}

void SDraw::DrawRectangle(IplImage *img, double x1, double y1, double x2, double y2, CvScalar c, int thickness)
{
  cvRectangle( img, cvPoint((int)(x1+.5), (int)(y1+.5)), cvPoint((int)(x2+.5), (int)(y2+.5)), c,
              thickness);

  /*cvLine(img,cvPoint((int)(x1+.5), (int)(y1+.5)), cvPoint((int)(x2+.5), (int)(y1+.5)),c, thickness);
  cvLine(img,cvPoint((int)(x2+.5), (int)(y1+.5)), cvPoint((int)(x2+.5), (int)(y2+.5)),c, thickness);
  cvLine(img,cvPoint((int)(x2+.5), (int)(y2+.5)), cvPoint((int)(x1+.5), (int)(y2+.5)),c, thickness);
  cvLine(img,cvPoint((int)(x1+.5), (int)(y2+.5)), cvPoint((int)(x1+.5), (int)(y1+.5)),c, thickness);*/
}


void SDraw::DrawPoly(IplImage *img, P::Array<Vector2> &vs,CvScalar c, int thickness)
{
  unsigned s= vs.Size();
  for (unsigned j=0; j<s; j++){
    DrawLine(img, vs[j].x, vs[j].y, vs[j+1<s?j+1:0].x, vs[j+1<s?j+1:0].y, c,thickness);
  }
}


void SDraw::DrawFillPoly(IplImage *img, P::Array<Vector2> &vs,CvScalar c)
{
  CvPoint *pts[1];
  int npts[1];
  pts[0] = (CvPoint*)cvAlloc(vs.Size()*sizeof(pts[0][0]));
  npts[0]=vs.Size();

  for (unsigned i=0; i<vs.Size(); i++){
    pts[0][i].x=(int)(vs[i].x+.5);
    pts[0][i].y=(int)(vs[i].y+.5);
  }

  cvFillPoly(img, pts, npts, 1, c);

  //cvFillConvexPoly( img, pts, vs.Size(), c);
  cvFree(&pts[0]);
}


void SDraw::DrawSIFT(IplImage *img, double x, double y, double l, double o, CvScalar c, int thickness)
{
  P::Vector2 p, r[4];

  //scale descriptor (box)
  r[0] = P::Vector2(-l,-l);
  r[1] = P::Vector2(+l,-l);
  r[2] = P::Vector2(+l,+l);
  r[3] = P::Vector2(-l,+l);
  p = P::Vector2(l,0);

  //rotate box and set position
  for (unsigned i=0; i<4; i++){
    r[i] = P::Rotate(r[i],-o);
    r[i] = P::Vector2(r[i].x+x, r[i].y+y);
  }

  p = P::Rotate(p,-o);
  p = P::Vector2(p.x+x, p.y+y);

  //draw
  for (unsigned i=0; i<4; i++)
    cvLine(img,cvPoint((int)(r[i].x+.5), (int)(r[i].y+.5)), cvPoint((int)(r[i<3?i+1:0].x+.5), (int)(r[i<3?i+1:0].y+.5)),c, thickness);
  

  cvLine(img,cvPoint((int)(x+.5), (int)(y+.5)), cvPoint((int)(p.x+.5), (int)(p.y+.5)),c, thickness);

  //...or draw circle
  //cvCircle( img, cvPoint((int)(x+.5),(int)(y+.5)), l, c, thickness);
}

void SDraw::DrawCircle(IplImage *img, double x, double y, double r, 
                       CvScalar c, int thickness)
{
  cvCircle( img, cvPoint((int)(x+.5),(int)(y+.5)), r, c, thickness);
}

void SDraw::DrawEllipse(IplImage *img, double x, double y, double a, double b, 
                        double angle, CvScalar c, int thickness)
{
  cvEllipse(img, cvPoint((int)(x+.5),(int)(y+.5)), cvSize((int)(a+.5),(int)(b+.5)),
                  angle, 0, 360,  c, thickness, CV_AA, 0);
}

void SDraw::DrawArc(IplImage *img, double x, double y, double r, double start_angle,
             double angular_span, CvScalar c, int thickness)
{
  cvEllipse(img, cvPoint((int)(x+.5),(int)(y+.5)), cvSize((int)(r+.5),(int)(r+.5)),
            0, (int)(-start_angle*180/M_PI+.5), 
            (int)((-start_angle-angular_span)*180/M_PI+.5), c, thickness, CV_AA, 0);
}

void SDraw::WriteText(IplImage *img, const char* text, double x, double y, CvScalar c)
{
  CvFont font;
  cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 1.1, 1.2, 0, 1, CV_AA );
  cvPutText( img, text, cvPoint((int)(x+.5),(int)(y+.5)), &font, c);
}


}