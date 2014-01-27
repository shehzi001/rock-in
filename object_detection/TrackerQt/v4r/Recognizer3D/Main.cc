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
 * @file main
 * @author Johann Prankl
 * @date Jan 27 2010
 * @version 0.1
 * @brief
 *
 * @see
 **/

#include <vector>
#include <time.h>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <limits.h>
#include "Recognizer3D/PNamespace.hh"
#include "Recognizer3D/Math.hh"
#include "Recognizer3D/Homography.hh"
#include "Recognizer3D/DetectGPUSIFT.hh"
#include "Recognizer3D/DetectSIFT.hh"
#include "Recognizer3D/Config.hh"
#include "Recognizer3D/ODetect3D.hh"
#include "Recognizer3D/Object3D.hh"
#include "Recognizer3D/ModelObject3D.hh"
#include "Recognizer3D/SPolygon.hh"
#include "Recognizer3D/cvgeometry.h"

//#define SOURCE_CAPTURE
#define SOURCE_FILES

//#define LOG_IMAGES

#define LOOP false //true
#define STEP false 

#define USE_GPUSIFT

//#define ASOF_CONFIG "cfg/HideBox04.cfg"
//#define  ASOF_CONFIG "cfg/Box05.cfg"
#define ASOF_CONFIG "cfg/MoveCamBox02.cfg"

#define DO_UNDISTORT


using namespace P;

bool firstRun=true;
IplImage *img = 0;
IplImage *tImg = 0;
P::Array<P::Vector2> contour;
Array<KeypointDescriptor*> keys;

bool CreateModel(Array<KeypointDescriptor*> &keys, Object3D &object);


static const unsigned PATH_MAX = 256;


/******************************** main *********************************/
int main ( int argc, char** argv ) 
{

  /***************** set undistortion ***********************/
  CvMat *pMapX=0, *pMapY=0;
  CvMat *pIntrinsicDistort = cvCreateMat(3,3, CV_64FC1);
  CvMat *pDistortion = cvCreateMat(1,4, CV_64FC1);
  CvMat *C = cvCreateMat(3,3, CV_32F);

  //-------------- move-/turn- box---------------------------
  //undistortion paramter
  cvmSet(pIntrinsicDistort, 0, 0, 596.6549157813346937);
  cvmSet(pIntrinsicDistort, 0, 1, 0.);
  cvmSet(pIntrinsicDistort, 0, 2, 353.1384893909934704);
  cvmSet(pIntrinsicDistort, 1, 0, 0.);
  cvmSet(pIntrinsicDistort, 1, 1, 598.0324507661965754);
  cvmSet(pIntrinsicDistort, 1, 2, 225.6147546068901875);
  cvmSet(pIntrinsicDistort, 2, 0, 0.);
  cvmSet(pIntrinsicDistort, 2, 1, 0.);
  cvmSet(pIntrinsicDistort, 2, 2, 1.);

  cvmSet(pDistortion, 0, 0, -0.3074401347696656);
  cvmSet(pDistortion, 0, 1, 0.1405620272214222);
  cvmSet(pDistortion, 0, 2, -9.2684639835669924e-04);
  cvmSet(pDistortion, 0, 3, -1.8418869765222761e-04);

  //camera matrix for undistorted images
  cvmSet(C, 0, 0, 596.561604);
  cvmSet(C, 0, 1, 0.);
  cvmSet(C, 0, 2, 353.497082);
  cvmSet(C, 1, 0, 0.);
  cvmSet(C, 1, 1, 597.739272);
  cvmSet(C, 1, 2, 226.281385);
  cvmSet(C, 2, 0, 0.);
  cvmSet(C, 2, 1, 0.);
  cvmSet(C, 2, 2, 1.);

  CvCapture* tCapture = 0;
  char file[PATH_MAX], pFilename[PATH_MAX];

  cvNamedWindow ( "Source", 1 );

  IplImage *tFrame = 0;
  IplImage *grey = 0;

  //create recogniser
  P::Config cfg(ASOF_CONFIG);
  #ifdef USE_GPUSIFT
  P::DetectGPUSIFT det;
  #else
  P::DetectSIFT det;
  #endif

  Object3D object;
  ODetect3D odetect;
  odetect.SetCameraParameter(C);

  #ifdef SOURCE_CAPTURE
  tCapture = cvCaptureFromCAM(0);
  if(tCapture == 0) exit(0);
  //cvSetCaptureProperty(tCapture, CV_CAP_PROP_FRAME_WIDTH, 640);
  //cvSetCaptureProperty(tCapture, CV_CAP_PROP_FRAME_HEIGHT, 480);
  #endif

  int key;
  unsigned startidx, cnt;
  startidx=cnt=cfg.GetValueInt("IMAGE_START_INDEX");
  unsigned endidx=cfg.GetValueInt("IMAGE_END_INDEX");
//  struct timespec start1, end1;
  strcpy(file,cfg.GetValueString("IMAGE_FILES").c_str());
  unsigned image_step = cfg.GetValueInt("IMAGE_STEP");


  for (;;) {
    snprintf(pFilename,PATH_MAX, file, cnt);

    #ifdef SOURCE_CAPTURE
      tFrame = cvQueryFrame ( tCapture );
      if ( !tFrame ) break;
    #else
    #ifdef SOURCE_FILES
      cout<<"File: "<<pFilename<<endl;
      tFrame=cvLoadImage ( pFilename, 1 );
    #endif
    #endif
    if ( !tImg ) {
        tImg = cvCreateImage ( cvGetSize ( tFrame ), 8, 3 );
        grey = cvCreateImage ( cvGetSize ( tFrame ), 8, 1 );
        img = cvCreateImage ( cvGetSize ( tFrame ), 8, 3 );
        pMapX  = cvCreateMat ( tFrame->height,  tFrame->width, CV_32FC1 );
        pMapY  = cvCreateMat ( tFrame->height,  tFrame->width, CV_32FC1 );
        cvInitUndistortMapExact (pIntrinsicDistort, pDistortion, pMapX,  pMapY);
    }
    #ifdef DO_UNDISTORT
    cvRemap ( tFrame, tImg, pMapX, pMapY );
    #else
    cvCopy ( tFrame, tImg, 0 );
    #endif
    cvConvertImage(tImg,grey);

//    clock_gettime(CLOCK_REALTIME, &start1); //CLOCK_THREAD_CPUTIME_ID
    /******************** do all the hard stuff ***************************/

    det.Operate(grey,keys);
    //det.Draw(tImg,keys);
    if(keys.Size()>0 && firstRun)
    {
      CreateModel(keys, object);
      firstRun=false;
    }else if (keys.Size()>0 && !firstRun)
    {
      odetect.SetDebugImage(tImg);
      if(odetect.Detect(keys, object))
      {
        SDraw::DrawPoly(tImg, object.contour.v, CV_RGB(0,255,0), 2);
      }
    }

      
    /**********************************************************************/
//    clock_gettime(CLOCK_REALTIME, &end1);
//    cout<<"Time [s]: "<<P::timespec_diff(&end1, &start1)<<endl;


    #ifdef LOG_IMAGES
    char filename[1024];
    snprintf(filename,1024,"log/tImg_%04d.jpg",cnt);
    cvSaveImage(filename,tImg);
    #endif

    cvShowImage ( "Source", tImg );

    if (cnt<endidx) cnt+=image_step;
    do{
      key = cvWaitKey ( 10 );
    }while ((((char)key)!=' ') && (((char)key)!='c') && (((char)key) != 27) && STEP);
    if ( ( char ) key == 27 ) {
      break;
    }
    #ifdef SOURCE_FILES
    cvReleaseImage ( &tFrame );
    #endif
  }

  #ifdef SOURCE_CAPTURE
  cvReleaseCapture(&tCapture);
  #endif
  if (tImg!=0){
    cvReleaseImage ( &tImg );
    cvReleaseImage ( &grey );
    cvReleaseImage ( &img);
    cvReleaseMat ( &pMapX ), cvReleaseMat ( &pMapY );
    cvReleaseMat ( &pIntrinsicDistort), cvReleaseMat ( &pDistortion);
    cvReleaseMat (&C);
  }
  cvReleaseCapture ( &tCapture );
  cvDestroyWindow ( "Source" );

  return 0;
}



/************************** SOME METHODES *************************/


bool CreateModel(Array<KeypointDescriptor*> &keys, Object3D &object)
{
  Vector2 p;
  Matrix H;
  Array<Vector2> vs3d;
  Array<KeypointDescriptor*> ks;

  //TurnBox04
  //contour.PushBack(Vector2(454,188)); vs3d.PushBack(Vector2(0.,0.));
  //contour.PushBack(Vector2(558,201)); vs3d.PushBack(Vector2(.127,0.));
  //contour.PushBack(Vector2(528,379)); vs3d.PushBack(Vector2(.127,0.07));
  //contour.PushBack(Vector2(430,368)); vs3d.PushBack(Vector2(0.,0.07));
  //MoveCamBox02
  contour.PushBack(Vector2(295,345)); vs3d.PushBack(Vector2(0.,0.));
  contour.PushBack(Vector2(255,250)); vs3d.PushBack(Vector2(0.,0.07));
  contour.PushBack(Vector2(433,239)); vs3d.PushBack(Vector2(.127,0.07));
  contour.PushBack(Vector2(464,350)); vs3d.PushBack(Vector2(.127,0.));


  SPolygon poly(contour);

  ComputeHomography(contour,vs3d,H);

  for (unsigned i=0; i<keys.Size(); i++)
  {
    if (poly.Inside(keys[i]->p))
    {
      ks.PushBack(keys[i]);
      p = MapPoint(keys[i]->p, H);
      ks.Last()->SetPos((float)p.x, (float)p.y, 0.);
      ks.Last()->Draw(tImg,*ks.Last(),CV_RGB(0,0,255));
    }
  }

  ModelObject3D model;

  //model.AddToModel(ks,object);

  //load/save object model
  //model.SaveModel("GuteLaune3D.dat",object);
  //model.LoadModel("GuteLaune3D.dat",object);
  //model.LoadModel("jasmin6.sift",object);
  model.LoadModel("GuteLaune.sift",object);

  SDraw::DrawPoly(tImg, contour,CV_RGB(0,0,255),2);
  object.id=0;

  cout<<"Create object model ...."<<endl;
  cout<<"Codebook size: "<<object.codebook.Size()<<endl;

  return true;
}





