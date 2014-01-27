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

#include "Recognizer3D.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "SDraw.hh"
#include "cvgeometry.h"
#include "v4r/TomGine/tgModelLoader.h"
#include "v4r/TomGine/tgCollission.h"

using namespace blortRecognizer;

void Recognizer3D::Convert(P::PoseCv& p1, TomGine::tgPose& p2){

  TomGine::mat3 cR;
  TomGine::vec3 ct;

  cR[0] = cvmGet(p1.R,0,0); cR[3] = cvmGet(p1.R,0,1); cR[6] = cvmGet(p1.R,0,2);
  cR[1] = cvmGet(p1.R,1,0); cR[4] = cvmGet(p1.R,1,1); cR[7] = cvmGet(p1.R,1,2);
  cR[2] = cvmGet(p1.R,2,0); cR[5] = cvmGet(p1.R,2,1); cR[8] = cvmGet(p1.R,2,2);

  ct.x = cvmGet(p1.t,0,0);
  ct.y = cvmGet(p1.t,1,0);
  ct.z = cvmGet(p1.t,2,0);

  p2.SetPose(cR,ct);
}

Recognizer3D::Recognizer3D(const blortRecognizer::CameraParameter& camParam, bool display)
{
  pIntrinsicDistort = cvCreateMat(3,3, CV_64FC1);
  pDistortion = cvCreateMat(1,4, CV_64FC1);
  C = cvCreateMat(3,3, CV_32F);

  m_cp = camParam;

  cvmSet(pIntrinsicDistort, 0, 0, camParam.fx);
  cvmSet(pIntrinsicDistort, 0, 1, 0.);
  cvmSet(pIntrinsicDistort, 0, 2, camParam.cx);
  cvmSet(pIntrinsicDistort, 1, 0, 0.);
  cvmSet(pIntrinsicDistort, 1, 1, camParam.fy);
  cvmSet(pIntrinsicDistort, 1, 2, camParam.cy);
  cvmSet(pIntrinsicDistort, 2, 0, 0.);
  cvmSet(pIntrinsicDistort, 2, 1, 0.);
  cvmSet(pIntrinsicDistort, 2, 2, 1.);

  cvmSet(pDistortion, 0, 0, camParam.k1);
  cvmSet(pDistortion, 0, 1, camParam.k2);
  cvmSet(pDistortion, 0, 2, camParam.k3);
  cvmSet(pDistortion, 0, 3, 0.0);

  //camera matrix for undistorted images
  cvmSet(C, 0, 0, camParam.fx);
  cvmSet(C, 0, 1, 0.);
  cvmSet(C, 0, 2, camParam.cx);
  cvmSet(C, 1, 0, 0.);
  cvmSet(C, 1, 1, camParam.fy);
  cvmSet(C, 1, 2, camParam.cy);
  cvmSet(C, 2, 0, 0.);
  cvmSet(C, 2, 1, 0.);
  cvmSet(C, 2, 2, 1.);

  m_detect.SetCameraParameter(C);

  pMapX  = cvCreateMat (camParam.h,  camParam.w, CV_32FC1 );
  pMapY  = cvCreateMat (camParam.h,  camParam.w, CV_32FC1 );
  cvInitUndistortMapExact(pIntrinsicDistort, pDistortion, pMapX,  pMapY);

  m_gauss_kernel = 5;
  m_gauss_dev = 1.5;

  m_display = display;
  if(m_display){
    cvNamedWindow ( "Recognizer3D", 1 );
    cvResizeWindow( "Recognizer3D", camParam.w, camParam.h );
  }

  m_model_loaded = false;
  m_quiet = true;
}

Recognizer3D::~Recognizer3D()
{
  cvReleaseMat(&pIntrinsicDistort);
  cvReleaseMat(&pDistortion);
  cvReleaseMat(&C);
  cvReleaseMat(&pMapX);
  cvReleaseMat(&pMapY);

  if(m_display)
    cvDestroyWindow ( "Recognizer3D" );
}

void Recognizer3D::setCameraParameter(const blortRecognizer::CameraParameter& camParam)
{
  m_cp = camParam;

  cvmSet(pIntrinsicDistort, 0, 0, camParam.fx);
  cvmSet(pIntrinsicDistort, 0, 1, 0.);
  cvmSet(pIntrinsicDistort, 0, 2, camParam.cx);
  cvmSet(pIntrinsicDistort, 1, 0, 0.);
  cvmSet(pIntrinsicDistort, 1, 1, camParam.fy);
  cvmSet(pIntrinsicDistort, 1, 2, camParam.cy);
  cvmSet(pIntrinsicDistort, 2, 0, 0.);
  cvmSet(pIntrinsicDistort, 2, 1, 0.);
  cvmSet(pIntrinsicDistort, 2, 2, 1.);

  cvmSet(pDistortion, 0, 0, camParam.k1);
  cvmSet(pDistortion, 0, 1, camParam.k2);
  cvmSet(pDistortion, 0, 2, camParam.k3);
  cvmSet(pDistortion, 0, 3, 0.0);

  //camera matrix for undistorted images
  cvmSet(C, 0, 0, camParam.fx);
  cvmSet(C, 0, 1, 0.);
  cvmSet(C, 0, 2, camParam.cx);
  cvmSet(C, 1, 0, 0.);
  cvmSet(C, 1, 1, camParam.fy);
  cvmSet(C, 1, 2, camParam.cy);
  cvmSet(C, 2, 0, 0.);
  cvmSet(C, 2, 1, 0.);
  cvmSet(C, 2, 2, 1.);

  m_detect.SetCameraParameter(C);

  pMapX  = cvCreateMat (camParam.h,  camParam.w, CV_32FC1 );
  pMapY  = cvCreateMat (camParam.h,  camParam.w, CV_32FC1 );
  cvInitUndistortMapExact(pIntrinsicDistort, pDistortion, pMapX,  pMapY);
}

bool Recognizer3D::recognize(IplImage* tFrame, TomGine::tgPose& pose, float &conf)
{
  P::PoseCv cvPose;
  bool detected = false;
  conf = 0.0f;

  if(!m_model_loaded || m_sift_model.codebook.Empty()){
    printf("[Recognizer3D::recognize] Error: no sift points in codebook of object (did you load the model before?\n");
    return false;
  }

  IplImage* tImg = 0;
  IplImage* grey = 0;

  tImg = cvCreateImage ( cvGetSize ( tFrame ), 8, 3 );
  grey = cvCreateImage ( cvGetSize ( tFrame ), 8, 1 );
  
  // undistort
  cvRemap(tFrame, tImg, pMapX, pMapY );

  // convert to gray scale image
  cvConvertImage(tImg, grey);

  sift.Operate(grey, m_image_keys);
  m_detect.SetDebugImage(tImg);

  if(m_display)
    sift.Draw(tImg, m_image_keys);

  if(m_detect.Detect(m_image_keys, m_sift_model)){

    if(m_sift_model.conf > 0.03){
      conf = m_sift_model.conf;


      if(m_display){
        P::SDraw::DrawPoly(tImg, m_sift_model.contour.v, CV_RGB(0,255,0),2);
        m_detect.DrawInlier(tImg, CV_RGB(0,255,0));
      }
      CopyPoseCv(m_sift_model.pose, cvPose);
      Convert(cvPose, pose);
      if(!m_quiet)
        printf("[Recognizer3D::recognize] object found (conf: %f)\n", m_sift_model.conf);
      detected = true;
    }else{
      if(m_display)
        P::SDraw::DrawPoly(tImg, m_sift_model.contour.v, CV_RGB(255,0,0),2);

      if(!m_quiet)
        printf("[Recognizer3D::recognize] No object (conf: %f)\n", m_sift_model.conf);
      detected = false;
    }

  }else{
    conf = m_sift_model.conf;

    if(m_display)
      P::SDraw::DrawPoly(tImg, m_sift_model.contour.v, CV_RGB(255,0,0),2);

    if(!m_quiet)
      printf("[Recognizer3D::recognize] No object (conf: %f)\n", m_sift_model.conf);
    detected = false;
  }

  if(m_display){
    cvShowImage("Recognizer3D", tImg);

    //		do{
    //		  key = cvWaitKey ( 10 );
    //		}while ( (((char)key) != ' ') && (((char)key) != 27) );
  }

  cvReleaseImage(&tImg);
  cvReleaseImage(&grey);

  return detected;
}

bool Recognizer3D::learnSifts(IplImage* tFrame, const TomGine::tgModel& m1, const TomGine::tgPose& pose)
{	
  P::Array<P::KeypointDescriptor*> m_tmp_keys;

  TomGine::mat3 R, Rt;
  TomGine::vec3 t;
  TomGine::tgModel m2 = m1;
  m_lastsiftexlist.clear();

  // Transform model vertices to camera coordinats
  pose.GetPose(R,t);
  Rt = R.transpose();
  for(unsigned int i=0; i<m2.m_vertices.size(); i++)
  {
    m2.m_vertices[i].pos = (R * m2.m_vertices[i].pos) + t;
    m2.m_vertices[i].normal = R * m2.m_vertices[i].normal;
  }

  IplImage* tImg = 0;
  IplImage* grey = 0;
  float dcpfx = 1/m_cp.fx;
  float dcpfy = 1/m_cp.fy;

  tImg = cvCreateImage ( cvGetSize ( tFrame ), 8, 3 );
  grey = cvCreateImage ( cvGetSize ( tFrame ), 8, 1 );
  
  // undistort
  cvRemap(tFrame, tImg, pMapX, pMapY );

  // convert to gray scale image
  cvConvertImage(tImg, grey);

  sift.Operate(grey, m_image_keys);

  for(unsigned int i=0; i<m_image_keys.Size(); i++)
  {
    TomGine::vec3 point, normal;
    std::vector<TomGine::vec3> pl;		// point list
    std::vector<TomGine::vec3> nl;		// normal list
    std::vector<double> zl; // z-value list (depth)
    float u,v,x,y;

    u = (float)m_image_keys[i]->p.x;
    v = (float)m_image_keys[i]->p.y;
    x = (u-m_cp.cx) * dcpfx;
    y = (v-m_cp.cy) * dcpfy;

    // Create ray from pixel
    TomGine::tgRay ray;
    ray.start = TomGine::vec3(0.0f,0.0f,0.0f);
    ray.dir =  TomGine::vec3(x,y,1.0f);
    ray.dir.normalize();

    if(	TomGine::tgCollission::IntersectRayModel(pl, nl, zl, ray, m2)
        && !pl.empty()
        && !zl.empty())
    {
      // determine nearest intersection point
      unsigned int idx_min = 0;
      float z_min = zl[0];
      for(unsigned int idx=0; idx<zl.size(); idx++){
        if(zl[idx] < z_min){
          idx_min = idx;
          z_min = zl[idx];
        }
      }

      if(z_min > 0.0f){
        // Transform to object space
        point = (Rt * (pl[idx_min] - t));
        normal = (Rt * nl[idx_min]);
        TomGine::vec3 r = (Rt * ray.dir) * zl[idx_min];

        Siftex s;
        s.pos = point;
        s.normal = normal;
        s.viewray = r;

        normal.normalize();
        r.normalize();

        if( (r * normal) < -0.1f)
        {
          // Save sift
          m_siftexlist.push_back(s);
          m_lastsiftexlist.push_back(s);
          m_image_keys[i]->SetPos(point.x, point.y, point.z);
          m_tmp_keys.PushBack(m_image_keys[i]);
        }
      }
    }

  }

  if(m_tmp_keys.Size() > 0)
  {
    m_sift_model_learner.AddToModel(m_tmp_keys, m_sift_model);
    if(!m_quiet)
      printf("[Recognizer3D::learnSifts] added %d sifts to model\n", m_tmp_keys.Size());
    m_model_loaded = true;
  }
  else
  {
    if(!m_quiet)
      printf("[Recognizer3D::learnSifts] Warning, no sifts found\n");
  }

  if(m_display){
    sift.Draw(tImg, m_tmp_keys);
    cvShowImage("Recognizer3D", tImg);

    //    int key;
    //		do{
    //      key = cvWaitKey ( );
    //		}while ( (((char)key) != ' ') && (((char)key) != 27) );
  }

  cvReleaseImage(&tImg);
  cvReleaseImage(&grey);

  return true;
}

bool Recognizer3D::loadModelFromFile(const char* sift_file)
{
  m_model_loaded = m_sift_model_learner.LoadModel(sift_file, m_sift_model);
  if(!m_quiet)
    printf("[Recognizer3D::loadModelFromFile] sift model loaded from '%s'\n", sift_file);
  return true;
}

bool Recognizer3D::saveModelToFile(const char* sift_file)
{
  m_sift_model_learner.SaveModel(sift_file, m_sift_model);
  printf("[Recognizer3D::saveModelToFile] sift model saved to '%s'\n", sift_file);
  return true;
}
