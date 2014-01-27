/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2011, Thomas Mörwald
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
#include "recognizer.h"

using namespace blortRecognizer;
using namespace std;

Recognizer::Recognizer(const Tracking::Tracker::Parameter &params)
{

  CameraParameter recParams;
  recParams.w = params.camPar.width;
  recParams.h = params.camPar.height;
  recParams.fx = params.camPar.fx;
  recParams.fy = params.camPar.fy;
  recParams.cx = params.camPar.cx;
  recParams.cy = params.camPar.cy;
  recParams.k1 = params.camPar.k1;
  recParams.k2 = params.camPar.k2;
  recParams.k3 = params.camPar.k3;
  recParams.p1 = params.camPar.p1;
  recParams.p2 = params.camPar.p2;
  m_recognizer = new Recognizer3D(recParams, false);

  m_cam_pose.SetPose(params.camPar.rot, params.camPar.pos);

  connect(this, SIGNAL(start_call()), this, SLOT(start()));
}

Recognizer::~Recognizer()
{
  delete m_recognizer;
}

void Recognizer::learn(const cv::Mat& img, const TomGine::tgModel& model, const TomGine::tgPose& pose)
{
  {
    QMutexLocker ml(&mutex);
    img.copyTo(m_img);
    m_model = model;
    m_pose = pose;
    cmd = LEARN;
  }

  emit start_call();
  wait();
}

void Recognizer::recognize(const cv::Mat& img, TomGine::tgPose& pose, float &confidence)
{
  {
    QMutexLocker ml(&mutex);
    img.copyTo(m_img);
    cmd = RECOGNIZE;
  }

  emit start_call();
  wait();

  pose = m_cam_pose * m_pose;
  confidence = m_confidence;

  //  TomGine::mat3 R;
  //  TomGine::vec3 t;
  //  m_pose.GetPose(R, t);
  //  cout << "rec_pose: " << endl;
  //  cout << "  " << t.x << " " << t.y << " " << t.z << endl;
  //  cout << "  " << R[0] << " " << R[3] << " " << R[6] << endl;
  //  cout << "  " << R[1] << " " << R[4] << " " << R[7] << endl;
  //  cout << "  " << R[2] << " " << R[5] << " " << R[8] << endl;

  //  m_cam_pose.GetPose(R, t);
  //  cout << "cam_pose: " << endl;
  //  cout << "  " << t.x << " " << t.y << " " << t.z << endl;
  //  cout << "  " << R[0] << " " << R[3] << " " << R[6] << endl;
  //  cout << "  " << R[1] << " " << R[4] << " " << R[7] << endl;
  //  cout << "  " << R[2] << " " << R[5] << " " << R[8] << endl;

  //  pose.GetPose(R, t);
  //  cout << "world_pose: " << endl;
  //  cout << "  " << t.x << " " << t.y << " " << t.z << endl;
  //  cout << "  " << R[0] << " " << R[3] << " " << R[6] << endl;
  //  cout << "  " << R[1] << " " << R[4] << " " << R[7] << endl;
  //  cout << "  " << R[2] << " " << R[5] << " " << R[8] << endl;

  //  cout << "confidence: " << confidence << endl;
}

void Recognizer::load(const std::string& filename)
{
  m_recognizer->loadModelFromFile(filename.c_str());

  cout << "[Recognizer::load] SIFTS: " << m_recognizer->getNumberOfSifts() << endl;
}

void Recognizer::save(const std::string& filename)
{
  cout << "[Recognizer::save] SIFTS: " << m_recognizer->getNumberOfSifts() << endl;

  if(m_recognizer->getNumberOfSifts()>0)
    m_recognizer->saveModelToFile(filename.c_str());
  else
    cout << "[Recognizer::save] Cannot save empty SIFT model." << endl;
}

void Recognizer::run()
{
  QMutexLocker ml(&mutex);
  if(cmd==LEARN && !m_img.empty())
  {
    IplImage img(m_img);
    m_recognizer->learnSifts(&img, m_model, m_pose);
    cout << "[Recognizer::LEARN] SIFTS: " << m_recognizer->getNumberOfSifts() << endl;
  }
  if(cmd==RECOGNIZE)
  {
    IplImage img(m_img);
    m_recognizer->recognize(&img, m_pose, m_confidence);
  }
}




