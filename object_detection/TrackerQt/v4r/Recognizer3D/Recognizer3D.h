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
 /**
 * @file Recognizer3D.h
 * @author Thomas Moerwald (Vienna University of Technology)
 * @date Mai 2010
 * @version 0.1
 * @brief Main file of Recognizer for 3D pose estimation using SIFT features
 * @namespace blortRecognizer
 */

#ifndef _RECOGNIZER_3D_
#define _RECOGNIZER_3D_

#include "DetectGPUSIFT.hh"
#include "ODetect3D.hh"
#include "Object3D.hh"
#include "ModelObject3D.hh"
#include "KeypointDescriptor.hh"
#include "PoseCv.hh"
#include "v4r/TomGine/tgModel.h"
#include "v4r/TomGine/tgPose.h"


namespace blortRecognizer{

/** @brief intrinsic camera parameter 
*   lookup OpenCV Camera Calibration and 3D Reconstruction
*   http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html */
struct CameraParameter {
	int w,h;
	float fx, fy;
	float cx, cy;
	float k1, k2, k3;
	float p1, p2;
};

struct Siftex{
  TomGine::vec3 pos;
  TomGine::vec3 normal;
  TomGine::vec3 viewray;
};

/** @brief Recognizer for 3D pose estimation using SIFT features */
class Recognizer3D{

public:
	/** @brief Construction of Recognizer
	*   @param camParam camera parameter for camera calibration
	*   @param display display sifts and found object in a window */
	Recognizer3D(const CameraParameter& camParam, bool display = false);
	~Recognizer3D();
	
	void setCameraParameter(const blortRecognizer::CameraParameter& camParam);
	
	void setGaussian(int kernel, float stdDeviation){ m_gauss_kernel = kernel; m_gauss_dev = stdDeviation; }
	
  void setQuiet(bool b) { m_quiet = b; }

	/** @brief recognizes a object by using the loaded sift model file
	*   @param tFrame Image/Pixel map to search for the sift model
	*   @param pose returned pose of the object (if found)
	*   @param conf returned confidence of pose of the object
	*   @return true if object found, false if not */
	bool recognize(IplImage* tFrame, TomGine::tgPose& pose, float &conf);

	/** @brief add sift features to sift model of an object
	*   @param tFrame image/pixel map to search for new sift features
	*   @param model shape description of model using faces and vertices (see TomGine::tgModel)
	*   @param pose pose of the model as seen in the image */
  bool learnSifts(IplImage* tFrame, const TomGine::tgModel& model, const  TomGine::tgPose& pose);
	
	/** @brief load a sift model
	*   @param sift_file relative path and name to sift file (i.e.: "../Resources/sift/TeaBox.sift")
	*   @return success of loading the file */
	bool loadModelFromFile(const char* sift_file);
	
	/** @brief save a sift model
	*   @param sift_file relative path and name to sift file (i.e.: "../Resources/sift/TeaBox.sift")
	*   @return success of saving the file */
	bool saveModelToFile(const char* sift_file);
	
	/** @brief get position and normal vector of sift features in model
	*   @param pl 3D point list of sift features (relative to object)
	*   @param nl 3D normal vector list of sift features (in object space) */
	void getSifts(std::vector<Siftex>& sl){ sl = m_siftexlist; }
	
	/** @brief get position and normal vector of sift features in model detected in the last call of learnSifts()
	*   @param pl 3D point list of sift features (relative to object)
	*   @param nl 3D normal vector list of sift features (in object space) */
	void getLastSifts(std::vector<Siftex> &sl){ sl = m_lastsiftexlist; }

  inline unsigned getNumberOfSifts()
  {
    return m_sift_model.getCodebookSize();
  }
	
private:
	Recognizer3D();
	
	P::DetectGPUSIFT sift;
	P::ModelObject3D m_sift_model_learner;
	P::ODetect3D m_detect;
	P::Object3D m_sift_model;
	
	P::Array<P::KeypointDescriptor*> m_image_keys;
	
	std::vector<Siftex> m_lastsiftexlist;
	std::vector<Siftex> m_siftexlist;
	
	CvMat *pIntrinsicDistort;
	CvMat *pDistortion;
	CvMat *C;
	CvMat *pMapX, *pMapY;
	CameraParameter m_cp;
	
	bool m_model_loaded;
	bool m_display;
	
	int m_gauss_kernel;
	float m_gauss_dev;
	
  bool m_quiet;

	void Convert(P::PoseCv& p1, TomGine::tgPose& p2);

	
};

}

#endif /* _RECOGNIZER_3D_ */
