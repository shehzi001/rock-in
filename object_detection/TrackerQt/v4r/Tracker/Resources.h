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
#ifndef __RESOURCES_H__
#define __RESOURCES_H__

#include "headers.h"
#include "TrackerModel.h"
#include "ModelLoader.h"
#include "v4r/TomGine/tgSingleton.h"
#include "v4r/TomGineIP/tgShader.h"
#include "v4r/TomGineIP/tgImageProcessor.h"
#include <opencv2/opencv.hpp>

#ifndef FN_LEN
#define FN_LEN 256
#endif

#define g_Resources Resources::GetInstance()

namespace Tracking
{

typedef std::vector<TomGine::tgShader*> ShaderList;
typedef std::vector<char*> NameList;

class Resources : public TomGine::tgSingleton<Resources>
{
  friend class TomGine::tgSingleton<Resources>;
private:
  Resources ();

  // Singleton Resources (one instance in programm)
  CvCapture* m_capture;
  IplImage* m_image;

  TomGine::tgImageProcessor *m_ip;

  // Resources lists
  ShaderList m_shaderList;

  // Name lists
  NameList m_shaderNameList;

//  std::string m_shaderPath;

  bool m_showlog;

  int SearchName (NameList* list, const char* filename);

public:
  ~Resources ();
  static Resources*
  GetInstance ()
  {
    return TomGine::tgSingleton<Resources>::GetInstance ();
  }

  // Initialisation
  IplImage* InitCapture (const char* file);

  IplImage* InitCapture (float width = 640.0, float height = 480.0, int camID = CV_CAP_ANY);

  TomGine::tgImageProcessor* InitImageProcessor (int width, int height);

  // Release-functions
  void ReleaseCapture ();
  void ReleaseImageProcessor ();

  // Set-function
  void ShowLog (bool b)
  {
    m_showlog = b;
  }

  // Get-functions
  IplImage* GetNewImage ();
  bool GetNewImage(IplImage* img);
  IplImage* GetImage ();
  TomGine::tgImageProcessor* GetImageProcessor ();

  TomGine::tgShader* GetShader (int id);

  // Add-functions
  int AddShader (const char* shadername, const char* vertex_file = NULL, const char* fragment_file = NULL,
                 const char* header = NULL);

  // Release-functions
  void ReleaseShader ();
  void ReleaseShader (int id);

  // Search-functions
  int SearchShaderName (const char* filename);
};

} // namespace Tracking

#endif
