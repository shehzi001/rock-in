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

#include "Resources.h"

#include "v4r/TomGine/tgError.h"

using namespace Tracking;

// *** PRIVATE ***

int Resources::SearchName(NameList* list, const char* filename){
	// parse through name list
	int i = 0;
	NameList::iterator it_name = list->begin();
	while(it_name != list->end()){
		if(!strcmp((*it_name),filename))
			return i;
		it_name++;
		i++;
	}

	// not found
	return -1;
}

// *** PUBLIC ***

Resources::Resources(){
	m_capture = 0;
	m_image = 0;
	m_ip = 0;
	m_showlog = false;
}

Resources::~Resources(){
	ReleaseCapture();
	//ReleaseScreen();
	ReleaseImageProcessor();

	ReleaseShader();

	if(m_showlog) printf("Resources released\n");
}


// *** Initialisation ***
IplImage* Resources::InitCapture(const char* file){
	m_capture = cvCaptureFromFile(file);
	if(!m_capture) {
		char errmsg[128];
		sprintf(errmsg, "[Resources::InitCapture] Error could not read '%s'\n", file );
		throw std::runtime_error(errmsg);
	}

	double w = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_WIDTH );
	double h = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_HEIGHT );

	if(m_showlog) printf("Camera settings: %.1f x %.1f\n", w, h);

	m_image = cvQueryFrame(m_capture);
	cvConvertImage(m_image, m_image, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
	//cvFlip(m_image, m_image, 1);
	return m_image;
}

IplImage* Resources::InitCapture(float width, float height, int camID){
	m_capture = cvCreateCameraCapture(camID);
	if(!m_capture) {
		char errmsg[128];
		sprintf(errmsg, "[Resources::InitCapture] Error could not initialise camera %d\n", camID );
		throw std::runtime_error(errmsg);
	}

	cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_WIDTH, width );
	cvSetCaptureProperty(m_capture, CV_CAP_PROP_FRAME_HEIGHT, height );

	double w = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_WIDTH );
	double h = cvGetCaptureProperty( m_capture, CV_CAP_PROP_FRAME_HEIGHT );

	if(m_showlog) printf("Camera settings: %.1f x %.1f\n", w, h);

	m_image = cvQueryFrame(m_capture);
	cvConvertImage(m_image, m_image, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
	//cvFlip(m_image, m_image, 1);
	return m_image;
}

TomGine::tgImageProcessor* Resources::InitImageProcessor(int width, int height)
{
	if(width==0||height==0){
		printf("[Resources::GetImageProcessor] Error TomGine::tgImageProcessor needs width and height for initialisation\n");
		return 0;
	}

	if(!m_ip)
		m_ip = new TomGine::tgImageProcessor( width, height );

	return m_ip;
}

// *** Release-functions ***
void Resources::ReleaseCapture(){
	if(m_capture)
		cvReleaseCapture(&m_capture);
	m_capture = 0;
}

void Resources::ReleaseImageProcessor(){
	if(m_ip)
		delete(m_ip);
	m_ip = 0;
}


// *** Get-functions ***
IplImage* Resources::GetNewImage(){

	if(!m_capture){
		printf("[Resources::GetNewImage] Error camera not initialised\n" );
		return 0;
	}
	IplImage* img;

	try{
		img = cvQueryFrame(m_capture);
	}
	catch(char const* e){
		printf("[Resources::GetNewImage()] Warning: %s", e);
	}

	if(img != NULL){
		m_image = img;
// 		cvConvertImage(m_image, m_image, CV_CVTIMG_SWAP_RB);
	}

	//cvFlip(m_image, m_image, 1);
	return m_image;
}

 bool Resources::GetNewImage(IplImage* img){

  if(!m_capture){
    printf("[Resources::GetNewImage] Error camera not initialised\n" );
    return 0;
  }

  try{
    img = cvQueryFrame(m_capture);
  }
  catch(char const* e){
    printf("[Resources::GetNewImage()] Warning: %s", e);
  }

  if(img != NULL)
  {
    m_image = img;
    return true;
  }

  img = m_image;
  return false;
}

IplImage* Resources::GetImage(){
	if(!m_image){
		return GetNewImage();
	}
	return m_image;
}

TomGine::tgImageProcessor* Resources::GetImageProcessor(){
	if(!m_ip){
		printf("[Resources::GetImageProcessor] Error TomGine::tgImageProcessor not initialised\n" );
		return 0;
    }
	return m_ip;
}

TomGine::tgShader* Resources::GetShader(int id){
	return m_shaderList[id];
}

// *** Add
int	Resources::AddShader(	const char* shadername,
							const char* vertex_file,
							const char* fragment_file,
							const char* header)
{
	int shaderID=-1;
	/*
	// check if texture allready loaded before by comparing filename
	int shaderID = SearchShaderName(shadername);
	if(shaderID != -1)
		return shaderID;	// return existing texture ID
	*/

	// texture doesn't exist and needs to be loaded
  std::string vertex_fullname, fragment_fullname, header_fullname;

//	printf("[Resources::AddShader] %s\n", fragment_fullname);

  std::string shader_path = TomGine::tgImageProcessor::getShaderPath();

  if(vertex_file)
    vertex_fullname = shader_path + std::string(vertex_file);
  if(fragment_file)
    fragment_fullname = shader_path + std::string(fragment_file);
	if(header)
    header_fullname = shader_path + std::string(header);

  TomGine::tgShader* shader = new TomGine::tgShader(vertex_fullname, fragment_fullname, header_fullname);

	char* name = new char[FN_LEN];
	strcpy(name, shadername);

	// put model into texture list
	m_shaderNameList.push_back(name);
	m_shaderList.push_back(shader);

	shaderID = m_shaderList.size()-1;

	if(m_showlog) printf("TomGine::tgShader %i loaded: %s\n", shaderID, name);

	return shaderID;
}

// *** Release
void Resources::ReleaseShader(){
	// release TomGine::tgShader
	ShaderList::iterator it_shader = m_shaderList.begin();
	while(it_shader != m_shaderList.end()){
		delete(*it_shader);
		it_shader++;
	}
	// release Shadernames
	NameList::iterator it_name = m_shaderNameList.begin();
	while(it_name != m_shaderNameList.end()){
		delete(*it_name);
		it_name++;
	}
}

void Resources::ReleaseShader(int id){
	if(m_shaderList[id])
		delete(m_shaderList[id]);
	m_shaderList[id] = 0;
	if(m_showlog) printf("TomGine::tgShader %i released\n", id);
#ifdef DEBUG
	TomGine::tgCheckError("[Resources::ReleaseShader]");
#endif
}


// *** Search-functions ***
int	Resources::SearchShaderName(const char* filename){
	return SearchName(&m_shaderNameList, filename);
}
