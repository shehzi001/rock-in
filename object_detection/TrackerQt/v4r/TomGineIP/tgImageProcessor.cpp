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

#include "v4r/TomGine/tgError.h"
#include "v4r/TomGine/tgTimer.h"
#include "tgImageProcessor.h"
#include <stdexcept>

using namespace TomGine;

tgImageProcessor::tgImageProcessor(unsigned w, unsigned h)
{
  std::string path_gauss = std::string(TOMGINE_IP_SHADER) + "ipGauss.frag";
  std::string path_gradX = std::string(TOMGINE_IP_SHADER) + "ipGradientsX.frag";
  std::string path_gradY = std::string(TOMGINE_IP_SHADER) + "ipGradientsY.frag";
  std::string path_sobel = std::string(TOMGINE_IP_SHADER) + "ipSobel.frag";
  std::string path_sobel_normal = std::string(TOMGINE_IP_SHADER) + "ipSobelNormal.frag";
  std::string path_sobel_adaptive = std::string(TOMGINE_IP_SHADER) + "ipSobelAdaptive.frag";
  std::string path_thinning = std::string(TOMGINE_IP_SHADER) + "ipThinning.frag";
  std::string path_spreading = std::string(TOMGINE_IP_SHADER) + "ipSpreading.frag";
  std::string path_rgb2hsv = std::string(TOMGINE_IP_SHADER) + "ipRGB2HSV.frag";
  std::string path_param2polar = std::string(TOMGINE_IP_SHADER) + "ipParam2Polar.frag";
  std::string path_add = std::string(TOMGINE_IP_SHADER) + "ipAdd.frag";
  std::string path_invert = std::string(TOMGINE_IP_SHADER) + "ipInvert.frag";
  std::string path_multiply = std::string(TOMGINE_IP_SHADER) + "ipMultiply.frag";
  std::string path_threshold = std::string(TOMGINE_IP_SHADER) + "ipThreshold.frag";
  std::string path_dilate = std::string(TOMGINE_IP_SHADER) + "ipDilate.frag";
  std::string path_dot_convolution = std::string(TOMGINE_IP_SHADER) + "ipDotConvolution.frag";

  m_shadeGauss = new tgShader("", path_gauss.c_str());
  m_shadeGradientsX = new tgShader("", path_gradX.c_str());
  m_shadeGradientsY = new tgShader("", path_gradY.c_str());
  m_shadeSobel = new tgShader("", path_sobel.c_str());
  m_shadeSobelNormal = new tgShader("", path_sobel_normal.c_str());
  m_shadeSobelAdaptive = new tgShader("", path_sobel_adaptive.c_str());
  m_shadeThinning = new tgShader("", path_thinning.c_str());
  m_shadeSpreading = new tgShader("", path_spreading.c_str());
  m_shadeRGB2HSV = new tgShader("", path_rgb2hsv.c_str());
  m_shadeParam2Polar = new tgShader("", path_param2polar.c_str());
  m_shadeAdd = new tgShader("", path_add.c_str());
  m_shadeInvert = new tgShader("", path_invert.c_str());
  m_shadeMultiply = new tgShader("", path_multiply.c_str());
  m_shadeThreshold = new tgShader("", path_threshold.c_str());
  m_shadeDilate = new tgShader("", path_dilate.c_str());
  m_shadeDotConvolution = new tgShader("", path_dot_convolution.c_str());

  init(w, h);
  initShader();
  glGenFramebuffers(1, &m_fbo_id);
  m_use_fbo = false;

  //	initFBO(avg_resolution);
#ifdef DEBUG
  tgCheckError("[tgImageProcessor::tgImageProcessor()]");
#endif
}

tgImageProcessor::~tgImageProcessor()
{
  //	if(glIsFramebuffer(fbo)) glDeleteFramebuffers(1, &fbo);
  //	if(glIsTexture(fbo_tex)) glDeleteTextures(1,&fbo_tex);
  //	if(glIsTexture(fbo_tex_depth)) glDeleteTextures(1,&fbo_tex_depth);

  if (m_shadeGauss)
    delete (m_shadeGauss);
  if (m_shadeGradientsX)
    delete (m_shadeGradientsX);
  if (m_shadeGradientsY)
    delete (m_shadeGradientsY);
  if (m_shadeSobel)
    delete (m_shadeSobel);
  if (m_shadeSobelNormal)
    delete (m_shadeSobelNormal);
  if (m_shadeSobelAdaptive)
    delete (m_shadeSobelAdaptive);
  if (m_shadeThinning)
    delete (m_shadeThinning);
  if (m_shadeSpreading)
    delete (m_shadeSpreading);
  if (m_shadeRGB2HSV)
    delete (m_shadeRGB2HSV);
  if (m_shadeParam2Polar)
    delete (m_shadeParam2Polar);
  if (m_shadeAdd)
    delete (m_shadeAdd);
  if (m_shadeInvert)
    delete (m_shadeInvert);
  if (m_shadeMultiply)
    delete (m_shadeMultiply);
  if (m_shadeThreshold)
    delete (m_shadeThreshold);
  if (m_shadeDilate)
    delete (m_shadeDilate);
  if (m_shadeDotConvolution)
    delete (m_shadeDotConvolution);

  if (glIsFramebuffer(m_fbo_id))
    glDeleteFramebuffers(1, &m_fbo_id);
#ifdef DEBUG
  tgCheckError("[tgImageProcessor::~tgImageProcessor()]");
#endif
}

// Main initialisation function
void tgImageProcessor::init(unsigned width, unsigned height)
{

  m_width = width;
  m_height = height;

  // Initialise camera
  m_cam_ortho.Set(vec3(0, 0, 1), vec3(0, 0, 0), vec3(0, 1, 0), 45.0f, m_width, m_height, 0.1f, 10.0f,
      tgCamera::GL_ORTHO);
}

// Load and compile shaders and set parameters
void tgImageProcessor::initShader()
{
  float sq2 = 1.0f / sqrt(2.0f);

  // distance of neighbouring pixels
  GLfloat dist[9] = { sq2, 1.0f, sq2, 1.0f, 0.0f, 1.0f, sq2, 1.0f, sq2 };

  float hi = 10.0f / 22; // = sqrt((3+10+3)^2 + (3+10+3)^2) = 22.6
  float lo = 3.0f / 22;
  GLfloat sobelY[9] = { -lo, 0.0f, lo, -hi, 0.0f, hi, -lo, 0.0f, lo };
  // dont modify structure of sobelY -> division in sobel.frag
  GLfloat sobelX[9] = { -lo, -hi, -lo, 0.0f, 0.0f, 0.0f, lo, hi, lo };

  float v = 1.0f / 24.0f;
  float dotConv[25] = { v, v, v, v, v, v, v, v, v, v, v, v, 0, v, v, v, v, v, v, v, v, v, v, v, v };

  if (m_shadeThreshold)
  {
    m_shadeThreshold->bind();
    m_shadeThreshold->setUniform("tex1", 0);
    m_shadeThreshold->unbind();
  }
  // Add shader
  if (m_shadeAdd)
  {
    m_shadeAdd->bind();
    m_shadeAdd->setUniform("tex1", 0);
    m_shadeAdd->setUniform("tex2", 1);
    m_shadeAdd->unbind();
  }
  // Multiply shader
  if (m_shadeMultiply)
  {
    m_shadeMultiply->bind();
    m_shadeMultiply->setUniform("tex1", 0);
    m_shadeMultiply->setUniform("tex2", 1);
    m_shadeMultiply->setUniform("tex1grad", false);
    m_shadeMultiply->setUniform("tex2grad", false);
    m_shadeMultiply->unbind();
  }
  // Invert shader
  if (m_shadeInvert)
  {
    m_shadeInvert->bind();
    m_shadeInvert->setUniform("tex", 0);
    m_shadeInvert->unbind();
  }
  // Gauss shader
  if (m_shadeGauss)
  {
    m_shadeGauss->bind();
    m_shadeGauss->setUniform("frame", 0);
    m_shadeGauss->unbind();
  }
#ifdef DEBUG
  tgCheckError("[tgImageProcessor::initShader] Gauss:");
#endif

  if (m_shadeGradientsX)
  {
    m_shadeGradientsX->bind();
    m_shadeGradientsX->setUniform("frame", 0);
    m_shadeGradientsX->unbind();
  }

  if (m_shadeGradientsY)
  {
    m_shadeGradientsY->bind();
    m_shadeGradientsY->setUniform("frame", 0);
    m_shadeGradientsY->unbind();
  }
#ifdef DEBUG
  tgCheckError("[tgImageProcessor::initShader] Gradients:");
#endif

  // Sobel shader
  if (m_shadeSobel)
  {
    m_shadeSobel->bind();
    m_shadeSobel->setUniform("frame", 0);
    m_shadeSobel->setUniform("mask", 1);
    m_shadeSobel->setUniform("binary", false);
    m_shadeSobel->setUniform("mSobelX", mat3(sobelX), GL_FALSE);
    m_shadeSobel->setUniform("mSobelY", mat3(sobelY), GL_FALSE);
    m_shadeSobel->setUniform("fThreshold", 0.1f);
    m_shadeSobel->unbind();
  }
#ifdef DEBUG
  tgCheckError("[tgImageProcessor::initShader] Sobel:");
#endif
  // Sobel shader
  if (m_shadeSobelNormal)
  {
    m_shadeSobelNormal->bind();
    m_shadeSobelNormal->setUniform("frame", 0);
    m_shadeSobelNormal->setUniform("mSobelX", mat3(sobelX), GL_FALSE);
    m_shadeSobelNormal->setUniform("mSobelY", mat3(sobelY), GL_FALSE);
    m_shadeSobelNormal->setUniform("fThreshold", 0.1f);
    m_shadeSobelNormal->unbind();
  }
#ifdef DEBUG
  tgCheckError("[tgImageProcessor::initShader] SobelNormal:");
#endif
  // Sobel shader
  if (m_shadeSobelAdaptive)
  {
    m_shadeSobelAdaptive->bind();
    m_shadeSobelAdaptive->setUniform("frame", 0);
    m_shadeSobelAdaptive->setUniform("mSobelX", mat3(sobelX), GL_FALSE);
    m_shadeSobelAdaptive->setUniform("mSobelY", mat3(sobelY), GL_FALSE);
    m_shadeSobelAdaptive->setUniform("fThreshold", 0.1f);
    m_shadeSobelAdaptive->setUniform("fOffset", 0.0f);
    m_shadeSobelAdaptive->unbind();
  }
#ifdef DEBUG
  tgCheckError("[tgImageProcessor::initShader] SobelAdaptive:");
#endif
  // Thinning shader
  if (m_shadeThinning)
  {
    m_shadeThinning->bind();
    m_shadeThinning->setUniform("frame", 0);
    m_shadeThinning->setUniform("mask", 1);
    m_shadeThinning->setUniform("binary", false);
    m_shadeThinning->unbind();
  }
  tgCheckError("[tgImageProcessor::initShader] Thinning:");

  // Spreading shader
  if (m_shadeSpreading)
  {
    m_shadeSpreading->bind();
    m_shadeSpreading->setUniform("mDistance", mat3(dist), GL_FALSE);
    m_shadeSpreading->setUniform("fThreshold", 0.1f);
    m_shadeSpreading->setUniform("fDistScale", 0.5f);
    m_shadeSpreading->unbind();
  }
  tgCheckError("[tgImageProcessor::initShader] Spreading:");

  // Spreading shader
  if (m_shadeDilate)
  {
    m_shadeDilate->bind();
    m_shadeDilate->setUniform("frame", 0);
    m_shadeDilate->setUniform("fDistScale", 0.5f);
    m_shadeDilate->unbind();
  }
  tgCheckError("[tgImageProcessor::initShader] Dilate:");

  // Spreading shader
  if (m_shadeDotConvolution)
  {
    m_shadeDotConvolution->bind();
    m_shadeDotConvolution->setUniform("frame", 0);
    m_shadeDotConvolution->setUniform("fThreshold", 0.0f);
    m_shadeDotConvolution->setUniform("kernel", 25, dotConv);
    m_shadeDotConvolution->unbind();
  }
  tgCheckError("[tgImageProcessor::initShader] Dot Convolution:");

}

void tgImageProcessor::applyOperation(tgShader* shader, const tgTexture2D& src, tgTexture2D& result)
{
  int w = src.GetWidth();
  int h = src.GetHeight();

  if (m_use_fbo)
  {
    GLenum f = src.GetFormat();
    result.Load(NULL, w, h, f);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo_id);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, result.GetTextureID(), 0);
#ifdef DEBUG
    tgCheckError("[tgImageProcessor::applyOperation] attach color texture");
#endif
  }

  m_cam_ortho.Activate();
  if (shader)
    shader->bind();
  src.Bind();
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);
  drawQuad(w, h);
  if (!m_use_fbo)
    result.CopyTexImage2D(w, h);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);

  if (shader)
    shader->unbind();

  if (m_use_fbo)
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void tgImageProcessor::drawQuad(float w, float h)
{
  glBegin(GL_QUADS);
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(0, 0, 0.0f);
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(w, 0, 0.0f);
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(w, h, 0.0f);
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(0, h, 0.0f);
  glEnd();
}

void tgImageProcessor::drawQuad(float x, float y, float w, float h)
{

  glBegin(GL_QUADS);
  glColor3f(1.0f, 1.0f, 1.0f);
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(x, y, 0.0f);
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(x + w, y, 0.0f);
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(x + w, y + h, 0.0f);
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(x, y + h, 0.0f);
  glEnd();
}

// Display list for flipping image upside down
void tgImageProcessor::drawQuadUpsideDown(float w, float h)
{
  glBegin(GL_QUADS);
  glTexCoord2f(0.0f, 1.0f);
  glVertex3f(0, 0, 0.0f);
  glTexCoord2f(1.0f, 1.0f);
  glVertex3f(w, 0, 0.0f);
  glTexCoord2f(1.0f, 0.0f);
  glVertex3f(w, h, 0.0f);
  glTexCoord2f(0.0f, 0.0f);
  glVertex3f(0, h, 0.0f);
  glEnd();
}

// Set functions
void tgImageProcessor::setCamOrtho()
{
  m_cam_ortho.Activate();
}


std::string tgImageProcessor::getShaderPath()
{
  return std::string(TOMGINE_IP_SHADER);
}

// *** Image Processing functions ***
void tgImageProcessor::flipUpsideDown(const tgTexture2D& source, tgTexture2D& result)
{
  int w = source.GetWidth();
  int h = source.GetHeight();

  if (m_use_fbo)
  {
    GLenum f = source.GetFormat();
    result.Load(NULL, w, h, f);
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo_id);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, result.GetTextureID(), 0);
#ifdef DEBUG
    tgCheckError("[tgImageProcessor::flipUpsideDown] attach color texture");
#endif
  }

  m_cam_ortho.Activate();
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);
  source.Bind();
  drawQuadUpsideDown(w, h);
  if (!m_use_fbo)
    result.CopyTexImage2D(w, h);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);

  if (m_use_fbo)
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void tgImageProcessor::copy(const tgTexture2D& source, tgTexture2D& result)
{
  applyOperation(NULL, source, result);
}

void tgImageProcessor::copy(const tgTexture2D& source, tgTexture2D& result, int x, int y, unsigned w, unsigned h)
{
  int ws = source.GetWidth();
  int hs = source.GetHeight();

  if (m_use_fbo)
    printf(
        "[tgImageProcessor::copy(const tgTexture2D&, tgTexture2D&, int, int, unsigned, unsigned)] Warning: usage of frame-buffer-object not possible.\n");

  m_cam_ortho.Activate();
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);
  source.Bind();
  drawQuad(ws, hs);
  result.CopyTexImage2D(x, y, w, h);
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);
}

void tgImageProcessor::threshold(const tgTexture2D& source, tgTexture2D& result, float th)
{
  m_shadeThreshold->bind();
  m_shadeThreshold->setUniform("th", th);
  m_shadeThreshold->unbind();
  applyOperation(m_shadeThreshold, source, result);
}

void tgImageProcessor::add(const tgTexture2D& src1, const tgTexture2D& src2, tgTexture2D& result, float k1, float k2)
{
  src2.Bind(1);
  m_shadeAdd->bind();
  m_shadeAdd->setUniform("k1", k1);
  m_shadeAdd->setUniform("k2", k2);
  m_shadeAdd->unbind();
  applyOperation(m_shadeAdd, src1, result);
}

void tgImageProcessor::multiply(const tgTexture2D& src1, const tgTexture2D& src2, tgTexture2D& result, bool src1grad,
                                bool src2grad)
{
  m_shadeMultiply->bind();
  m_shadeMultiply->setUniform("tex1grad", src1grad);
  m_shadeMultiply->setUniform("tex2grad", src2grad);
  m_shadeMultiply->setUniform("byfactor", false);
  m_shadeMultiply->unbind();
  src2.Bind(1);
  applyOperation(m_shadeMultiply, src1, result);
}
void tgImageProcessor::multiply(const tgTexture2D& src, float k, tgTexture2D &result)
{
  m_shadeMultiply->bind();
  m_shadeMultiply->setUniform("tex1grad", false);
  m_shadeMultiply->setUniform("tex2grad", false);
  m_shadeMultiply->setUniform("byfactor", true);
  m_shadeMultiply->setUniform("k", k);
  m_shadeMultiply->unbind();
  applyOperation(m_shadeMultiply, src, result);
}

void tgImageProcessor::invert(const tgTexture2D& source, tgTexture2D& result)
{
  applyOperation(m_shadeInvert, source, result);
}

float tgImageProcessor::averageLuminance(GLenum internal)
{
  int vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);

  if (vp[2] != vp[3])
    printf("[tgImageProcessor::average] Warning, screen width and height not equal (%d x %d)\n", vp[2], vp[3]);

  unsigned p = ilog2(vp[2]);

  float result;
  tgTexture2D tex_result;
  tex_result.CopyTexImage2D(vp[2], vp[2], internal);
  glGenerateMipmap(GL_TEXTURE_2D);
  glGetTexImage(GL_TEXTURE_2D, p, GL_LUMINANCE, GL_FLOAT, &result);

  return result;
}

vec3 tgImageProcessor::average()
{
  int vp[4];
  glGetIntegerv(GL_VIEWPORT, vp);

  if (vp[2] != vp[3])
    printf("[tgImageProcessor::average] Warning, screen width and height not equal (%d x %d)\n", vp[2], vp[3]);

  unsigned p = ilog2(vp[2]);

  vec3 result;
  tgTexture2D tex_result;
  tex_result.CopyTexImage2D(vp[2], vp[2], GL_RGB32F);
  glGenerateMipmap(GL_TEXTURE_2D);
  glGetTexImage(GL_TEXTURE_2D, p, GL_RGB, GL_FLOAT, &result);

  return result;
}

float tgImageProcessor::average(std::vector<float> &data)
{
  unsigned s = data.size();
  unsigned e = s >> 2;
  if (s % 4)
  {
    printf("[tgImageProcessor::average] not divideable by 4\n");
    data.resize(s + s % 4);
    s = data.size();
    e = s >> 2;
  }

  unsigned p = ilog2(e);
  float rgba[4];
  tgTexture2D avg;

  if (pow(2, p) != e)
    printf("[tgImageProcessor::average] element size not power of two\n");

  //		if(pow(2,p)!=data.size()){
  //
  //			if(p%2){
  //				printf("[tgImageProcessor::average] ungerade: %d\n", p);
  //				p++;
  //			}else{
  //				printf("[tgImageProcessor::average] gerade: %d\n", p);
  //			}
  //			unsigned w = pow(2,p>>1);
  //			printf("[tgImageProcessor::average] texture size: %dx%d\n", w,w);
  //
  //			std::vector<float> tmp = data;
  //			tmp.resize(pow(2,p), 0.0f);
  //			avg.Load(&tmp[0], w, w, GL_R32F, GL_RED, GL_FLOAT);
  //			tgCheckError("[tgImageProcessor::average] load data to GPU\n");
  //
  //			glGenerateMipmap(GL_TEXTURE_2D);
  //			tgCheckError("[tgImageProcessor::average] compute mipmaps\n");
  //
  //			glGetTexImage(GL_TEXTURE_2D, p>>1, GL_RED, GL_FLOAT, &result);
  //			tgCheckError("[tgImageProcessor::average] get result from GPU\n");
  //
  //			result = result * tmp.size()/s;
  //
  //		}else{
  unsigned w = pow(2, p >> 1);
  avg.Load(&data[0], w, w, GL_RGBA32F, GL_RGBA, GL_FLOAT);
  tgCheckError("[tgImageProcessor::average] load data to GPU\n");

  glFinish();

  tgTimer timer;
  timer.Update();
  glGenerateMipmap(GL_TEXTURE_2D);
  tgCheckError("[tgImageProcessor::average] compute mipmaps\n");

  glGetTexImage(GL_TEXTURE_2D, p >> 1, GL_RGBA, GL_FLOAT, &rgba);
  tgCheckError("[tgImageProcessor::average] get result from GPU\n");

  //		}


  float result = (rgba[0] + rgba[1] + rgba[2] + rgba[3]) * 0.25;

  printf("[tgImageProcessor::average] time: %f\n", timer.Update());

  return result;
}

void tgImageProcessor::gauss(const tgTexture2D& source, tgTexture2D& result)
{
  applyOperation(m_shadeGauss, source, result);
}

void tgImageProcessor::gradients(const tgTexture2D& source, tgTexture2D& gradientsX, tgTexture2D& gradientsY)
{
  applyOperation(m_shadeGradientsX, source, gradientsX);
  applyOperation(m_shadeGradientsY, source, gradientsY);
}

void tgImageProcessor::sobel(const tgTexture2D& source, tgTexture2D& result, float threshold, bool normalise,
                             bool binary)
{
  m_shadeSobel->bind();
  m_shadeSobel->setUniform("fThreshold", threshold);
  m_shadeSobel->setUniform("norm", normalise);
  m_shadeSobel->setUniform("binary", binary);
  m_shadeSobel->unbind();
  applyOperation(m_shadeSobel, source, result);
}

void tgImageProcessor::sobel(const tgTexture2D& source, tgTexture2D& result, tgTexture2D& mask, float threshold,
                             bool normalise, bool binary)
{
  m_shadeSobel->bind();
  m_shadeSobel->setUniform("fThreshold", threshold);
  m_shadeSobel->setUniform("norm", normalise);
  m_shadeSobel->setUniform("binary", binary);
  m_shadeSobel->setUniform("masked", true);
  m_shadeSobel->unbind();
  mask.Bind(1);
  applyOperation(m_shadeSobel, source, result);
  m_shadeSobel->bind();
  m_shadeSobel->setUniform("masked", false);
  m_shadeSobel->unbind();
}

void tgImageProcessor::sobelNormal(const tgTexture2D &source, tgTexture2D& result, float threshold)
{
  m_shadeSobelNormal->bind();
  m_shadeSobelNormal->setUniform("fThreshold", threshold);
  m_shadeSobelNormal->unbind();
  applyOperation(m_shadeSobelNormal, source, result);
}

void tgImageProcessor::sobelAdaptive(const tgTexture2D& source, tgTexture2D& result, float adaptive_threshold,
                                     float offset, bool normalise, bool binary)
{
  m_shadeSobelAdaptive->bind();
  m_shadeSobelAdaptive->setUniform("fThreshold", adaptive_threshold);
  m_shadeSobelAdaptive->setUniform("fOffset", offset);
  m_shadeSobelAdaptive->setUniform("norm", normalise);
  m_shadeSobelAdaptive->setUniform("binary", binary);
  m_shadeSobelAdaptive->unbind();
  applyOperation(m_shadeSobelAdaptive, source, result);
}

void tgImageProcessor::rgbdEdges(const tgTexture2D& color, const tgTexture2D& depth, const tgTexture2D& curvature,
                                 tgTexture2D& result, float k1, float k2, float k3)
{
  tgTexture2D texEdgesGauss, texEdgesColor, texEdgesDepth, texEdgesDepthc, texEdgesCurvature, texResult;

  // color edges
  gauss(color, texEdgesGauss);
  sobel(texEdgesGauss, texEdgesColor, 0.0f, false, true);

  // depth edges
  sobel(depth, texEdgesDepth, 0.0f, false, true);

  // depth edges = color edges * depth edges
  multiply(texEdgesColor, texEdgesDepth, texEdgesDepthc);

  // curvature edges = color edges * curvature
  multiply(texEdgesColor, curvature, texEdgesCurvature);

  // edges = k1*color edges + k2*depth edges + k3*curvature edges
  add(texEdgesColor, texEdgesDepthc, texResult, k1, k2);
  add(texEdgesCurvature, texResult, result, k3);
}

void tgImageProcessor::thinning(const tgTexture2D& source, tgTexture2D& result, bool normalise, bool binary)
{
  m_shadeThinning->bind();
  m_shadeThinning->setUniform("binary", binary);
  m_shadeThinning->setUniform("norm", normalise);
  m_shadeThinning->unbind();
  applyOperation(m_shadeThinning, source, result);
}

void tgImageProcessor::param2polar(const tgTexture2D& source, tgTexture2D& result)
{
  source.Bind();
  applyOperation(m_shadeParam2Polar, source, result);
}

void tgImageProcessor::rgb2hsv(const tgTexture2D& source, tgTexture2D& result)
{
  source.Bind();
  applyOperation(m_shadeRGB2HSV, source, result);
}

void tgImageProcessor::spreading(const tgTexture2D& source, tgTexture2D& result, bool binary, float fDistScale,
                                 float fThreshold)
{
  m_shadeSpreading->bind();
  m_shadeSpreading->setUniform("binary", binary);
  m_shadeSpreading->setUniform("fDistScale", fDistScale);
  m_shadeSpreading->setUniform("fThreshold", fThreshold);
  m_shadeSpreading->unbind();
  applyOperation(m_shadeSpreading, source, result);
}

void tgImageProcessor::dilate(const tgTexture2D& source, tgTexture2D& result, float scale)
{
  m_shadeDilate->bind();
  m_shadeDilate->setUniform("fDistScale", scale);
  m_shadeDilate->unbind();
  applyOperation(m_shadeDilate, source, result);
}

void tgImageProcessor::dotConvolute(const tgTexture2D& source, tgTexture2D& result, std::vector<float> kernel,
                                    float threshold)
{
  if (kernel.size() != 25)
    printf("[tgImageProcessor::dotConvolute] Error, kernel size not 5x5 (25)\n");

  m_shadeDotConvolution->bind();
  m_shadeDotConvolution->setUniform("kernel", 25, &kernel[0]);
  m_shadeDotConvolution->setUniform("fThreshold", threshold);
  m_shadeDotConvolution->unbind();
  tgCheckError("[tgImageProcessor::dotConvolute] A");
  applyOperation(m_shadeDotConvolution, source, result);
  tgCheckError("[tgImageProcessor::dotConvolute] B");
}

void tgImageProcessor::render(const tgTexture2D& tex)
{
  m_cam_ortho.Activate();
  glDisable(GL_DEPTH_TEST);
  glEnable(GL_TEXTURE_2D);
  tex.Bind();
  drawQuad(tex.GetWidth(), tex.GetHeight());
  glDisable(GL_TEXTURE_2D);
  glEnable(GL_DEPTH_TEST);
}

//void tgImageProcessor::render(const TomGine::tgTexture2D &tex, int x, int y, unsigned w, unsigned h){
//	m_cam_ortho.Activate();
//	glDisable(GL_DEPTH_TEST);
//	glEnable(GL_TEXTURE_2D);
//		tex.Bind();
//		drawQuad((float)x, (float)y, (float)w, (float)h);
//	glDisable(GL_TEXTURE_2D);
//	glEnable(GL_DEPTH_TEST);
//}

//void tgImageProcessor::avgActivate(){
//	if(m_avg_init){
//		glBindFramebuffer(GL_FRAMEBUFFER, fbo);
//		tgCheckError("tgImageProcessor::avgActivate() A");
//		glClearColor(0,0,0,0);
//		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//		tgCheckError("tgImageProcessor::avgActivate() B");
//	}
//}
//
//void tgImageProcessor::avgGet(float *avg, int lvl){
//	if(m_avg_init){
//		if(lvl==0){
//			// for some reason glGetTexImage doesen't get the value of the very last mipmap stage
//// 			float tmp[16];
//// 			glBindTexture(GL_TEXTURE_2D, fbo_tex);
//// 			glGenerateMipmap(GL_TEXTURE_2D);
//// 			glGetTexImage(GL_TEXTURE_2D, fbo_stage-1, GL_RGBA, GL_FLOAT, tmp);
//// 			avg[0] = 0;
//// 			for(unsigned i=0; i<4; i++)
//// 				avg[0] += 0.25 * tmp[i];
//			printf("[tgImageProcessor::avgGet] Warning: not implemented\n");
//		}else{
//			glBindTexture(GL_TEXTURE_2D, fbo_tex);
//// 			glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGB16,0, 0, fbo_res,fbo_res,0);
//			glGenerateMipmap(GL_TEXTURE_2D);
//			glGetTexImage(GL_TEXTURE_2D, fbo_stage-lvl, GL_RED, GL_FLOAT, avg);
//			tgCheckError("tgImageProcessor::avgGet()");
//		}
//	}
//}
//
//void tgImageProcessor::avgDeactivate(){
//	if(m_avg_init){
//		glBindFramebuffer(GL_FRAMEBUFFER, 0);
//	}
//}

