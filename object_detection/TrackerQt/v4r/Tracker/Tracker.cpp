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
#include "Tracker.h"
#include <stdexcept>
#include "v4r/TomGine/tgError.h"

using namespace Tracking;
using namespace TomGine;
using namespace std;

// *** PUBLIC ***

Tracker::Parameter::Parameter ()
{
  // Default parameter
  camPar.width = 640;
  camPar.height = 480;
  model_id_count = 0;
  hypotheses_id_count = 0;
  num_particles = 100;
  num_recursions = 2;
  //  hypotheses_trials = 5;
  convergence = 5;
  edge_tolerance = 45.0f * M_PI / 180.0f;
  num_spreadings = 3;
  m_spreadlvl = 2;
  variation = Particle (vec3 (0.01f, 0.01f, 0.01f), vec3 (0.1f, 0.1f, 0.1f), 0.01f, 0.01f, 0.01f,
                        TomGine::tgQuaternion ());
  minTexGrabAngle = 3.0f * M_PI_4;
  max_kernel_size = 3;
  kernel_size = 1;
  modelPath = string ("../Resources/model/");
  //  texturePath = string ("../Resources/texture/");
  //  shaderPath = string ("../Resources/shader/");
  image_sobel_th = 0.02f;
  model_sobel_th = 0.03f;
  pred_no_convergence = 0.1f;
  keep_best_particles = 0.1f;
  lpf = 0.4;
  method = EDGECOLOR;
}

namespace Tracking
{
std::ostream& operator << (std::ostream& os, Tracker::Parameter& param)
{
  os << param.camPar;

  os << endl;
  os << "# ----- Tracking Parameter ----- #" << endl;
  os << "# particles" << endl;
  os << param.num_particles << " " << param.num_recursions << endl;
  os << "# convergence" << endl;
  os << param.convergence << endl;
  os << "# edge tolerance" << endl;
  os << param.edge_tolerance << endl;
  os << "# spreadings" << endl;
  os << param.num_spreadings << " " << param.m_spreadlvl << endl;
  os << "# variation r" << endl;
  os << param.variation.r.x << " " << param.variation.r.y << " " << param.variation.r.z << endl;
  os << "# variation t" << endl;
  os << param.variation.t.x << " " << param.variation.t.y << " " << param.variation.t.z << endl;
  os << "# minimum texture grab angle" << endl;
  os << param.minTexGrabAngle << endl;
  os << "# max kernel size" << endl;
  os << param.max_kernel_size << endl;
  os << "# sobel threshold" << endl;
  os << param.image_sobel_th << " " << param.model_sobel_th << endl;
  os << "# non-converging particles" << endl;
  os << param.pred_no_convergence << endl;
  os << "# best particles kept" << endl;
  os << param.keep_best_particles << endl;
  os << "# filtering level (low-pass)" << endl;
  os << param.lpf << endl;
  os << "# method" << endl;
  os << param.method << endl;
  os << "# model path" << endl;
  os << param.modelPath << endl;

  return os;
}

std::istream& operator >> (std::istream& is, Tracker::Parameter& param)
{
  is >> param.camPar;

  string linebuffer;

  getline(is,linebuffer);
  getline(is,linebuffer);

  getline(is,linebuffer);
  is >> param.num_particles;
  is >> param.num_recursions;
  is.get();

  getline(is,linebuffer);
  is >> param.convergence;
  is.get();

  getline(is,linebuffer);
  is >> param.edge_tolerance;
  is.get();

  getline(is,linebuffer);
  is >> param.num_spreadings;
  is >> param.m_spreadlvl;
  is.get();

  getline(is,linebuffer);
  is >> param.variation.r.x;
  is >> param.variation.r.y;
  is >> param.variation.r.z;
  is.get();

  getline(is,linebuffer);
  is >> param.variation.t.x;
  is >> param.variation.t.y;
  is >> param.variation.t.z;
  is.get();

  getline(is,linebuffer);
  is >> param.minTexGrabAngle;
  is.get();

  getline(is,linebuffer);
  is >> param.max_kernel_size;
  is.get();

  getline(is,linebuffer);
  is >> param.image_sobel_th;
  is >> param.model_sobel_th;
  is.get();

  getline(is,linebuffer);
  is >> param.pred_no_convergence;
  is.get();

  getline(is,linebuffer);
  is >> param.keep_best_particles;
  is.get();

  getline(is,linebuffer);
  is >> param.lpf;
  is.get();

  getline(is,linebuffer);
  size_t meth;
  is >> meth;
  is.get();
  if(meth==0)
    param.method = EDGE;
  else if(meth==1)
    param.method = COLOR;
  else if(meth==2)
    param.method = EDGECOLOR;

  getline(is,linebuffer);
  is >> param.modelPath;
  is.get();

  return is;
}
} // namespace Tracking

void Tracker::Parameter::print()
{
  cout << endl << "Tracker::Parameter" << endl;
  cout << "-----------------------------------" << endl;
  cout << "width: " << camPar.width << endl;
  cout << "height: " << camPar.height << endl;
  cout << "model_id_count: " << model_id_count << endl;
  cout << "hypotheses_id_count: " << hypotheses_id_count << endl;
  cout << "num_particles: " << num_particles << endl;
  cout << "num_recursions: " << num_recursions << endl;
  //  cout << "hypotheses_trials: " << hypotheses_trials << endl;
  cout << "convergence: " << convergence << endl;
  cout << "edge_tolerance: " << edge_tolerance << endl;
  cout << "num_spreadings: " <<  num_spreadings << endl;
  cout << "m_spreadlvl: " << m_spreadlvl << endl;
  cout << "variation r: " << variation.r.x << " " << variation.r.y << " " << variation.r.z << endl;
  cout << "variation t: " << variation.t.x << " " << variation.t.y << " " << variation.t.z << endl;
  cout << "minTexGrabAngle: " << minTexGrabAngle << endl;
  cout << "max_kernel_size: " << max_kernel_size << endl;
  cout << "kernel_size: " << kernel_size << endl;
  cout << "image_sobel_th: " << image_sobel_th << endl;
  cout << "model_sobel_th: " << model_sobel_th << endl;
  cout << "pred_no_convergence: " << pred_no_convergence << endl;
  cout << "keep_best_particles: " << keep_best_particles << endl;
  cout << "lpf: " << lpf << endl;
  cout << "method: " << method << endl;
  cout << "modelpath: " << modelPath << endl;
  cout << "-----------------------------------" << endl << endl;
}

Tracker::Tracker ()
{
  // Default flags
  m_lock = false;
  m_showparticles = false;
  m_showmodel = 1;
  m_draw_edges = false;
  m_tracker_initialized = false;
}

Tracker::~Tracker ()
{
  unsigned i;

  for (i = 0; i < m_tex_frame_ip.size (); i++)
  {
    delete (m_tex_frame_ip[i]);
  }
#ifdef DEBUG
  tgCheckError("[Tracker::~Tracker] Textures:");
#endif
  for (i = 0; i < m_modellist.size (); i++)
  {
    delete (m_modellist[i]);
  }
#ifdef DEBUG
  tgCheckError("[Tracker::~Tracker] Models:");
#endif
  delete g_Resources;
#ifdef DEBUG
  tgCheckError("[Tracker::~Tracker] Resources:");
#endif
  // 	printf("Tracker::~Tracker()\n");
}

bool
Tracker::initGL ()
{
  glEnable (GL_DEPTH_TEST);
  glDepthFunc (GL_LEQUAL);
  glEnable (GL_CULL_FACE);
  glCullFace (GL_BACK);
  glDisable(GL_BLEND);

  const GLubyte *str;
  int glOcclusionQueryAvailable;

  // Check for Extension
  str = glGetString (GL_EXTENSIONS);
  glOcclusionQueryAvailable = (strstr ((const char *)str, "GL_ARB_occlusion_query") != NULL);
  if (!glOcclusionQueryAvailable)
  {
    char errmsg[256];
    sprintf (
          errmsg,
          "[OpenGLControl] Error OpenGL extension 'GL_ARB_occlusion_query' not available. Your graphic card does not support this extension or the hardware driver for your graphic card is not installed properly!");
    throw std::runtime_error (errmsg);
  }

  return true;
}

void
Tracker::setFrameTime (double dTime)
{
  for (unsigned i = 0; i < m_modellist.size (); i++)
  {
    m_modellist[i]->predictor->updateTime (dTime);
  }
}

void Tracker::setParameter(const Tracker::Parameter &_params)
{
  params = _params;

  for(size_t i=0; i<m_modellist.size(); i++)
  {
    ModelEntry* me = m_modellist[i];
    me->num_recursions = params.num_recursions;
    me->num_particles = params.num_particles;
    me->setLPF(params.lpf);
  }
}

bool
Tracker::setCameraParameters (TomGine::tgCamera::Parameter cam_par)
{

  if (cam_par.zFar <= 0.0)
  {
    printf ("[Tracker::setCameraParameters] Error 'Far Clipping Plane' not valid: %f", cam_par.zFar);
    return false;
  }
  if (cam_par.zNear < 0.0)
  {
    printf ("[Tracker::setCameraParameters] Error 'Near Clipping Plane' not valid: %f", cam_par.zNear);
    return false;
  }

  if (cam_par.zNear >= cam_par.zFar)
  {
    printf ("[Tracker::setCameraParameters] Error 'Clipping Panes' not valid: %f > %f", cam_par.zNear, cam_par.zFar);
    return false;
  }

  m_cam_perspective.Set (cam_par);
  return true;
}

bool
Tracker::init (const Parameter& trackParam)
{

  if (m_tracker_initialized)
  {
    printf ("[Tracker::init()] Warning: Tracker already initialized\n");
    return false;
  }

  // load tracker parameter
  this->params = trackParam;
  // initialise tracker
  return init ();
}

bool
Tracker::init ()
{

  if (m_tracker_initialized)
  {
    printf ("[Tracker::init()] Warning: Tracker already initialized\n");
    return false;
  }

  // OpenGL / devIL
  initGL ();

  //  // Set pathes to file resources
  //  g_Resources->SetShaderPath (params.shaderPath.c_str ());

  // Load camera parameter
  m_cam_perspective.Set (params.camPar);

  // Initialize Image Processor
  g_Resources->InitImageProcessor (params.camPar.width, params.camPar.height);
  m_ip = g_Resources->GetImageProcessor ();
  //  params.shaderPath = m_ip->getShaderPath();
  //  std::cout << "[Tracker::init] shaderPath: " << params.shaderPath << std::endl;
  //  if(params.shaderPath.empty())
  //    std::cout << "[Tracker::init] params.shaderPath.empty()" << std::endl;

  g_Resources->ShowLog (false);

  // 	m_ip->avgInit(1024);

  // Textures
  m_tex_frame.Bind ();
  glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

  m_tex_frame_ip.push_back (new TomGine::tgTexture2D ());
  for (int i = 0; i < (int)params.num_spreadings - 1; i++)
    m_tex_frame_ip.push_back (new TomGine::tgTexture2D ());

  if (params.num_spreadings == 0)
    params.m_spreadlvl = 0;
  else if (params.m_spreadlvl >= params.num_spreadings)
    params.m_spreadlvl = params.num_spreadings - 1;

  return (m_tracker_initialized = initInternal ());
}

void
Tracker::loadImage (unsigned char* image, GLenum format)
{
  m_tex_frame.Load (image, params.camPar.width, params.camPar.height, GL_RGBA, format);
}

void
Tracker::reset ()
{
  for (unsigned i = 0; i < m_modellist.size (); i++)
  {
    reset (i);
  }
}

void
Tracker::reset (int id)
{
  if (id >= 0 && id < m_modellist.size ())
  {
    m_modellist[id]->predictor->sample (m_modellist[id]->distribution, params.num_particles,
                                        m_modellist[id]->initial_pose, params.variation);
    m_modellist[id]->resetPose ();
  }
}

int
Tracker::addModel (TomGine::tgModel& m, tgPose& p, std::string label, bool bfc)
{

  if (!m_tracker_initialized)
  {
    char errmsg[128];
    sprintf (errmsg, "[Tracker::addModel()] Error tracker not initialised!");
    throw std::runtime_error (errmsg);
  }

  ModelEntry* modelEntry = new ModelEntry (m, params.lpf);
  modelEntry->label = label;
  modelEntry->model.setBFC (bfc);

  modelEntry->pose = p;
  modelEntry->initial_pose = p;

  // Calculate Zoom Direction and pass it to the particle filter
  vec3 vCam = m_cam_perspective.GetPos ();
  vec3 vObj = vec3 (p.t.x, p.t.y, p.t.z);
  modelEntry->vCam2Model = vObj - vCam;
  modelEntry->predictor->setCamViewVector (modelEntry->vCam2Model);
  modelEntry->predictor->sample (modelEntry->distribution, params.num_particles, p, params.variation);
  modelEntry->predictor->setNoConvergence (params.pred_no_convergence);
  modelEntry->predictor->setKeepParticles (params.keep_best_particles);

  modelEntry->id = params.model_id_count++;
  modelEntry->num_particles = params.num_particles;
  modelEntry->num_recursions = params.num_recursions;
  m_modellist.push_back (modelEntry);

  return modelEntry->id;
}

int
Tracker::addModelFromFile (const char* filename, tgPose& p, std::string label, bool bfc)
{
  if (!m_tracker_initialized)
  {
    char errmsg[128];
    sprintf (errmsg, "[Tracker::addModelFromFile()] Error tracker not initialised!");
    throw std::runtime_error (errmsg);
  }

  ModelEntry* modelEntry = new ModelEntry (params.lpf);
  modelEntry->label = label;
  modelEntry->model.setBFC (bfc);

  ModelLoader modelloader;
  if (!modelloader.LoadPly (modelEntry->model, filename))
    return -1;

  modelEntry->pose = p;
  modelEntry->initial_pose = p;
  // Calculate Zoom Direction and pass it to the particle filter
  vec3 vCam = m_cam_perspective.GetPos ();
  vec3 vObj = vec3 (p.t.x, p.t.y, p.t.z);
  modelEntry->vCam2Model = vObj - vCam;
  modelEntry->predictor->setCamViewVector (modelEntry->vCam2Model);
  modelEntry->predictor->sample (modelEntry->distribution, params.num_particles, p, params.variation);
  modelEntry->predictor->setNoConvergence (params.pred_no_convergence);
  modelEntry->predictor->setKeepParticles (params.keep_best_particles);

  modelEntry->id = params.model_id_count++;
  modelEntry->num_particles = params.num_particles;
  modelEntry->num_recursions = params.num_recursions;
  m_modellist.push_back (modelEntry);

  return modelEntry->id;
}

void
Tracker::addPoseHypothesis (int id, tgPose &p, std::string label, bool bfc)
{
  if (!m_tracker_initialized)
  {
    char errmsg[128];
    sprintf (errmsg, "[Tracker::addPoseHypothesis()] Error tracker not initialised!");
    throw std::runtime_error (errmsg);
  }
  ModelEntryList::iterator it = m_modellist.begin ();
  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      ModelEntry* modelEntry = new ModelEntry (params.lpf);
      modelEntry->label = label;
      modelEntry->model.setBFC (bfc);
      modelEntry->model = m_modellist[id]->model; // TODO maybe buggy
      modelEntry->pose = p;
      modelEntry->initial_pose = p;
      modelEntry->predictor->sample (modelEntry->distribution, params.num_particles, p, params.variation);
      modelEntry->predictor->setNoConvergence (params.pred_no_convergence);

      modelEntry->id = params.hypotheses_id_count++;
      modelEntry->num_particles = params.num_particles;
      modelEntry->num_recursions = params.num_recursions;
      modelEntry->hypothesis_id = id;
      m_hypotheses.push_back (modelEntry);
      return;
    }
    it++;
  }
  printf ("[Tracker::addHypothesis()] Warning model for hypothesis not found: %d - %d \n", id, (int)m_modellist.size ());
}

void
Tracker::getModelBoundingBox2D (int id, TomGine::tgRect2Di &bb, bool pow2)
{
  ModelEntryList::iterator it = m_modellist.begin ();
  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      this->m_cam_perspective.Activate ();
      (*it)->model.getBoundingBox2D ((*it)->pose, bb, pow2);
      return;
    }
    it++;
  }
}

void
Tracker::removeModel (int id)
{
  ModelEntryList::iterator it = m_modellist.begin ();
  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      delete (*it);
      m_modellist.erase (it);
      return;
    }
    it++;
  }
}

ModelEntry*
Tracker::getModelEntry (int id)
{
  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      return (*it);
    }
    it++;
  }
}

void
Tracker::getModelPose (int id, tgPose& p)
{
  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      p = (*it)->pose;
      // 			p = (*it)->distribution.getParticle(0);
      return;
    }
    it++;
  }
}

TomGine::tgPose
Tracker::getModelPoseLPF (int id)
{
  ModelEntryList::iterator it = m_modellist.begin ();
  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      return (*it)->pose;
    }
    it++;
  }
}

void
Tracker::getModelPoseCamCoords (int id, tgPose& p)
{
  mat4 mv;
  mat3 R;
  vec3 t;

  mat3 gl2cv;
  gl2cv[0] = 1.0;
  gl2cv[3] = 0.0;
  gl2cv[6] = 0.0;
  gl2cv[1] = 0.0;
  gl2cv[4] = -1.0;
  gl2cv[7] = 0.0;
  gl2cv[2] = 0.0;
  gl2cv[5] = 0.0;
  gl2cv[8] = -1.0;

  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      m_cam_perspective.Activate ();
      p = m_modellist[0]->pose;
      p.Activate ();
      glGetFloatv (GL_MODELVIEW_MATRIX, mv);
      p.Deactivate ();

      R = gl2cv * mat3 (mv);
      t = gl2cv * vec3 (mv[12], mv[13], mv[14]);

      // 			printf("mv: %f %f %f\n", mv[12], mv[13], mv[14]);
      // 			printf("t:  %f %f %f\n", t.x, t.y, t.z);

      p.SetPose (R, t);
      return;
    }
    it++;
  }
}

void
Tracker::getModelVelocity (int id, float &translational, float &angular)
{
  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      translational = (*it)->speed_translational;
      angular = (*it)->speed_angular;
      return;
    }
    it++;
  }
}

const TomGine::tgCamera Tracker::getCamera() const
{
  return m_cam_perspective;
}

void
Tracker::getModelTrackingState (int id, TrackingState &ts)
{
  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      ts = (*it)->ts;
      return;
    }
    it++;
  }
}

void
Tracker::drawTest ()
{
  tgPose p;
  mat4 mv;
  mat3 R, Rinv;
  vec3 t;

  std::vector<vec3> v;

  m_cam_perspective.Activate ();

  if (!m_modellist.empty ())
  {

    TomGine::tgModel model = m_modellist[0]->model;
    for (unsigned i = 0; i < model.m_vertices.size (); i++)
    {
      v.push_back (model.getVertex (i).pos);
    }

    p = m_modellist[0]->pose;

    p.Activate ();
    glGetFloatv (GL_MODELVIEW_MATRIX, mv);
    p.Deactivate ();

    R = mat3 (mv);
    t = vec3 (mv[12], mv[13], mv[14]);
    p.SetPose (R, t);

    glMatrixMode (GL_MODELVIEW);
    glLoadIdentity ();
    glDisable (GL_DEPTH_TEST);

    glPointSize (5.0);
    glBegin (GL_POINTS);
    glColor3f (1.0f, 0.0f, 0.0f);
    glVertex3f (t.x, t.y, t.z);
    glEnd ();

    glPointSize (2.0);

    glBegin (GL_POINTS);
    glColor3f (1.0f, 0.0f, 0.0f);

    for (unsigned int i = 0; i < v.size (); i++)
    {
      v[i] = (R * v[i]) + t;

      glVertex3f (v[i].x, v[i].y, v[i].z);
    }
    glEnd ();

  }
}

void
Tracker::getModelInitialPose (int id, tgPose& p)
{
  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      p = (*it)->initial_pose;
      return;
    }
    it++;
  }
}

void
Tracker::getModelConfidenceMean (int id, float& c)
{
  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      c = (*it)->distribution.getMeanC ();
      return;
    }
    it++;
  }
}

void
Tracker::getModelConfidenceMedian (int id, float& c)
{
  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      c = (*it)->distribution.getMedianC ();
      return;
    }
    it++;
  }
}

void
Tracker::getModelConfidenceMax (int id, float& c)
{
  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      c = (*it)->distribution.getMaxC ();
      return;
    }
    it++;
  }
}

bool
Tracker::getModelPoint3D (int id, int x_win, int y_win, double& x3, double& y3, double& z3)
{
  ModelEntryList::iterator it;

  for (it = m_modellist.begin (); it < m_modellist.end (); it++)
  {
    if (id == (*it)->id)
    {
      // Activate Camera
      TomGine::tgCamera cam = m_cam_perspective;
      cam.SetZRange (0.0, 1.0);
      cam.Activate ();

      // Clear Depth Buffer
      glClear (GL_DEPTH_BUFFER_BIT);
      glEnable (GL_DEPTH_TEST);

      // Apply pose
      (*it)->pose.Activate ();

      // Draw Model Faces
      glEnable (GL_LIGHTING);
      (*it)->model.drawFaces ();
      glDisable (GL_LIGHTING);

      // ************************
      int viewport[4];
      double modelview[16];
      double projection[16];
      double result[3];

      glGetDoublev (GL_MODELVIEW_MATRIX, &modelview[0]);
      glGetDoublev (GL_PROJECTION_MATRIX, &projection[0]);
      glGetIntegerv (GL_VIEWPORT, &viewport[0]);
      y_win = viewport[3] - y_win; // flip y for OpenGL

      // Read value of depth buffer at position (x_win, y_win)
      glReadPixels (x_win, y_win, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z3);

      if (z3 > 0.99)
        return false;

      // calculate intersection of camera viewing vector and model surface
      gluUnProject ((double)x_win, (double)y_win, (double)z3, modelview, projection, viewport, &result[0], &result[1],
          &result[2]);

      x3 = result[0];
      y3 = result[1];
      z3 = result[2];

      (*it)->pose.Deactivate ();

      return true;
    }
  }
  return false;
}

void
Tracker::getModelMask (int id, TomGine::tgTexture2D &mask)
{
  ModelEntryList::iterator it;

  for (it = m_modellist.begin (); it < m_modellist.end (); it++)
  {
    if (id == (*it)->id)
    {
      glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      m_cam_perspective.Activate ();
      glDisable (GL_LIGHTING);
      glColor3f (1, 1, 1);
      glEnable (GL_DEPTH_TEST);
      glDepthMask (1);
      (*it)->pose.Activate ();
      (*it)->model.DrawFaces ();
      (*it)->pose.Deactivate ();
      glDisable (GL_DEPTH_TEST);
      glDepthMask (0);

      mask.CopyTexImage2D (params.camPar.width, params.camPar.height);
    }
  }
}

void
Tracker::getModelEdgeMask (int id, TomGine::tgTexture2D &mask)
{
  ModelEntryList::iterator it;

  for (it = m_modellist.begin (); it < m_modellist.end (); it++)
  {
    if (id == (*it)->id)
    {
      computeModelEdgeMask ((*it), mask);
    }
  }
}

bool
Tracker::getModelMaskOwnEdges (int id)
{
  ModelEntryList::iterator it;

  for (it = m_modellist.begin (); it < m_modellist.end (); it++)
  {
    if (id == (*it)->id)
    {
      return (*it)->mask_geometry_edges;
    }
  }
}

void
Tracker::setModelInitialPose (int id, const tgPose& p)
{
  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      (*it)->setInitialPose (p);
      return;
    }
    it++;
  }
}

void
Tracker::setModelPredictor (int id, Predictor* predictor)
{
  ModelEntryList::iterator it = m_modellist.begin ();
  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      (*it)->predictor = predictor;
      return;
    }
    it++;
  }
}

void
Tracker::setModelPredictorNoConvergence (int id, float no_conv)
{
  ModelEntryList::iterator it = m_modellist.begin ();
  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      (*it)->predictor->setNoConvergence (no_conv);
      return;
    }
    it++;
  }
}

void
Tracker::setModelMask (int id, TomGine::tgTexture2D *mask)
{

}

void
Tracker::setModelMaskOwnEdges (int id, bool masked)
{
  ModelEntryList::iterator it = m_modellist.begin ();
  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      (*it)->mask_geometry_edges = masked;
      return;
    }
    it++;
  }
}

void
Tracker::setModelLock (int id, bool lock)
{
  ModelEntryList::iterator it = m_modellist.begin ();
  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      (*it)->lock = lock;
      return;
    }
    it++;
  }
}

void
Tracker::setModelRecursionsParticle (int id, int num_recursions, int num_particles)
{
  ModelEntryList::iterator it = m_modellist.begin ();
  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      (*it)->num_recursions = num_recursions;
      (*it)->num_particles = num_particles;
      return;
    }
    it++;
  }
}

void
Tracker::saveModel (int id, std::string filename)
{
  ModelLoader modelloader;
  ModelEntryList::iterator it = m_modellist.begin ();

  while (it != m_modellist.end ())
  {
    if (id == (*it)->id)
    {
      modelloader.SavePly ((*it)->model, filename.c_str ());
      return;
    }
    it++;
  }
}

void
Tracker::saveModels (const char* pathname)
{
  ModelLoader modelloader;
  ModelEntryList::iterator it = m_modellist.begin ();
  string name;
  while (it != m_modellist.end ())
  {
    name = string (pathname);
    name.append ((*it)->label);
    (*it)->model.unwarpTexturesBox_hacky (name.c_str ());
    modelloader.SavePly ((*it)->model, name.c_str ());
    it++;
  }
}

void
Tracker::saveScreenshot (const char* filename)
{
  IplImage* img = cvCreateImage (cvSize (params.camPar.width, params.camPar.height), IPL_DEPTH_8U, 3);
  glReadPixels (0, 0, params.camPar.width, params.camPar.height, GL_RGB, GL_UNSIGNED_BYTE, img->imageData);
  cvConvertImage (img, img, CV_CVTIMG_FLIP | CV_CVTIMG_SWAP_RB);
  cvSaveImage (filename, img);
  cvReleaseImage (&img);
}

void
Tracker::setLockFlag (bool val)
{
  ModelEntryList::iterator it = m_modellist.begin ();
  while (it != m_modellist.end ())
  {
    (*it)->lock = val;
    it++;
  }
  m_lock = val;
}

void
Tracker::drawModel (tgPose p)
{
  m_cam_perspective.Activate ();

  glEnable (GL_DEPTH_TEST);
  glClear (GL_DEPTH_BUFFER_BIT);

  p.Activate ();

  for (unsigned i = 0; i < m_modellist.size (); i++)
  {

    glColorMask (0, 0, 0, 0);
    m_modellist[i]->model.drawFaces ();
    glColorMask (1, 1, 1, 1);
    m_modellist[i]->model.drawEdges ();
  }

  p.Deactivate ();
}

void
Tracker::drawModel (const TomGine::tgModel &m, const tgPose &p)
{
  m_cam_perspective.Activate ();

  glEnable (GL_LIGHTING);
  p.Activate ();

  m.DrawFaces ();

  p.Deactivate ();
  glDisable (GL_LIGHTING);
}

void
Tracker::drawModelWireframe (const TomGine::tgModel &m, const tgPose &p, float linewidth)
{

  glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
  glLineWidth (linewidth);

  m_cam_perspective.Activate ();

  glDisable (GL_LIGHTING);

  p.Activate ();

  m.DrawFaces ();

  p.Deactivate ();

  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
}

void
Tracker::drawCoordinateSystem (float linelength, float linewidth, TomGine::tgPose pose)
{
  glDisable (GL_DEPTH_TEST);
  glDisable (GL_CULL_FACE);
  m_cam_perspective.Activate ();

  float l = linelength;
  glLineWidth (linewidth);

  vec3 o = pose.t;

  vec3 x = pose * vec3 (l, 0, 0);
  vec3 y = pose * vec3 (0, l, 0);
  vec3 z = pose * vec3 (0, 0, l);

  glColor3f (1, 0, 0);
  glBegin (GL_LINES);
  glVertex3f (o.x, o.y, o.z);
  glVertex3f (x.x, x.y, x.z);
  glEnd ();

  glColor3f (0, 1, 0);
  glBegin (GL_LINES);
  glVertex3f (o.x, o.y, o.z);
  glVertex3f (y.x, y.y, y.z);
  glEnd ();

  glColor3f (0, 0, 1);
  glBegin (GL_LINES);
  glVertex3f (o.x, o.y, o.z);
  glVertex3f (z.x, z.y, z.z);
  glEnd ();

}

// render coordinate frame
void
Tracker::drawCoordinates (float linelength)
{
  glDisable (GL_DEPTH_TEST);
  glDisable (GL_CULL_FACE);
  m_cam_perspective.Activate ();

  //	float l1 = 0.06f;
  //	float l2 = 0.02f;
  //	float b1 = 0.001f;
  //	float b2 = 0.003f;

  drawCoordinateSystem (linelength, 2.0f);

  // 	// X - Axis
  // 	glPushMatrix();
  // 		glColor3f(1.0,0.0,0.0);
  // 		glBegin(GL_TRIANGLE_FAN);
  // 			glVertex3f(l1,		0.0,	 b1);
  // 			glVertex3f(0.0,		0.0,	 b1);
  // 			glVertex3f(0.0,		0.0,	-b1);
  // 			glVertex3f(l1,		0.0,	-b1);
  // 			glVertex3f(l1,		0.0,	-b1-b2);
  // 			glVertex3f(l1+l2,	0.0,	0.0);
  // 			glVertex3f(l1,		0.0,	 b1+b2);
  // 		glEnd();
  // 	glPopMatrix();
  //
  // 	// Y - Axis
  // 	glPushMatrix();
  // 		glColor3f(0.0,1.0,0.0);
  // 		glRotatef(90, 0.0, 0.0, 1.0);
  // 		glBegin(GL_TRIANGLE_FAN);
  // 			glVertex3f(l1,		0.0,	 b1);
  // 			glVertex3f(0.0,		0.0,	 b1);
  // 			glVertex3f(0.0,		0.0,	-b1);
  // 			glVertex3f(l1,		0.0,	-b1);
  // 			glVertex3f(l1,		0.0,	-b1-b2);
  // 			glVertex3f(l1+l2,	0.0,	0.0);
  // 			glVertex3f(l1,		0.0,	 b1+b2);
  // 		glEnd();
  // 	glPopMatrix();
  //
  // 	// Z - Axis
  // 	glPushMatrix();
  // 		glColor3f(0.0,0.0,1.0);
  // 		glRotatef(-90, 0.0, 1.0, 0.0);
  // 		glBegin(GL_TRIANGLE_FAN);
  // 			glVertex3f(l1,		0.0,	 b1);
  // 			glVertex3f(0.0,		0.0,	 b1);
  // 			glVertex3f(0.0,		0.0,	-b1);
  // 			glVertex3f(l1,		0.0,	-b1);
  // 			glVertex3f(l1,		0.0,	-b1-b2);
  // 			glVertex3f(l1+l2,	0.0,	0.0);
  // 			glVertex3f(l1,		0.0,	 b1+b2);
  // 		glEnd();
  // 	glPopMatrix();

}

void
Tracker::drawImage ()
{
  glDepthMask (0);
  glColor3f (1.0, 1.0, 1.0);

  m_ip->render (m_tex_frame);

  glDepthMask (1);
}

void
Tracker::drawEdgeImage ()
{
  glDepthMask (0);
  glColor3f (1.0, 1.0, 1.0);

  m_ip->render (*m_tex_frame_ip[params.m_spreadlvl]);

  glDepthMask (1);
}

void
Tracker::drawHSVImage ()
{
  glDepthMask (0);
  glColor3f (1.0, 1.0, 1.0);

  TomGine::tgTexture2D tex;
  m_ip->rgb2hsv (m_tex_frame, tex);

  glDepthMask (1);
}

void
Tracker::drawCalibrationPattern (float point_size)
{

  if (m_calib_points.empty ())
    return;

  glDisable (GL_DEPTH_TEST);
  glDisable (GL_CULL_FACE);

  m_cam_perspective.Activate ();

  glPointSize (point_size);

  glBegin (GL_POINTS);

  for (unsigned i = 0; i < m_calib_points.size (); i++)
    glVertex3f (m_calib_points[i].x, m_calib_points[i].y, m_calib_points[i].z);

  glEnd ();
}

void
Tracker::loadCalibrationPattern (const char* mdl_file)
{
  FILE* pfile = 0;

  pfile = fopen (mdl_file, "r");

  if (pfile != NULL)
  {
    unsigned n = 0;
    vec3 p;
    rewind (pfile);
    fscanf (pfile, "%d", &n);
    m_calib_points.clear ();
    for (unsigned i = 0; i < n; i++)
    {
      fscanf (pfile, "%f %f %f", &p.x, &p.y, &p.z);
      m_calib_points.push_back (p);
    }
  }
  else
  {
    printf ("[Tracker::loadCalibrationPattern] Warning couldn't load model file '%s'\n", mdl_file);
  }

  fclose (pfile);
}

void
Tracker::drawPixel (float u, float v, vec3 color, float size)
{
  glDisable (GL_DEPTH_TEST);
  glDisable (GL_LIGHTING);
  m_ip->setCamOrtho ();

  glPointSize (size);
  glColor3f (color.x, color.y, color.z);

  glBegin (GL_POINTS);
  glVertex3f (u, v, 0.0);
  glEnd ();
}

void
Tracker::drawPoint (float x, float y, float z, float size)
{
  glDisable (GL_DEPTH_TEST);
  glDisable (GL_LIGHTING);
  m_cam_perspective.Activate ();

  glPointSize (size);
  // 	glColor3f(1.0,0.0,0.0);

  glBegin (GL_POINTS);
  glVertex3f (x, y, z);
  glEnd ();
}

void
Tracker::getGlError ()
{
  int err = glGetError ();
  switch (err)
  {
  case GL_NO_ERROR:
    break;
  case GL_INVALID_ENUM:
    printf ("glGetError: GL_INVALID_ENUM\n");
    break;
  case GL_INVALID_VALUE:
    printf ("glGetError: GL_INVALID_VALUE\n");
    break;
  case GL_INVALID_OPERATION:
    printf ("glGetError: GL_INVALID_OPERATION\n");
    break;
  case GL_STACK_OVERFLOW:
    printf ("glGetError: GL_STACK_OVERFLOW\n");
    break;
  case GL_STACK_UNDERFLOW:
    printf ("glGetError: GL_STACK_UNDERFLOW\n");
    break;
  case GL_OUT_OF_MEMORY:
    printf ("glGetError: GL_OUT_OF_MEMORY\n");
    break;
  default:
    printf ("glGetError: no known error\n");
    break;
  }
}

void
Tracker::computeModelEdgeMask (ModelEntry* modelEntry, TomGine::tgTexture2D &mask)
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  m_cam_perspective.Activate ();
  glEnable (GL_LIGHTING);
  glEnable (GL_DEPTH_TEST);
  glDepthMask (1);
  modelEntry->pose.Activate ();
  modelEntry->model.DrawFaces ();
  modelEntry->pose.Deactivate ();
  glDisable (GL_DEPTH_TEST);
  glDepthMask (0);
  glDisable (GL_LIGHTING);

  mask.CopyTexImage2D (params.camPar.width, params.camPar.height);
  m_ip->sobel (mask, mask, 0.01, true, true);
}

// Show performance and likelihood
void
Tracker::printStatistics ()
{
  printf ("\n\nStatistics: \n");

  for (unsigned i = 0; i < m_modellist.size (); i++)
  {
    tgPose pMean = m_modellist[i]->distribution.getMean ();
    printf ("	Object %d '%s'\n", i, m_modellist[i]->label.c_str ());
    printf ("		FPS: %.1f\n", 1.0f / m_ftime);
    printf ("		Textured: %d\n", m_modellist[i]->model.m_textured);
    printf ("		Recursions: %i\n", m_modellist[i]->num_recursions);
    printf ("		Particles: %i\n", m_modellist[i]->distribution.size ());
    printf ("		Position: %f, %f, %f\n", m_modellist[i]->pose.t.x, m_modellist[i]->pose.t.y, m_modellist[i]->pose.t.z);
    printf ("		Orientation: ");
    m_modellist[i]->pose.q.printAxisAngle ();
    printf ("		Variance: %f \n", m_modellist[i]->distribution.getPositionVariance ());
    printf ("		Confidence (Median): %f \n", m_modellist[i]->distribution.getMedianC ());
    printf ("		Confidence (Mean): %f \n", m_modellist[i]->distribution.getMean ().c);
    printf ("		Confidence (Max): %f \n", m_modellist[i]->distribution.getMax ().c);
    printf ("		Weight: %f \n", m_modellist[i]->distribution.getMax ().w);
    printf ("		Spread: %d / %d\n", params.m_spreadlvl, params.num_spreadings);
    printf ("		Kernel: %d \n", params.kernel_size);
    printf ("		ImageSobelTh: %f \n", params.image_sobel_th);
    printf ("		ModelSobelTh: %f \n", params.model_sobel_th);
  }
}

void
Tracker::printParams ()
{

  float dpi = 180.0f * M_1_PI;

  printf ("\n[Constraints]\n");
  printf ("r.x = %f\n", params.variation.r.x * dpi);
  printf ("r.y = %f\n", params.variation.r.y * dpi);
  printf ("t.z = %f\n", params.variation.r.z * dpi);
  printf ("t.x = %f\n", params.variation.t.x);
  printf ("t.y = %f\n", params.variation.t.y);
  printf ("t.z = %f\n", params.variation.t.z);
  printf ("z = %f\n", params.variation.z);

  printf ("\n[ParticleFilter]\n");
  printf ("recursions = %d\n", params.num_recursions);
  printf ("particles = %d\n", params.num_particles);
  printf ("convergence = %d\n", params.convergence);
  printf ("PredictorNoConvergence = %f\n", params.pred_no_convergence);
  printf ("KeepBestParticles = %f\n", params.keep_best_particles);
  printf ("PoseLpfFactor = %f\n", params.lpf);

  printf ("\n[ResourcePath]\n");
  printf ("ModelPath = %s\n", params.modelPath.c_str ());
  //  printf ("TexturePath = %s\n", params.texturePath.c_str ());
  //  printf ("ShaderPath = %s\n", params.shaderPath.c_str ());

  printf ("\n[ImageProcessing]\n");
  printf ("EdgeMatchingTolerance = %f\n", params.edge_tolerance * dpi);
  printf ("MinTextureGrabAngle = %f\n", params.minTexGrabAngle * dpi);
  printf ("NumberOfSpreadings = %d\n", params.num_spreadings);
  printf ("MaxKernelSize = %d\n", params.max_kernel_size);
  printf ("ModelSobelThreshold = %f\n", params.model_sobel_th);
  printf ("ImageSobelThreshold = %f\n", params.image_sobel_th);
  printf ("Method = %d\n", params.method);
}

