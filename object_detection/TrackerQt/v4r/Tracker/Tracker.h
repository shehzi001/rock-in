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
 * @file Tracker.h
 * @author Thomas Mörwald
 * @date October 2009
 * @version 0.1
 * @brief Main file of Tracker.
 * @namespace Tracking
 */
#ifndef _TRACKER_H_
#define _TRACKER_H_

#include "headers.h"
#include "Timer.h"
#include "Resources.h"
#include "ModelEntry.h"
#include "v4r/TomGine/tgMathlib.h"
#include "v4r/TomGine/tgLight.h"
#include "v4r/TomGine/tgFont.h"
#include "v4r/TomGine/tgTexture.h"
#include "v4r/TomGineIP/tgHistDesc2D.h"


/** @brief namespace Tracking */
namespace Tracking{

typedef std::vector<ModelEntry*> ModelEntryList;

/** @brief Main class of Tracker, defining API */
class Tracker{
public:
  struct Parameter{
    TomGine::tgCamera::Parameter camPar;
    int model_id_count;
    int hypotheses_id_count;
    int num_particles;					///< number of particles to draw for each frame
    int num_recursions;					///< number of recursions for each image
    //    unsigned hypotheses_trials;			///< number of trials a hypothesis gets for convergence, until comparisson to original
    int convergence;					///< convergence factor
    float edge_tolerance;				///< maximal angular deviation of edges to match in degrees
    unsigned int num_spreadings;		///< number of spreadings during image processing
    unsigned int m_spreadlvl;			///< Width of edges in pixels
    Particle variation;					///< standard deviation of particle distribution in meter
    float minTexGrabAngle;				///< Angular threshold between view vector and face normal for grabing texture
    unsigned int max_kernel_size;		///< Max. edgel comparison kernel (distance of neighbouring pixels taken into account)
    int kernel_size;					///< Edgel comparison kernel (distance of neighbouring pixels taken into account)
    std::string modelPath;				///< Path to model recources
    //    std::string texturePath;			///< Path to texture recources
    //		std::string shaderPath;				///< Path to shader recources
    float model_sobel_th;				///< Threshold for sobel edge detection for model
    float image_sobel_th;				///< Threshold for sobel edge detection for image
    float pred_no_convergence;			///< Percentage of particles of distribution voting for no convergence
    float keep_best_particles;			///< Percentage of particles kept when resampling (low=more robust, more jitter; high=less jitter, less robust)
    float lpf;							///< low pass filtering factor for pose filtering
    Method method;						///< Tracking method (edges / color / color+edges(default)

    // Constructor (assigning default values)
    Parameter();

    // Writes the parameters to a stream.
    friend std::ostream& operator << (std::ostream& os, Parameter& param);

    // Reads the parameters from a stream.
    friend std::istream& operator >> (std::istream& is, Parameter& param);

    void print();
  };



public:
  Tracker();
  ~Tracker();

  // Main functions (init, image_processing, tracking, reset)
  /** @brief Initialize tracker with an INI file and image/window width and height in pixel */
  bool init(const Parameter& trackParam);

  /** @brief Perform image processing with edge detection */
  void loadImage(unsigned char* image, GLenum format=GL_BGR);
  /** @brief Perform image processing with edge detection
  *	@param image image data
  *	@param format data format */
  virtual void image_processing(unsigned char* image, GLenum format=GL_BGR)=0;
  /** @brief Perform image processing with edge detection, painting a virtual object into the image
  *	@param image image data
  *	@param model geometry of virtual object
  *	@param pose position and orientation of the virtual object in world space
  *	@param format data format */
  virtual void image_processing(unsigned char* image, const TomGine::tgModel &model, const TomGine::tgPose &pose, GLenum format=GL_BGR)=0;
  /** @brief Perform image processing with edge detection, painting the model with id as virtual object into image
  *	@param image image data
  *	@param model_id id of the model
  *	@param pose position and orientation of the virtual object in world space
  *	@param format data format*/
  virtual void image_processing(unsigned char* image, int model_id, const TomGine::tgPose &pose, GLenum format=GL_BGR)=0;

  virtual void model_processing(ModelEntry* modelEntry, TomGine::tgTexture2D &tex_model_ip){}

  virtual void tsd_processing(ModelEntry* modelEntry, TomGine::tgHistDesc2D &desc_g_image,
                              TomGine::tgHistDesc2D &desc_g_model, TomGine::tgHistDesc2D &desc_c_image, TomGine::tgHistDesc2D &desc_c_model,
                              unsigned segx = 5, unsigned segy = 5, double vis_ratio = 0.75){}

  /** @brief Tracks all models by matching their edges against edges of images */
  virtual bool track()=0;
  /** @brief Tracks model by id by matching their edges against edges of images
  *	@param id the id of the model given by addModel() or addModelFromFile() */
  virtual bool track(int id)=0;

  /** @brief Resets the pose of all models to the initial pose */
  void reset();
  /** @brief Resets the pose of a model to the initial pose
  *	@param id the id of the model given by addModel() or addModelFromFile() */
  void reset(int id);

  // Model handling
  /** @brief Adds a geometrical model to the tracker
  *	@return id of the added model (-1 if not successfull)	*/
  int addModel(TomGine::tgModel& m, TomGine::tgPose& pose,  std::string label, bool bfc=true);

  /** @brief Adds a geometrical model from file (ply-fileformat) to the tracker
  *	@param filename absolute filename of the model (or relative to the execution path)
  *	@param pose place where the model is initially put to
  *	@param label label of the model
  *	@param bfc enable/disable backfaceculling (look up OpenGL Backface Culling)
  *	@return  id of the added model (-1 if not successfull)	*/
  int addModelFromFile(const char* filename, TomGine::tgPose& pose, std::string label, bool bfc=true);

  /** @brief Remove model from tracker by id */
  void removeModel(int id);

  /** @brief Adds a pose hypothesis to model */
  void addPoseHypothesis(int id, TomGine::tgPose &p, std::string label, bool bfc);

  /** @brief Get bounding box of object in image space in pixel */
  void getModelBoundingBox2D( int id, TomGine::tgRect2Di &bb, bool pow2=false );

  /** @brief Get mean confidence value of distribution of a model */
  void getModelConfidenceMean(int id, float& c);

  /** @brief Get mean confidence value of distribution of a model */
  void getModelConfidenceMedian(int id, float& c);

  /** @brief Get mean confidence value of distribution of a model */
  void getModelConfidenceMax(int id, float& c);

  /** @brief Calculates mask produced by the geometry edges of a model */
  void getModelEdgeMask(int id, TomGine::tgTexture2D &mask);

  /** @brief DO NOT USE THIS FUNCTION! */
  ModelEntry* getModelEntry(int id);

  /** @brief Get the initial pose of a model */
  void getModelInitialPose(int id, TomGine::tgPose& p);

  /** @brief Calculates mask produced by the geometry edges of a model */
  void getModelMask(int id, TomGine::tgTexture2D &mask);

  /** @brief Gets flag to mask geometry edges of model */
  bool getModelMaskOwnEdges(int id);

  /** @brief Get 3D point from 2D window coordinates */
  bool getModelPoint3D(int id, int x_win, int y_win, double& x3, double& y3, double& z3);

  /** @brief Get current pose of a model */
  void getModelPose(int id, TomGine::tgPose& p);

  /** @brief Get low pass filtered pose of a model */
  TomGine::tgPose getModelPoseLPF(int id);

  /** @brief Get current pose of a model in camera coordinates */
  void getModelPoseCamCoords(int id, TomGine::tgPose& p);

  /** @brief Get a tracking quality state of a model */
  void getModelTrackingState(int id, TrackingState &ts);

  /** @brief Get current velocity of a model ( in m/s and rad/s ) */
  void getModelVelocity(int id, float &translational, float &angular);

  /** @brief Get camera of tracker (perspective) */
  const TomGine::tgCamera getCamera() const;


  /** @brief Set the initial pose of a model */
  void setModelInitialPose(int id, const TomGine::tgPose &p);

  /** @brief Set a model predictor */
  void setModelPredictor(int id, Predictor* predictor);

  /** @brief Locks the model with id */
  void setModelLock(int id, bool lock);

  /** @brief Sets the number of recursions and particles used for tracking */
  void setModelRecursionsParticle(int id, int num_recursions, int num_particles);

  /** @brief Set the number of particles (in percent) which are voting for no convergence (for capturing fast movement)*/
  void setModelPredictorNoConvergence(int id, float no_conv);

  /** @brief Sets mask for a model to define edges which are not considered for the model */
  void setModelMask(int id, TomGine::tgTexture2D *mask=0);

  /** @brief Sets flag to mask geometry edges of model */
  void setModelMaskOwnEdges(int id, bool masked);

  /** @brief Save model to file */
  void saveModel(int id, std::string filename);
  /** @brief Save all models to file */
  void saveModels(const char* pathname);

  /** @brief Takes screenshot and saves it to file */
  void saveScreenshot(const char* filename);

  /** @brief Captures image and attaches it to model as texture */
  virtual void textureFromImage(bool force=false){}
  virtual void textureFromImage(int id, const TomGine::tgPose &pose, bool use_num_pixels=true){}
  virtual void textureFromImage (int id, const TomGine::tgPose &pose, const std::vector<unsigned> &face_indices){}

  /** @brief Remove textures from model */
  virtual void untextureModels(){}

  // Drawing to screen (result, ...)
  /** @brief Draw all models */
  virtual void drawResult(float linewidth=1.0f)=0;
  void drawModel(TomGine::tgPose p);
  void drawModel(const TomGine::tgModel &m, const TomGine::tgPose &p);
  virtual void drawModelEntry(int id, int mode, float linewidth=1.0f){}
  void drawModelWireframe(const TomGine::tgModel &m, const TomGine::tgPose &p, float linewidth=1.0f);
  void drawCoordinateSystem(float linelength=0.5f, float linewidth=1.0f, TomGine::tgPose pose=TomGine::tgPose());
  void drawCoordinates(float linelength=1.0f);
  void drawImage();
  void drawEdgeImage();
  void drawHSVImage();
  virtual void drawModelEdgeImage()=0;
  void drawPixel(float u, float v, TomGine::vec3 color=TomGine::vec3(1.0,1.0,1.0), float size=1.0f);
  void drawPoint(float x, float y, float z, float size=1.0);
  void drawCalibrationPattern(float point_size=1.0f);
  void loadCalibrationPattern(const char* mdl_file);
  void drawTest();
  void printStatistics();
  void printParams();

  // set parameters for texture tracking
  virtual void setKernelSize(int val){ }

  // Set Parameters
  void setParameter(const Tracker::Parameter &_params);
  void setFrameTime(double dTime);
  bool setCameraParameters(TomGine::tgCamera::Parameter cam_par);
  void setSpreadLvl(unsigned int val){ params.m_spreadlvl = val; }

  void setCamPerspective(){ m_cam_perspective.Activate(); }
  void setDrawImage(bool val){ m_drawimage = val; }

  // Get Parameters
  float getCamZNear(){ return m_cam_perspective.GetZNear(); }
  float getCamZFar(){ return m_cam_perspective.GetZFar(); }
  unsigned int getSpreadLvl(){ return params.m_spreadlvl; }
  Method getMethod(){ return params.method; }

  // get Flags
  bool getLockFlag(){ return m_lock; }
  bool getEdgesImageFlag(){ return m_draw_edges; }
  bool getDrawParticlesFlag(){ return m_showparticles; }
  int  getModelModeFlag(){ return m_showmodel; }
  bool getTrackingStateDetection(){ return m_tsd; }

  // set Flags
  void setLockFlag(bool val);
  void setEdgesImageFlag(bool val){ m_draw_edges = val; }
  void setDrawParticlesFlag(bool val){ m_showparticles = val; }
  void setModelModeFlag(int val){ m_showmodel = val; }
  void setTrackingStateDetection(bool val){ m_tsd = val; }

  // Functions for analysing PDF
  virtual void evaluatePDF( int id,
                            float x_min, float y_min,
                            float x_max, float y_max,
                            int res,
                            const char* meshfile, const char* xfile){}

  virtual std::vector<float> getPDFxy(	Particle pose,
                                        float x_min, float y_min,
                                        float x_max, float y_max,
                                        int res,
                                        const char* filename=NULL, const char* filename2=NULL){ std::vector<float> a; return a;}

  virtual void savePDF(	std::vector<float> vPDFMap,
                        float x_min, float y_min,
                        float x_max, float y_max,
                        int res,
                        const char* meshfile, const char* xfile){}

protected:
  Parameter params;

  std::vector<TomGine::vec3> m_calib_points;

  TomGine::tgLight m_light;
  TomGine::tgImageProcessor* m_ip;

  TomGine::tgTexture2D m_tex_frame;
  std::vector<TomGine::tgTexture2D*> m_tex_frame_ip;

  float m_ftime;
  Timer m_timer;

  // ModelEntry
  ModelEntryList m_modellist;
  ModelEntryList m_hypotheses;

  TomGine::tgCamera m_cam_perspective;
  TomGine::tgCamera m_cam_default;

  // Flags
  bool m_lock;
  bool m_showparticles;
  int  m_showmodel;
  bool m_draw_edges;
  bool m_tracker_initialized;
  bool m_drawimage;
  bool m_tsd;
  bool m_pose_filter;

  // Functions
  void getGlError();

  void computeModelEdgeMask(ModelEntry* modelEntry, TomGine::tgTexture2D &mask);

  virtual bool initInternal()=0;
  bool init();
  bool initGL();

};

} // namespace Tracking

#endif