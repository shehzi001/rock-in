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
#ifndef __TRACKER_MODEL_H__
#define __TRACKER_MODEL_H__

namespace Tracking
{
  class TrackerModel;
}
#include "headers.h"
#include "v4r/TomGine/tgTextureModel.h"
#include "v4r/TomGine/tgTexture.h"
#include "v4r/TomGine/tgMathlib.h"
#include "v4r/TomGine/tgPose.h"
#include "v4r/TomGine/tgCamera.h"
#include "v4r/TomGineIP/tgShader.h"

#ifndef FN_LEN
#define FN_LEN 256
#endif

namespace Tracking
{

  /** @brief 3D Model with special methods for tracking and texturing */
  class TrackerModel : public TomGine::tgModel
  {
  private:
    TrackerModel (const TrackerModel& m);

    TrackerModel&
    operator= (const TomGine::tgModel& m); // no implementation (should not be used)


  public:
    TrackerModel ();
    TrackerModel (const TomGine::tgModel& m);
    ~TrackerModel ();

    TrackerModel&
    operator= (const TrackerModel& m);

    void
    CopyTo(TomGine::tgTextureModel &m);

    virtual void
    Merge (const tgModel &m);

    void
    releasePassList ();

    struct Pass
    { // Renderpass
      std::vector<unsigned> f; // Faces to draw with this pass
      TomGine::mat4 modelviewprojection; // Modelview and projection matrix for texCoords
      float x, y, w, h; // Bounding box of SubTexture
      TomGine::tgTexture2D texture; // TomGine::tgTexture2D to use
    };

    typedef std::vector<Pass*> PassList;

    // Variables
    PassList m_passlist;
    std::vector<int> m_facepixellist;

    TomGine::tgTexture2D* m_tex_original; // original texture of model (not modified by tracker)
    TomGine::tgTexture2D* m_texture; // texture of model modified by tracker (edge-texture)
    bool m_textured;
    bool m_fulltextured;

    // computes, updates
    void
    computeEdges ();
    void
    computeBoundingSphere ();
    void
    Update ();

    // draws
    virtual void
    Print () const;
    virtual void
    drawNormals (float normal_length = 0.01);
    virtual void
    drawFaces (bool colorful = false);
    void
    drawFace (int i);
    void
    drawEdges ();
    void
    drawTexturedFaces ();
    void
    drawUntexturedFaces ();
    void
    drawPass (bool colorful = false);
    void
    drawCoordinates ();

    std::vector<unsigned>
    getFaceUpdateList (const TomGine::tgPose& pose, TomGine::vec3 view, float minTexGrabAngle = 3.0 * M_PI_4,
                       bool use_num_pixels = true);

    void
    getBoundingBox2D (TomGine::tgPose& pose, TomGine::tgRect2Di &bb, bool pow2 = false);

    /** @brief capture texture from image */
    void
    textureFromImage (TomGine::tgTexture2D &image, int width, int height, const TomGine::tgPose& pose,
                      const std::vector<unsigned> &faceUpdateList, TomGine::tgCamera* m_cam);

    void
    useTexCoords (bool useTC);
    void
    unwarpTexturesBox_hacky (const char* name);

    // gets
    bool
    getTextured ()
    {
      return m_textured;
    }
    bool
    getFullTextured ()
    {
      return m_fulltextured;
    }
    TomGine::tgTexture2D*
    getTexture ()
    {
      return m_texture;
    }
    TomGine::tgTexture2D*
    getOriginalTexture ()
    {
      return m_tex_original;
    }
    float
    getBoundingSphereRadius ()
    {
      return m_boundingSphereRadius;
    }

    // sets
    void
    setBFC (bool bfc)
    {
      m_bfc = bfc;
    }
    void
    setTexture (TomGine::tgTexture2D* tex)
    {
      m_texture = tex;
    }
    void
    setOriginalTexture (TomGine::tgTexture2D* tex)
    {
      m_tex_original = tex;
    }
    void
    restoreTexture ()
    {
      m_texture = m_tex_original;
    }

  protected:
    GLint m_dlTexturedFaces;
    GLint m_dlUntexturedFaces;
    GLint m_dlPass;
    GLint m_dlFaces;
    GLint m_dlEdges;
    GLint m_dlNormals;

    TomGine::tgShader* m_shadeTexturing;
    int m_shadeTexturingID;
    bool m_bfc;
    float m_boundingSphereRadius;

    // Functions
    bool
    isRedundant (TomGine::tgLine* e1);
    void
    UpdateDisplayLists ();
  };

} // namespace Tracking

#endif
