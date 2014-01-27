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

#ifndef _TG_SHADER_H__
#define _TG_SHADER_H__

#include "v4r/TomGine/headers.h"
#include "v4r/TomGine/tgMathlib.h"
#include <string>

namespace TomGine {

class tgShader
{
private:
  GLhandleARB fragment;
  GLhandleARB vertex;
  GLhandleARB program;

public:
  /** @brief Creating the shader programs.
   *  @param	vertex_file	Path to vertex shader file.
   *  @param fragment_file	Path to fragment shader file.
   *  @param vertex_header_file	Path to header file, which is included before vertex shader string.
   *  @param fragment_header_file	Path to header file, which is included before fragment shader string. */
  tgShader(const std::string &vertex_file,
           const std::string &fragment_file,
           const std::string &vert_header_file = "",
           const std::string &frag_header_file = "",
           const std::string &to_replace = "",
           const std::string &replace_with = "");

  /** @brief Destroys shader programs. */
  ~tgShader();

  /** @brief Binds/Activates shader program for usage. */
  void bind();
  /** @brief Unbinds/Disables shader program. */
  void unbind();

  /** @brief	Returns the location of an attribute variable of the bound shader. */
  GLuint getAttribLoc(const char*);
  /** @brief	Returns the location of an uniform variable of the bound shader. */
  GLint getUniformLoc(const char*);
  /** @brief Specify the value of a 'uniform int' variable for the current program object */
  void setUniform(const char*, int);
  /** @brief Specify the value of a 'uniform unsigned' variable for the current program object */
  void setUniform(const char*, unsigned);
  /** @brief Specify the value of a 'uniform int[]' variable for the current program object */
  void setUniform(const char*, int, const int*);
  /** @brief Specify the value of a 'uniform float' variable for the current program object */
  void setUniform(const char*, float);
  /** @brief Specify the value of a 'uniform float[]' variable for the current program object */
  void setUniform(const char*, int, const float*);
  /** @brief Specify the value of a 'uniform vec2' variable for the current program object */
  void setUniform(const char*, vec2);
  /** @brief Specify the value of a 'uniform vec2[]' variable for the current program object */
  void setUniform(const char* var, int n, vec2* f);
  /** @brief Specify the value of a 'uniform vec3' variable for the current program object */
  void setUniform(const char*, vec3);
  /** @brief Specify the value of a 'uniform vec3[]' variable for the current program object */
  void setUniform(const char* var, int n, vec3* f);
  /** @brief Specify the value of a 'uniform vec4' variable for the current program object */
  void setUniform(const char*, vec4);
  /** @brief Specify the value of a 'uniform vec4[]' variable for the current program object */
  void setUniform(const char* var, int n, vec4* f);
  /** @brief Specify the value of a 'uniform mat3' variable for the current program object */
  void setUniform(const char*, mat3, bool transpose = false);
  /** @brief Specify the value of a 'uniform mat3[]' variable for the current program object */
  void setUniform(const char* var, int n, mat3* f, bool transpose = false);
  /** @brief Specify the value of a 'uniform mat4' variable for the current program object */
  void setUniform(const char*, mat4, bool transpose = false);
  /** @brief Specify the value of a 'uniform mat4[]' variable for the current program object */
  void setUniform(const char* var, int n, mat4* f, bool transpose = false);

};

/** @brief read text file as string */
std::string read_text_file(const std::string &filename);

} //namespace TomGine

#endif //__SHADER_H__
