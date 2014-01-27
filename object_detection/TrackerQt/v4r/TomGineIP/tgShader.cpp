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
#include "tgShader.h"
#include <string.h>
#include <stdarg.h>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

using namespace TomGine;

tgShader::tgShader(const std::string &vertex_file, const std::string &fragment_file,
                   const std::string &vert_header_file, const std::string &frag_header_file,
                   const std::string &to_replace, const std::string &replace_with)
{
  GLint status;
  std::string vs = read_text_file(vertex_file);
  std::string fs = read_text_file(fragment_file);
  std::string vhs = read_text_file(vert_header_file);
  std::string fhs = read_text_file(frag_header_file);

  // check if all files have been loaded
  if (!vertex_file.empty() && vs.empty())
  {
    std::ostringstream os;
    os << "[tgShader::tgShader] Error loading file for vertex shader '";
    os << vertex_file << "'" << std::endl;
    os << "  Have you installed TomGineIP (make install)?" << std::endl;
    throw std::runtime_error(os.str().c_str());
  }
  if (!fragment_file.empty() && fs.empty())
  {
    std::ostringstream os;
    os << "[tgShader::tgShader] Error loading file for fragment shader '";
    os << fragment_file << "'" << std::endl;
    os << "  Have you installed TomGineIP (make install)?" << std::endl;
    throw std::runtime_error(os.str().c_str());
  }
  if (!vert_header_file.empty() && vhs.empty())
  {
    std::ostringstream os;
    os << "[tgShader::tgShader] Error loading header file for vertex shader '";
    os << vert_header_file << "'" << std::endl;
    os << "  Have you installed TomGineIP (make install)?" << std::endl;
    throw std::runtime_error(os.str().c_str());
  }
  if (!frag_header_file.empty() && fhs.empty())
  {
    std::ostringstream os;
    os << "[tgShader::tgShader] Error loading header file for fragment shader '";
    os << frag_header_file << "'" << std::endl;
    os << "  Have you installed TomGineIP (make install)?" << std::endl;
    throw std::runtime_error(os.str().c_str());
  }

  // add headers
  if (!vert_header_file.empty())
    vs = vhs + vs;
  if (!frag_header_file.empty())
    fs = fhs + fs;

  // replace
  if (!to_replace.empty())
  {
    size_t f = vs.find(to_replace);
    while (f != std::string::npos)
    {
      vs.replace(f, to_replace.length(), replace_with);
      f = vs.find(to_replace);
    }

    f = fs.find(to_replace);
    while (f != std::string::npos)
    {
      fs.replace(f, to_replace.length(), replace_with);
      f = fs.find(to_replace);
    }
  }

  // load vertex shader
  if (!vs.empty())
  {
    const char* vc = vs.c_str();
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vc, NULL);
    glCompileShader(vertex);

    glGetShaderiv(vertex, GL_COMPILE_STATUS, &status);
    if (!status)
    {
      printf("[tgShader::tgShader] Error compiling vertex shader '%s'\n", vertex_file.c_str());
      program = 0;
      return;
    }
  } else
    vertex = 0;

  // load fragment shader
  if (!fs.empty())
  {
    const char* fc = fs.c_str();
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fc, NULL);
    glCompileShader(fragment);

    glGetShaderiv(fragment, GL_COMPILE_STATUS, &status);
    if (!status)
    {
      printf("[tgShader::tgShader] Error compiling fragment shader '%s'\n", fragment_file.c_str());
      program = 0;
      return;
    }
  } else
    fragment = 0;

  if (fragment == 0 && vertex == 0)
  {
    program = 0;
    return;
  }

  program = glCreateProgram();
  if (vertex != 0)
  {
    glAttachShader(program, vertex);
    glDeleteShader(vertex);
  }
  if (fragment != 0)
  {
    glAttachShader(program, fragment);
    glDeleteShader(fragment);
  }
  glLinkProgram(program);

  glGetProgramiv(program, GL_LINK_STATUS, &status);
  if (!status)
  {
    printf("Error linking program with '%s' and '%s'\n", vertex_file.c_str(), fragment_file.c_str());
    glDeleteShader(program);
    program = 0;
    return;
  }

  glValidateProgram(program);
  glGetProgramiv(program, GL_VALIDATE_STATUS, &status);
  if (!status)
  {
    printf("Error, validating program '%s' and '%s'\n", vertex_file.c_str(), fragment_file.c_str());
    glDeleteProgram(program);
    program = 0;
  }

  tgCheckError("[tgShader::tgShader]");
}

tgShader::~tgShader()
{
  if (program != 0)
    glDeleteProgram(program);

  tgCheckError("[tgShader::~tgShader]");
}

void tgShader::bind()
{
  if (program)
    glUseProgram(program);
}

void tgShader::unbind()
{
  if (program)
    glUseProgram(0);
}

GLuint tgShader::getAttribLoc(const char *attr)
{
  return glGetAttribLocation(program, attr);
}

GLint tgShader::getUniformLoc(const char* var)
{
  return glGetUniformLocation(program, var);
}

void tgShader::setUniform(const char* var, int f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform1i(loc, f);
}

void tgShader::setUniform(const char* var, unsigned f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform1i(loc, (int) f);
}

void tgShader::setUniform(const char* var, int n, const int* f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform1iv(loc, n, f);
}

void tgShader::setUniform(const char* var, float f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform1f(loc, f);
}

void tgShader::setUniform(const char* var, int n, const float* f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform1fv(loc, n, f);
  //printf("f: %f %f %f\n", f[0],f[1],f[2]);
}

void tgShader::setUniform(const char* var, vec2 f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform2fv(loc, 1, f);
}

void tgShader::setUniform(const char* var, int n, vec2* f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform1fv(loc, 2 * n, f[0]);
}

void tgShader::setUniform(const char* var, vec3 f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform3fv(loc, 1, f);
}

void tgShader::setUniform(const char* var, int n, vec3* f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform1fv(loc, 3 * n, f[0]);
}

void tgShader::setUniform(const char* var, vec4 f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform4fv(loc, 1, f);
}

void tgShader::setUniform(const char* var, int n, vec4* f)
{
  int loc = glGetUniformLocation(program, var);
  glUniform1fv(loc, 4 * n, f[0]);
}

void tgShader::setUniform(const char* var, mat3 f, bool transpose)
{
  int loc = glGetUniformLocation(program, var);
  glUniformMatrix3fv(loc, 1, transpose, f);
}

void tgShader::setUniform(const char* var, int n, mat3* f, bool transpose)
{
  int loc = glGetUniformLocation(program, var);
  glUniformMatrix3fv(loc, n, transpose, f[0]);
}

void tgShader::setUniform(const char* var, mat4 f, bool transpose)
{
  int loc = glGetUniformLocation(program, var);
  glUniformMatrix4fv(loc, 1, transpose, f);
}

void tgShader::setUniform(const char* var, int n, mat4* f, bool transpose)
{
  int loc = glGetUniformLocation(program, var);
  glUniformMatrix4fv(loc, n, transpose, f[0]);
}

std::string TomGine::read_text_file(const std::string &filename)
{
  if (filename.empty())
    return "";

  std::ifstream ifs(filename.c_str(), std::ios::in);
  if (!ifs)
    return "";

  std::string out;
  ifs.seekg(0, std::ios::end);
  out.resize(ifs.tellg());
  ifs.seekg(0, std::ios::beg);
  ifs.read(&out[0], out.size());
  ifs.close();
  return out;
}

