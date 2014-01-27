/*
 * Software License Agreement (GNU General Public License)
 *
 *  Copyright (c) 2013, Johann Prankl
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
 * @author johann.prankl
 *
 */
/**
 * $Id: Except.cc,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 */

#include <stdarg.h>
#include <stdio.h>
#include "Except.hh"

namespace P
{

/**
 * Except constructor.
 * @param file     (in) __FILE__ macro
 * @param function (in) __FUNCTION__ macro
 * @param line     (in) __LINE__ macro
 * @param format   (in) printf-style format string
 */
Except::Except(const char *file, const char *function, int line,
               const char *format, ...) throw()
{
  static char what[1024];
  static char msg[1024];
  va_list arg_list;
  va_start(arg_list, format);
  vsnprintf(what, 1024, format, arg_list);
  va_end(arg_list);
  snprintf(msg, 1024, "%s:%s:%d: %s", file, function, line, what);
  _what = msg;
}

}

