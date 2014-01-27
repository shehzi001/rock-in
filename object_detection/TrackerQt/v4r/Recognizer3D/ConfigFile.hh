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
 * $Id: ConfigFile.hh,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 */

#ifndef P_CONFIG_FILE_HH
#define P_CONFIG_FILE_HH

#include <string>
#include <fstream>
#include "PNamespace.hh"

namespace P
{

class ConfigFile
{
private:
  fstream file;
  int line_cnt;

private:
  void RemoveEOL(string &str);
  bool IsComment(string &str);
public:
  ConfigFile(const string &name);
  ~ConfigFile();
  bool GetLine(string &str);
  int GetLineCnt() {return line_cnt;}
};

/// Get the next word from a string, starting at pos.
bool GetWord(const string &str, string &word, string::size_type &pos);

}

#endif

