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
 * $Id: Config.cc,v 1.1.1.1 2008/02/29 10:12:02 jp Exp $
 */

#include "ConfigFile.hh"
#include "Config.hh"

namespace P 
{

Config::Config(const char *filename)
{
  Load(filename);
}

void Config::Load(const char *filename)
{
  ConfigFile file(filename);
  string line, name, value;
  string::size_type pos;
  while(file.GetLine(line))
  {
    pos = 0;
    GetWord(line, name, pos);
    GetWord(line, value, pos);
    items[name] = value;
  }
}


bool Config::ExistItem(const string &name)
{
  map<string,string>::iterator it = items.find(name);
  if( it == items.end() ) return false;
  return true; 
}

}

