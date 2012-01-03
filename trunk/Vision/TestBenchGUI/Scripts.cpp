//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchGUI
// File:           Scripts.cpp
// Description:    Class and structs for using scripts
// Author:         Franc Pape
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of TestBenchGUI.
//
// TestBenchGUI is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// TestBenchGUI is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with TestBenchGUI.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#include "Scripts.h"

std::vector<Script> ScriptParser::loadScripts(const std::string &filename){
    using boost::property_tree::ptree;
    ptree pt;

    read_xml(filename, pt);
    BOOST_FOREACH(ptree::value_type &v, pt.get_child("scripts")){
        std::string path = v.second.get<std::string>("<xmlattr>.path");
        std::string name = v.second.get<std::string>("<xmlattr>.name");
        bool training = v.second.get("<xmlattr>.training", false);
        std::string python = v.second.get("<xmlattr>.python", "python");
        std::string desc;
        std::vector<Param> params;

        BOOST_FOREACH(ptree::value_type &v1, v.second){
            if(v1.first == "desc"){
                desc = v1.second.data();
            } else if(v1.first == "param") {
                Param p(
                    v1.second.get<std::string>("<xmlattr>.name"),
                    v1.second.get<std::string>("<xmlattr>.type"),
                    ""
                );
                params.push_back(p);
            }
        }

        scripts.push_back(Script(path, name, training, desc, python, params));
    }

    return scripts;
}
