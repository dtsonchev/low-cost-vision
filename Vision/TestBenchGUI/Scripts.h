//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchGUI
// File:           Scripts.hpp
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

#ifndef SCRIPT_H
#define SCRIPT_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <set>
#include <exception>
#include <iostream>

struct Param {
    Param(std::string name, std::string type, std::string value) :
        name(name), type(type), value(value) {}
    std::string name, type, value;
};

struct Script {
    Script(std::string path, std::string name, bool training, std::string description, std::string python, std::vector<Param> params) :
        path(path), name(name), training(training), description(description), python(python), params(params)
    {}

    std::string path;
    std::string name;
    bool training;
    std::string description;
    std::string python;
    std::vector<Param> params;
};

class ScriptParser {
public:
    ScriptParser(){}
    std::vector<Script> loadScripts(const std::string &filename);
    std::vector<Script> getScripts() {return scripts;}
private:
    std::vector<Script> scripts;
};

#endif
