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