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