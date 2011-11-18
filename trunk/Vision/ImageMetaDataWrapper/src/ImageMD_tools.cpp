#include <iostream>
#include <sstream>
#include <map>
#include <vector>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <ImageMD/Types.hpp>
#include <ImageMD/ImageMD_Tools.hpp>

#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

std::vector<ImageMetaData::ImageMD> ImageMetaData::getMetaData(const std::string& path,
		const std::string& rootTag) {
	using std::string;
	using std::stringstream;
	using std::vector;
	using std::map;
	using boost::property_tree::ptree;

	vector<ImageMD> md;
	ptree pt;

	read_xml(path, pt);
	foreach(ptree::value_type &img, pt.get_child(rootTag)){
		ImageMD imd(img.second.get<string>("<xmlattr>.path"));

		foreach(ptree::value_type &img_child, img.second) {
			if(img_child.first == "category") {
				imd.categories[img_child.second.get<string>("<xmlattr>.name")] =
						img_child.second.get<string>("<xmlattr>.value");
			} else if(img_child.first == "property") {
				imd.properties[img_child.second.get<string>("<xmlattr>.name")] =
						AnyTypeFromString(img_child.second.get<string>("<xmlattr>.value"));
			} else if(img_child.first == "object") {
				map<string, AnyType> properties;

				foreach(ptree::value_type &obj_child, img_child.second) {
					if(obj_child.first == "property") {
						properties[obj_child.second.get<string>("<xmlattr>.name")] =
								AnyTypeFromString(obj_child.second.get<string>("<xmlattr>.value"));
					}
				}

				imd.objects.push_back(properties);
			}
		}

		md.push_back(imd);
	}

	return md;
}

ImageMetaData::AnyType ImageMetaData::AnyTypeFromString(const std::string& str){
	using std::stringstream;

	stringstream ss;
	ss << str;

	// Check for integer
	int i;
	ss >> i;
	if(!ss.fail()){
		return AnyType(i);
	}

	// Check for double
	double d;
	ss >> d;
	if(!ss.fail()){
		return AnyType(d);
	}

	// Otherwise it must be a string
	return AnyType(ss.str());
}
