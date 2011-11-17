#pragma once

#include <vector>
#include <map>
#include <set>
#include <ImageMD/Types.hpp>

namespace ImageMetaData {

//========================================================================
// Functions for obtaining and manipulating image metadata
//========================================================================
/**
 * Extracts the meta data of images from an XML file and puts it in ImageMD objects
 * @param path the path of the XML file containing the meta data
 * @param rootTag the first tag in the XML file
 * @return a vector of ImageMD objects with the extracted meta data
 */
std::vector<ImageMD> getMetaData(std::string path, std::string rootTag);

/**
 * Parses the contents of a string and puts it in a AnyType as either an int, a double or a string
 * @param str the string to be interpreted
 * @return an AnyType object containing the parsed information
 */
AnyType AnyTypeFromString(std::string str);

/**
 * Checks if a map contains a certain key
 * @param m the map to be checked
 * @param k the key to be checked
 * @return true if it contains the key, false otherwise
 */
template<class K, class V>
bool ContainsKey(const std::map<K, V>& m, const K& k){
	return m.find(k) != m.end();
}

}
