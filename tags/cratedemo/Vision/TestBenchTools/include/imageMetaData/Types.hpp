//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchTools
// File:           Types.hpp
// Description:    Classes and typedefs for more convenient usage
// Author:         Franc Pape
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of TestBenchTools.
//
// TestBenchTools is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// TestBenchTools is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with TestBenchTools.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

#pragma once

#include <vector>
#include <map>
#include <boost/any.hpp>

namespace imageMetaData {

//========================================================================
// AnyType class (holds any type)
//========================================================================
/**
 * The AnyType class can hold any type and can then be used
 * as if it were that type, to some extent.
 */
class AnyType {
public:
	/**
	 * Empty constructor
	 */
	AnyType() :
			content() {
	}

	/**
	 * Specific constructor for const char*, to store it as string
	 * @param value the value
	 */
	AnyType(const char* value) :
			content(std::string(value)) {
	}

	/**
	 * Constructor that can receive any type
	 * @param value the value
	 */
	template<typename ValueType>
	AnyType(const ValueType& value) :
			content(value) {
	}

	/**
	 * Constructor that receives a boost::any object
	 * @param value the value
	 */
	AnyType(const boost::any & value) :
			content(value) {
	}

	/**
	 * Copy constructor
	 * @param value the object to be copied
	 */
	AnyType(const AnyType& value) :
			content(value.content) {
	}
//========================================================================
// Assignment operator
//========================================================================
	template<typename ValueType>
	AnyType & operator=(const ValueType& rhs) {
		content = rhs;
		return *this;
	}

//========================================================================
// Cast operator
//========================================================================
	template<typename ValueType>
	operator ValueType() {
		return boost::any_cast<ValueType>(content);
	}

//========================================================================
// Comparison operators
//========================================================================
	bool operator==(const char* other) const {
		return boost::any_cast<std::string>(content) == std::string(other);
	}

	template<typename ValueType>
	friend bool operator==(const AnyType& lhs, const ValueType& rhs) {
		return boost::any_cast<ValueType>(lhs.content) == rhs;
	}

	template<typename ValueType>
	friend bool operator==(const ValueType& lhs, const AnyType& rhs) {
		return rhs == lhs;
	}

	template<typename ValueType>
	friend bool operator!=(const AnyType& lhs, const ValueType& rhs) {
		return !(lhs == rhs);
	}

	template<typename ValueType>
	friend bool operator!=(const ValueType& lhs, const AnyType& rhs) {
		return !(rhs == lhs);
	}

//========================================================================
// Binary arithmetic operators
//========================================================================
	friend std::string operator+(const AnyType& lhs, const char* rhs) {
		return boost::any_cast<std::string>(lhs.content) + std::string(rhs);
	}

	friend std::string operator+(const char* lhs, const AnyType& rhs) {
		return std::string(lhs) + boost::any_cast<std::string>(rhs.content);
	}

	template<typename ValueType>
	friend ValueType operator+(const AnyType& lhs, const ValueType& rhs) {
		return boost::any_cast<ValueType>(lhs.content) + rhs;
	}

	template<typename ValueType>
	friend ValueType operator+(const ValueType &lhs, const AnyType& rhs) {
		return rhs + lhs;
	}

	template<typename ValueType>
	friend ValueType operator-(const AnyType& lhs, const ValueType& rhs) {
		return boost::any_cast<ValueType>(lhs.content) - rhs;
	}

	template<typename ValueType>
	friend ValueType operator-(const ValueType &lhs, const AnyType& rhs) {
		return rhs - lhs;
	}

	template<typename ValueType>
	friend ValueType operator*(const AnyType& lhs, const ValueType& rhs) {
		return boost::any_cast<ValueType>(lhs.content) * rhs;
	}

	template<typename ValueType>
	friend ValueType operator*(const ValueType &lhs, const AnyType& rhs) {
		return rhs * lhs;
	}

	template<typename ValueType>
	friend ValueType operator/(const AnyType& lhs, const ValueType& rhs) {
		return boost::any_cast<ValueType>(lhs.content) / rhs;
	}

	template<typename ValueType>
	friend ValueType operator/(const ValueType &lhs, const AnyType& rhs) {
		return rhs / lhs;
	}

//========================================================================
// Unary arithmetic operators
//========================================================================
	AnyType& operator+=(const char* rhs) {
		std::string temp = boost::any_cast<std::string>(content);
		temp += rhs;
		content = temp;
		return *this;
	}

	template<typename ValueType>
	AnyType& operator+=(const ValueType& rhs) {
		ValueType temp = boost::any_cast<ValueType>(content);
		temp += rhs;
		content = temp;
		return *this;
	}

	template<typename ValueType>
	AnyType& operator-=(const ValueType& rhs) {
		ValueType temp = boost::any_cast<ValueType>(content);
		temp -= rhs;
		content = temp;
		return *this;
	}

	template<typename ValueType>
	AnyType& operator*=(const ValueType &rhs) {
		ValueType temp = boost::any_cast<ValueType>(content);
		temp *= rhs;
		content = temp;
		return *this;
	}

	template<typename ValueType>
	AnyType& operator/=(const ValueType& rhs) {
		ValueType temp = boost::any_cast<ValueType>(content);
		temp /= rhs;
		content = temp;
		return *this;
	}

private:
	boost::any content;
};

//========================================================================
// Typedefs for convenience usage
//========================================================================

/**
 * This contains a single category with a specific sub category
 * first : the name of the category
 * second: the name of the sub category
 */
typedef std::pair<std::string, std::string> Category;

/**
 *  This contains a list of categories with specific sub category
 */
typedef std::map<std::string, AnyType> Properties;

/**
 * This contains a single sub category with results
 * first : the name of the sub category
 * second: a pair of integers, which contains:
 * 	first : the amount of correct images in this sub category
 * 	second: the total amount of images in this sub category
 */
typedef std::pair<std::string, std::pair<int, int> > SubCategoryResults;

/**
 * This contains a single category with sub categories and their results
 * first : the name of the category
 * second: a list of sub categories with their results
 */
typedef std::pair<std::string, std::map<std::string, std::pair<int, int> > > CategoryResults;

/**
 * This contains a list of categories with sub categories and their results
 */
typedef std::map<std::string, std::map<std::string, std::pair<int, int> > > CategoriesResults;

//========================================================================
// ImageMD class (holds metadata of an image)
//========================================================================

/**
 * The ImageMD class holds metadata of an image
 */
class ImageMD {
public:
	/**
	 * Constructor that takes the path of the image
	 * @param path the path of the image
	 */
	ImageMD(std::string path, std::string name) :
		path(path), name(name)
	{
	}

	std::string path;												///< The path to the image
	std::string name;												///< Image name
	std::map<std::string, std::string> categories;		///< The categories this image is in
	Properties properties;										///< The properties of the entire image
	std::vector<Properties> objects;							///< A list of objects with their properties
};

}
