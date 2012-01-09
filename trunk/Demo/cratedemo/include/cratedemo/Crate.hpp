#pragma once
#include <string.h>
#include <string>
#include <cratedemo/CrateContent.hpp>
#include <datatypes/point2.hpp>
#include <datatypes/point3.hpp>
#include <datatypes/size3.hpp>

namespace cratedemo {
class Crate {

public:
	virtual ~Crate();
	/**
	 * Puts a CrateContent* in the list of containing items on location index.
	 * @param index Location of the to be inputed CrateContent*.
	 * @param crateContent Pointer of the to be inputed CrateContent.
	 */
	void put(size_t index, CrateContent* crateContent);
	/**
	 *
	 */
	CrateContent* get(size_t index) const;
	/**
	 * Returns the location of the object on index.
	 * @param index
	 * @return the location of the object on index.
	 */
	virtual datatypes::point3f getCrateContentLocation(size_t index) const = 0;
	/**
	 * Remove the content on location index.
	 * @param index
	 */
	void remove(size_t index);
	/**
	 * Returns if the Crate is empty.
	 * @return
	 */
	bool isEmpty() const;
protected:
	Crate(std::string name, datatypes::point2f position, float angle, datatypes::size3f size, size_t maxNumberOfObjects);

	std::string name;
	datatypes::point2f position;
	float angle;
	datatypes::size3f size;
	size_t maxNumberOfObjects;
	CrateContent** data;
};
}
