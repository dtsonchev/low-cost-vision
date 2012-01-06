#include <cratedemo/GridCrate.hpp>
#include <cratedemo/Crate.hpp>
#include <cratedemo/CrateExceptions.hpp>
#include <datatypes/point2.hpp>
#include <datatypes/point3.hpp>
#include <datatypes/size3.hpp>
#include <string>

namespace cratedemo {

GridCrate::GridCrate(std::string name, datatypes::point2f position, float angle, datatypes::size3f size, size_t maxNumberOfObjects, size_t gridWidth, size_t gridHeight, double distanceToSide, double distanceToNext, double radiusOfBallContainer, double bottomThickness) :
		Crate(name, position, angle, size, maxNumberOfObjects) {

	this->gridWidth = gridWidth;
	this->gridHeight = gridHeight;
	this->distanceToSide = distanceToSide;
	this->distanceToNext = distanceToNext;
	this->bottomThickness = bottomThickness;
	this->radiusOfBallContainer = radiusOfBallContainer;
}

datatypes::point3f GridCrate::getCrateContentGripLocation(size_t index) {
	using datatypes::point2f;
	using datatypes::point3f;

	if (data[index] == NULL) {
		throw cratedemo::LocationIsEmptyException();
	}

	point2f location2D;
	location2D.x = -(size.width / 2) + (index % gridWidth) * (2 * radiusOfBallContainer + distanceToNext) + distanceToSide + radiusOfBallContainer;
	location2D.y = -(size.depth / 2) + (index / gridHeight) * (2 * radiusOfBallContainer + distanceToNext) + distanceToSide + radiusOfBallContainer;
	location2D.rotate(angle);
	point3f location3D(location2D.x, location2D.y, bottomThickness);
	location3D += data[index]->getGripPoint();

	return location3D;
}
}
