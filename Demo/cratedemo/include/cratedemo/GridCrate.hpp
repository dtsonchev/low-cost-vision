#pragma once
#include <cratedemo/Crate.hpp>
#include <string>
namespace cratedemo {
class GridCrate : public Crate {
public:
	GridCrate(
		std::string name,
		datatypes::point2f position,
		float angle,
		datatypes::size3f size,
		size_t maxNumberOfObjects,
		size_t gridWidth,
		size_t gridHeight,
		double distanceToSide,
		double distanceToNext,
		double radiusOfBallContainer,
		double bottomThickness);
	datatypes::point3f getCrateContentGripLocation(size_t index);

private:
	size_t gridWidth;
	size_t gridHeight;
	double distanceToSide;
	double distanceToNext;
	double bottomThickness;
	double radiusOfBallContainer;
};
}
