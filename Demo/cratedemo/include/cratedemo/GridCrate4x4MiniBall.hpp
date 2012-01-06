#pragma once
#include <cratedemo/GridCrate.hpp>

namespace cratedemo {
class GridCrate4x4MiniBall : public GridCrate {
private:
	static const float CRATE_WIDTH;
	static const float CRATE_DEPTH;
	static const float CRATE_HEIGHT;
	static const int MAX_NUMBER_OF_OBJECTS;
	static const int GRID_WIDTH = 4;
	static const int GRID_HEIGHT = 4;
	static const float DISTANCE_TO_SIDE;
	static const float DISTANCE_TO_NEXT;
	static const float RADIUS_OF_BALL_CONTAINER;
	static const float BOTTOM_THICKNESS;

public:
	GridCrate4x4MiniBall(
			std::string name,
			datatypes::point2f position,
			float angle) : GridCrate(	name,
										position,
										angle,
										datatypes::size3f(CRATE_WIDTH,CRATE_DEPTH,CRATE_HEIGHT),
										MAX_NUMBER_OF_OBJECTS,
										GRID_WIDTH,
										GRID_HEIGHT,
										DISTANCE_TO_SIDE,
										DISTANCE_TO_NEXT,
										RADIUS_OF_BALL_CONTAINER,
										BOTTOM_THICKNESS) {}
};

}

/*
 * GridCrate(
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


 */
