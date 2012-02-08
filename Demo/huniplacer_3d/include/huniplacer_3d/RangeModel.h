#pragma once

#include <GL/gl.h>

#include <huniplacer/huniplacer.h>

#include "Face.h"

using namespace huniplacer;

class RangeModel {
public:
	RangeModel(){}
	virtual ~RangeModel(){}

	void init(effector_boundaries *eb);
	std::vector<Face> get_faces();

private:
	std::vector<Face> faces;
	void add_face(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz);
	void add_quad(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz, float dx, float dy, float dz);
};
