#ifndef FACE_H_
#define FACE_H_

#include <huniplacer/point3.h>

class Face{
public:
	huniplacer::point3 a, b, c;

	Face(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz):
		a(ax,ay,az),
		b(bx,by,bz),
		c(cx,cy,cz)
	{}

private:


};

#endif /* FACE_H_ */
