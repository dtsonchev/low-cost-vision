#include "huniplacer_3d/Kabouter.h"
#include "GL/glut.h"

Kabouter::Kabouter(){
	x = 40;
	y = 0;
	z = 0;
}

void Kabouter::DrawSelf(){
	glBegin(GL_QUADS);
		glVertex3f(x +  3.5f , y +  3.5f , z +  3.5f ); // Top Right Of The Quad (Bottom)
		glVertex3f(x +  3.5f , y		 , z +  3.5f ); // Top Left Of The Quad (Bottom)
		glVertex3f(x +  3.5f , y		 , z 		 ); // Bottom Left Of The Quad (Bottom)
		glVertex3f(x +  3.5f , y +  3.5f , z		 ); // Bottom Right Of The Quad (Bottom)
	glEnd();
}



Kabouter::~Kabouter() {

}

