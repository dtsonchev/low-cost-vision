#include <huniplacer_3d/Render.h>

#include <stdio.h>
#include <time.h>

#include <GL/freeglut.h>

#include <huniplacer_3d/huniplacer_3d.h>
#include <huniplacer_3d/Keyhandlers.h>

float yRotation = 0;
float xRotation = 0;
int screen_width = 1;
int screen_height = 1;
struct timeval new_timeval, previous_timeval;

GLuint cubelist;

void drawCube(float x, float y, float z, float width, float height,
		float depth) {
	glBegin(GL_QUADS); // Draw A Quad

	glVertex3f(x + (0.5f * width), y + (0.5f * height), z + (-0.5f * depth)); // Top Right Of The Quad (Top)
	glVertex3f(x + (-0.5f * width), y + (0.5f * height), z + (-0.5f * depth)); // Top Left Of The Quad (Top)
	glVertex3f(x + (-0.5f * width), y + (0.5f * height), z + (0.5f * depth)); // Bottom Left Of The Quad (Top)
	glVertex3f(x + (0.5f * width), y + (0.5f * height), z + (0.5f * depth)); // Bottom Right Of The Quad (Top)

	glVertex3f(x + (0.5f * width), y + (-0.5f * height), z + (0.5f * depth)); // Top Right Of The Quad (Bottom)
	glVertex3f(x + (-0.5f * width), y + (-0.5f * height), z + (0.5f * depth)); // Top Left Of The Quad (Bottom)
	glVertex3f(x + (-0.5f * width), y + (-0.5f * height), z + (-0.5f * depth)); // Bottom Left Of The Quad (Bottom)
	glVertex3f(x + (0.5f * width), y + (-0.5f * height), z + (-0.5f * depth)); // Bottom Right Of The Quad (Bottom)

	glVertex3f(x + (0.5f * width), y + (0.5f * height), z + (0.5f * depth)); // Top Right Of The Quad (Front)
	glVertex3f(x + (-0.5f * width), y + (0.5f * height), z + (0.5f * depth)); // Top Left Of The Quad (Front)
	glVertex3f(x + (-0.5f * width), y + (-0.5f * height), z + (0.5f * depth)); // Bottom Left Of The Quad (Front)
	glVertex3f(x + (0.5f * width), y + (-0.5f * height), z + (0.5f * depth)); // Bottom Right Of The Quad (Front)

	glVertex3f(x + (0.5f * width), y + (-0.5f * height), z + (-0.5f * depth)); // Top Right Of The Quad (Back)
	glVertex3f(x + (-0.5f * width), y + (-0.5f * height), z + (-0.5f * depth)); // Top Left Of The Quad (Back)
	glVertex3f(x + (-0.5f * width), y + (0.5f * height), z + (-0.5f * depth)); // Bottom Left Of The Quad (Back)
	glVertex3f(x + (0.5f * width), y + (0.5f * height), z + (-0.5f * depth)); // Bottom Right Of The Quad (Back)

	glVertex3f(x + (-0.5f * width), y + (0.5f * height), z + (0.5f * depth)); // Top Right Of The Quad (Left)
	glVertex3f(x + (-0.5f * width), y + (0.5f * height), z + (-0.5f * depth)); // Top Left Of The Quad (Left)
	glVertex3f(x + (-0.5f * width), y + (-0.5f * height), z + (-0.5f * depth)); // Bottom Left Of The Quad (Left)
	glVertex3f(x + (-0.5f * width), y + (-0.5f * height), z + (0.5f * depth)); // Bottom Right Of The Quad (Left)

	glVertex3f(x + (0.5f * width), y + (0.5f * height), z + (-0.5f * depth)); // Top Right Of The Quad (Right)
	glVertex3f(x + (0.5f * width), y + (0.5f * height), z + (0.5f * depth)); // Top Left Of The Quad (Right)
	glVertex3f(x + (0.5f * width), y + (-0.5f * height), z + (0.5f * depth)); // Bottom Left Of The Quad (Right)
	glVertex3f(x + (0.5f * width), y + (-0.5f * height), z + (-0.5f * depth)); // Bottom Right Of The Quad (Right)
	glEnd(); // Done Drawing The Quad
}


void render() {
	glClearColor(0.0, 0.0, 0.0, 1);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();

	float mtime, seconds, useconds;
	gettimeofday(&new_timeval, NULL);

	seconds  = new_timeval.tv_sec  - previous_timeval.tv_sec;
	useconds = new_timeval.tv_usec - previous_timeval.tv_usec;

	mtime = seconds + useconds/1000000;

	getHuniplacerData()->modeldata.deltatime = mtime;

	float dif = mtime * 40;

	previous_timeval = new_timeval;

	if (getKeyData()->key_left)
		yRotation += dif;
	if (getKeyData()->key_right)
		yRotation -= dif;

	if (getKeyData()->key_plus)
		getHuniplacerData()->modeldata.robot->setSpeed(getHuniplacerData()->modeldata.robot->getSpeed() + dif*3);
	if (getKeyData()->key_min)
		getHuniplacerData()->modeldata.robot->setSpeed(getHuniplacerData()->modeldata.robot->getSpeed() - dif*3);

	if (getKeyData()->key_up)
		xRotation += dif;
	if (getKeyData()->key_down)
		xRotation -= dif;

	if (getKeyData()->key_w)
		getHuniplacerData()->modeldata.pivot.y += dif;
	if (getKeyData()->key_s)
		getHuniplacerData()->modeldata.pivot.y -= dif;

	if (getKeyData()->key_d)
		getHuniplacerData()->modeldata.pivot.x += dif;
	if (getKeyData()->key_a)
		getHuniplacerData()->modeldata.pivot.x -= dif;

	if (getKeyData()->key_q)
		getHuniplacerData()->modeldata.pivot.z += dif;
	if (getKeyData()->key_e)
		getHuniplacerData()->modeldata.pivot.z -= dif;

	if (getHuniplacerData()->modeldata.pivot.x < huniplacer::measures::MIN_X)
		getHuniplacerData()->modeldata.pivot.x = huniplacer::measures::MIN_X;
	if (getHuniplacerData()->modeldata.pivot.x > huniplacer::measures::MAX_X)
		getHuniplacerData()->modeldata.pivot.x = huniplacer::measures::MAX_X;

	if (getHuniplacerData()->modeldata.pivot.y < huniplacer::measures::MIN_Y)
		getHuniplacerData()->modeldata.pivot.y = huniplacer::measures::MIN_Y;
	if (getHuniplacerData()->modeldata.pivot.y > huniplacer::measures::MAX_Y)
		getHuniplacerData()->modeldata.pivot.y = huniplacer::measures::MAX_Y;

	if (getHuniplacerData()->modeldata.pivot.z < huniplacer::measures::MIN_Z)
		getHuniplacerData()->modeldata.pivot.z = huniplacer::measures::MIN_Z;
	if (getHuniplacerData()->modeldata.pivot.z > huniplacer::measures::MAX_Z)
		getHuniplacerData()->modeldata.pivot.z = huniplacer::measures::MAX_Z;

	if(getHuniplacerData()->modeldata.robot->getSpeed() < 20)
		getHuniplacerData()->modeldata.robot->setSpeed(20);
	if(getHuniplacerData()->modeldata.robot->getSpeed() > 500)
		getHuniplacerData()->modeldata.robot->setSpeed(500);

	glRotatef(xRotation, 1, 0, 0);
	glRotatef(yRotation, 0, 1, 0);

	glColor3f(1.0f, 0, 0);
	glPushMatrix();
	glTranslatef(getHuniplacerData()->modeldata.pivot.x + 0.5, getHuniplacerData()->modeldata.pivot.z, getHuniplacerData()->modeldata.pivot.y);
	drawCube(0, 0, 0, 5, 1, 1);
	glPopMatrix();

	glPushMatrix();
	glColor3f(0, 1, 0);
	glTranslatef(getHuniplacerData()->modeldata.pivot.x, getHuniplacerData()->modeldata.pivot.z + 0.5, getHuniplacerData()->modeldata.pivot.y);
	drawCube(0, 0, 0, 1, 5, 1);
	glPopMatrix();

	glPushMatrix();
	glColor3f(0, 0, 1);
	glTranslatef(getHuniplacerData()->modeldata.pivot.x, getHuniplacerData()->modeldata.pivot.z, getHuniplacerData()->modeldata.pivot.y + 0.5);
	drawCube(0, 0, 0, 1, 1, 5);
	glPopMatrix();

	getHuniplacerData()->modeldata.robot->draw();

	glColor4f(0.3, 0.3, 0.3, 0.1);

	glPushMatrix();
	glDepthMask(GL_FALSE);
	glCallList(cubelist);
	glDepthMask(GL_TRUE);
	glPopMatrix();

	glLoadIdentity();
	char str[256];

	glDisable(GL_DEPTH_TEST);
	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	glRasterPos2i(-(screen_width / 5), 65);
	sprintf(str, "pivot location x:%g", getHuniplacerData()->modeldata.pivot.x);
	glutBitmapString(GLUT_BITMAP_HELVETICA_10, (unsigned char*) str);
	glRasterPos2i(-(screen_width / 5), 55);
	sprintf(str, "pivot location y:%g", getHuniplacerData()->modeldata.pivot.y);
	glutBitmapString(GLUT_BITMAP_HELVETICA_10, (unsigned char*) str);
	glRasterPos2i(-(screen_width / 5), 45);
	sprintf(str, "pivot location z:%g", getHuniplacerData()->modeldata.pivot.z);
	glutBitmapString(GLUT_BITMAP_HELVETICA_10, (unsigned char*) str);

	float rx, ry, rz;
	getHuniplacerData()->modeldata.robot->getPosition(&rx, &ry, &rz);
	glRasterPos2i(-(screen_width / 5), 35);
	sprintf(str, "robot location x:%g", rx);
	glutBitmapString(GLUT_BITMAP_HELVETICA_10, (unsigned char*) str);
	glRasterPos2i(-(screen_width / 5), 25);
	sprintf(str, "robot location y:%g", ry);
	glutBitmapString(GLUT_BITMAP_HELVETICA_10, (unsigned char*) str);
	glRasterPos2i(-(screen_width / 5), 15);
	sprintf(str, "robot location z:%g", rz);
	glutBitmapString(GLUT_BITMAP_HELVETICA_10, (unsigned char*) str);
	glRasterPos2i(-(screen_width / 5), 5);
	sprintf(str, "speed:%g", getHuniplacerData()->modeldata.robot->getSpeed());
	glutBitmapString(GLUT_BITMAP_HELVETICA_10, (unsigned char*) str);

	glEnable(GL_DEPTH_TEST);
	glutSwapBuffers();

	ros::spinOnce();
	if(!ros::ok()){
		glutDestroyWindow(glutGetWindow());
	}
}

void resize(int width, int height) {
	screen_width = width;
	screen_height = height;
	if (height == 0)
		height = 1;
	float ratio = width * 1.0 / height;

	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0f, -ratio, 0.1f, 9000.0f);
	gluLookAt(0, huniplacer::measures::MAX_Z, -300, 0,
			huniplacer::measures::MAX_Z, 0, 0, 1, 0);

	glEnable(GL_MULTISAMPLE);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glShadeModel(GL_SMOOTH);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if (cubelist == 0) {
		cubelist = glGenLists(1);
		glNewList(cubelist, GL_COMPILE);

		const bool* bitmap = getHuniplacerData()->modeldata.eb->get_bitmap();
		width = getHuniplacerData()->modeldata.eb->get_width();
		height = getHuniplacerData()->modeldata.eb->get_height();
		int depth = getHuniplacerData()->modeldata.eb->get_depth();

		std::vector<Face> faces = getHuniplacerData()->modeldata.range.get_faces();

		for (unsigned int i = 0; i < faces.size(); i++) {
			glBegin(GL_LINE_STRIP); // Draw A Triangle
			glNormal3f(
					(faces.at(i).a.x - (width / 2)) * getHuniplacerData()->modeldata.eb->get_voxel_size(),
					(faces.at(i).a.z) * getHuniplacerData()->modeldata.eb->get_voxel_size()
							+ huniplacer::measures::MIN_Z,
					(faces.at(i).a.y - (depth / 2)) * getHuniplacerData()->modeldata.eb->get_voxel_size());
			glVertex3f(
					(faces.at(i).a.x - (width / 2)) * getHuniplacerData()->modeldata.eb->get_voxel_size(),
					(faces.at(i).a.z) * getHuniplacerData()->modeldata.eb->get_voxel_size()
							+ huniplacer::measures::MIN_Z,
					(faces.at(i).a.y - (depth / 2)) * getHuniplacerData()->modeldata.eb->get_voxel_size()); // Top Left
			glVertex3f(
					(faces.at(i).b.x - (width / 2)) * getHuniplacerData()->modeldata.eb->get_voxel_size(),
					(faces.at(i).b.z) * getHuniplacerData()->modeldata.eb->get_voxel_size()
							+ huniplacer::measures::MIN_Z,
					(faces.at(i).b.y - (depth / 2)) * getHuniplacerData()->modeldata.eb->get_voxel_size()); // Top Right
			glVertex3f(
					(faces.at(i).c.x - (width / 2)) * getHuniplacerData()->modeldata.eb->get_voxel_size(),
					(faces.at(i).c.z) * getHuniplacerData()->modeldata.eb->get_voxel_size()
							+ huniplacer::measures::MIN_Z,
					(faces.at(i).c.y - (depth / 2)) * getHuniplacerData()->modeldata.eb->get_voxel_size()); // Bottom Right
			glEnd();
		}

		glEndList();
	}
}
