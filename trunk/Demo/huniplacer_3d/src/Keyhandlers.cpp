#include <huniplacer_3d/Keyhandlers.h>
#include <huniplacer_3d/huniplacer_3d.h>

#include <GL/freeglut.h>

KeyData keys;

KeyData* getKeyData(){
	return &keys;
}

void keyHandler(unsigned char key, int x, int y) {
	switch (key) {
	case 'w':
		keys.key_w = true;
		break;
	case 's':
		keys.key_s = true;
		break;
	case 'a':
		keys.key_a = true;
		break;
	case 'd':
		keys.key_d = true;
		break;
	case 'q':
		keys.key_q = true;
		break;
	case 'e':
		keys.key_e = true;
		break;
	case '+':
		keys.key_plus = true;
		break;
	case '-':
		keys.key_min = true;
		break;
	case '\r':
		getHuniplacerData()->modeldata.robot->move_actual_robot();
		break;
	}
}

void keyUpHandler(unsigned char key, int x, int y) {
	switch (key) {
	case 'w':
		keys.key_w = false;
		break;
	case 's':
		keys.key_s = false;
		break;
	case 'a':
		keys.key_a = false;
		break;
	case 'd':
		keys.key_d = false;
		break;
	case 'q':
		keys.key_q = false;
		break;
	case 'e':
		keys.key_e = false;
		break;
	case '+':
		keys.key_plus = false;
		break;
	case '-':
		keys.key_min = false;
		break;
	}
}

void specialHandler(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_LEFT:
		keys.key_left = true;
		break;
	case GLUT_KEY_RIGHT:
		keys.key_right = true;
		break;
	case GLUT_KEY_UP:
		keys.key_up = true;
		break;
	case GLUT_KEY_DOWN:
		keys.key_down = true;
		break;
	case GLUT_KEY_END:
		exit(1);
		break;
	case GLUT_KEY_F11:
		glutFullScreenToggle();
		break;
	}
}

void specialUpHandler(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_LEFT:
		keys.key_left = false;
		break;
	case GLUT_KEY_RIGHT:
		keys.key_right = false;
		break;
	case GLUT_KEY_UP:
		keys.key_up = false;
		break;
	case GLUT_KEY_DOWN:
		keys.key_down = false;
		break;
	}
}
