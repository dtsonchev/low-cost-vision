#pragma once

struct KeyData{
	bool key_w, key_s, key_a, key_d, key_q, key_e, key_left, key_up, key_down, key_right, key_min, key_plus;
};

KeyData* getKeyData();

void keyHandler(unsigned char key, int x, int y);
void keyUpHandler(unsigned char key, int x, int y);
void specialHandler(int key, int x, int y);
void specialUpHandler(int key, int x, int y);
