#pragma once

#include <huniplacer/huniplacer.h>

#include <huniplacer_3d/CrateModel.h>
#include <huniplacer_3d/defines.h>

using namespace huniplacer;

class CrateModel;

class RobotModel {
public:

	RobotModel(inverse_kinematics_model* ikm,
			effector_boundaries* boundaries, imotor3* motor);
	void setPosition(int x, int y, int z);
	void getPosition(float*x, float*y, float*z);
	float getSpeed();
	void setSpeed(float speed);

	void moveTo(int x, int y, int z);

	void draw();

	void addCrate(CrateModel* crate);
	void removeCrate(CrateModel* crate);

	void setNewAnimation(std::vector<double> x,std::vector<double> y,std::vector<double> z,std::vector<double> speed);

	bool move_actual_robot();
private:
	void updateAnimation();
	point3 updateAnimationIteration(int it, double distance, double time);
	bool is_valid_angle(double angle);
	void drawMotor();
	void drawHip(float angle);
	void drawAnkle(float angle, float hipAngle);
	void drawEffector();
	void drawCrate(CrateModel* crate);
	void drawCube(float x, float y, float z, float width, float height,
			float depth);
	inverse_kinematics_model* ikm;
	effector_boundaries* boundaries;
	imotor3* motor;

	std::list<CrateModel*> m_Crates;

	float posX, posY, posZ;
	float speed;

	std::vector<double> anim_x;
	std::vector<double> anim_y;
	std::vector<double> anim_z;
	std::vector<double> anim_speed;
	float anim_time;
};
