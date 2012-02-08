/*
 * RobotModel.cpp
 *
 *  Created on: 19 jan. 2012
 *      Author: joris
 */

#include <GL/gl.h>

#include <huniplacer_3d/RobotModel.h>
#include <huniplacer_3d/CrateModel.h>
#include <huniplacer_3d/huniplacer_3d.h>

float test = 0.0f;

RobotModel::RobotModel(inverse_kinematics_model* ikm,
		effector_boundaries* boundaries, imotor3* motor) :
		ikm(ikm), boundaries(boundaries), motor(motor) {
	posX = 0;
	posY = 0;
	posZ = (huniplacer::measures::MIN_Z + huniplacer::measures::MAX_Z) / 2;
}

bool RobotModel::is_valid_angle(double angle) {
	return angle > motor->get_min_angle() && angle < motor->get_max_angle();
}

float RobotModel::getSpeed() {
	return speed;
}

void RobotModel::setSpeed(float speed) {
	this->speed = speed;
}

void RobotModel::setPosition(int x, int y, int z) {
	try {
		huniplacer::motionf m;
		ikm->point_to_motion(point3(x, y, z), m);

		if (boundaries->check_path(point3(posX, posY, posZ), point3(x, y, z))) {
			if (is_valid_angle(m.angles[0]) && is_valid_angle(m.angles[1])
					&& is_valid_angle(m.angles[2])) {
				posX = x;
				posY = y;
				posZ = z;
			}
		}

	} catch (std::exception& e) {

	}
}

void RobotModel::getPosition(float* x, float* y, float* z) {
	*x = posX;
	*y = posY;
	*z = posZ;
}

void RobotModel::addCrate(CrateModel* crate) {
	m_Crates.push_back(crate);
}
void RobotModel::removeCrate(CrateModel* crate) {
	m_Crates.remove(crate);
}

void RobotModel::draw() {

//Draw 3 motors

	motionf motion;
	point3 point3(posX, posY, posZ);

	ikm->point_to_motion(point3, motion);

	glPushMatrix();
	glRotatef(60, 0, 1, 0);
	glTranslatef(0, 0, huniplacer::measures::BASE);
	drawMotor();
	drawHip(motion.angles[1]);
	glPopMatrix();
	drawAnkle(60, motion.angles[1]);

	glPushMatrix();
	glRotatef(180, 0, 1, 0);
	glTranslatef(0, 0, huniplacer::measures::BASE);
	drawMotor();
	drawHip(motion.angles[0]);
	glPopMatrix();
	drawAnkle(180, motion.angles[0]);

	glPushMatrix();
	glRotatef(-60, 0, 1, 0);
	glTranslatef(0, 0, huniplacer::measures::BASE);
	drawMotor();
	drawHip(motion.angles[2]);
	glPopMatrix();
	drawAnkle(-60, motion.angles[2]);

	drawEffector();

	glBegin(GL_QUADS);
	glColor4f(0.3, 0.3, 0.3, 0.3);
	glVertex3f(-100, GROUND_PANE, -100);
	glVertex3f(100, GROUND_PANE, -100);
	glVertex3f(100, GROUND_PANE, 100);
	glVertex3f(-100, GROUND_PANE, 100);
	glEnd();

	std::list<CrateModel*>::iterator i;
	for (i = m_Crates.begin(); i != m_Crates.end(); i++) {
		drawCrate((*i));
	}

	updateAnimation();
}

void RobotModel::drawMotor() {
	glColor3f(0.8, 0.8, 0.8);
	drawCube(-15, 0, 0, 20, 20, 20);
	glColor3f(0.4, 0.2, 0.8);
	drawCube(0, 0, 0, 10, 5, 5);
}

void RobotModel::drawHip(float angle) {
	glPushMatrix();
	glColor3f(0, 0.6, 0);
	glRotatef(-huniplacer::utils::deg(angle), 1, 0, 0);
	glTranslatef(0, -huniplacer::measures::HIP / 2, 0);
	drawCube(0, 0, 0, 5, huniplacer::measures::HIP, 5);
	glTranslatef(0, -huniplacer::measures::HIP / 2, 0);
	glPopMatrix();
}

void RobotModel::drawAnkle(float angle, float hipAngle) {
	glColor3f(0, 0.4, 0);
//	float y_dif = sin(hipAngle) * huniplacer::measures::HIP;
//	float z_dif = sqrt(huniplacer::measures::HIP*huniplacer::measures::HIP - y_dif * y_dif);
//	float start_x = huniplacer::measures::BASE;
//	float start_y = y_dif + huniplacer::measures::BASE;
//	float start_z = z_dif;
//
	angle = huniplacer::utils::rad(angle);
//
//	float begin_y = cos(angle) * start_x - sin(angle) * start_y;
//	float begin_x = sin(angle) * start_x + cos(angle) * start_y;
//
//	float end_x = posX;
//	float end_y = posY;
//	float end_z = posZ;
//
//	drawCube(begin_x,-start_z, begin_y, 20,20,20);
//
//	glColor3f(1,0,0);
//	drawCube(end_x,end_z, end_y, 20,20,20);
//	glColor3f(1,0,1);

	float y_dif = sin(hipAngle) * huniplacer::measures::HIP;
	float z_dif = sqrt(
			huniplacer::measures::HIP * huniplacer::measures::HIP
					- y_dif * y_dif);

	float radius = (huniplacer::measures::BASE + y_dif);

	float start_x = sin(angle) * radius;
	float start_z = cos(angle) * radius;
	float start_y = -z_dif;

	float des_x = posX + sin(angle) * huniplacer::measures::EFFECTOR;
	float des_z = posY + cos(angle) * huniplacer::measures::EFFECTOR;
	float des_y = posZ;

	float diff_x = start_x - des_x;
	float diff_y = start_y - des_y;
	float diff_z = start_z - des_z;

	glBegin(GL_LINES);
	glVertex3f(start_x, start_y, start_z);
	glVertex3f(des_x, des_y, des_z);
	glEnd();
}

void RobotModel::drawEffector() {
	glColor3f(0.8, 0.2, 0);
	glBegin(GL_TRIANGLE_STRIP);
	for (int i = 0; i <= 365; i += 5) {
		float rad = huniplacer::utils::rad(i);
		if (i % 10 == 5) {
			glVertex3f(posX + sin(rad) * huniplacer::measures::EFFECTOR, posZ,
					posY + cos(rad) * huniplacer::measures::EFFECTOR);
		} else {
			glVertex3f(posX + sin(rad) * (huniplacer::measures::EFFECTOR - 10),
					posZ,
					posY + cos(rad) * (huniplacer::measures::EFFECTOR - 10));
		}
	}
	glEnd();
	glBegin(GL_TRIANGLE_STRIP);
	for (int i = 0; i <= 365; i += 5) {
		float rad = huniplacer::utils::rad(i);
		if (i % 10 == 5) {
			glVertex3f(posX + sin(rad) * 2, posZ - 5, posY + cos(rad) * 2);
		} else {
			glVertex3f(posX + sin(rad) * 2, posZ + 5, posY + cos(rad) * 2);
		}
	}
	glEnd();
}

void RobotModel::drawCrate(CrateModel* crate) {
	glPushMatrix();
	glColor4f(0.5, 0.5, 0.5, crate->getAlpha());
	glTranslatef(crate->getLocation().x, crate->getLocation().y,
			crate->getLocation().z);
	glRotatef(huniplacer::utils::deg(crate->getRotation()), 0, 1, 0);
	drawCube(0, 0, 0, crate->getSize().x, crate->getSize().y,
			crate->getSize().z);
	if (crate->balletjes != NULL) {
		for (int x = 0; x < crate->getNrBallsWidth(); x++) {
			for (int y = 0; y < crate->getNrBallsDepth(); y++) {

				if (crate->balletjes[x + y * crate->getNrBallsWidth()])
					glColor3f(0, 1, 0);
				else
					glColor3f(1, 0, 0);

				glBegin(GL_QUADS);
				glVertex3f(
						-crate->getSize().x / 2
								+ (crate->getSize().x
										- crate->getSize().x
												/ crate->getNrBallsWidth() / 8)
										/ crate->getNrBallsWidth() * x
								+ crate->getSize().x / crate->getNrBallsWidth()
								- (crate->getSize().x / crate->getNrBallsWidth()
										/ 8),
						crate->getSize().y / 2 + 0.1,
						-crate->getSize().z / 2
								+ crate->getSize().z / crate->getNrBallsDepth()
										* y
								+ (crate->getSize().z / crate->getNrBallsDepth()
										/ 8));
				glVertex3f(
						-crate->getSize().x / 2
								+ (crate->getSize().x
										- crate->getSize().x
												/ crate->getNrBallsWidth() / 8)
										/ crate->getNrBallsWidth() * x
								+ (crate->getSize().x / crate->getNrBallsWidth()
										/ 8),
						crate->getSize().y / 2 + 0.1,
						-crate->getSize().z / 2
								+ crate->getSize().z / crate->getNrBallsDepth()
										* y
								+ (crate->getSize().z / crate->getNrBallsDepth()
										/ 8));
				glVertex3f(
						-crate->getSize().x / 2
								+ (crate->getSize().x
										- crate->getSize().x
												/ crate->getNrBallsWidth() / 8)
										/ crate->getNrBallsWidth() * x
								+ (crate->getSize().x / crate->getNrBallsWidth()
										/ 8),
						crate->getSize().y / 2 + 0.1,
						-crate->getSize().z / 2
								+ crate->getSize().z / crate->getNrBallsDepth()
										* y
								+ crate->getSize().z / crate->getNrBallsDepth()
								- (crate->getSize().z / crate->getNrBallsDepth()
										/ 8));
				glVertex3f(
						-crate->getSize().x / 2
								+ (crate->getSize().x
										- crate->getSize().x
												/ crate->getNrBallsWidth() / 8)
										/ crate->getNrBallsWidth() * x
								+ crate->getSize().x / crate->getNrBallsWidth()
								- (crate->getSize().x / crate->getNrBallsWidth()
										/ 8),
						crate->getSize().y / 2 + 0.1,
						-crate->getSize().z / 2
								+ crate->getSize().z / crate->getNrBallsDepth()
										* y
								+ crate->getSize().z / crate->getNrBallsDepth()
								- (crate->getSize().z / crate->getNrBallsDepth()
										/ 8));
				glEnd();
			}
		}
	}
	glPopMatrix();
}

void RobotModel::drawCube(float x, float y, float z, float width, float height,
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

bool RobotModel::move_actual_robot() {
	deltarobotnode::motionSrv srv;
	srv.request.motions.speed.push_back(speed);
	srv.request.motions.x.push_back(posX);
	srv.request.motions.y.push_back(posY);
	srv.request.motions.z.push_back(posZ);
	srv.request.motions.speed.push_back(speed);
	srv.request.motions.x.push_back(getHuniplacerData()->modeldata.pivot.x);
	srv.request.motions.y.push_back(getHuniplacerData()->modeldata.pivot.y);
	srv.request.motions.z.push_back(getHuniplacerData()->modeldata.pivot.z);
	if (getHuniplacerData()->rosdata.moveToSrv.call(srv)) {
//			ROS_INFO("Sum: %ld", srv.response.succeeded);
	} else {
//			ROS_ERROR("Failed to call service motion");
		return false;
	}
	return srv.response.succeeded;
}

void RobotModel::setNewAnimation(std::vector<double> x, std::vector<double> y,
		std::vector<double> z, std::vector<double> speed) {
	anim_speed = speed;
	anim_x = x;
	anim_y = y;
	anim_z = z;
	anim_time = 0;
}

void RobotModel::updateAnimation() {
	point3 pos = updateAnimationIteration(0, 0, anim_time);
	setPosition(pos.x, pos.y, pos.z);
	anim_time += getHuniplacerData()->modeldata.deltatime;
}

point3 RobotModel::updateAnimationIteration(int it, double distance,
		double time) {
	if (anim_x.size() > it + 1) {
		double dX = anim_x.at(it + 1) - anim_x.at(it);
		double dY = anim_y.at(it + 1) - anim_y.at(it);
		double dZ = anim_z.at(it + 1) - anim_z.at(it);

		double thisDistance = sqrt(pow(dX, 2) + pow(dY, 2) + pow(dZ, 2));
		double thisTime = thisDistance / anim_speed.at(it);
		double maxDistance = time * anim_speed.at(it);

		ROS_INFO("%f %f", maxDistance, thisDistance);

		if (time > thisTime) {
			//GO on with next iteration
			return updateAnimationIteration(it + 1, distance + thisDistance,
					time - thisTime);
		} else {
			double factor = maxDistance / thisDistance;
			double newX = anim_x.at(it) + dX * factor;
			double newY = anim_y.at(it) + dY * factor;
			double newZ = anim_z.at(it) + dZ * factor;
			return point3(newX, newY, newZ);
		}
	} else if (anim_x.size() > it) {
		//Probably the end
		return point3(anim_x.at(it), anim_y.at(it), anim_z.at(it));
	} else {
		return point3(0, 0, 0);
	}
}

