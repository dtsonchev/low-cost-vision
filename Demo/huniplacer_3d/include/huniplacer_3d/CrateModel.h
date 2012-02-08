#pragma once

#include <huniplacer/huniplacer.h>

#include <huniplacer_3d/RobotModel.h>
#include <huniplacer_3d/defines.h>

using namespace huniplacer;

class RobotModel;

class CrateModel {
public:
	CrateModel(RobotModel* model,
			point3 size,
			point3 location = point3(0, GROUND_PANE, 0),
			double rotation = 0);

	~CrateModel();

	void setLocation(point3& loc);
	void setRotation(double rotation);
	void setAlpha(float alpha);

	double getRotation();
	point3 getLocation();
	point3 getSize();
	float getAlpha();

	void setNrBallsWidth(int rows);
	void setNrBallsDepth(int cols);

	int getNrBallsWidth();
	int getNrBallsDepth();
	bool* balletjes;
private:
	RobotModel* m_Model;
	point3 m_Location;
	point3 m_Size;
	float m_Alpha;
	double m_Rotation;
	int nr_balls_width;
	int nr_balls_depth;
};
