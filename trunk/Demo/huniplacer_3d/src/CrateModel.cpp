#include <huniplacer_3d/CrateModel.h>

CrateModel::CrateModel(
		RobotModel* model,
		point3 size,
		point3 location,
		double rotation) :
		m_Model(model), m_Location(location),m_Size(size), m_Rotation(rotation), nr_balls_depth(0), nr_balls_width(0){
	m_Model->addCrate(this);
}

CrateModel::~CrateModel() {
	m_Model->removeCrate(this);
}

double CrateModel::getRotation(){
	return m_Rotation;
}

int CrateModel::getNrBallsWidth(){
	return nr_balls_width;
}

int CrateModel::getNrBallsDepth(){
	return nr_balls_depth;
}

void CrateModel::setNrBallsWidth(int rows){
	nr_balls_width = rows;
}

void CrateModel::setNrBallsDepth(int cols){
	nr_balls_depth = cols;
}

point3 CrateModel::getLocation(){
	return m_Location;
}

point3 CrateModel::getSize(){
	return m_Size;
}

void CrateModel::setLocation(point3& loc){
	m_Location = loc;
}

void CrateModel::setRotation(double rot){
	m_Rotation = rot;
}

void CrateModel::setAlpha(float alpha){
	m_Alpha = alpha;
}

float CrateModel::getAlpha(){
	return m_Alpha;
}
