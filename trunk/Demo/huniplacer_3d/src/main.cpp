#include <GL/freeglut.h>

#include <ros/ros.h>

#include <huniplacer_3d/huniplacer_3d.h>

#include <huniplacer_3d/Callbacks.h>
#include <huniplacer_3d/DummyMotor.h>
#include <huniplacer_3d/Keyhandlers.h>
#include <huniplacer_3d/Render.h>

huniplacerData data;

huniplacerData* getHuniplacerData(){
	return &data;
}

void initRos(int argc, char** argv){
	ros::init(argc, argv, "hunplacer_3d");
	data.rosdata.node = new ros::NodeHandle();
	data.rosdata.moveToSrv = data.rosdata.node->serviceClient<deltarobotnode::motionSrv>("moveTo");
	data.rosdata.motionReceiver = data.rosdata.node->subscribe("dummy", 1000, moveCallback);
	data.rosdata.crateReceiver = data.rosdata.node->subscribe("crateEvent", 1000, crateCallback);	
	data.rosdata.positionReceiver = data.rosdata.node->subscribe("pubDeltaPos", 100, positionCallback);
	data.rosdata.topviewReceiver = data.rosdata.node->subscribe("topview_info", 1000, topviewCallback);
	ROS_INFO("Huniplacer 3d viewer listening");
}

void initModel(){
	data.modeldata.ikmodel = new huniplacer::inverse_kinematics_impl(
			huniplacer::measures::BASE, huniplacer::measures::HIP,
			huniplacer::measures::EFFECTOR, huniplacer::measures::ANKLE,
			huniplacer::measures::HIP_ANKLE_ANGLE_MAX);

	data.modeldata.motor = new DummyMotor();

	data.modeldata.eb = effector_boundaries::generate_effector_boundaries(*data.modeldata.ikmodel, *data.modeldata.motor,
			3.0);
	data.modeldata.robot = new RobotModel(data.modeldata.ikmodel, data.modeldata.eb, data.modeldata.motor);

	data.modeldata.range.init(data.modeldata.eb);
}

void initWindow(int argc, char** argv){
	glutInit(&argc, argv);
	//glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_MULTISAMPLE);
	glutInitDisplayString("rgba double depth=>12 samples>=8");
	glutInitWindowPosition(565, 0);
	glutInitWindowSize(500, 500);

	glutCreateWindow("HUniplacer 3D");

	// register callbacks
	glutSpecialFunc(specialHandler);
	glutSpecialUpFunc(specialUpHandler);
	glutKeyboardFunc(keyHandler);
	glutKeyboardUpFunc(keyUpHandler);
	glutDisplayFunc(render);
	glutReshapeFunc(resize);
	glutIdleFunc(render);
}

int main(int argc, char** argv) {
	initRos(argc, argv);
	initModel();
	initWindow(argc, argv);

	glutMainLoop();

	return 0;
}
