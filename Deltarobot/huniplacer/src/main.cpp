#include <cstdio>
#include <stdexcept>
#include <iostream>
#include <huniplacer/huniplacer.h>
#include <modbus/modbus.h>

using namespace huniplacer;

static void modbus_exhandler(std::runtime_error& ex)
{
	printf(
		"runtime error of type '%s' in modbus controller\n"
		"what(): %s\n", typeid(ex).name(), ex.what());
}

int main(int argc, char** argv)
{
	inverse_kinematics_impl kinematics(
		measures::BASE,
		measures::HIP,
		measures::EFFECTOR,
		measures::ANKLE,
		measures::HIP_ANKLE_ANGLE_MAX);

	modbus_t* modbus_rtu = modbus_new_rtu(
		"/dev/ttyS0",
		crd514_kd::rtu_config::BAUDRATE,
		crd514_kd::rtu_config::PARITY,
		crd514_kd::rtu_config::DATA_BITS,
		crd514_kd::rtu_config::STOP_BITS);

	steppermotor3 motors(modbus_rtu, utils::rad(-45), utils::rad(75), modbus_exhandler);
	//steppermotor3 motors(modbus_rtu, utils::rad(-45), utils::rad(95), modbus_exhandler); //TODO: WATCH OUT, MAX ANGLES ARE WRONG
	/*motors.power_on();

	motionf m;
	m.deceleration[0] = m.deceleration[1] = m.deceleration[2] =
			m.acceleration[0] = m.acceleration[1] = m.acceleration[2] = utils::rad(3600);
	m.angles[0] = utils::rad(90);
	m.angles[1] = utils::rad(45);
	m.angles[2] = utils::rad(10);
	m.speed[0] = m.speed[1] = m.speed[2] = utils::rad(90);

	motors.moveto_within(m, 5, false);

	*/
	deltarobot robot(kinematics, motors);
	robot.power_on();

	robot.moveto(point3(0, 0, -160), 20);
	getchar();
	robot.moveto(point3(30, -30, -160), 20);
	getchar();
	robot.moveto(point3(30, 30, -160), 20);
	getchar();

    robot.wait_for_idle();

	return 0;
}
