#include <cstdio>
#include <stdexcept>
#include <huniplacer/huniplacer.h>

using namespace huniplacer;

static void modbus_exhandler(std::runtime_error& ex)
{
	printf(
		"runtime error in modbus controller\n"
		"what(): %s\n", ex.what());
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

	steppermotor3 motors(modbus_rtu, -45, 75, modbus_exhandler);

	deltarobot robot(kinematics, motors);

#define M(x, y) robot.moveto(point3(x, y, -100), 360)
	M(-20, -20);
	M(-20, 20);
	M(20, -20);
	M(20, 20);
#undef M

	return 0;
}
