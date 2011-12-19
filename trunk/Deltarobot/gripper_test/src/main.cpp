#include <iostream>
#include <gripper/gripper.h>

using namespace std;

int main(void)
{
	gripper grip("192.168.0.2", 502);
	grip.connect();
	grip.disconnect();
	grip.connect();
	grip.grap();
	usleep(5*1000*1000);
	grip.release();
	return 0;
}


