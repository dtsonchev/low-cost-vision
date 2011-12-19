#pragma once

#include <modbus/modbus.h>
#include <string>

/**
 * Software interface for gripper.
 */
class gripper
{
public:
	/**
	 * Constructor
	 * @param ip_adress the ip-adress of the IO unit.
	 * @param port the port number of the IO unit.
	 */
	gripper(const std::string & ip_adress, const int port);
	/**
	 * Destructor
	 */
	~gripper();

	/**
	 * Connects to the IO unit
	 */
	void connect();
	/**
	 * Disconnects to the IO unit
	 */
	void disconnect();

	/**
	 * Switches the gripper on
	 */
	void grap();
	/**
	 * Releases the gripper
	 */
	void release();

private:
	std::string ip_adress;
	modbus_t* modbus;
	const static int GRIPPER_MODBUS_ADRESS = 8001;
	int port;
	bool connected;
};
