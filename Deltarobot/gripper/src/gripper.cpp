#include <gripper/gripper.h>
#include <cerrno>
#include <stdexcept>
#include <string>
#include <sstream>

gripper::gripper(const std::string & ip_adress, const int port) :
	ip_adress(ip_adress),
	port(port),
	connected(false)
{
}

gripper::~gripper()
{
	try
	{
		if(connected)
		{
			disconnect();
		}
	}
	catch(...)
	{
	}
}

void gripper::connect()
{
	if(!connected)
	{
		modbus = modbus_new_tcp(ip_adress.c_str(), port);
		if(modbus == NULL)
		{
			throw std::runtime_error("Unable to allocate libmodbus context");
		}
		if(modbus_connect(modbus) == -1)
		{
			throw std::runtime_error("Connection failed");
		}
		connected = true;
	}
	else
	{
		throw std::runtime_error("Already connected");
	}
}

void gripper::disconnect()
{
	if(connected)
	{
		modbus_close(modbus);
		modbus_free(modbus);
		connected = false;
	}
	else
	{
		throw std::runtime_error("Already disconnected");
	}
}

void gripper::grap()
{
	if(!connected)
	{
		throw std::runtime_error("Could not grab: disconnected");
	}

	if(modbus_write_register(modbus, GRIPPER_MODBUS_ADRESS, 1) == -1)
	{
		std::stringstream ss;
		ss << "Could not grab: modbus error: ";
		ss << modbus_strerror(errno);
		throw std::runtime_error(ss.str());
	}
}

void gripper::release()
{
	if(!connected)
	{
		throw std::runtime_error("Could not release: disconnected");
	}

	if(modbus_write_register(modbus, GRIPPER_MODBUS_ADRESS, 0) == -1)
	{
		std::stringstream ss;
		ss << "Could not release: modbus error: ";
		ss << modbus_strerror(errno);
		throw std::runtime_error(ss.str());
	}
}

