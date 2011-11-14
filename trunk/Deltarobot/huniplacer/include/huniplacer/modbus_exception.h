#pragma once

#include <stdexcept>
#include <string>
#include <sstream>
#include <cerrno>

#include <modbus/modbus.h>

namespace huniplacer
{
	/**
	 * @brief exception to indicate modbus errors
	 *
	 * modbus_ctrl can throw this exception whenever a modbus related error occurs
	 **/
    class modbus_exception : public std::runtime_error
    {
        private:
    		/// @brief modbus error code
            const int error_code;
            /// @brief modbus error string (obtained using modbus_strerror)
            std::string msg;
            
        public:
            modbus_exception(void) :
                std::runtime_error(""),
                error_code(errno)
            {
                std::stringstream ss;
                ss << "modbus error[" << error_code << "]: " << modbus_strerror(error_code);
                msg = ss.str();
            }
            
            virtual ~modbus_exception(void) throw()
            {
            }
            
            const char* what()
            {
                return msg.c_str();
            }
            
            int get_error_code(void)
            {
                return error_code;
            }
    };
}
