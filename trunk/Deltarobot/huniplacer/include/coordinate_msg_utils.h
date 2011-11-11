#pragma once

//TODO seperate implementation from declaration

#include <stdexcept>
#include <huniplacer/coordinate_msg.h>
#include "point3.h"

namespace huniplacer
{
    namespace coordinate_msg_types
    {
        enum _coordinate_msg_types
        {
            POWER_OFF,
            POWER_ON,
            MOVE,
            STOP
        };
    }

    class coordinate_msg_exception : public std::runtime_error
    {
        public:
            coordinate_msg_exception(const std::string& msg) : std::runtime_error(msg) {}
            ~coordinate_msg_exception(void) throw() {}
    };

    class coordinate_msg_utils
    {
        public:
            static void init_coordinate_msg(coordinate_msg& msg, int type)
            {
                msg.type = type;
            }
            
            static void init_coordinate_msg_move(coordinate_msg& msg, const point3& p, double speed)
            {
                msg.type = coordinate_msg_types::MOVE;
                
                msg.data.push_back(p.x);
                msg.data.push_back(p.y);
                msg.data.push_back(p.z);
                msg.data.push_back(speed);
            }
            
            static point3 get_point(const coordinate_msg& msg)
            {
                if(msg.type != coordinate_msg_types::MOVE)
                {
                    throw coordinate_msg_exception("message type != move");
                }
                if(msg.data.size() != 4)
                {
                    throw coordinate_msg_exception("message data size != 4 (float64's)");
                }
                
                return point3(msg.data.at(0), msg.data.at(1), msg.data.at(2));
            }
            
            static double get_speed(const coordinate_msg& msg)
            {
                if(msg.type != coordinate_msg_types::MOVE)
                {
                    throw coordinate_msg_exception("message type != move");
                }
                if(msg.data.size() != 4)
                {
                    throw coordinate_msg_exception("message data size != 4 (dwords)");
                }
                
                return msg.data.at(3);
            }
    };
}





