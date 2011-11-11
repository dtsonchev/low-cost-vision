#pragma once

#include <cmath>

namespace huniplacer
{
	/// @brief 3 dimensional point class
    class point3
    {
        public:
            double x, y, z;

            point3(double x, double y, double z) : x(x), y(y), z(z) { }
            ~point3() { }
            
            point3 offset(point3& p);
            
            inline point3& operator+=(const point3& rhs)
            {
                x += rhs.x;
                y += rhs.y;
                z += rhs.z;
                return *this;
            }
            
            inline const point3 operator+(const point3& rhs) const
            {
                point3 res = *this;
                res += rhs;
                return res;
            }
            
            /**
             * @brief calculates the euclidean distance between *this and p
             * @return distance
             **/
            inline double distance(const point3& p) const
            {
                double dx = x - p.x;
                double dy = y - p.y;
                double dz = z - p.z;
                return sqrt(dx*dx + dy*dy + dz*dz);
            }
            
            /**
             * @brief rotate over the Y axis
             * @param phi rotation in degrees
             **/
            inline point3 rotate_y(double phi) const
            {
                return point3(x*cos(phi) - z*sin(phi), y, x*sin(phi) + z*cos(phi));
            }
            
            /**
             * @brief rotate over the Z axis
             * @param phi rotation in degrees
             **/
            inline point3 rotate_z(double phi) const
            {
                return point3(x*cos(phi) - y*sin(phi), x*sin(phi) + y*cos(phi), z);
            }            
    };
}
