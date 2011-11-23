#pragma once

#include <cmath>
#include <vector>

namespace pcrctransformation {
	/**
	* class that represents a 2 dimensional point.
	* Has utilities to transform the point.
	*/
	class point2f {
		public:
			/**
			 * contains the x value of the point
			 */
			double x;
			/**
			 * contains the y value of the point
			 */
			double y;

			/**
			 * constructor to make a point to 0,0
			 */
			point2f() : x(0), y(0) { }
			/**
			 * constructor to make a point at a specific location
			 * @param x the x value of the point
			 * @param y the y value of the point
			 */
			point2f(double x, double y) : x(x), y(y) { }
			/**
			 * destructor
			 */
			~point2f() { }

			inline const bool operator==(const point2f& rhs) const
			{
				return this->x == rhs.x && this->y == rhs.y;
			}

            inline point2f& operator+=(const point2f& rhs)
            {
                x += rhs.x;
                y += rhs.y;
                return *this;
            }

            inline point2f& operator-=(const point2f& rhs)
            {
                x -= rhs.x;
                y -= rhs.y;
                return *this;
            }

            inline point2f& operator*=(const point2f& rhs)
            {
                x *= rhs.x;
                y *= rhs.y;
                return *this;
            }

            inline const point2f operator+(const point2f& rhs) const
            {
                point2f res = *this;
                res += rhs;
                return res;
            }

            inline const point2f operator-(const point2f& rhs) const
            {
                point2f res = *this;
                res -= rhs;
                return res;
            }

            inline const point2f operator*(const point2f& rhs) const
            {
                point2f res = *this;
                res *= rhs;
                return res;
            }
            /**
             * calculates the distance between two points
             * @param p the second point
             * @return the distance between the points
             */
            inline double distance(const point2f& p) const
            {
                double dx = x - p.x;
                double dy = y - p.y;
                return sqrt(dx*dx + dy*dy);
            }
            /**
             * Calculate the center of two points
             * @param p the second point
             * @return returns a point2f with the location of the center
             */
            inline point2f mean(const point2f& p) const
			{
            	point2f result;
            	result = *this+p;
            	result.x /= 2;
            	result.y /= 2;
				return result;
			}
            /**
             * rotates the point around 0,0
             * @param angle the angle to rotate (radians)
             * @return returns a point2f with the new location
             */
            inline point2f rotate(double angle) const
            {
            	return point2f(x*cos(angle) - y*sin(angle), x*sin(angle) + y*cos(angle));
            }
            /**
             * a vector of point2f's
             */
            typedef std::vector<point2f> point2fvector;


	};
}
