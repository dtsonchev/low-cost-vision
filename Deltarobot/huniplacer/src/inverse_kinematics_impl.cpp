#include "inverse_kinematics_impl.h"

#include <cmath>
#include <cstdio>
#include <boost/math/special_functions/fpclassify.hpp>
#include "inverse_kinematics_exception.h"

#include "utils.h"

namespace huniplacer
{
    inverse_kinematics_impl::inverse_kinematics_impl(const double base, const double hip, const double effector, const double ankle, const double hip_ankle_angle_max) :
        inverse_kinematics_model(base, hip, effector, ankle, utils::rad(hip_ankle_angle_max))
    {
    }

    inverse_kinematics_impl::~inverse_kinematics_impl(void)
    {
    }
    
   /* double kinematics_impl::moveto(const point3& p, double angle)
    {
        //printf("\n");
        //printf("p = { %lf, %lf, %lf }\n", p.x, p.y, p.z);
        
        point3 g(p.x, p.z, p.y); //quickfix
        g = g.rotate_y(-angle);
        g.x += effector;
        //printf("g = { %lf, %lf, %lf }\n", g.x, g.y, g.z);
        
        //sqrt(a*a - g.z*g.z)
        double a = ankle*ankle - g.z*g.z;
        //printf("a = %lf\n", a);
        
        //sqrt((xg+effector-base)*(xg+effector-base) + yg*yg)
        double b = point3(g.x, g.y, 0).distance(point3(base, 0, 0));
        //printf("b = %lf\n", b);
        
        double alpha = acos((-a + hip*hip + b*b) / (2*hip*b));
        //printf("alpha = %lf\n", alpha);
        
        double beta = atan2(g.y, g.x - base);
        //printf("beta = %lf\n", beta);
        
        double rho = alpha - beta;
        //printf("rho = %lf\n", rho);
        //printf("\n");
        
        //return rho;
        return -rho; //quickfix
    }*/

	#define SQR(x) ((x)*(x))
    double inverse_kinematics_impl::moveto(const point3& p, double motor_angle)
    {
    	//ideas from Viacheslav Slavinsky are used
    	//conventions:
    	//	sitting in front of delta robot
    	//	x-axis goes from left to right
    	//	y-axis goes from front to back
    	//	z-axis goes from bottom to top
    	//	point (0,0,0) lies in the middle of all the motors at the motor's height

    	point3 p_fixed = p.rotate_z(-motor_angle);

    	p_fixed.y -= effector;
    	p_fixed.y += base;

    	double c = sqrt(SQR(p_fixed.x) + SQR(p_fixed.y) + SQR(p_fixed.z));

    	double alpha_acos_input =
    			(-(SQR(ankle) - SQR(p_fixed.x)) + SQR(hip) + SQR(c))
    			/
    			(2*hip*c);
    	if(alpha_acos_input < -1 || alpha_acos_input > 1)
    		throw inverse_kinematics_exception("point out of range", p);

    	double alpha = acos(alpha_acos_input);

    	double beta = atan2(p_fixed.z, p_fixed.y);
    	double rho = beta - alpha;

    	double knee_z = sin(rho) * hip;
    	double hip_ankle_angle = abs(atan(p_fixed.x / (p_fixed.z - knee_z)));
    	if(hip_ankle_angle > hip_ankle_angle_max)
    		throw inverse_kinematics_exception("angle between hip and ankle is out of range", p);

    	return rho;
    }
	#undef SQR
    
    void inverse_kinematics_impl::point_to_motion(const point3& p, motionf& mf)
    {
        point3 goal = p;

        try
        {
			mf.angles[0] = -90 - utils::deg(moveto(p, utils::rad(0 * 120)));
			mf.angles[1] = -90 - utils::deg(moveto(p, utils::rad(1 * 120)));
			mf.angles[2] = -90 - utils::deg(moveto(p, utils::rad(2 * 120)));
        }
        catch(inverse_kinematics_exception& ex)
        {
        	throw ex;
        }

        //quickfix
        mf.acceleration[0] = mf.acceleration[1] = mf.acceleration[2] = 3600;
        mf.deceleration[0] = mf.deceleration[1] = mf.deceleration[2] = 3600;

        if(
            boost::math::isnan(mf.angles[0]) ||
            boost::math::isnan(mf.angles[1]) ||
            boost::math::isnan(mf.angles[2]))
        {
            throw inverse_kinematics_exception("angle=NaN", p);
        }
    }
}

