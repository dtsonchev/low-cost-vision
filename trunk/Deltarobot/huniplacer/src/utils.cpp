#include <huniplacer/utils.h>

namespace huniplacer
{
    namespace utils
    {
        long time_now(void)
        {
            boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration duration(time.time_of_day());
            return duration.total_milliseconds();
        }
        
        void sleep(long ms)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
        }

        double deg(double rad)
		{
			return (rad / M_PI) * 180;
		}

		double rad(double deg)
		{
			return (deg / 180) * M_PI;
		}
    }
}
