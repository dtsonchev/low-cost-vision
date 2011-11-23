#pragma once

namespace huniplacer_gui
{
	namespace utils
	{
	    double convert_scale(double to_min, double to_max, double from_min, double from_max, double value)
	    {
	        return ((value - from_min) * (to_max - to_min)) / (from_max - from_min) + to_min;
	    }
	}
}
