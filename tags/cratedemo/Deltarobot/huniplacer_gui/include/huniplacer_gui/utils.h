//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer_gui
// File:           utils.h
// Description:    miscellaneous utilities
// Author:         Lukas Vermond, Kasper van Nieuwland & Glenn Meerstra
// Notes:          
//
// License:        GNU GPL v3
//
// This file is part of huniplacer_gui.
//
// huniplacer_gui is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// huniplacer_gui is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with huniplacer_gui.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


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
