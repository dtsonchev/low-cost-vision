//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer_gui
// File:           huniplacer_frame_impl.h
// Description:    class that implements the form it's functionality
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
#include <huniplacer_gui/huniplacer_frame.h>
#include <vector>
#include <huniplacer/huniplacer.h>
#include <huniplacer/effector_boundaries.h>
extern "C"
{
#include <modbus/modbus.h>
}

namespace huniplacer_gui
{
    class huniplacer_frame_impl : public huniplacer_frame
    {
        private:
            huniplacer::inverse_kinematics_model* ik_model;
            huniplacer::imotor3* motors;
            huniplacer::deltarobot* robot;

            double cur_x;
            double cur_y;
            double cur_z;
            std::vector<wxCoord> bounds;

            void init_deltarobot(const char* device);
            void destroy_deltarobot(void);

            static void popup_warn(const wxString& msg);
            static void popup_err(const wxString& msg);

            static void motionthread_exhandler(std::runtime_error& err);
            void update_pos_txtfields(void);
            //void update_z_slider(void);
            bool try_move(double x, double y, double z);

            void draw_boundaries();

        public:
            huniplacer_frame_impl(void);
            virtual ~huniplacer_frame_impl(void);

            //events
            void pos_panelOnLeftDown(wxMouseEvent& event);
            void pos_panelOnPaint(wxPaintEvent& event);
            void pos_panel_sideOnLeftDown(wxMouseEvent& event);
            void pos_panel_sideOnPaint(wxPaintEvent& event);
            //void slider_z_posOnLeftUp(wxMouseEvent& event);
            void button_moveOnButtonClick(wxCommandEvent& event);
            void button_connectOnButtonClick(wxCommandEvent& event);
            void button_disconnectOnButtonClick(wxCommandEvent& event);
            void button_onOnButtonClick(wxCommandEvent& event);
            void button_offOnButtonClick(wxCommandEvent& event);
    };
}
