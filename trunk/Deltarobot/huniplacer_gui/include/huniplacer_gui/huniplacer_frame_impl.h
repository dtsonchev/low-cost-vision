#pragma once
#include <huniplacer_gui/huniplacer_frame.h>
#include <vector>
#include <huniplacer/huniplacer.h>
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
            void update_z_slider(void);
            bool try_move(double x, double y, double z);

        public:
            huniplacer_frame_impl(void);
            virtual ~huniplacer_frame_impl(void);

            //events
            void pos_panelOnLeftDown(wxMouseEvent& event);
            void pos_panelOnPaint(wxPaintEvent& event);
            void slider_z_posOnLeftUp(wxMouseEvent& event);
            void button_moveOnButtonClick(wxCommandEvent& event);
            void button_circleOnButtonClick(wxCommandEvent& event);
            void button_connectOnButtonClick(wxCommandEvent& event);
            void button_disconnectOnButtonClick(wxCommandEvent& event);
            void button_onOnButtonClick(wxCommandEvent& event);
            void button_offOnButtonClick(wxCommandEvent& event);
    };
}
