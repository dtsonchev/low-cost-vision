///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Nov  4 2011)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#ifndef __huniplacer_frame__
#define __huniplacer_frame__

#include <wx/panel.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/slider.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/statbox.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

namespace huniplacer_gui
{
    class huniplacer_frame: public wxFrame
    {
        private:

        protected:
            wxPanel* pos_panel;
            wxSlider* slider_z_pos;
            wxStaticText* lab_x;
            wxTextCtrl* txtbox_x;
            wxStaticText* lab_y;
            wxTextCtrl* txtbox_y;
            wxStaticText* lab_z;
            wxTextCtrl* txtbox_z;
            wxStaticText* lab_speed;
            wxTextCtrl* txtbox_speed;
            wxButton* button_move;
            wxStaticText* lab_radius;
            wxTextCtrl* txtbox_circle;
            wxButton* button_circle;
            wxButton* button_reset;
            wxButton* button_on;
            wxButton* button_off;

            // Virtual event handlers, overide them in your derived class
            virtual void pos_panelOnLeftDown(wxMouseEvent& event)
            {
                event.Skip();
            }
            virtual void pos_panelOnPaint(wxPaintEvent& event)
            {
                event.Skip();
            }
            virtual void slider_z_posOnLeftUp(wxMouseEvent& event)
            {
                event.Skip();
            }
            virtual void button_moveOnButtonClick(wxCommandEvent& event)
            {
                event.Skip();
            }
            virtual void button_circleOnButtonClick(wxCommandEvent& event)
            {
                event.Skip();
            }
            virtual void button_resetOnButtonClick(wxCommandEvent& event)
            {
                event.Skip();
            }
            virtual void button_onOnButtonClick(wxCommandEvent& event)
            {
                event.Skip();
            }
            virtual void button_offOnButtonClick(wxCommandEvent& event)
            {
                event.Skip();
            }

        public:

            huniplacer_frame(wxWindow* parent, wxWindowID id = wxID_ANY,
                    const wxString& title = wxT("huniplacer diagnostic tool"),
                    const wxPoint& pos = wxDefaultPosition, const wxSize& size =
                            wxSize(540, 665),
                    long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
            ~huniplacer_frame();

    };
}
#endif //__huniplacer_frame__
