//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        huniplacer_gui
// File:           huniplacer_frame.h
// Description:    WxFormbuilder generated form. implementation in huniplacer_frame_impl
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


///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep  8 2010)
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
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/statbox.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// Class huniplacer_frame
///////////////////////////////////////////////////////////////////////////////
class huniplacer_frame: public wxFrame {
private:

protected:
	wxPanel* pos_panel;
	wxPanel* pos_panel_side;
	wxStaticText* lab_x;
	wxTextCtrl* txtbox_x;
	wxStaticText* lab_y;
	wxTextCtrl* txtbox_y;
	wxStaticText* lab_z;
	wxTextCtrl* txtbox_z;
	wxStaticText* lab_speed;
	wxTextCtrl* txtbox_speed;
	wxButton* button_move;
	wxButton* button_connect;
	wxButton* button_disconnect;
	wxButton* button_on;
	wxButton* button_off;
	wxStaticText* lab_status;

	// Virtual event handlers, overide them in your derived class
	virtual void pos_panelOnLeftDown(wxMouseEvent& event) {
		event.Skip();
	}
	virtual void pos_panelOnPaint(wxPaintEvent& event) {
		event.Skip();
	}
	virtual void pos_panel_sideOnLeftDown(wxMouseEvent& event) {
		event.Skip();
	}
	virtual void pos_panel_sideOnPaint(wxPaintEvent& event) {
		event.Skip();
	}
	virtual void button_moveOnButtonClick(wxCommandEvent& event) {
		event.Skip();
	}
	virtual void button_connectOnButtonClick(wxCommandEvent& event) {
		event.Skip();
	}
	virtual void button_disconnectOnButtonClick(wxCommandEvent& event) {
		event.Skip();
	}
	virtual void button_onOnButtonClick(wxCommandEvent& event) {
		event.Skip();
	}
	virtual void button_offOnButtonClick(wxCommandEvent& event) {
		event.Skip();
	}

public:

	huniplacer_frame(wxWindow* parent, wxWindowID id = wxID_ANY,
			const wxString& title = wxT("huniplacer diagnostic tool"),
			const wxPoint& pos = wxDefaultPosition,
			const wxSize& size = wxSize(-1, -1),
			long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
	~huniplacer_frame();

};

#endif //__huniplacer_frame__
