///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Dec 21 2009)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#pragma once

#include <wx/string.h>
#include <wx/stattext.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/filepicker.h>
#include <wx/sizer.h>
#include <wx/statline.h>
#include <wx/radiobut.h>
#include <wx/button.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// Class ChoiceFrame
///////////////////////////////////////////////////////////////////////////////
class ChoiceFrame: public wxFrame {
private:

protected:
	///Information text field
	wxStaticText* m_staticText28;
	///A object to select a directory with the correct contents
	wxDirPickerCtrl* dirPicker;
	///A cosmetic line
	wxStaticLine* m_staticline41;
	///A text field to display messages
	wxStaticText* MessageField;
	///A cosmetic line
	wxStaticLine* m_staticline4;
	///Radio button for creating a new xml file
	wxRadioButton* CreateNewXMLradioBtn;
	///Radio button for editing a existing xml
	wxRadioButton* EditExistingXMLradioBtn;
	///Radio button for adding new images to an existing xml
	wxRadioButton* AddToExistingXMLradioBtn;
	///A cosmetic line
	wxStaticLine* m_staticline43;
	///Exit button
	wxButton* ExitButton;
	///OK button
	wxButton* OKButton;

	/// Virtual event handlers, overide them in your derived class
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnXMLOption(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnExit(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnOK(wxMouseEvent& event) {
		event.Skip();
	}

public:

	/**
	 * The constructor
	 * @param parent the parent window
	 * @param id The windows id number
	 * @param title The title of the window
	 * @param pos the location of the top left corner
	 * @param size the size of the window
	 * @param style the frame style
	 */
	ChoiceFrame(wxWindow* parent, wxWindowID id = wxID_ANY,
			const wxString& title =
					wxT("Choose an option and select the correct files"),
			const wxPoint& pos = wxDefaultPosition,
			const wxSize& size = wxSize(-1, -1),
			long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
	///The deconstructor
	~ChoiceFrame();

};
