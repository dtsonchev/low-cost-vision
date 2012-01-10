///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Dec 21 2009)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#pragma once

#include <wx/bitmap.h>
#include <wx/image.h>
#include <wx/icon.h>
#include <wx/statbmp.h>
#include <wx/gdicmn.h>
#include <wx/font.h>
#include <wx/colour.h>
#include <wx/settings.h>
#include <wx/string.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>
#include <wx/slider.h>
#include <wx/statline.h>
#include <wx/combobox.h>
#include <wx/radiobut.h>
#include <wx/checkbox.h>
#include <wx/button.h>
#include <wx/panel.h>
#include <wx/scrolwin.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// Class CrateApp
///////////////////////////////////////////////////////////////////////////////
class CrateApp: public wxFrame {
private:

protected:
	wxStaticBitmap* ImageField;
	wxStaticText* MessageLabel;
	wxScrolledWindow* OptionScrollWindow;
	wxStaticText* m_staticText29;
	wxSlider* ColorSlider;
	wxStaticLine* m_staticline1;
	wxComboBox* BackgroundComboBox;
	wxComboBox* LightingComboBox;
	wxComboBox* PerspectiveComboBox;
	wxStaticLine* m_staticline2;
	wxRadioButton* ZoomBox_radioBtn;
	wxCheckBox* ZoomCheckBox;
	wxButton* ZoomButton;
	wxButton* OriginalImageButton;
	wxStaticLine* m_staticline21;
	wxComboBox* ObjectTypeCombo;
	wxRadioButton* CrateLineRDB;
	wxRadioButton* QRCodeCornerRDB;
	wxStaticText* QRCodeCornerLabel;
	wxRadioButton* OppositeCornerRDB;
	wxStaticText* OppositeCornerLabel;
	wxStaticLine* m_staticline12;
	wxStaticText* m_staticText11;
	wxTextCtrl* QRCodeTextBox;
	wxStaticLine* m_staticline121;
	wxButton* NextObjectButton;
	wxButton* SkipButton;
	wxButton* NextImageButton;
	wxButton* ResetButton;
	wxButton* DoneButton;
	wxBoxSizer* bSizer121;

	// Virtual event handlers, overide them in your derived class
	/**
	 * This function is called when the window size changes, all the values
	 * concerning coordinates are reset.
	 * @param event the event created when the window size changes
	 */
	virtual void OnSizeChange(wxSizeEvent& event) {
		event.Skip();
	}
	/**
	 * This function is called when the mouse leaves the image field
	 * @param event the event created when the mouse moves out the image field
	 */
	virtual void OnLeaveImageField(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * This function is called when the mouse is released over the image field
	 * calls the drawing function and sets the corners (QR and Opposite corner)
	 * @param event the event created when the mouse is pressed
	 */
	virtual void OnLeftMousePressed(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * This function is called when the mouse is released over the image field
	 * calls the drawing function and sets the corners (QR and Opposite corner)
	 * @param event the event created when the mouse is released
	 */
	virtual void OnLeftMouseRelease(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * This function is called when the mouse moves over the image field
	 * calls the drawing function and sets the corners (QR and Opposite corner)
	 * @param event the event created when the mouse moves over the image
	 */
	virtual void OnImageMotion(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * This function is called when the left mouse button is released when clicking on the slider
	 * this function then calls the drawing function
	 * @param event the event created when the mouse is released
	 */
	virtual void OnColorSlider(wxScrollEvent& event) {
		event.Skip();
	}
	/**
	 * The function is called when the zoom check box is pressed
	 * enables / disables the drawing function
	 * @param event the event that is created when the radio button is pessed
	 */
	virtual void OnZoomRadioButton(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * The function is called when the zoom check box is pressed
	 * enables / disables the actual drawing on the image
	 * @param event the event that is created when the check box is pressed
	 */
	virtual void OnZoomChange(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * The function that is called when the original button is pressed
	 * calculates the scale and loads the original image
	 * @param event the event that is created when the button is pessed
	 */
	virtual void OnZoom(wxCommandEvent& event) {
		event.Skip();
	}
	/**
	 * The function that is called when the original button is pressed
	 * calculates the scale and loads the original image
	 * @param event the event that is created when the button is pessed
	 */
	virtual void OnOriginal(wxCommandEvent& event) {
		event.Skip();
	}
	/**
	 * The function that is called when the type of object is changed in
	 * the object type combo box
	 * @param event the event that is created when the type changes
	 */
	virtual void OnObjectType( wxCommandEvent& event ) {
		event.Skip();
	}
	/**
	 * The function that is called when the Next button is pressed
	 * Checks if all the values are filled and writes them to the xml file
	 * @param event the event that is created when the button is pessed
	 */
	virtual void OnNextObjectButton(wxCommandEvent& event) {
		event.Skip();
	}
	/**
	 * The function that is called when the Skip button is pressed
	 * and loads the next image
	 * @param event the event that is created when the button is pessed
	 */
	virtual void OnSkip(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * The function that is called when the reset button is pressed
	 * this clears all the values al ready enter and starts over at the first crate
	 * @param event the event that is created when the button is pessed
	 */
	virtual void OnReset(wxCommandEvent& event) {
		event.Skip();
	}
	/**
	 * The function that is called when the done button is pressed
	 * Closes the GUI
	 * @param event the event that is created when the button is pessed
	 */
	virtual void OnDoneButton(wxCommandEvent& event) {
		event.Skip();
	}
	/**
	 * The function that is called when the Next image button is pressed
	 * calls the function to load the next image
	 * @param event the event that is created when the button is pessed
	 */
	virtual void OnNextImage( wxMouseEvent& event ) {
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
	CrateApp(wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title =
			wxT("Draw a line to substract crate values"),
			const wxPoint& pos = wxDefaultPosition,
			const wxSize& size = wxSize(-1, -1),
			long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
	///The Deconstructor
	~CrateApp();
};
