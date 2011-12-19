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
#include <wx/filepicker.h>
#include <wx/combobox.h>
#include <wx/radiobut.h>
#include <wx/checkbox.h>
#include <wx/button.h>
#include <wx/scrolwin.h>
#include <wx/frame.h>

///////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// Class CrateApp
///////////////////////////////////////////////////////////////////////////////
class CrateApp: public wxFrame {
private:

protected:
	///The image field
	wxStaticBitmap* ImageField;
	///The label to post messages in
	wxStaticText* MessageLabel;
	///A frame to allow scrolling
	wxScrolledWindow* OptionScrollWindow;
	///Information text field
	wxStaticText* AmountOfObjectsLabel;
	///A field were the user writes the amount of object in
	wxTextCtrl* AmountOfObjectsTxtField;
	///Information text field
	wxStaticText* m_staticText29;
	///Slider for changing the drawing color
	wxSlider* ColorSlider;
	///A cosmetic line
	wxStaticLine* m_staticline1;
	///Combo box where the background is selected
	wxComboBox* BackgroundComboBox;
	///Combo box where the lighting is selected
	wxComboBox* LightingComboBox;
	///Combo box where the perspective is selected
	wxComboBox* PerspectiveComboBox;
	///A cosmetic line
	wxStaticLine* m_staticline2;
	///The radio button for drawing a zoom box
	wxRadioButton* ZoomBox_radioBtn;
	///A check for showing the zoom box in the image
	wxCheckBox* ZoomCheckBox;
	///The button to press to actual zoom in on the image
	wxButton* ZoomButton;
	///The button to press to exit zoom mode
	wxButton* OriginalImageButton;
	///A cosmetic line
	wxStaticLine* m_staticline21;
	///Radio button for drawing a line from the QR code corner to the opposite corner
	wxRadioButton* LineRDB;
	///Radio button for selecting the QR code corner
	wxRadioButton* QRCodeCornerRDB;
	///Information text field
	wxStaticText* QRCodeCornerLabel;
	///Radio button for selecting the opposite from the QR corner
	wxRadioButton* OppositeCornerRDB;
	///Information text field
	wxStaticText* OppositeCornerLabel;
	///A cosmetic line
	wxStaticLine* m_staticline12;
	///Information text field
	wxStaticText* m_staticText11;
	///Text box where the contents of the QR code is placed
	wxTextCtrl* QRCodeTextBox;
	///A cosmetic line
	wxStaticLine* m_staticline121;
	///Button for going to the next object/image
	wxButton* NextObjectButton;
	///Button for skipping the current image an going to the next
	wxButton* SkipButton;
	///Button for reseting the values and starting over for the current image
	wxButton* ResetButton;
	///Button closses the window
	wxButton* DoneButton;
	///sizer used in calculating the max width of the image
	wxBoxSizer* bSizer121;

	// Virtual event handlers, overide them in your derived class
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnSizeChange(wxSizeEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnLeaveImageField(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnLeftMousePressed(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnLeftMouseRelease(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnImageMotion(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnColorSlider(wxScrollEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnZoomRadioButton(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnZoomChange(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnZoom(wxCommandEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnOriginal(wxCommandEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnNextObjectButton(wxCommandEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnSkip(wxMouseEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnReset(wxCommandEvent& event) {
		event.Skip();
	}
	/**
	 * See the implementation
	 * @param event
	 */
	virtual void OnDoneButton(wxCommandEvent& event) {
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
	///The deconstructor
	~CrateApp();

};
