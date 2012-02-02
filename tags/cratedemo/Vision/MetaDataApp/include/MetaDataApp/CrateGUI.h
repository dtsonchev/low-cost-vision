//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        MetaDataApp
// File:           CrateGUI.h
// Description:    The implementation of the functions in the GUI
// Author:         Glenn Meerstra
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of MetaDataApp.
//
// MetaDataApp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MetaDataApp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with MetaDataApp.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************


#ifndef CRATEGUI_H_
#define CRATEGUI_H_

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
#include <wx/radiobut.h>
#include <wx/stattext.h>
#include <wx/statline.h>
#include <wx/textctrl.h>
#include <wx/button.h>
#include <wx/frame.h>

#include <boost/filesystem.hpp>

#include <iostream>
#include <vector>

typedef enum markerOptions {
	LeftTop, RightTop, LeftBottom
} markerOptions;

/**
 * @brief This class is made for creating meta data for crates with markers and a barcode
 * @author Glenn
 * @version 2.0
 * @date 10-2011
 */
class CrateGUI: public wxFrame {
private:
	///@brief a constant for the width of the image
	static const int imageWidth = 470;
	///@brief a constant for the height of the image
	static const int imageHeight = 470;

	wxString barcode;
	int positionValues[3 * 4];

double	heightScale;
	double widthScale;
	double rotation;

	int LTX, LTY, RBX, RBY;
	int originalCenterX, originalCenterY;

	/**
	 * @brief when the left mouse button is pressed the variable is set and cleared when the mouse button is release
	 * \n when this variable is true the function on motion has to work.
	 */
	bool leftMouseDownInImage;

	/**
	 * @fn ClearPositionValues()
	 * @brief fills all the values in the positionValues array with 0
	 */
	void ClearPositionValues();
	/**
	 * @fn UpdateImageField()
	 * @brief sets the image in the image field
	 */
	void UpdateImageField();
	/**
	 * @fn DrawBoxesOnImage(int skipNr)
	 * @brief draws all the boxes on the image field
	 * @param the markerNr of the current marker (see markerOptions)
	 */
	void DrawBoxesOnImage(int skipNr);

protected:
	///images
	wxImage image;

	///Image fields
	wxStaticBitmap* imageField;

	///Box sizer with all the options
	wxBoxSizer* bSizer12;

	///Label Left top marker left top corner coordinate text field
	wxStaticText* LTLT_TxtField;
	///Label Left top marker right bottom corner coordinate text field
	wxStaticText* LTRB_TxtField;
	///Label Right top marker left top corner coordinate text field
	wxStaticText* RTLT_TxtField;
	///Label Right top marker right bottom corner coordinate text field
	wxStaticText* RTRB_TxtField;
	///Label Left bottom marker left top corner coordinate text field
	wxStaticText* LBLT_TxtField;
	///Label Left bottom marker right bottom corner coordinate text field
	wxStaticText* LBRB_TxtField;
	///Label for the path of the image
	wxStaticText* pathField;

	///Done button
	wxButton* DONE_button;
	///Reset button
	wxButton* ResetButton;

	///Barcode text field
	wxTextCtrl* QRCode_TxtField;

	///Radio buttons Left top marker
	wxRadioButton* LT_Radio;
	///Radio buttons Left top marker left top corner
	wxRadioButton* LTLTCorner_Radio;
	///Radio buttons Left top marker right bottom corner
	wxRadioButton* LTRLCorner_Radio;
	///Radio buttons Right top marker
	wxRadioButton* RT_Radio;
	///Radio buttons Right top marker left top corner
	wxRadioButton* RTLTCorner_Radio;
	///Radio buttons Right top marker right bottom corner
	wxRadioButton* RTRLCorner_Radio;
	///Radio buttons Left bottom marker
	wxRadioButton* LB_Radio;
	///Radio buttons Left bottom marker left top corner
	wxRadioButton* LBLTCorner_Radio;
	///Radio buttons Left bottom marker right bottom corner
	wxRadioButton* LBRLCorner_Radio;

	/**
	 * @fn OnLeftDown( wxMouseEvent& event )
	 * @brief when the left mouse button is pressed within the image field this function is called
	 * @param event the event created when the mouse is pressed
	 */
	virtual void OnLeftDown( wxMouseEvent& event );
	/**
	 * @fn OnLeftUp( wxMouseEvent& event )
	 * @brief when the left mouse button is released within the image field this function is called
	 * @param event the event created when the mouse is released
	 */
	virtual void OnLeftUp( wxMouseEvent& event );
	/**
	 * @fn OnLeftMotion( wxMouseEvent& event )
	 * @brief changes the value for drawing on the image
	 * @param event the event created when the mouse is in motion
	 */
	virtual void OnLeftMotion( wxMouseEvent& event );
	/**
	 * @fn OnDonePressed( wxMouseEvent& event )
	 * @brief checks if all the values are filled and closes it's self and shows it's parent
	 * @param event the event created when the Done button is pressed
	 */
	virtual void OnDonePressed( wxMouseEvent& event );
	/**
	 * @fn OnQRFocus( wxFocusEvent& event )
	 * @brief clears the value
	 * @param event the event created when the focus is set on the QR Code text field
	 */
	virtual void OnQRFocus( wxFocusEvent& event );
	/**
	 * @fn OnLeftImageField( wxMouseEvent& event )
	 * @brief The function which is called when the mouse leaves the image field
	 * @param event the event created when the image field is left
	 */
	virtual void OnLeftImageField( wxMouseEvent& event ) {leftMouseDownInImage = false;}
	/**
	 * @brief The function which is called when the window size changes
	 * @details values concerning the location of the current object are set to 0
	 * @param event the event created when the window size changes
	 */
	virtual void OnSizeChange( wxSizeEvent& event );
	/**
	 * @brief The function which is called when the Reset button is pressed
	 * @details resets all the input values for the whole image
	 * @param event the event created when the Done button is pressed
	 */
	virtual void OnReset( wxCommandEvent& event );

public:
	///@brief the constructor
	CrateGUI( wxWindow* parent, wxWindowID id = wxID_ANY, const wxString& title = wxT("Crate"), const wxPoint& pos = wxDefaultPosition, const wxSize& size = wxSize( -1,510 ), long style = wxDEFAULT_FRAME_STYLE|wxTAB_TRAVERSAL );
	///@brief the destructor
	~CrateGUI();

	/**
	 * @fn bool getValues(int *values, wxString &barcode)
	 * @brief gets the values for the location of the markers. and gets the barcode
	 * @param values a variable where the location variables are returned in
	 * @param barcode a variable where the barcode is returned in
	 * @return returns true if it succeded and all the values are filled
	 */
	bool getValues(int *values, wxString &barcode);
	/**
	 * @fn Start(wxImage cutImage, int x, int y, wxString imagePath, double rotation)
	 * @brief sets all the variables and shows the crate gui and hides the parent gui
	 * @param cutImage the part of the image where the crate is located
	 * @param x the x coordinate from the upper left corner of cutImage in the original
	 * @param y the y coordinate from the upper left corner of cutImage in the original
	 * @param imagePath the path to the object
	 * @param rotation the of the object
	 */
	void Start(wxImage cutImage, int x, int y, wxString imagePath, double rotation);

};

#endif /* CRATEGUI_H_ */
