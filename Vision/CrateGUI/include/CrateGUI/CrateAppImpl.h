//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CrateGUI
// File:           CrateAppImpl.h
// Description:    The implementation of the functions in the GUI
// Author:         Glenn Meerstra
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of CrateGUI.
//
// CrateGUI is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// CrateGUI is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with CrateGUI.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#pragma once

#include "CrateGUI/CrateApp.h"

#include "DetectQRCode/BarcodeDetector.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>

#include <vector>

#include <wx/geometry.h>

#include <iostream>
#include <string>

/**
 * Defines used in order to caculate the fiducial locations
 *
 */
//TODO: turn into const static private variables
#define LINE_LENGTH 7.1
#define FID_OFFSET 2.25
#define FID_RADIUS 0.75
#define QR_MARKER_SIDE 0.7
#define QR_CODE_SIDE 2.8

/**
 * This class is the implementation of the GUI functions
 */
class CrateAppImpl: public CrateApp
{
private:
	///The corner selected during the program at the same corner as the QR code
	wxPoint QRCorner;
	///The corner selected during the program at the opposite corner as the QR code
	wxPoint OppositeCorner;

	///The center point of the fiducial left of the QR code
	wxPoint2DDouble fid1;
	///The center point of the fiducial in the opposit corner of the QR corner
	wxPoint2DDouble fid2;
	///The center point of the fiducial above the QR code
	wxPoint2DDouble fid3;
	///The center point of the crate
	wxPoint2DDouble center;

	///The string extracted from the QR code
	std::string QRCode;

	///All the image paths
	std::vector<boost::filesystem::path> imagePaths;
	///The iterator that points to one of the images in imagePaths
	std::vector<boost::filesystem::path>::iterator imagePathsIt;

	///The path to the chosen directory
	boost::filesystem::path dirPath;
	///The path to the chosen xml file
	boost::filesystem::path xmlPath;

	///The xml contents where the image are added to
	boost::property_tree::ptree pt;
	///The image in the xml file which is currently being prossed by the user
	boost::property_tree::ptree* tempValue;

	///A variable to see if the program is in the zoom state
	bool zoom;
	///A variable to see if the current image is already available in the xml
	bool AlreadyInXML;
	///A variable to see if the mouse is pressed inside the image field
	bool mousePressedInImageField;

	///The scale at which the image is scaled
	double Scale;

	double LargestScale;

	///The top left x coordinate of the part at which the image is zoom into
	int zoomX;
	///The width of the part at which the image is zoom into
	int zoomWidth;
	///The top left y coordinate of the part at which the image is zoom into
	int zoomY;
	///The height of the part at which the image is zoom into
	int zoomHeight;
	///The xml option (add, create, edit)
	int XMLOption;
	///The crate number at which the user is currently working
	int currentCrateNumber;
	///The total amount of crate in the current image
	int amountOfCrates;
	///The offset of the top left corner of the imagefield to the top left corner of the window
	int coordinateOffset;

	///The original image
	wxImage image;
	///The zoomed part of the original image
	wxImage zoomImage;
	///The path of the image currently used
	wxString currentImagePath;

	/**
	 * Extracts the barcode from the image
	 * @param distance the pixel distance from the QR code corner to the opposit corner
	 * @param angle the angle at which the QR code is rotated at in radialen
	 * @return returns the barcode
	 */
	std::string getBarcode(double distance, double angle);
	/**
	 * gets the path from the chosen dir
	 * @param path the path where the chosen dir needs to be substracted from
	 */
	void getPathFromChosenDir(boost::filesystem::path& path);
	/**
	 * loads a new image in the image variable and calculates for the first time the scale
	 * @param path the image that needs to be loaded
	 */
	void setCurrentImagePath(const wxString& path);
	/**
	 * calculates the fiducial points
	 */
	void calculateFiducialPoints();
	/**
	 * Draws the fiducials and the zoom box on the image
	 */
	void drawCrateAttributes();
	/**
	 * refreshed the image field with the original image
	 */
	void UpdateImageField();
	/**
	 * selects the next image and cals setCurrentImagePath
	 */
	void NextImage();
	std::string DoubleToStringFormatted(double d);

protected:

public:
	/**
	 * an enum with XML options
	 */
	typedef enum XMLOptions{
		Add, Edit, NewXML
	} XMLOptions;

	/**
	 * The constructor
	 * @param parent the parent window
	 * @param id The windows id number
	 * @param title The title of the window
	 * @param pos the location of the top left corner
	 * @param size the size of the window
	 * @param style the frame style
	 */
	CrateAppImpl(
			wxWindow* parent,
			wxWindowID id = wxID_ANY,
			const wxString& title = wxT("Draw a line to substract crate values"),
			const wxPoint& pos = wxDefaultPosition,
			const wxSize& size = wxSize(-1, -1),
			long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
	/**
	 * The deconstructor
	 */
	virtual ~CrateAppImpl();

	/**
	 * This function is called when the window size changes, all the values
	 * concerning coordinates are reset.
	 * @param event the event created when the window size changes
	 */
	void OnSizeChange(wxSizeEvent& event);
	/**
	 * This function is called when the mouse leaves the image field
	 * @param event the event created when the mouse moves out the image field
	 */
	void OnLeaveImageField(wxMouseEvent& event);
	/**
	 * This function is called when the mouse is released over the image field
	 * calls the drawing function and sets the corners (QR and Opposite corner)
	 * @param event the event created when the mouse is pressed
	 */
	void OnLeftMousePressed(wxMouseEvent& event);
	/**
	 * This function is called when the mouse is released over the image field
	 * calls the drawing function and sets the corners (QR and Opposite corner)
	 * @param event the event created when the mouse is released
	 */
	void OnLeftMouseRelease(wxMouseEvent& event);
	/**
	 * This function is called when the mouse moves over the image field
	 * calls the drawing function and sets the corners (QR and Opposite corner)
	 * @param event the event created when the mouse moves over the image
	 */
	void OnImageMotion(wxMouseEvent& event);
	/**
	 * This function is called when the left mouse button is released when clicking on the slider
	 * this function then calls the drawing function
	 * @param event the event created when the mouse is released
	 */
	void OnColorSlider(wxScrollEvent& event);
	/**
	 * The function is called when the zoom check box is pressed
	 * enables / disables the drawing function
	 * @param event the event that is created when the radio button is pessed
	 */
	void OnZoomRadioButton(wxMouseEvent& event);
	/**
	 * The function is called when the zoom check box is pressed
	 * enables / disables the actual drawing on the image
	 * @param event the event that is created when the check box is pressed
	 */
	void OnZoomChange(wxMouseEvent& event);
	/**
	 * The function that is called when the original button is pressed
	 * calculates the scale and loads the original image
	 * @param event the event that is created when the button is pessed
	 */
	void OnZoom(wxCommandEvent& event);
	/**
	 * The function that is called when the original button is pressed
	 * calculates the scale and loads the original image
	 * @param event the event that is created when the button is pessed
	 */
	void OnOriginal(wxCommandEvent& event);
	/**
	 * The function that is called when the type of object is changed in
	 * the object type combo box
	 * @param event the event that is created when the type changes
	 */
	void OnObjectType( wxCommandEvent& event );
	/**
	 * The function that is called when the Next button is pressed
	 * Checks if all the values are filled and writes them to the xml file
	 * @param event the event that is created when the button is pessed
	 */
	void OnNextObjectButton(wxCommandEvent& event);
	/**
	 * The function that is called when the Skip button is pressed
	 * and loads the next image
	 * @param event the event that is created when the button is pessed
	 */
	void OnSkip(wxMouseEvent& event);
	/**
	 * The function that is called when the reset button is pressed
	 * this clears all the values al ready enter and starts over at the first crate
	 * @param event the event that is created when the button is pessed
	 */
	void OnReset(wxCommandEvent& event);
	/**
	 * The function that is called when the done button is pressed
	 * Closes the GUI
	 * @param event the event that is created when the button is pessed
	 */
	void OnDoneButton(wxCommandEvent& event);
	/**
	 * The function that is called when the Next image button is pressed
	 * calls the function to load the next image
	 * @param event the event that is created when the button is pessed
	 */
	void OnNextImage( wxMouseEvent& event );

	/**
	 * Transfer the chosen directory path to this class
	 * @param dirPath the path to transfer
	 */
	inline void SetDirPath(boost::filesystem::path& dirPath);
	/**
	 * Transfers the chosen xml path to this class
	 * @param xmlPath the path to transfer
	 */
	inline void SetXMLPath(boost::filesystem::path& xmlPath);
	/**
	 * Transfers the paths to the images to this class
	 * @param imagePaths the paths to transfer
	 */
	inline void ChangeImagePaths(std::vector<boost::filesystem::path>& imagePaths);
	/**
	 * Sets the option for editing the xml
	 * @param option the edit option
	 */
	inline void EditXML(int option);
	/**
	 * Adds a new perspective to the perspective combobox
	 * @param perspective the perspective to add
	 */
	inline void AddPerspective(wxString perspective);
	/**
	 * Adds a new light option to the light combobox
	 * @param light the light option to add
	 */
	inline void AddLight(wxString light);
	/**
	 * Adds a new background to the background combobox
	 * @param background the background to add
	 */
	inline void AddBackground(wxString background);

	/**
	 * This function initializes the iterator and creates a new crateTestSet.xml file if necessary
	 */
	void Start();

};

void CrateAppImpl::SetDirPath(boost::filesystem::path& dirPath){
	this->dirPath = dirPath;
}
void CrateAppImpl::SetXMLPath(boost::filesystem::path& xmlPath){
	this->xmlPath = xmlPath;
}
void CrateAppImpl::ChangeImagePaths(std::vector<boost::filesystem::path>& imagePaths){
	this->imagePaths = imagePaths;
}
void CrateAppImpl::EditXML(int option){
	this->XMLOption = option;
}
void CrateAppImpl::AddPerspective(wxString perspective){
	PerspectiveComboBox->Append(perspective);
}
void CrateAppImpl::AddLight(wxString light){
	LightingComboBox->Append(light);
}
void CrateAppImpl::AddBackground(wxString background){
	BackgroundComboBox->Append(background);
}
