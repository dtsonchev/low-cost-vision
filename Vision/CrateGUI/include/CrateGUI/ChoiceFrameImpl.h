//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CrateGUI
// File:           ChoiceFrameImpl.h
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

#include "CrateGUI/ChoiceFrame.h"
#include "CrateGUI/CrateAppImpl.h"

#include <boost/filesystem.hpp>

/**
 * This class is the implementation of the GUI parts for choosing the the\n
 * correct directory and xml file
 */
class ChoiceFrameImpl: public ChoiceFrame {
private:
	//The application to give the actual values to the images
	CrateAppImpl* crateApp;
	//path to the chosen directory
	boost::filesystem::path dirPath;
	//path to the chosen xmlPath
	boost::filesystem::path xmlPath;
	//path to the values.xml
	boost::filesystem::path valuesXMLPath;
	//All the images that are available in the image directory
	std::vector<boost::filesystem::path> imagePaths;

protected:

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
	ChoiceFrameImpl(wxWindow* parent, wxWindowID id = wxID_ANY,
			const wxString& title =
					wxT("Choose an option and select the correct files"),
			const wxPoint& pos = wxDefaultPosition,
			const wxSize& size = wxSize(-1, -1),
			long style = wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
	//Destructor
	virtual ~ChoiceFrameImpl(){}

	/**
	 * Adds image paths to the imagePaths vector
	 * @param itPath the path to the dir ectory at which an iterator searches in for images
	 * @param xmlPath to path to the xml in order to skip images already in available in the xml
	 */
	void LoadImagePaths(const boost::filesystem::path &itPath, const boost::filesystem::path &xmlPath);
	/**
	 * This function is called when the selected radio button changes
	 * @param event the event generated when the selected radio button changes
	 */
	void OnXMLOption(wxMouseEvent& event);
	/**
	 * This function is called when the exit button is pressed
	 * @param event the event created when the button is pressed
	 */
	void OnExit(wxMouseEvent& event);
	/**
	 * This function is called when the OK button is pressed
	 * @param event the event created when the button is pressed
	 */
	void OnOK(wxMouseEvent& event);

};
