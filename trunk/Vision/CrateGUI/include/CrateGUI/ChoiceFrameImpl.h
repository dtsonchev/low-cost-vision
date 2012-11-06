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
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************
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
