//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        MetaDataApp
// File:           Main.cpp
// Description:    This program indexes images located in a 'Images' directory next to 
// 			this directory there needs to be a .txt file which contains: 
// 			-backgroundTypes 
//  				Black 
//  				White 
//  				etc. 
//  			-LightOptions 
//  				TL light 
//  				No extra light 
//  				etc. 
//  			-PerspectiveOptions 
//  				2D
//	  			3D 
//  				etc.
//
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
#include <wx/wx.h>
#include <MetaDataApp/ChoiceFrame.h>
#include <MetaDataApp/ConverterGUI.h>

/**
 * @mainpage Meta data GUI
 * @section This program indexes images
 * This program indexes images located in a 'Images' directory next to
 * this directory there needs to be a .txt file which contains:
 * \n -objectTypes \n Lego \n Lego man \n etc. \n
 * \n -backgroundTypes \n Black \n White \n etc. \n
 * \n -LightOptions \n TL light \n No extra light \n etc. \n
 * \n -PerspectiveOptions \n 2D \n 3D \n etc.
 */

///@brief this class is made to launch the GUI
class MyApp: public wxApp {
	/**
	 * @brief this function launches the GUI
	 */
	virtual bool OnInit();
};

IMPLEMENT_APP(MyApp)

bool MyApp::OnInit() {
	ChoiceFrame *startFrame;
	startFrame = new ChoiceFrame(NULL, wxID_ANY, wxT(""),
			wxDefaultPosition, wxSize(250, 400),
			wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
	startFrame->Start();

	return true;
}
