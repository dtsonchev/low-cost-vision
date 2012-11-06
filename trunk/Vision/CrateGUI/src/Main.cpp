//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CrateGUI
// File:           Main.cpp
// Description:    The main that starts the GUI
// Author:         Glenn Meerstra
// Notes:          This program indexes images located in a 'Images' directory next to
//		   this directory there needs to be a .txt file which contains:
//		   -backgroundTypes 
//		   	Black
//			White
//			etc.
//		   -LightOptions
//			TL light
//			No extra light
//			etc.
//		  -PerspectiveOptions
//			2D
//			3D
//			etc.
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
#include <wx/wx.h>
#include <CrateGUI/ChoiceFrameImpl.h>
#include <CrateGUI/CrateAppImpl.h>

/**
 * @mainpage Meta data GUI
 * @section This program indexes images
 * This program indexes images located in a 'Images' directory next to
 * this directory there needs to be a .txt file which contains:
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
	ChoiceFrameImpl *startFrame;
	startFrame = new ChoiceFrameImpl( NULL );
	startFrame->Show(true);

	return true;
}
