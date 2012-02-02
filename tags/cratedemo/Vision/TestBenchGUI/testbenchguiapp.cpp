//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchGUI
// File:           testbenchguiapp.cpp
// Description:    Main application
// Author:         Franc Pape
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of TestBenchGUI.
//
// TestBenchGUI is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// TestBenchGUI is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with TestBenchGUI.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************

// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
#pragma hdrstop
#endif

#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

////@begin includes
////@end includes

#include "testbenchguiapp.h"

////@begin XPM images
////@end XPM images


/*
 * Application instance implementation
 */

////@begin implement app
IMPLEMENT_APP( TestbenchGUIApp )
////@end implement app


/*
 * TestbenchGUIApp type definition
 */

IMPLEMENT_CLASS( TestbenchGUIApp, wxApp )


/*
 * TestbenchGUIApp event table definition
 */

BEGIN_EVENT_TABLE( TestbenchGUIApp, wxApp )

////@begin TestbenchGUIApp event table entries
////@end TestbenchGUIApp event table entries

END_EVENT_TABLE()


/*
 * Constructor for TestbenchGUIApp
 */

TestbenchGUIApp::TestbenchGUIApp()
{
    Init();
}


/*
 * Member initialisation
 */

void TestbenchGUIApp::Init()
{
////@begin TestbenchGUIApp member initialisation
////@end TestbenchGUIApp member initialisation
}

/*
 * Initialisation for TestbenchGUIApp
 */

bool TestbenchGUIApp::OnInit()
{
////@begin TestbenchGUIApp initialisation
	// Remove the comment markers above and below this block
	// to make permanent changes to the code.

#if wxUSE_XPM
	wxImage::AddHandler(new wxXPMHandler);
#endif
#if wxUSE_LIBPNG
	wxImage::AddHandler(new wxPNGHandler);
#endif
#if wxUSE_LIBJPEG
	wxImage::AddHandler(new wxJPEGHandler);
#endif
#if wxUSE_GIF
	wxImage::AddHandler(new wxGIFHandler);
#endif
	MainFrame* mainWindow = new MainFrame( NULL );
	mainWindow->Show(true);
////@end TestbenchGUIApp initialisation

    return true;
}


/*
 * Cleanup for TestbenchGUIApp
 */

int TestbenchGUIApp::OnExit()
{
////@begin TestbenchGUIApp cleanup
	return wxApp::OnExit();
////@end TestbenchGUIApp cleanup
}

