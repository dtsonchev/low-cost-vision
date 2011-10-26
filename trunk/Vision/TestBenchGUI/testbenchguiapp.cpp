/////////////////////////////////////////////////////////////////////////////
// Name:        testbenchguiapp.cpp
// Purpose:     
// Author:      Franc
// Modified by: 
// Created:     Wed 12 Oct 2011 17:44:54 CEST
// RCS-ID:      
// Copyright:   
// Licence:     
/////////////////////////////////////////////////////////////////////////////

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

