/////////////////////////////////////////////////////////////////////////////
// Name:        testbenchguiapp.h
// Purpose:     
// Author:      Franc
// Modified by: 
// Created:     Wed 12 Oct 2011 17:44:54 CEST
// RCS-ID:      
// Copyright:   
// Licence:     
/////////////////////////////////////////////////////////////////////////////

#ifndef _TESTBENCHGUIAPP_H_
#define _TESTBENCHGUIAPP_H_


/*!
 * Includes
 */

////@begin includes
#include "wx/image.h"
#include "mainframe.h"
////@end includes

/*!
 * Forward declarations
 */

////@begin forward declarations
////@end forward declarations

/*!
 * Control identifiers
 */

////@begin control identifiers
////@end control identifiers

/*!
 * TestbenchGUIApp class declaration
 */

class TestbenchGUIApp: public wxApp
{    
    DECLARE_CLASS( TestbenchGUIApp )
    DECLARE_EVENT_TABLE()

public:
    /// Constructor
    TestbenchGUIApp();

    void Init();

    /// Initialises the application
    virtual bool OnInit();

    /// Called on exit
    virtual int OnExit();

////@begin TestbenchGUIApp event handler declarations

////@end TestbenchGUIApp event handler declarations

////@begin TestbenchGUIApp member function declarations

////@end TestbenchGUIApp member function declarations

////@begin TestbenchGUIApp member variables
////@end TestbenchGUIApp member variables
};

/*!
 * Application instance declaration 
 */

////@begin declare app
DECLARE_APP(TestbenchGUIApp)
////@end declare app

#endif
    // _TESTBENCHGUIAPP_H_
