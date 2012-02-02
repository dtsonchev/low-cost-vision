//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchGUI
// File:           testbenchguiapp.hpp
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
