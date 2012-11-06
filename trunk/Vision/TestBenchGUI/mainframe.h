//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchGUI
// File:           mainframe.hpp
// Description:    Frame for the main application
// Author:         Franc Pape
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

#ifndef _MAINFRAME_H_
#define _MAINFRAME_H_


/*!
 * Includes
 */

////@begin includes
#include "wx/frame.h"
#include "wx/listctrl.h"
////@end includes

#include "Scripts.h"

/*!
 * Forward declarations
 */

////@begin forward declarations
////@end forward declarations

/*!
 * Control identifiers
 */

////@begin control identifiers
#define ID_MAINFRAME 10000
#define PanelTop 10001
#define ButtonAddJob 10004
#define PanelJobs 10003
#define ListJobs 10006
#define ButtonClearJobs 10005
#define ButtonBash 10022
#define ButtonRunJobs 10019
#define SYMBOL_MAINFRAME_STYLE wxCAPTION|wxRESIZE_BORDER|wxSYSTEM_MENU|wxCLOSE_BOX
#define SYMBOL_MAINFRAME_TITLE _("Vision testbench")
#define SYMBOL_MAINFRAME_IDNAME ID_MAINFRAME
#define SYMBOL_MAINFRAME_SIZE wxSize(400, 300)
#define SYMBOL_MAINFRAME_POSITION wxDefaultPosition
////@end control identifiers


/*!
 * MainFrame class declaration
 */

class MainFrame: public wxFrame
{
    DECLARE_CLASS( MainFrame )
    DECLARE_EVENT_TABLE()

public:
    /// Constructors
    MainFrame();
    MainFrame( wxWindow* parent, wxWindowID id = SYMBOL_MAINFRAME_IDNAME, const wxString& caption = SYMBOL_MAINFRAME_TITLE, const wxPoint& pos = SYMBOL_MAINFRAME_POSITION, const wxSize& size = SYMBOL_MAINFRAME_SIZE, long style = SYMBOL_MAINFRAME_STYLE );

    bool Create( wxWindow* parent, wxWindowID id = SYMBOL_MAINFRAME_IDNAME, const wxString& caption = SYMBOL_MAINFRAME_TITLE, const wxPoint& pos = SYMBOL_MAINFRAME_POSITION, const wxSize& size = SYMBOL_MAINFRAME_SIZE, long style = SYMBOL_MAINFRAME_STYLE );

    /// Destructor
    ~MainFrame();

    /// Initialises member variables
    void Init();

    /// Creates the controls and sizers
    void CreateControls();

////@begin MainFrame event handler declarations

    /// wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonAddJob
    void OnButtonAddJobClick( wxCommandEvent& event );

    /// wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonClearJobs
    void OnButtonClearJobsClick( wxCommandEvent& event );

    /// wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonBash
    void OnButtonBashClick( wxCommandEvent& event );

    /// wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonRunJobs
    void OnButtonRunJobsClick( wxCommandEvent& event );

////@end MainFrame event handler declarations

////@begin MainFrame member function declarations

    std::vector<Script> GetScripts() const { return scripts ; }
    void SetScripts(std::vector<Script> value) { scripts = value ; }

    /// Retrieves bitmap resources
    wxBitmap GetBitmapResource( const wxString& name );

    /// Retrieves icon resources
    wxIcon GetIconResource( const wxString& name );
////@end MainFrame member function declarations

    /// Should we show tooltips?
    static bool ShowToolTips();

////@begin MainFrame member variables
public:
    std::vector<Script> scripts;
////@end MainFrame member variables
    std::string callPythonFunc(
        const char* modulePath, const char* moduleName, const char* func, std::vector<Param> params, unsigned int argCount
    );
};

class PythonErr : public std::runtime_error
{
	public:
        PythonErr(const std::string& msg) : std::runtime_error(msg) {}
		~PythonErr(void) throw() {}
};

#endif
    // _MAINFRAME_H_
