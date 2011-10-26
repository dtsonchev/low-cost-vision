/////////////////////////////////////////////////////////////////////////////
// Name:        mainframe.h
// Purpose:     
// Author:      Franc
// Modified by: 
// Created:     Fri 14 Oct 2011 10:41:09 CEST
// RCS-ID:      
// Copyright:   
// Licence:     
/////////////////////////////////////////////////////////////////////////////

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
