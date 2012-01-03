//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchGUI
// File:           filedialog.hpp
// Description:    Dialog for selecting an xml file
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

#ifndef _FILEDIALOG_H_
#define _FILEDIALOG_H_


/*!
 * Includes
 */

////@begin includes
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
#define ID_FILEDIALOG 10017
#define SYMBOL_FILEDIALOG_STYLE wxCAPTION|wxRESIZE_BORDER|wxSYSTEM_MENU|wxCLOSE_BOX|wxTAB_TRAVERSAL
#define SYMBOL_FILEDIALOG_TITLE _("filedialog")
#define SYMBOL_FILEDIALOG_IDNAME ID_FILEDIALOG
#define SYMBOL_FILEDIALOG_SIZE wxSize(400, 300)
#define SYMBOL_FILEDIALOG_POSITION wxDefaultPosition
////@end control identifiers


/*!
 * filedialog class declaration
 */

class filedialog: public wxDialog
{
    DECLARE_DYNAMIC_CLASS( filedialog )
    DECLARE_EVENT_TABLE()

public:
    /// Constructors
    filedialog();
    filedialog( wxWindow* parent, wxWindowID id = SYMBOL_FILEDIALOG_IDNAME, const wxString& caption = SYMBOL_FILEDIALOG_TITLE, const wxPoint& pos = SYMBOL_FILEDIALOG_POSITION, const wxSize& size = SYMBOL_FILEDIALOG_SIZE, long style = SYMBOL_FILEDIALOG_STYLE );

    /// Creation
    bool Create( wxWindow* parent, wxWindowID id = SYMBOL_FILEDIALOG_IDNAME, const wxString& caption = SYMBOL_FILEDIALOG_TITLE, const wxPoint& pos = SYMBOL_FILEDIALOG_POSITION, const wxSize& size = SYMBOL_FILEDIALOG_SIZE, long style = SYMBOL_FILEDIALOG_STYLE );

    /// Destructor
    ~filedialog();

    /// Initialises member variables
    void Init();

    /// Creates the controls and sizers
    void CreateControls();

////@begin filedialog event handler declarations
////@end filedialog event handler declarations

////@begin filedialog member function declarations
    /// Retrieves bitmap resources
    wxBitmap GetBitmapResource( const wxString& name );

    /// Retrieves icon resources
    wxIcon GetIconResource( const wxString& name );
////@end filedialog member function declarations

    /// Should we show tooltips?
    static bool ShowToolTips();

////@begin filedialog member variables
////@end filedialog member variables
};

#endif
    // _FILEDIALOG_H_
