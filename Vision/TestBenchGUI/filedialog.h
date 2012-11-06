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
