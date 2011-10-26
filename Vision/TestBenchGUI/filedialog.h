/////////////////////////////////////////////////////////////////////////////
// Name:        filedialog.h
// Purpose:     
// Author:      Franc
// Modified by: 
// Created:     Tue 18 Oct 2011 14:52:00 CEST
// RCS-ID:      
// Copyright:   
// Licence:     
/////////////////////////////////////////////////////////////////////////////

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
