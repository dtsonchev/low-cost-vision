/////////////////////////////////////////////////////////////////////////////
// Name:        filedialog.cpp
// Purpose:     
// Author:      Franc
// Modified by: 
// Created:     Tue 18 Oct 2011 14:52:00 CEST
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

#include "filedialog.h"

////@begin XPM images

////@end XPM images


/*
 * filedialog type definition
 */

IMPLEMENT_DYNAMIC_CLASS( filedialog, wxDialog )


/*
 * filedialog event table definition
 */

BEGIN_EVENT_TABLE( filedialog, wxDialog )

////@begin filedialog event table entries
////@end filedialog event table entries

END_EVENT_TABLE()


/*
 * filedialog constructors
 */

filedialog::filedialog()
{
    Init();
}

filedialog::filedialog( wxWindow* parent, wxWindowID id, const wxString& caption, const wxPoint& pos, const wxSize& size, long style )
{
    Init();
    Create(parent, id, caption, pos, size, style);
}


/*
 * filedialog creator
 */

bool filedialog::Create( wxWindow* parent, wxWindowID id, const wxString& caption, const wxPoint& pos, const wxSize& size, long style )
{
////@begin filedialog creation
    SetExtraStyle(wxWS_EX_BLOCK_EVENTS);
    wxDialog::Create( parent, id, caption, pos, size, style );

    CreateControls();
    if (GetSizer())
    {
        GetSizer()->SetSizeHints(this);
    }
    Centre();
////@end filedialog creation
    return true;
}


/*
 * filedialog destructor
 */

filedialog::~filedialog()
{
////@begin filedialog destruction
////@end filedialog destruction
}


/*
 * Member initialisation
 */

void filedialog::Init()
{
////@begin filedialog member initialisation
////@end filedialog member initialisation
}


/*
 * Control creation for filedialog
 */

void filedialog::CreateControls()
{    
////@begin filedialog content construction
    filedialog* itemDialog1 = this;

    wxBoxSizer* itemBoxSizer2 = new wxBoxSizer(wxVERTICAL);
    itemDialog1->SetSizer(itemBoxSizer2);

////@end filedialog content construction
}


/*
 * Should we show tooltips?
 */

bool filedialog::ShowToolTips()
{
    return true;
}

/*
 * Get bitmap resources
 */

wxBitmap filedialog::GetBitmapResource( const wxString& name )
{
    // Bitmap retrieval
////@begin filedialog bitmap retrieval
    wxUnusedVar(name);
    return wxNullBitmap;
////@end filedialog bitmap retrieval
}

/*
 * Get icon resources
 */

wxIcon filedialog::GetIconResource( const wxString& name )
{
    // Icon retrieval
////@begin filedialog icon retrieval
    wxUnusedVar(name);
    return wxNullIcon;
////@end filedialog icon retrieval
}
