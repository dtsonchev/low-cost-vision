//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchGUI
// File:           filedialog.cpp
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
