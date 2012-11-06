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
