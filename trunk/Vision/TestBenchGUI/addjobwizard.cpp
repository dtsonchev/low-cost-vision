//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchGUI
// File:           addjobwizard.cpp
// Description:    Wizard for creating a testing job
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

#include "addjobwizard.h"
#include <boost/algorithm/string.hpp>

unsigned int scriptIndex;
std::vector<Script> scripts;

////@begin XPM images
////@end XPM images


/*
 * AddJobWizard type definition
 */

IMPLEMENT_DYNAMIC_CLASS( AddJobWizard, wxWizard )


/*
 * AddJobWizard event table definition
 */

BEGIN_EVENT_TABLE( AddJobWizard, wxWizard )

////@begin AddJobWizard event table entries
    EVT_WIZARD_FINISHED( ID_ADDJOBWIZARD, AddJobWizard::OnAddjobwizardFinished )

////@end AddJobWizard event table entries

END_EVENT_TABLE()


/*
 * AddJobWizard constructors
 */

AddJobWizard::AddJobWizard()
{
    Init();
}

AddJobWizard::AddJobWizard( wxWindow* parent, wxWindowID id, const wxPoint& pos )
{
    Init();
    Create(parent, id, pos);
}


/*
 * AddJobWizard creator
 */

bool AddJobWizard::Create( wxWindow* parent, wxWindowID id, const wxPoint& pos )
{
////@begin AddJobWizard creation
    SetExtraStyle(wxWS_EX_BLOCK_EVENTS);
    wxBitmap wizardBitmap(GetBitmapResource(wxT("wizard.png")));
    wxWizard::Create( parent, id, _("Add job"), wizardBitmap, pos, wxDEFAULT_DIALOG_STYLE|wxCAPTION|wxSYSTEM_MENU|wxCLOSE_BOX );

    CreateControls();
////@end AddJobWizard creation
    return true;
}


/*
 * AddJobWizard destructor
 */

AddJobWizard::~AddJobWizard()
{
////@begin AddJobWizard destruction
////@end AddJobWizard destruction
}


/*
 * Member initialisation
 */

void AddJobWizard::Init()
{
////@begin AddJobWizard member initialisation
    trainXmlPath = "";
    testXmlPath = "";
////@end AddJobWizard member initialisation
}


/*
 * Control creation for AddJobWizard
 */

void AddJobWizard::CreateControls()
{
////@begin AddJobWizard content construction
    AddJobWizard* itemWizard1 = this;

    WizardPage* itemWizardPageSimple2 = new WizardPage( itemWizard1 );
    itemWizard1->GetPageAreaSizer()->Add(itemWizardPageSimple2);

    WizardPage1* itemWizardPageSimple12 = new WizardPage1( itemWizard1 );
    itemWizard1->GetPageAreaSizer()->Add(itemWizardPageSimple12);

    WizardPage2* itemWizardPageSimple19 = new WizardPage2( itemWizard1 );
    itemWizard1->GetPageAreaSizer()->Add(itemWizardPageSimple19);

    wxWizardPageSimple* lastPage = NULL;
    if (lastPage)
        wxWizardPageSimple::Chain(lastPage, itemWizardPageSimple2);
    lastPage = itemWizardPageSimple2;
    if (lastPage)
        wxWizardPageSimple::Chain(lastPage, itemWizardPageSimple12);
    lastPage = itemWizardPageSimple12;
    if (lastPage)
        wxWizardPageSimple::Chain(lastPage, itemWizardPageSimple19);
    lastPage = itemWizardPageSimple19;
////@end AddJobWizard content construction
}


/*
 * Runs the wizard.
 */

bool AddJobWizard::Run()
{
    wxWindowList::compatibility_iterator node = GetChildren().GetFirst();
    while (node)
    {
        wxWizardPage* startPage = wxDynamicCast(node->GetData(), wxWizardPage);
        if (startPage) return RunWizard(startPage);
        node = node->GetNext();
    }
    return false;
}


/*
 * Should we show tooltips?
 */

bool AddJobWizard::ShowToolTips()
{
    return true;
}

/*
 * Get bitmap resources
 */

wxBitmap AddJobWizard::GetBitmapResource( const wxString& name )
{
    // Bitmap retrieval
////@begin AddJobWizard bitmap retrieval
    wxUnusedVar(name);
    if (name == _T("wizard.png"))
    {
        wxBitmap bitmap(_T("wizard.png"), wxBITMAP_TYPE_PNG);
        return bitmap;
    }
    return wxNullBitmap;
////@end AddJobWizard bitmap retrieval
}

/*
 * Get icon resources
 */

wxIcon AddJobWizard::GetIconResource( const wxString& name )
{
    // Icon retrieval
////@begin AddJobWizard icon retrieval
    wxUnusedVar(name);
    return wxNullIcon;
////@end AddJobWizard icon retrieval
}


/*
 * WizardPage type definition
 */

IMPLEMENT_DYNAMIC_CLASS( WizardPage, wxWizardPageSimple )


/*
 * WizardPage event table definition
 */

BEGIN_EVENT_TABLE( WizardPage, wxWizardPageSimple )

////@begin WizardPage event table entries
    EVT_CHOICE( ChoiceScriptSelect, WizardPage::OnChoiceScriptSelectSelected )

////@end WizardPage event table entries

END_EVENT_TABLE()


/*
 * WizardPage constructors
 */

WizardPage::WizardPage()
{
    Init();
}

WizardPage::WizardPage( wxWizard* parent )
{
    Init();
    Create( parent );
}


/*
 * WizardPage creator
 */

bool WizardPage::Create( wxWizard* parent )
{
////@begin WizardPage creation
    wxBitmap wizardBitmap(wxNullBitmap);
    wxWizardPageSimple::Create( parent, NULL, NULL, wizardBitmap );

    CreateControls();
    if (GetSizer())
        GetSizer()->Fit(this);
////@end WizardPage creation
    return true;
}


/*
 * WizardPage destructor
 */

WizardPage::~WizardPage()
{
////@begin WizardPage destruction
////@end WizardPage destruction
}


/*
 * Member initialisation
 */

void WizardPage::Init()
{
////@begin WizardPage member initialisation
////@end WizardPage member initialisation

    ScriptParser sp;
    scripts = sp.loadScripts("scripts.xml");
}


/*
 * Control creation for WizardPage
 */

void WizardPage::CreateControls()
{
////@begin WizardPage content construction
    WizardPage* itemWizardPageSimple2 = this;

    wxBoxSizer* itemBoxSizer3 = new wxBoxSizer(wxVERTICAL);
    itemWizardPageSimple2->SetSizer(itemBoxSizer3);

    wxStaticText* itemStaticText4 = new wxStaticText( itemWizardPageSimple2, wxID_STATIC, _("Script selection"), wxDefaultPosition, wxDefaultSize, 0 );
    itemStaticText4->SetFont(wxFont(16, wxDEFAULT, wxNORMAL, wxNORMAL, false, wxT("Ubuntu")));
    itemBoxSizer3->Add(itemStaticText4, 0, wxALIGN_LEFT|wxALL, 5);

    wxStaticLine* itemStaticLine5 = new wxStaticLine( itemWizardPageSimple2, wxID_STATIC, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
    itemBoxSizer3->Add(itemStaticLine5, 0, wxGROW|wxALL, 5);

    wxPanel* itemPanel6 = new wxPanel( itemWizardPageSimple2, ID_PANEL, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTAB_TRAVERSAL );
    itemBoxSizer3->Add(itemPanel6, 0, wxALIGN_LEFT, 5);

    wxBoxSizer* itemBoxSizer7 = new wxBoxSizer(wxHORIZONTAL);
    itemPanel6->SetSizer(itemBoxSizer7);

    wxStaticText* itemStaticText8 = new wxStaticText( itemPanel6, wxID_STATIC, _("Select script"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer7->Add(itemStaticText8, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxArrayString itemChoice9Strings;
    wxChoice* itemChoice9 = new wxChoice( itemPanel6, ChoiceScriptSelect, wxDefaultPosition, wxDefaultSize, itemChoice9Strings, 0 );
    itemBoxSizer7->Add(itemChoice9, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxStaticBox* itemStaticBoxSizer10Static = new wxStaticBox(itemWizardPageSimple2, wxID_ANY, _("Description"));
    wxStaticBoxSizer* itemStaticBoxSizer10 = new wxStaticBoxSizer(itemStaticBoxSizer10Static, wxVERTICAL);
    itemBoxSizer3->Add(itemStaticBoxSizer10, 1, wxGROW|wxALL, 5);

    wxStaticText* itemStaticText11 = new wxStaticText( itemWizardPageSimple2, LabelScriptDescription, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
    itemStaticText11->Wrap(300);
    itemStaticBoxSizer10->Add(itemStaticText11, 1, wxGROW|wxALL, 5);

////@end WizardPage content construction
    wxChoice* ch = (wxChoice*)FindWindowById(ChoiceScriptSelect);
    for(unsigned int i = 0; i < scripts.size(); i++){
        ch->Append(wxString(scripts[i].name.c_str(), wxConvUTF8));
    }
    scriptIndex = 0;
    ch->SetSelection(scriptIndex);
    wxStaticText* txt = (wxStaticText*)FindWindowById(LabelScriptDescription);
    txt->SetLabel(wxString(scripts[scriptIndex].description.c_str(), wxConvUTF8));
    txt->Wrap(250);
}


/*
 * Should we show tooltips?
 */

bool WizardPage::ShowToolTips()
{
    return true;
}

/*
 * Get bitmap resources
 */

wxBitmap WizardPage::GetBitmapResource( const wxString& name )
{
    // Bitmap retrieval
////@begin WizardPage bitmap retrieval
    wxUnusedVar(name);
    return wxNullBitmap;
////@end WizardPage bitmap retrieval
}

/*
 * Get icon resources
 */

wxIcon WizardPage::GetIconResource( const wxString& name )
{
    // Icon retrieval
////@begin WizardPage icon retrieval
    wxUnusedVar(name);
    return wxNullIcon;
////@end WizardPage icon retrieval
}


/*
 * WizardPage1 type definition
 */

IMPLEMENT_DYNAMIC_CLASS( WizardPage1, wxWizardPageSimple )


/*
 * WizardPage1 event table definition
 */

BEGIN_EVENT_TABLE( WizardPage1, wxWizardPageSimple )

////@begin WizardPage1 event table entries
    EVT_WIZARD_PAGE_CHANGED( -1, WizardPage1::OnAjWizardPage2PageChanged )
    EVT_WIZARD_PAGE_CHANGING( -1, WizardPage1::OnAjWizardPage2PageChanging )

    EVT_TEXT( TextFieldParams, WizardPage1::OnTextFieldParamsTextUpdated )

////@end WizardPage1 event table entries

END_EVENT_TABLE()


/*
 * WizardPage1 constructors
 */

WizardPage1::WizardPage1()
{
    Init();
}

WizardPage1::WizardPage1( wxWizard* parent )
{
    Init();
    Create( parent );
}


/*
 * WizardPage1 creator
 */

bool WizardPage1::Create( wxWizard* parent )
{
////@begin WizardPage1 creation
    wxBitmap wizardBitmap(wxNullBitmap);
    wxWizardPageSimple::Create( parent, NULL, NULL, wizardBitmap );

    CreateControls();
    if (GetSizer())
        GetSizer()->Fit(this);
////@end WizardPage1 creation
    return true;
}


/*
 * WizardPage1 destructor
 */

WizardPage1::~WizardPage1()
{
////@begin WizardPage1 destruction
////@end WizardPage1 destruction
}


/*
 * Member initialisation
 */

void WizardPage1::Init()
{
////@begin WizardPage1 member initialisation
////@end WizardPage1 member initialisation
    std::cout << "WizardPage1::Init" << std::endl;
}


/*
 * Control creation for WizardPage1
 */

void WizardPage1::CreateControls()
{
////@begin WizardPage1 content construction
    WizardPage1* itemWizardPageSimple12 = this;

    wxBoxSizer* itemBoxSizer13 = new wxBoxSizer(wxVERTICAL);
    itemWizardPageSimple12->SetSizer(itemBoxSizer13);

    wxStaticText* itemStaticText14 = new wxStaticText( itemWizardPageSimple12, wxID_STATIC, _("Script parameters"), wxDefaultPosition, wxDefaultSize, 0 );
    itemStaticText14->SetFont(wxFont(16, wxDEFAULT, wxNORMAL, wxNORMAL, false, wxT("Ubuntu")));
    itemBoxSizer13->Add(itemStaticText14, 0, wxALIGN_LEFT|wxALL, 5);

    wxStaticLine* itemStaticLine15 = new wxStaticLine( itemWizardPageSimple12, wxID_STATIC, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
    itemBoxSizer13->Add(itemStaticLine15, 0, wxGROW|wxALL, 5);

    wxStaticText* itemStaticText16 = new wxStaticText( itemWizardPageSimple12, wxID_STATIC, _("Enter parameters, seperated by ';'"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer13->Add(itemStaticText16, 0, wxALIGN_LEFT|wxALL, 5);

    wxTextCtrl* itemTextCtrl17 = new wxTextCtrl( itemWizardPageSimple12, TextFieldParams, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer13->Add(itemTextCtrl17, 0, wxGROW|wxALL, 5);

    wxStaticText* itemStaticText18 = new wxStaticText( itemWizardPageSimple12, LabelParams, _("var1;var2"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer13->Add(itemStaticText18, 0, wxALIGN_LEFT|wxALL, 5);

////@end WizardPage1 content construction
}


/*
 * Should we show tooltips?
 */

bool WizardPage1::ShowToolTips()
{
    return true;
}

/*
 * Get bitmap resources
 */

wxBitmap WizardPage1::GetBitmapResource( const wxString& name )
{
    // Bitmap retrieval
////@begin WizardPage1 bitmap retrieval
    wxUnusedVar(name);
    return wxNullBitmap;
////@end WizardPage1 bitmap retrieval
}

/*
 * Get icon resources
 */

wxIcon WizardPage1::GetIconResource( const wxString& name )
{
    // Icon retrieval
////@begin WizardPage1 icon retrieval
    wxUnusedVar(name);
    return wxNullIcon;
////@end WizardPage1 icon retrieval
}

void WizardPage1::AddParamField(wxBoxSizer* parent, const wxString& name, const wxString& type ){
    wxString s = name + _("(") + type + _("):");
    wxStaticText* itemStaticText17 = new wxStaticText( this, wxID_ANY, s, wxDefaultPosition, wxDefaultSize, 0 );
    parent->Add(itemStaticText17, 0, wxALIGN_LEFT|wxALL, 5);

    wxTextCtrl* itemTextCtrl18 = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
    parent->Add(itemTextCtrl18, 0, wxALIGN_LEFT|wxALL|wxEXPAND, 5);
}

/*
 * wxEVT_COMMAND_CHOICE_SELECTED event handler for ChoiceScriptSelect
 */

void WizardPage::OnChoiceScriptSelectSelected( wxCommandEvent& event )
{
    wxStaticText* txt = (wxStaticText*)FindWindowById(LabelScriptDescription);
    scriptIndex = event.GetSelection();
    txt->SetLabel(wxString(scripts[scriptIndex].description.c_str(), wxConvUTF8));
    txt->Wrap(250);
}





/*
 * WizardPage2 type definition
 */

IMPLEMENT_DYNAMIC_CLASS( WizardPage2, wxWizardPageSimple )


/*
 * WizardPage2 event table definition
 */

BEGIN_EVENT_TABLE( WizardPage2, wxWizardPageSimple )

////@begin WizardPage2 event table entries
    EVT_WIZARD_PAGE_CHANGED( -1, WizardPage2::OnAjWizardPage3PageChanged )

    EVT_TEXT( TextFieldTestXML, WizardPage2::OnTextFieldTestXMLTextUpdated )

    EVT_BUTTON( ButtonBrowseTestXML, WizardPage2::OnButtonBrowseTestXMLClick )

    EVT_TEXT( TextFieldTrainXML, WizardPage2::OnTextFieldTrainXMLTextUpdated )

    EVT_BUTTON( ButtonBrowseTrainXML, WizardPage2::OnButtonBrowseTrainXMLClick )

////@end WizardPage2 event table entries

END_EVENT_TABLE()


/*
 * WizardPage2 constructors
 */

WizardPage2::WizardPage2()
{
    Init();
}

WizardPage2::WizardPage2( wxWizard* parent )
{
    Init();
    Create( parent );
}


/*
 * WizardPage2 creator
 */

bool WizardPage2::Create( wxWizard* parent )
{
////@begin WizardPage2 creation
    wxBitmap wizardBitmap(wxNullBitmap);
    wxWizardPageSimple::Create( parent, NULL, NULL, wizardBitmap );

    CreateControls();
    if (GetSizer())
        GetSizer()->Fit(this);
////@end WizardPage2 creation
    return true;
}


/*
 * WizardPage2 destructor
 */

WizardPage2::~WizardPage2()
{
////@begin WizardPage2 destruction
////@end WizardPage2 destruction
}


/*
 * Member initialisation
 */

void WizardPage2::Init()
{
////@begin WizardPage2 member initialisation
////@end WizardPage2 member initialisation
}


/*
 * Control creation for WizardPage2
 */

void WizardPage2::CreateControls()
{
////@begin WizardPage2 content construction
    WizardPage2* itemWizardPageSimple19 = this;

    wxBoxSizer* itemBoxSizer20 = new wxBoxSizer(wxVERTICAL);
    itemWizardPageSimple19->SetSizer(itemBoxSizer20);

    wxStaticText* itemStaticText21 = new wxStaticText( itemWizardPageSimple19, wxID_STATIC, _("XML Selection"), wxDefaultPosition, wxDefaultSize, 0 );
    itemStaticText21->SetFont(wxFont(16, wxDEFAULT, wxNORMAL, wxNORMAL, false, wxT("Ubuntu")));
    itemBoxSizer20->Add(itemStaticText21, 0, wxALIGN_LEFT|wxALL, 5);

    wxStaticLine* itemStaticLine22 = new wxStaticLine( itemWizardPageSimple19, wxID_STATIC, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL );
    itemBoxSizer20->Add(itemStaticLine22, 0, wxGROW|wxALL, 5);

    wxPanel* itemPanel23 = new wxPanel( itemWizardPageSimple19, ID_PANEL1, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTAB_TRAVERSAL );
    itemBoxSizer20->Add(itemPanel23, 1, wxGROW, 5);

    wxBoxSizer* itemBoxSizer24 = new wxBoxSizer(wxVERTICAL);
    itemPanel23->SetSizer(itemBoxSizer24);

    wxStaticText* itemStaticText25 = new wxStaticText( itemPanel23, wxID_STATIC, _("Testing XML:"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer24->Add(itemStaticText25, 0, wxALIGN_LEFT|wxALL, 5);

    wxPanel* itemPanel26 = new wxPanel( itemPanel23, ID_PANEL2, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTAB_TRAVERSAL );
    itemBoxSizer24->Add(itemPanel26, 0, wxGROW, 5);

    wxBoxSizer* itemBoxSizer27 = new wxBoxSizer(wxHORIZONTAL);
    itemPanel26->SetSizer(itemBoxSizer27);

    wxTextCtrl* itemTextCtrl28 = new wxTextCtrl( itemPanel26, TextFieldTestXML, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer27->Add(itemTextCtrl28, 1, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxButton* itemButton29 = new wxButton( itemPanel26, ButtonBrowseTestXML, _("Browse..."), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer27->Add(itemButton29, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxStaticText* itemStaticText30 = new wxStaticText( itemPanel23, wxID_STATIC, _("Training XML"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer24->Add(itemStaticText30, 0, wxALIGN_LEFT|wxALL, 5);

    wxPanel* itemPanel31 = new wxPanel( itemPanel23, ID_PANEL3, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTAB_TRAVERSAL );
    itemBoxSizer24->Add(itemPanel31, 0, wxGROW, 5);

    wxBoxSizer* itemBoxSizer32 = new wxBoxSizer(wxHORIZONTAL);
    itemPanel31->SetSizer(itemBoxSizer32);

    wxTextCtrl* itemTextCtrl33 = new wxTextCtrl( itemPanel31, TextFieldTrainXML, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
    itemTextCtrl33->Enable(false);
    itemBoxSizer32->Add(itemTextCtrl33, 1, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxButton* itemButton34 = new wxButton( itemPanel31, ButtonBrowseTrainXML, _("Browse..."), wxDefaultPosition, wxDefaultSize, 0 );
    itemButton34->Enable(false);
    itemBoxSizer32->Add(itemButton34, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

////@end WizardPage2 content construction
}


/*
 * Should we show tooltips?
 */

bool WizardPage2::ShowToolTips()
{
    return true;
}

/*
 * Get bitmap resources
 */

wxBitmap WizardPage2::GetBitmapResource( const wxString& name )
{
    // Bitmap retrieval
////@begin WizardPage2 bitmap retrieval
    wxUnusedVar(name);
    return wxNullBitmap;
////@end WizardPage2 bitmap retrieval
}

/*
 * Get icon resources
 */

wxIcon WizardPage2::GetIconResource( const wxString& name )
{
    // Icon retrieval
////@begin WizardPage2 icon retrieval
    wxUnusedVar(name);
    return wxNullIcon;
////@end WizardPage2 icon retrieval
}



/*
 * wxEVT_WIZARD_FINISHED event handler for ID_ADDJOBWIZARD
 */

void AddJobWizard::OnAddjobwizardFinished( wxWizardEvent& event )
{
    std::cout << "OnAddjobwizardFinished: " << "start" << std::endl;

    Script s(scripts[scriptIndex].path, scripts[scriptIndex].name, scripts[scriptIndex].training,
        scripts[scriptIndex].description, scripts[scriptIndex].python, scripts[scriptIndex].params);

    if(s.training)
        s.params.push_back(Param("trainXmlPath", "string", trainXmlPath));
    s.params.push_back(Param("testXmlPath", "string", testXmlPath));

    MainFrame* mf = (MainFrame*)GetParent();
    mf->scripts.push_back(s);

    std::cout << "OnAddjobwizardFinished: " << "end" << std::endl;
}


/*
 * wxEVT_COMMAND_TEXT_UPDATED event handler for TextFieldTrainXML
 */

void WizardPage2::OnTextFieldTrainXMLTextUpdated( wxCommandEvent& event )
{
    AddJobWizard* ajw = (AddJobWizard*)GetParent();
    wxTextCtrl* txt = (wxTextCtrl*)FindWindowById(TextFieldTrainXML);
    ajw->SetTrainXmlPath(std::string(txt->GetValue().mb_str()));
}


/*
 * wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonBrowseTrainXML
 */

void WizardPage2::OnButtonBrowseTrainXMLClick( wxCommandEvent& event )
{
    wxFileDialog openFileDialog(this, _("Open XML file"), _(""), _(""), _("XML files (*.xml)|*.xml"), wxFD_OPEN|wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return;

    wxTextCtrl* txt = (wxTextCtrl*)FindWindowById(TextFieldTrainXML);
    txt->SetValue(openFileDialog.GetPath());
}


/*
 * wxEVT_WIZARD_PAGE_CHANGED event handler for AjWizardPage3
 */

void WizardPage2::OnAjWizardPage3PageChanged( wxWizardEvent& event )
{
    wxWindow* txt = FindWindowById(TextFieldTrainXML);
    wxWindow* but = FindWindowById(ButtonBrowseTrainXML);

    if(scripts[scriptIndex].training){
        txt->Enable();
        but->Enable();
    } else {
        txt->Disable();
        but->Disable();
    }
}


/*
 * wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonBrowseTestXML
 */

void WizardPage2::OnButtonBrowseTestXMLClick( wxCommandEvent& event )
{
    wxFileDialog openFileDialog(this, _("Open XML file"), _(""), _(""), _("XML files (*.xml)|*.xml"), wxFD_OPEN|wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return;

    wxTextCtrl* txt = (wxTextCtrl*)FindWindowById(TextFieldTestXML);
    txt->SetValue(openFileDialog.GetPath());
}


/*
 * wxEVT_COMMAND_TEXT_UPDATED event handler for TextFieldTestXML
 */

void WizardPage2::OnTextFieldTestXMLTextUpdated( wxCommandEvent& event )
{
    AddJobWizard* ajw = (AddJobWizard*)GetParent();
    wxTextCtrl* txt = (wxTextCtrl*)FindWindowById(TextFieldTestXML);
    ajw->SetTestXmlPath(std::string(txt->GetValue().mb_str()));
}


/*
 * wxEVT_WIZARD_PAGE_CHANGED event handler for AjWizardPage2
 */

void WizardPage1::OnAjWizardPage2PageChanged( wxWizardEvent& event )
{
    std::string s;
    for(unsigned int i = 0; i < (scripts[scriptIndex].params.size() - 1); i++){
        s += scripts[scriptIndex].params[i].name + "; ";
    }
    s += scripts[scriptIndex].params[scripts[scriptIndex].params.size() - 1].name;

    wxStaticText* txt = (wxStaticText*)FindWindowById(LabelParams);
    txt->SetLabel(wxString(s.c_str(), wxConvUTF8));
    txt->Wrap(250);
}


/*
 * wxEVT_COMMAND_TEXT_UPDATED event handler for TextFieldParams
 */

void WizardPage1::OnTextFieldParamsTextUpdated( wxCommandEvent& event )
{

}


/*
 * wxEVT_WIZARD_PAGE_CHANGING event handler for AjWizardPage2
 */

void WizardPage1::OnAjWizardPage2PageChanging( wxWizardEvent& event )
{
    wxTextCtrl* txt = (wxTextCtrl*)FindWindowById(TextFieldParams);
    std::string s = std::string(txt->GetValue().mb_str());
    boost::erase_all(s, " ");

    std::vector<std::string> params;
    boost::split(params, s,  boost::is_any_of(";"));

    if(params.size() != scripts[scriptIndex].params.size()){
        event.Veto();
        std::stringstream ss;
        ss << "Please provide " << scripts[scriptIndex].params.size() << " parameters";
        wxMessageBox(wxString(ss.str().c_str(), wxConvUTF8), _("Incorrect number of parameters"), wxOK, this);
        return;
    }

    for(unsigned int i = 0; i < params.size(); i++){
        scripts[scriptIndex].params[i].value = params[i];
    }
}

