//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        TestBenchGUI
// File:           addjobwizard.hpp
// Description:    Wizard for creating a testing job
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

#ifndef _ADDJOBWIZARD_H_
#define _ADDJOBWIZARD_H_


/*!
 * Includes
 */
#include "mainframe.h"
////@begin includes
#include "wx/wizard.h"
#include "wx/statline.h"
////@end includes

#include "Scripts.h"

/*!
 * Forward declarations
 */

class MainFrame;

////@begin forward declarations
class WizardPage;
class WizardPage1;
class WizardPage2;
////@end forward declarations

/*!
 * Control identifiers
 */

////@begin control identifiers
#define ID_ADDJOBWIZARD 10007
#define AjWizardPage1 10008
#define ID_PANEL 10011
#define ChoiceScriptSelect 10009
#define LabelScriptDescription 10012
#define AjWizardPage2 10010
#define TextFieldParams 10002
#define LabelParams 10013
#define AjWizardPage3 10014
#define ID_PANEL1 10015
#define ID_PANEL2 10017
#define TextFieldTestXML 10018
#define ButtonBrowseTestXML 10016
#define ID_PANEL3 10035
#define TextFieldTrainXML 10021
#define ButtonBrowseTrainXML 10020
#define SYMBOL_ADDJOBWIZARD_IDNAME ID_ADDJOBWIZARD
////@end control identifiers

//#define ID_TEXTCTRL 10013

/*!
 * AddJobWizard class declaration
 */

class AddJobWizard: public wxWizard
{
    DECLARE_DYNAMIC_CLASS( AddJobWizard )
    DECLARE_EVENT_TABLE()

public:
    /// Constructors
    AddJobWizard();
    AddJobWizard( wxWindow* parent, wxWindowID id = SYMBOL_ADDJOBWIZARD_IDNAME, const wxPoint& pos = wxDefaultPosition );

    /// Creation
    bool Create( wxWindow* parent, wxWindowID id = SYMBOL_ADDJOBWIZARD_IDNAME, const wxPoint& pos = wxDefaultPosition );

    /// Destructor
    ~AddJobWizard();

    /// Initialises member variables
    void Init();

    /// Creates the controls and sizers
    void CreateControls();

////@begin AddJobWizard event handler declarations

    /// wxEVT_WIZARD_FINISHED event handler for ID_ADDJOBWIZARD
    void OnAddjobwizardFinished( wxWizardEvent& event );

////@end AddJobWizard event handler declarations

////@begin AddJobWizard member function declarations

    /// Runs the wizard
    bool Run();

    std::string GetTrainXmlPath() const { return trainXmlPath ; }
    void SetTrainXmlPath(std::string value) { trainXmlPath = value ; }

    std::string GetTestXmlPath() const { return testXmlPath ; }
    void SetTestXmlPath(std::string value) { testXmlPath = value ; }

    /// Retrieves bitmap resources
    wxBitmap GetBitmapResource( const wxString& name );

    /// Retrieves icon resources
    wxIcon GetIconResource( const wxString& name );
////@end AddJobWizard member function declarations

    /// Should we show tooltips?
    static bool ShowToolTips();

////@begin AddJobWizard member variables
    std::string trainXmlPath;
    std::string testXmlPath;
////@end AddJobWizard member variables
};

/*!
 * WizardPage class declaration
 */

class WizardPage: public wxWizardPageSimple
{
    DECLARE_DYNAMIC_CLASS( WizardPage )
    DECLARE_EVENT_TABLE()

public:
    /// Constructors
    WizardPage();

    WizardPage( wxWizard* parent );

    /// Creation
    bool Create( wxWizard* parent );

    /// Destructor
    ~WizardPage();

    /// Initialises member variables
    void Init();

    /// Creates the controls and sizers
    void CreateControls();

////@begin WizardPage event handler declarations

    /// wxEVT_COMMAND_CHOICE_SELECTED event handler for ChoiceScriptSelect
    void OnChoiceScriptSelectSelected( wxCommandEvent& event );

////@end WizardPage event handler declarations

////@begin WizardPage member function declarations

    /// Retrieves bitmap resources
    wxBitmap GetBitmapResource( const wxString& name );

    /// Retrieves icon resources
    wxIcon GetIconResource( const wxString& name );
////@end WizardPage member function declarations

    /// Should we show tooltips?
    static bool ShowToolTips();

////@begin WizardPage member variables
////@end WizardPage member variables
};

/*!
 * WizardPage1 class declaration
 */

class WizardPage1: public wxWizardPageSimple
{
    DECLARE_DYNAMIC_CLASS( WizardPage1 )
    DECLARE_EVENT_TABLE()

public:
    /// Constructors
    WizardPage1();

    WizardPage1( wxWizard* parent );

    /// Creation
    bool Create( wxWizard* parent );

    /// Destructor
    ~WizardPage1();

    /// Initialises member variables
    void Init();

    /// Creates the controls and sizers
    void CreateControls();

////@begin WizardPage1 event handler declarations

    /// wxEVT_WIZARD_PAGE_CHANGED event handler for AjWizardPage2
    void OnAjWizardPage2PageChanged( wxWizardEvent& event );

    /// wxEVT_WIZARD_PAGE_CHANGING event handler for AjWizardPage2
    void OnAjWizardPage2PageChanging( wxWizardEvent& event );

    /// wxEVT_COMMAND_TEXT_UPDATED event handler for TextFieldParams
    void OnTextFieldParamsTextUpdated( wxCommandEvent& event );

////@end WizardPage1 event handler declarations

////@begin WizardPage1 member function declarations

    /// Retrieves bitmap resources
    wxBitmap GetBitmapResource( const wxString& name );

    /// Retrieves icon resources
    wxIcon GetIconResource( const wxString& name );
////@end WizardPage1 member function declarations

    /// Should we show tooltips?
    static bool ShowToolTips();

////@begin WizardPage1 member variables
////@end WizardPage1 member variables
private:
    void AddParamField( wxBoxSizer* parent, const wxString& name, const wxString& type );
    wxScrolledWindow* ScrollWindowParams;
    wxBoxSizer* Page2Sizer;
};

/*!
 * WizardPage2 class declaration
 */

class WizardPage2: public wxWizardPageSimple
{
    DECLARE_DYNAMIC_CLASS( WizardPage2 )
    DECLARE_EVENT_TABLE()

public:
    /// Constructors
    WizardPage2();

    WizardPage2( wxWizard* parent );

    /// Creation
    bool Create( wxWizard* parent );

    /// Destructor
    ~WizardPage2();

    /// Initialises member variables
    void Init();

    /// Creates the controls and sizers
    void CreateControls();

////@begin WizardPage2 event handler declarations

    /// wxEVT_WIZARD_PAGE_CHANGED event handler for AjWizardPage3
    void OnAjWizardPage3PageChanged( wxWizardEvent& event );

    /// wxEVT_COMMAND_TEXT_UPDATED event handler for TextFieldTestXML
    void OnTextFieldTestXMLTextUpdated( wxCommandEvent& event );

    /// wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonBrowseTestXML
    void OnButtonBrowseTestXMLClick( wxCommandEvent& event );

    /// wxEVT_COMMAND_TEXT_UPDATED event handler for TextFieldTrainXML
    void OnTextFieldTrainXMLTextUpdated( wxCommandEvent& event );

    /// wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonBrowseTrainXML
    void OnButtonBrowseTrainXMLClick( wxCommandEvent& event );

////@end WizardPage2 event handler declarations

////@begin WizardPage2 member function declarations

    /// Retrieves bitmap resources
    wxBitmap GetBitmapResource( const wxString& name );

    /// Retrieves icon resources
    wxIcon GetIconResource( const wxString& name );
////@end WizardPage2 member function declarations

    /// Should we show tooltips?
    static bool ShowToolTips();

////@begin WizardPage2 member variables
////@end WizardPage2 member variables
};

#endif
    // _ADDJOBWIZARD_H_
