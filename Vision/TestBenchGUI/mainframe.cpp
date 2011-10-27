/////////////////////////////////////////////////////////////////////////////
// Name:        mainframe.cpp
// Purpose:     
// Author:      Franc Pape
// Modified by: 
// Created:     Fri 14 Oct 2011 10:41:09 CEST
// RCS-ID:      
// Copyright:   
// Licence:     
/////////////////////////////////////////////////////////////////////////////

#include <Python.h>
// For compilers that support precompilation, includes "wx/wx.h".
#include "wx/wxprec.h"

#ifdef __BORLANDC__
#pragma hdrstop
#endif

#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

////@begin includes
#include "addjobwizard.h"
#include "wx/imaglist.h"
////@end includes

#include "mainframe.h"
#include "boost/filesystem.hpp"
#include <fstream>

////@begin XPM images
////@end XPM images


/*
 * MainFrame type definition
 */

IMPLEMENT_CLASS( MainFrame, wxFrame )


/*
 * MainFrame event table definition
 */

BEGIN_EVENT_TABLE( MainFrame, wxFrame )

////@begin MainFrame event table entries
    EVT_BUTTON( ButtonAddJob, MainFrame::OnButtonAddJobClick )

    EVT_BUTTON( ButtonClearJobs, MainFrame::OnButtonClearJobsClick )

    EVT_BUTTON( ButtonBash, MainFrame::OnButtonBashClick )

    EVT_BUTTON( ButtonRunJobs, MainFrame::OnButtonRunJobsClick )

////@end MainFrame event table entries

END_EVENT_TABLE()


/*
 * MainFrame constructors
 */

MainFrame::MainFrame()
{
    Init();
}

MainFrame::MainFrame( wxWindow* parent, wxWindowID id, const wxString& caption, const wxPoint& pos, const wxSize& size, long style )
{
    Init();
    Create( parent, id, caption, pos, size, style );
}


/*
 * MainFrame creator
 */

bool MainFrame::Create( wxWindow* parent, wxWindowID id, const wxString& caption, const wxPoint& pos, const wxSize& size, long style )
{
////@begin MainFrame creation
    wxFrame::Create( parent, id, caption, pos, size, style );

    CreateControls();
    Centre();
////@end MainFrame creation
    return true;
}


/*
 * MainFrame destructor
 */

MainFrame::~MainFrame()
{
////@begin MainFrame destruction
////@end MainFrame destruction
}


/*
 * Member initialisation
 */

void MainFrame::Init()
{
////@begin MainFrame member initialisation
////@end MainFrame member initialisation
}


/*
 * Control creation for MainFrame
 */

void MainFrame::CreateControls()
{    
////@begin MainFrame content construction
    MainFrame* itemFrame1 = this;

    wxBoxSizer* itemBoxSizer2 = new wxBoxSizer(wxVERTICAL);
    itemFrame1->SetSizer(itemBoxSizer2);

    wxPanel* itemPanel3 = new wxPanel( itemFrame1, PanelTop, wxDefaultPosition, wxDefaultSize, wxNO_BORDER|wxTAB_TRAVERSAL );
    itemBoxSizer2->Add(itemPanel3, 0, wxGROW|wxALL, 5);

    wxBoxSizer* itemBoxSizer4 = new wxBoxSizer(wxHORIZONTAL);
    itemPanel3->SetSizer(itemBoxSizer4);

    wxButton* itemButton5 = new wxButton( itemPanel3, ButtonAddJob, _("Add job"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer4->Add(itemButton5, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxStaticBox* itemStaticBoxSizer6Static = new wxStaticBox(itemFrame1, PanelJobs, _("Jobs"));
    wxStaticBoxSizer* itemStaticBoxSizer6 = new wxStaticBoxSizer(itemStaticBoxSizer6Static, wxVERTICAL);
    itemBoxSizer2->Add(itemStaticBoxSizer6, 1, wxGROW|wxALL, 5);

    wxListCtrl* itemListCtrl7 = new wxListCtrl( itemFrame1, ListJobs, wxDefaultPosition, wxSize(100, 100), wxLC_REPORT|wxLC_SINGLE_SEL|wxSUNKEN_BORDER );
    itemStaticBoxSizer6->Add(itemListCtrl7, 1, wxGROW|wxALL, 5);

    wxBoxSizer* itemBoxSizer8 = new wxBoxSizer(wxHORIZONTAL);
    itemStaticBoxSizer6->Add(itemBoxSizer8, 0, wxGROW, 5);

    wxButton* itemButton9 = new wxButton( itemFrame1, ButtonClearJobs, _("Clear jobs"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer8->Add(itemButton9, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxButton* itemButton10 = new wxButton( itemFrame1, ButtonBash, _("Generate bash"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer8->Add(itemButton10, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    itemBoxSizer8->Add(5, 5, 1, wxALIGN_CENTER_VERTICAL|wxALL, 5);

    wxButton* itemButton12 = new wxButton( itemFrame1, ButtonRunJobs, _("Run"), wxDefaultPosition, wxDefaultSize, 0 );
    itemBoxSizer8->Add(itemButton12, 0, wxALIGN_CENTER_VERTICAL|wxALL, 5);

////@end MainFrame content construction

    wxListCtrl* lj = (wxListCtrl*)FindWindowById(ListJobs);
    lj->InsertColumn(0, _("Name"));
    lj->InsertColumn(1, _("Test"));
    lj->InsertColumn(2, _("Train"));
    
    //scripts = new 
}


/*
 * Should we show tooltips?
 */

bool MainFrame::ShowToolTips()
{
    return true;
}

/*
 * Get bitmap resources
 */

wxBitmap MainFrame::GetBitmapResource( const wxString& name )
{
    // Bitmap retrieval
////@begin MainFrame bitmap retrieval
    wxUnusedVar(name);
    return wxNullBitmap;
////@end MainFrame bitmap retrieval
}

/*
 * Get icon resources
 */

wxIcon MainFrame::GetIconResource( const wxString& name )
{
    // Icon retrieval
////@begin MainFrame icon retrieval
    wxUnusedVar(name);
    return wxNullIcon;
////@end MainFrame icon retrieval
}


/*
 * wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonAddJob
 */

void MainFrame::OnButtonAddJobClick( wxCommandEvent& event )
{
    AddJobWizard* window = new AddJobWizard(this);
    bool finished = window->Run();
    window->Destroy();
    
    if(finished && !scripts.empty()){
        wxListCtrl* lj = (wxListCtrl*)FindWindowById(ListJobs);
        Script s = scripts[scripts.size() - 1];
        int index = lj->InsertItem(0, wxString(s.name.c_str(), wxConvUTF8));
        lj->SetItem(index, 1, wxString(s.params[s.params.size() - 1].value.c_str(), wxConvUTF8));
        if(s.training)
            lj->SetItem(index, 2, wxString(s.params[s.params.size() - 2].value.c_str(), wxConvUTF8));
    }
}


/*
 * wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonClearJobs
 */

void MainFrame::OnButtonClearJobsClick( wxCommandEvent& event )
{
    int answer = wxMessageBox(_("This will clear all jobs, are you sure?"), _("Confirm"), 
        wxOK|wxCANCEL|wxICON_EXCLAMATION, this);
    if(answer == wxOK){
        scripts.clear();
        
        wxListCtrl* lj = (wxListCtrl*)FindWindowById(ListJobs);
        lj->DeleteAllItems();
    }
}




/*
 * wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonRunJobs
 */

void MainFrame::OnButtonRunJobsClick( wxCommandEvent& event )
{
    if(!scripts.empty()){
        for(unsigned int i = 0; i < scripts.size(); i++){
            try{
                unsigned int argCount = scripts[i].params.size() + (scripts[i].training ? 2 : 1);
                callPythonFunc(scripts[i].path.c_str(), scripts[i].name.c_str(), "getResult", scripts[i].params, argCount);
            } catch(PythonErr &e) {
                std::cerr << "Error running script:\n";
                std::cerr <<  e.what();
            }
        }
        wxMessageBox(_("Done running scripts"), _("Done"), wxOK, this);
    }
}

std::string MainFrame::callPythonFunc(
    const char* modulePath, const char* moduleName, const char* func, std::vector<Param> params, unsigned int argCount
){
    std::string result;
    PyObject *pName, *pImporter, *pModule, *pDict, *pFunc, *pValue, *pArgs;
    pName = pImporter = pModule = pDict = pFunc = pValue = pArgs = NULL;

    Py_Initialize();
    
    // Import module from file
    pName = PyString_FromString("imp");
    pImporter = PyImport_Import(pName);
  
    pArgs = PyTuple_New(2);
    PyTuple_SetItem(pArgs, 0, PyString_FromString(moduleName));
    PyTuple_SetItem(pArgs, 1, PyString_FromString( (std::string(modulePath) + std::string(moduleName) + ".py").c_str() ));

    pModule = PyObject_CallObject( PyDict_GetItemString(PyModule_GetDict(pImporter), "load_source"), pArgs);
    
    //Clean up
    Py_DECREF(pArgs);
    Py_DECREF(pImporter);
    Py_DECREF(pName);
    pArgs = NULL;
    
    if(pModule == NULL){
        PyErr_Print();
        Py_Finalize();
        throw PythonErr("No module named " + std::string(moduleName) + " in folder " + std::string(modulePath));
    }
  
    // Borrowed references
	pDict = PyModule_GetDict(pModule);
	pFunc = PyDict_GetItemString(pDict, func);
  
    if (PyCallable_Check(pFunc)) {
        // New reference, call Py_DECREF() when not used anymore
        pArgs = PyTuple_New( argCount );

        if(pArgs != NULL){
        // Parse arguments into tuple
            unsigned int i;
            for(i = 0; i < params.size(); i++){
                PyTuple_SetItem(pArgs, i,
                    PyString_FromString( params[i].value.c_str() ));
            }
            for(unsigned int j = i; j < argCount; j++){
                PyTuple_SetItem(pArgs, j, Py_None);
            }

            // New reference, call Py_DECREF() when not used anymore
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
        }
    
        if (pValue != NULL) {
            char* temp = PyString_AsString(pValue);
            result = std::string(temp);
            Py_DECREF(pValue);
        } else {
			PyErr_Print();
			Py_DECREF(pModule);
            Py_Finalize();
			
            throw PythonErr("Python call failed.\nDid you provide the right number of arguments?");
        }
    } else {
        PyErr_Print();

        // Clean up
        Py_DECREF(pModule);
        Py_Finalize();
		
        throw PythonErr("No function '" + std::string(func) + "' in module '" + std::string(moduleName) + "'");
    }

    // Clean up
    Py_DECREF(pModule);
    Py_Finalize();

    return result;
}


/*
 * wxEVT_COMMAND_BUTTON_CLICKED event handler for ButtonBash
 */

void MainFrame::OnButtonBashClick( wxCommandEvent& event )
{
    using namespace boost::filesystem;
    wxFileDialog saveFileDialog(this, _("Save bash script"), _(""), _(""), _("Shell script (*.sh)|*.sh"), wxFD_SAVE);
    if (saveFileDialog.ShowModal() == wxID_CANCEL)
        return;
    
    std::string bashPath = std::string(saveFileDialog.GetPath().mb_str());
    
    if(path(bashPath).extension() == ""){
        bashPath += ".sh";
    }
    
    std::ofstream bashFile;
    bashFile.open(bashPath.c_str());
    bashFile << "#!/bin/sh\n\n";
    
    for(unsigned int i = 0; i < scripts.size(); i++){
        bashFile << "python " << "\"" << scripts[i].path << scripts[i].name << ".py\"";
        for(unsigned int j = 0; j < scripts[i].params.size(); j++){
            bashFile << " " << "\"" << scripts[i].params[j].value << "\"";
        }
        bashFile << "\n";
    }
    
    bashFile.close();
    chmod(bashPath.c_str(), 0775);
}

