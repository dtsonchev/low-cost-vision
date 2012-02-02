///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Dec 21 2009)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "CrateGUI/ChoiceFrame.h"

///////////////////////////////////////////////////////////////////////////

ChoiceFrame::ChoiceFrame(wxWindow* parent, wxWindowID id, const wxString& title,
		const wxPoint& pos, const wxSize& size, long style) :
		wxFrame(parent, id, title, pos, size, style) {
	this->SetSizeHints(wxDefaultSize, wxDefaultSize);

	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer(wxVERTICAL);

	m_staticText28 =
			new wxStaticText(
					this,
					wxID_ANY,
					wxT("Select a directory with an Image \ndirectory and a Values.xml"),
					wxDefaultPosition, wxDefaultSize, 0);
	m_staticText28->Wrap(-1);
	bSizer1->Add(m_staticText28, 0, wxALL, 5);

	dirPicker = new wxDirPickerCtrl(this, wxID_ANY, wxEmptyString,
			wxT("Select a folder"), wxDefaultPosition, wxSize(-1, -1),
			wxDIRP_DEFAULT_STYLE);
	bSizer1->Add(dirPicker, 0, wxALL | wxEXPAND, 5);

	wxBoxSizer* bSizer41;
	bSizer41 = new wxBoxSizer(wxVERTICAL);

	bSizer1->Add(bSizer41, 1, wxEXPAND, 5);

	m_staticline41 = new wxStaticLine(this, wxID_ANY, wxDefaultPosition,
			wxDefaultSize, wxLI_HORIZONTAL);
	bSizer1->Add(m_staticline41, 0, wxEXPAND | wxALL, 5);

	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer(wxVERTICAL);

	MessageField =
			new wxStaticText(
					this,
					wxID_ANY,
					wxT("Choose wheter you want to load\nan existing  .xml to add and edit it,\nor you want to start over."),
					wxDefaultPosition, wxDefaultSize, 0);
	MessageField->Wrap(-1);
	bSizer11->Add(MessageField, 1, wxEXPAND | wxALL, 5);

	bSizer1->Add(bSizer11, 0, wxEXPAND, 5);

	m_staticline4 = new wxStaticLine(this, wxID_ANY, wxDefaultPosition,
			wxDefaultSize, wxLI_HORIZONTAL);
	bSizer1->Add(m_staticline4, 0, wxEXPAND | wxALL, 5);

	wxBoxSizer* bSizer18;
	bSizer18 = new wxBoxSizer(wxVERTICAL);

	CreateNewXMLradioBtn = new wxRadioButton(this, wxID_ANY,
			wxT("Create new XML"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer18->Add(CreateNewXMLradioBtn, 0, wxALL, 5);

	EditExistingXMLradioBtn = new wxRadioButton(this, wxID_ANY,
			wxT("Edit existing XML"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer18->Add(EditExistingXMLradioBtn, 0, wxALL, 5);

	AddToExistingXMLradioBtn = new wxRadioButton(this, wxID_ANY,
			wxT("Add to existing XML"), wxDefaultPosition, wxDefaultSize,
			0);
	bSizer18->Add(AddToExistingXMLradioBtn, 0, wxALL, 5);

	bSizer1->Add(bSizer18, 0, wxEXPAND, 5);

	m_staticline43 = new wxStaticLine(this, wxID_ANY, wxDefaultPosition,
			wxDefaultSize, wxLI_HORIZONTAL);
	bSizer1->Add(m_staticline43, 0, wxEXPAND | wxALL, 5);

	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer(wxHORIZONTAL);

	ExitButton = new wxButton(this, wxID_ANY, wxT("Exit"), wxDefaultPosition,
			wxSize(100, -1), 0);
	bSizer12->Add(ExitButton, 0, wxALL, 5);

	OKButton = new wxButton(this, wxID_ANY, wxT("OK"), wxDefaultPosition,
			wxSize(100, -1), 0);
	bSizer12->Add(OKButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	bSizer1->Add(bSizer12, 0, wxEXPAND, 5);

	this->SetSizer(bSizer1);
	this->Layout();
	bSizer1->Fit(this);

	// Connect Events
	CreateNewXMLradioBtn->Connect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( ChoiceFrame::OnXMLOption ), NULL, this);
	EditExistingXMLradioBtn->Connect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( ChoiceFrame::OnXMLOption ), NULL, this);
	AddToExistingXMLradioBtn->Connect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( ChoiceFrame::OnXMLOption ), NULL, this);
	ExitButton->Connect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( ChoiceFrame::OnExit ), NULL, this);
	OKButton->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler( ChoiceFrame::OnOK ),
			NULL, this);
}

ChoiceFrame::~ChoiceFrame() {
	// Disconnect Events
	CreateNewXMLradioBtn->Disconnect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( ChoiceFrame::OnXMLOption ), NULL, this);
	EditExistingXMLradioBtn->Disconnect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( ChoiceFrame::OnXMLOption ), NULL, this);
	AddToExistingXMLradioBtn->Disconnect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( ChoiceFrame::OnXMLOption ), NULL, this);
	ExitButton->Disconnect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( ChoiceFrame::OnExit ), NULL, this);
	OKButton->Disconnect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( ChoiceFrame::OnOK ), NULL, this);
}
