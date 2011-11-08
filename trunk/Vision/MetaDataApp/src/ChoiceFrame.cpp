#include "ChoiceFrame.h"

ChoiceFrame::ChoiceFrame(wxWindow* parent, wxWindowID id, const wxString& title,
		const wxPoint& pos, const wxSize& size, long style) :
		wxFrame(parent, id, title, pos, size, style) {
	this->SetSizeHints(wxDefaultSize, wxDefaultSize);

	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer(wxVERTICAL);

	wxStaticText* m_staticText28 = new wxStaticText( this, wxID_ANY, wxT("Select a directory with an Image \ndirectory and a Values.xml"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText28->Wrap( -1 );
	bSizer1->Add( m_staticText28, 0, wxALL, 5 );

	dirPicker = new wxDirPickerCtrl(this, wxID_ANY, wxEmptyString,
			wxT("Select a folder, with an Images directory"),
			wxDefaultPosition, wxSize(-1, -1), wxDIRP_DEFAULT_STYLE);
	bSizer1->Add(dirPicker, 0, wxALL | wxEXPAND, 5);

	wxStaticText* m_staticText27 = new wxStaticText( this, wxID_ANY, wxT("Select a .xml file to edit"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText27->Wrap( -1 );
	bSizer1->Add( m_staticText27, 0, wxALL, 5 );

	XMLPicker = new wxFilePickerCtrl(this, wxID_ANY, wxEmptyString,
			wxT("Select a file to write, edit or add images to"),
			wxT("XML (*.xml, *.XML)|*.xml; *.XML"), wxDefaultPosition,
			wxDefaultSize, wxFLP_DEFAULT_STYLE);
	bSizer1->Add(XMLPicker, 0, wxALL | wxEXPAND, 5);

	bSizer1->Add(
			new wxStaticLine(this, wxID_ANY, wxDefaultPosition, wxSize(-1, 1),
					wxLI_HORIZONTAL), 0, wxALL | wxEXPAND, 5);

	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer(wxVERTICAL);

	MessageField = new wxStaticText(this, wxID_ANY,
			wxT("Choose whether you want to load\n"
					"an existing  .xml to add and edit it,\n"
					"or you want to start over."), wxDefaultPosition,
			wxDefaultSize, 0);
	MessageField->Wrap(-1);
	bSizer11->Add(MessageField, 1, wxEXPAND | wxALL, 5);

	bSizer1->Add(bSizer11, 1, wxEXPAND, 5);

	bSizer1->Add(
			new wxStaticLine(this, wxID_ANY, wxDefaultPosition, wxSize(-1, 1),
					wxLI_HORIZONTAL), 0, wxALL | wxEXPAND, 5);

	wxBoxSizer* bSizer18;
	bSizer18 = new wxBoxSizer(wxVERTICAL);

	CreateNewXMLradioBtn = new wxRadioButton(this, wxID_ANY,
			wxT("Override existing XML"), wxDefaultPosition, wxDefaultSize,
			0);
	bSizer18->Add(CreateNewXMLradioBtn, 0, wxALL, 5);

	EditExistingXMLradioBtn = new wxRadioButton(this, wxID_ANY,
			wxT("Edit existing XML"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer18->Add(EditExistingXMLradioBtn, 0, wxALL, 5);

	AddToExistingXMLradioBtn = new wxRadioButton(this, wxID_ANY,
			wxT("Add to existing XML"), wxDefaultPosition, wxDefaultSize,
			0);
	bSizer18->Add(AddToExistingXMLradioBtn, 0, wxALL, 5);

	bSizer1->Add(bSizer18, 1, wxEXPAND, 5);

	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer(wxHORIZONTAL);

	ExitButton = new wxButton(this, wxID_ANY, wxT("Exit"), wxDefaultPosition,
			wxSize(100, -1), 0);
	bSizer12->Add(ExitButton, 0, wxALL, 5);

	wxBoxSizer* bSizer121;
	bSizer121 = new wxBoxSizer(wxVERTICAL);

	bSizer12->Add(bSizer121, 1, wxEXPAND, 5);

	OKButton = new wxButton(this, wxID_ANY, wxT("OK"), wxDefaultPosition,
			wxSize(100, -1), 0);
	bSizer12->Add(OKButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	bSizer1->Add(bSizer12, 0, wxEXPAND, 5);

	this->SetSizer(bSizer1);
	this->Layout();

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
