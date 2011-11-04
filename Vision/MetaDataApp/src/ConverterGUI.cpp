#include "ConverterGUI.h"

GUIFrame::GUIFrame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	//parentWindow = parent;

	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxHORIZONTAL );

	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer( wxVERTICAL );

	wxBoxSizer* bSizer111;
	bSizer111 = new wxBoxSizer( wxVERTICAL );

	bSizer111->SetMinSize( wxSize( 590,445 ) );
	wxImage::AddHandler(new wxJPEGHandler);
	ImageField = new wxStaticBitmap( this, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxSize( 640,480 ), 0 );
	bSizer111->Add( ImageField, 0, wxTOP|wxRIGHT|wxLEFT, 5 );

	bSizer11->Add( bSizer111, 0, wxALIGN_CENTER_VERTICAL|wxEXPAND, 5 );

	wxBoxSizer* bSizer112;
	bSizer112 = new wxBoxSizer( wxVERTICAL );

	MessageLabel = new wxStaticText( this, wxID_ANY, wxT(""
			"Select an directory with\n"
			"An image directory,\n"
			"And .txt file with the backgrounds an object types\n"
			"And press the start button"), wxDefaultPosition, wxDefaultSize, 0 );
	MessageLabel->Wrap( -1 );
	bSizer112->Add( MessageLabel, 1, wxALIGN_CENTER_VERTICAL|wxEXPAND|wxALL, 5 );

	bSizer11->Add( bSizer112, 1, wxEXPAND, 5 );

	bSizer1->Add( bSizer11, 1, wxEXPAND, 5 );

	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxVERTICAL );

	wxBoxSizer* bSizer121;
	bSizer121 = new wxBoxSizer( wxVERTICAL );

	wxBoxSizer* bSizer1211;
	bSizer1211 = new wxBoxSizer( wxHORIZONTAL );

	AmountOfObjectsLabel = new wxStaticText( this, wxID_ANY, wxT("Amount Of Object"), wxDefaultPosition, wxDefaultSize, 0 );
	AmountOfObjectsLabel->Wrap( -1 );
	bSizer1211->Add( AmountOfObjectsLabel, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	AmountOfObjectsTxtField = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1211->Add( AmountOfObjectsTxtField, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1211, 0, wxEXPAND, 5 );

	bSizer121->Add( new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxSize( -1,1 ), wxLI_HORIZONTAL ), 0, wxALL|wxEXPAND, 5 );

	wxBoxSizer* bSizer1212;
	bSizer1212 = new wxBoxSizer( wxHORIZONTAL );

	BackgroundTextField = new wxStaticText( this, wxID_ANY, wxT("Path to seperated\nfore- background:"), wxDefaultPosition, wxDefaultSize, 0 );
	BackgroundTextField->Wrap( -1 );
	bSizer1212->Add( BackgroundTextField, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	filePicker = new wxFilePickerCtrl( this, wxID_ANY, wxEmptyString, wxT("Select a file"), wxT("*.PNG"), wxDefaultPosition, wxDefaultSize, wxFLP_DEFAULT_STYLE );
	bSizer1212->Add( filePicker, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1212, 0, wxEXPAND, 5 );

	BackgroundComboBox = new wxComboBox( this, wxID_ANY, wxT("Background"), wxDefaultPosition, wxSize( -1,-1 ), 0, NULL, wxCB_READONLY );
	bSizer121->Add( BackgroundComboBox, 0, wxALL, 5 );

	LightingComboBox = new wxComboBox( this, wxID_ANY, wxT("Light"), wxDefaultPosition, wxSize( -1,-1 ), 0, NULL, wxCB_READONLY  );
	bSizer121->Add( LightingComboBox, 0, wxALL, 5 );

	PerspectiveComboBox = new wxComboBox( this, wxID_ANY, wxT("Perspective"), wxDefaultPosition, wxSize( -1,-1 ), 0, NULL, wxCB_READONLY  );
	bSizer121->Add( PerspectiveComboBox, 0, wxALL, 5 );

	bSizer121->Add( new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxSize( -1,1 ), wxLI_HORIZONTAL ), 0, wxALL|wxEXPAND, 5 );

	ObjectComboBox = new wxComboBox( this, wxID_ANY, wxT("Object"), wxDefaultPosition, wxSize( -1,-1 ), 0, NULL, wxCB_READONLY  );
	bSizer121->Add( ObjectComboBox, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	wxBoxSizer* bSizer1221;
	bSizer1221 = new wxBoxSizer( wxHORIZONTAL );

	SurroundBox_radioBtn = new wxRadioButton( this, wxID_ANY, wxT("Surround box"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1221->Add( SurroundBox_radioBtn, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	CrateButton = new wxButton( this, wxID_ANY, wxT("Crate"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1221->Add( CrateButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1221, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer1213;
	bSizer1213 = new wxBoxSizer( wxHORIZONTAL );

	LUC_RadioBtn = new wxRadioButton( this, wxID_ANY, wxT("Left Upper Corner"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1213->Add( LUC_RadioBtn, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	LUC_Label = new wxStaticText( this, wxID_ANY, wxT("(000, 000)"), wxDefaultPosition, wxDefaultSize, 0 );
	LUC_Label->Wrap( -1 );
	bSizer1213->Add( LUC_Label, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1213, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer1214;
	bSizer1214 = new wxBoxSizer( wxHORIZONTAL );

	RLC_RadioBtn = new wxRadioButton( this, wxID_ANY, wxT("Right Lower Corner"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1214->Add( RLC_RadioBtn, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	RLC_Label = new wxStaticText( this, wxID_ANY, wxT("(000, 000)"), wxDefaultPosition, wxDefaultSize, 0 );
	RLC_Label->Wrap( -1 );
	bSizer1214->Add( RLC_Label, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1214, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer1215;
	bSizer1215 = new wxBoxSizer( wxHORIZONTAL );

	CenterLine_radioBtn = new wxRadioButton( this, wxID_ANY, wxT("Rotation line"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1215->Add( CenterLine_radioBtn, 1, wxALL, 5 );

	NoRotationCheckBox = new wxCheckBox( this, wxID_ANY, wxT("No Rotation"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1215->Add( NoRotationCheckBox, 0, wxALL, 5 );

	bSizer121->Add( bSizer1215, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer1216;
	bSizer1216 = new wxBoxSizer( wxHORIZONTAL );

	CenterTop_RadioBtn = new wxRadioButton( this, wxID_ANY, wxT("Rotation Top"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1216->Add( CenterTop_RadioBtn, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	CenterTop_Label = new wxStaticText( this, wxID_ANY, wxT("(000, 000)"), wxDefaultPosition, wxDefaultSize, 0 );
	CenterTop_Label->Wrap( -1 );
	bSizer1216->Add( CenterTop_Label, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1216, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer1217;
	bSizer1217 = new wxBoxSizer( wxHORIZONTAL );

	CenterBottom_RadioBtn = new wxRadioButton( this, wxID_ANY, wxT("Center Bottom"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1217->Add( CenterBottom_RadioBtn, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	CenterBottom_Label = new wxStaticText( this, wxID_ANY, wxT("(000, 000)"), wxDefaultPosition, wxDefaultSize, 0 );
	CenterBottom_Label->Wrap( -1 );
	bSizer1217->Add( CenterBottom_Label, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1217, 0, wxEXPAND, 5 );

	bSizer121->Add( new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxSize( -1,1 ), wxLI_HORIZONTAL ), 0, wxEXPAND | wxALL, 5 );

	wxBoxSizer* bSizer1218;
	bSizer1218 = new wxBoxSizer( wxHORIZONTAL );

	Rotation_Label = new wxStaticText( this, wxID_ANY, wxT("Rotation (degrees)"), wxDefaultPosition, wxDefaultSize, 0 );
	Rotation_Label->Wrap( -1 );
	bSizer1218->Add( Rotation_Label, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	RotationValue_label = new wxStaticText( this, wxID_ANY, wxT("360.0"), wxDefaultPosition, wxDefaultSize, 0 );
	RotationValue_label->Wrap( -1 );
	bSizer1218->Add( RotationValue_label, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1218, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer1219;
	bSizer1219 = new wxBoxSizer( wxHORIZONTAL );

	CenterPoint_label = new wxStaticText( this, wxID_ANY, wxT("Center point"), wxDefaultPosition, wxDefaultSize, 0 );
	CenterPoint_label->Wrap( -1 );
	bSizer1219->Add( CenterPoint_label, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	CenterPointValue_label = new wxStaticText( this, wxID_ANY, wxT("(000,000)"), wxDefaultPosition, wxDefaultSize, 0 );
	CenterPointValue_label->Wrap( -1 );
	bSizer1219->Add( CenterPointValue_label, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1219, 0, wxEXPAND, 5 );

	bSizer121->Add( new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxSize( -1,1 ), wxLI_HORIZONTAL ), 0, wxEXPAND | wxALL, 5 );

	wxBoxSizer* bSizer1220;
	bSizer1220 = new wxBoxSizer( wxHORIZONTAL );

	NextObjectButton = new wxButton( this, wxID_ANY, wxT("Next Object"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1220->Add( NextObjectButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	SkipButton = new wxButton( this, wxID_ANY, wxT("Skip"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1220->Add( SkipButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1220, 0, wxEXPAND, 5 );

	bSizer12->Add( bSizer121, 1, wxEXPAND, 5 );

	bSizer1->Add( bSizer12, 0, wxEXPAND, 5 );

	this->SetSizer( bSizer1 );
	this->Layout();

	ImageField->Connect( wxEVT_LEAVE_WINDOW, wxMouseEventHandler( GUIFrame::OnLeftImageField ), NULL, this );
	ImageField->Connect(wxEVT_MOTION, wxMouseEventHandler( GUIFrame::OnImageMotion), NULL, this );
	ImageField->Connect(wxEVT_LEFT_UP, wxMouseEventHandler( GUIFrame::OnLeftMouseRelease), NULL, this );
	ImageField->Connect(wxEVT_LEFT_DOWN, wxMouseEventHandler( GUIFrame::OnLeftMousePressed), NULL, this );
	NextObjectButton->Connect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GUIFrame::OnNextObjectButton), NULL, this );
	SkipButton->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( GUIFrame::OnSkip ), NULL, this );

	ObjectComboBox->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( GUIFrame::OnComboSelect ), NULL, this );
	CrateButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GUIFrame::OnCrateButton ), NULL, this );

	AmountOfObjectsTxtField->Connect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GUIFrame::OnAmountOfObjects ), NULL, this );
	NoRotationCheckBox->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( GUIFrame::NoRotation ), NULL, this );
}

GUIFrame::~GUIFrame()
{
	std::stringstream s;
	s << dirPath << "/TestSet.xml";
	boost::property_tree::write_xml(s.str().c_str(), pt);
	this->GetParent()->Show(true);
	this->GetParent()->SetFocus();

	ImageField->Disconnect( wxEVT_LEAVE_WINDOW, wxMouseEventHandler( GUIFrame::OnLeftImageField ), NULL, this );
	ImageField->Disconnect(wxEVT_MOTION, wxMouseEventHandler( GUIFrame::OnImageMotion), NULL, this );
	ImageField->Disconnect(wxEVT_LEFT_UP, wxMouseEventHandler( GUIFrame::OnLeftMouseRelease), NULL, this );
	ImageField->Disconnect(wxEVT_LEFT_DOWN, wxMouseEventHandler( GUIFrame::OnLeftMouseRelease), NULL, this );
	NextObjectButton->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GUIFrame::OnNextObjectButton), NULL, this );
	SkipButton->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( GUIFrame::OnSkip ), NULL, this );

	ObjectComboBox->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( GUIFrame::OnComboSelect ), NULL, this );
	CrateButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( GUIFrame::OnCrateButton ), NULL, this );

	AmountOfObjectsTxtField->Disconnect( wxEVT_KILL_FOCUS, wxFocusEventHandler( GUIFrame::OnAmountOfObjects ), NULL, this );
	NoRotationCheckBox->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( GUIFrame::NoRotation ), NULL, this );
}
