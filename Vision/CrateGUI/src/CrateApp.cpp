///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Dec 21 2009)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "CrateGUI/CrateApp.h"

///////////////////////////////////////////////////////////////////////////

CrateApp::CrateApp( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 600,600 ), wxDefaultSize );

	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxHORIZONTAL );

	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer( wxVERTICAL );

	wxBoxSizer* bSizer111;
	bSizer111 = new wxBoxSizer( wxVERTICAL );

	ImageField = new wxStaticBitmap( this, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxSize( -1,-1 ), 0 );
	bSizer111->Add( ImageField, 0, wxTOP|wxRIGHT|wxLEFT, 5 );

	bSizer11->Add( bSizer111, 0, wxALIGN_CENTER_VERTICAL|wxEXPAND, 5 );

	wxBoxSizer* bSizer112;
	bSizer112 = new wxBoxSizer( wxVERTICAL );

	MessageLabel = new wxStaticText( this, wxID_ANY, wxT("Message: press start"), wxDefaultPosition, wxDefaultSize, 0 );
	MessageLabel->Wrap( -1 );
	bSizer112->Add( MessageLabel, 0, wxALIGN_CENTER_VERTICAL|wxALL|wxEXPAND, 5 );

	bSizer11->Add( bSizer112, 1, wxEXPAND, 5 );

	bSizer1->Add( bSizer11, 1, wxEXPAND, 5 );

	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxVERTICAL );

	bSizer121 = new wxBoxSizer( wxVERTICAL );

	OptionScrollWindow = new wxScrolledWindow( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxHSCROLL|wxVSCROLL );
	OptionScrollWindow->SetScrollRate( 0, 10 );
	wxBoxSizer* bSizer1211;
	bSizer1211 = new wxBoxSizer( wxVERTICAL );

	wxBoxSizer* bSizer121102;
	bSizer121102 = new wxBoxSizer( wxHORIZONTAL );

	m_staticText29 = new wxStaticText( OptionScrollWindow, wxID_ANY, wxT("Box Color:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText29->Wrap( -1 );
	bSizer121102->Add( m_staticText29, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	ColorSlider = new wxSlider( OptionScrollWindow, wxID_ANY, 0, 0, 255, wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL );
	bSizer121102->Add( ColorSlider, 1, wxALL, 5 );

	bSizer1211->Add( bSizer121102, 0, wxEXPAND, 5 );

	m_staticline1 = new wxStaticLine( OptionScrollWindow, wxID_ANY, wxDefaultPosition, wxSize( -1,1 ), wxLI_HORIZONTAL );
	bSizer1211->Add( m_staticline1, 0, wxALL|wxEXPAND, 5 );

	BackgroundComboBox = new wxComboBox( OptionScrollWindow, wxID_ANY, wxT("Background"), wxDefaultPosition, wxSize( -1,-1 ), 0, NULL, wxCB_READONLY );
	bSizer1211->Add( BackgroundComboBox, 0, wxALL|wxEXPAND, 5 );

	LightingComboBox = new wxComboBox( OptionScrollWindow, wxID_ANY, wxT("Lighting"), wxDefaultPosition, wxSize( -1,-1 ), 0, NULL, 0 );
	bSizer1211->Add( LightingComboBox, 0, wxALL|wxEXPAND, 5 );

	PerspectiveComboBox = new wxComboBox( OptionScrollWindow, wxID_ANY, wxT("Perspective"), wxDefaultPosition, wxSize( -1,-1 ), 0, NULL, 0 );
	bSizer1211->Add( PerspectiveComboBox, 0, wxALL|wxEXPAND, 5 );

	m_staticline2 = new wxStaticLine( OptionScrollWindow, wxID_ANY, wxDefaultPosition, wxSize( -1,1 ), wxLI_HORIZONTAL );
	bSizer1211->Add( m_staticline2, 0, wxEXPAND | wxALL, 5 );

	wxBoxSizer* bSizer121104;
	bSizer121104 = new wxBoxSizer( wxHORIZONTAL );

	ZoomBox_radioBtn = new wxRadioButton( OptionScrollWindow, wxID_ANY, wxT("Zoom box"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer121104->Add( ZoomBox_radioBtn, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	ZoomCheckBox = new wxCheckBox( OptionScrollWindow, wxID_ANY, wxT("On"), wxDefaultPosition, wxDefaultSize, 0 );
	ZoomCheckBox->SetValue(true);
	bSizer121104->Add( ZoomCheckBox, 0, wxALL, 5 );

	bSizer1211->Add( bSizer121104, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer121105;
	bSizer121105 = new wxBoxSizer( wxHORIZONTAL );

	ZoomButton = new wxButton( OptionScrollWindow, wxID_ANY, wxT("Zoom"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer121105->Add( ZoomButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	wxBoxSizer* bSizer1211051;
	bSizer1211051 = new wxBoxSizer( wxVERTICAL );

	bSizer121105->Add( bSizer1211051, 1, wxEXPAND, 5 );

	OriginalImageButton = new wxButton( OptionScrollWindow, wxID_ANY, wxT("Original"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer121105->Add( OriginalImageButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer1211->Add( bSizer121105, 0, wxEXPAND, 5 );

	m_staticline21 = new wxStaticLine( OptionScrollWindow, wxID_ANY, wxDefaultPosition, wxSize( -1,1 ), wxLI_HORIZONTAL );
	bSizer1211->Add( m_staticline21, 0, wxEXPAND | wxALL, 5 );

	ObjectTypeCombo = new wxComboBox( OptionScrollWindow, wxID_ANY, wxT("Crate"), wxDefaultPosition, wxDefaultSize, 0, NULL, 0 );
	ObjectTypeCombo->Append( wxT("Crate") );
	ObjectTypeCombo->Append( wxT("Marker") );
	ObjectTypeCombo->Append( wxT("QR code") );
	bSizer1211->Add( ObjectTypeCombo, 0, wxALL|wxEXPAND, 5 );

	wxBoxSizer* bSizer24;
	bSizer24 = new wxBoxSizer( wxVERTICAL );

	CrateLineRDB = new wxRadioButton( OptionScrollWindow, wxID_ANY, wxT("Line"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer24->Add( CrateLineRDB, 0, wxALL|wxEXPAND, 5 );

	wxBoxSizer* bSizer48;
	bSizer48 = new wxBoxSizer( wxHORIZONTAL );

	QRCodeCornerRDB = new wxRadioButton( OptionScrollWindow, wxID_ANY, wxT("QR code corner"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer48->Add( QRCodeCornerRDB, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	QRCodeCornerText = new wxStaticText( OptionScrollWindow, wxID_ANY, wxT(""), wxDefaultPosition, wxDefaultSize, 0 );
	QRCodeCornerText->Wrap( -1 );
	bSizer48->Add( QRCodeCornerText, 1, wxALL|wxEXPAND, 5 );

	QRCodeCornerLabel = new wxStaticText( OptionScrollWindow, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	QRCodeCornerLabel->Wrap( -1 );
	QRCodeCornerLabel->SetMinSize( wxSize( 70,-1 ) );

	bSizer48->Add( QRCodeCornerLabel, 0, wxALL|wxALIGN_CENTER_VERTICAL|wxEXPAND, 5 );

	bSizer24->Add( bSizer48, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer481;
	bSizer481 = new wxBoxSizer( wxHORIZONTAL );

	OppositeCornerRDB = new wxRadioButton( OptionScrollWindow, wxID_ANY, wxT("Opposite corner"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer481->Add( OppositeCornerRDB, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	OppositeCornerText = new wxStaticText( OptionScrollWindow, wxID_ANY, wxT(""), wxDefaultPosition, wxDefaultSize, 0 );
	OppositeCornerText->Wrap( -1 );
	bSizer481->Add( OppositeCornerText, 1, wxALL|wxEXPAND, 5 );

	OppositeCornerLabel = new wxStaticText( OptionScrollWindow, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	OppositeCornerLabel->Wrap( -1 );
	OppositeCornerLabel->SetMinSize( wxSize( 70,-1 ) );

	bSizer481->Add( OppositeCornerLabel, 0, wxALL|wxALIGN_CENTER_VERTICAL|wxEXPAND, 5 );

	bSizer24->Add( bSizer481, 0, wxEXPAND, 5 );

	bSizer1211->Add( bSizer24, 0, wxEXPAND, 5 );

	m_staticline12 = new wxStaticLine( OptionScrollWindow, wxID_ANY, wxDefaultPosition, wxSize( -1,1 ), wxLI_HORIZONTAL );
	bSizer1211->Add( m_staticline12, 0, wxEXPAND | wxALL, 5 );

	m_staticText11 = new wxStaticText( OptionScrollWindow, wxID_ANY, wxT("Contents QR code:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText11->Wrap( -1 );
	bSizer1211->Add( m_staticText11, 0, wxALL|wxEXPAND, 5 );

	QRCodeTextBox = new wxTextCtrl( OptionScrollWindow, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1211->Add( QRCodeTextBox, 0, wxALL|wxEXPAND, 5 );

	m_staticline121 = new wxStaticLine( OptionScrollWindow, wxID_ANY, wxDefaultPosition, wxSize( -1,1 ), wxLI_HORIZONTAL );
	bSizer1211->Add( m_staticline121, 0, wxEXPAND | wxALL, 5 );

	wxBoxSizer* bSizer121114;
	bSizer121114 = new wxBoxSizer( wxHORIZONTAL );

	NextObjectButton = new wxButton( OptionScrollWindow, wxID_ANY, wxT("Add object"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer121114->Add( NextObjectButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	wxBoxSizer* bSizer1211141;
	bSizer1211141 = new wxBoxSizer( wxVERTICAL );

	bSizer121114->Add( bSizer1211141, 1, wxEXPAND, 5 );

	SkipButton = new wxButton( OptionScrollWindow, wxID_ANY, wxT("Skip"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer121114->Add( SkipButton, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer1211->Add( bSizer121114, 0, wxEXPAND, 5 );

	NextImageButton = new wxButton( OptionScrollWindow, wxID_ANY, wxT("Next image"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1211->Add( NextImageButton, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );

	wxBoxSizer* bSizer121115;
	bSizer121115 = new wxBoxSizer( wxHORIZONTAL );

	ResetButton = new wxButton( OptionScrollWindow, wxID_ANY, wxT("Reset image"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer121115->Add( ResetButton, 0, wxALL, 5 );

	wxBoxSizer* bSizer1211151;
	bSizer1211151 = new wxBoxSizer( wxVERTICAL );

	bSizer121115->Add( bSizer1211151, 1, wxEXPAND, 5 );

	DoneButton = new wxButton( OptionScrollWindow, wxID_ANY, wxT("Done"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer121115->Add( DoneButton, 0, wxALL, 5 );

	bSizer1211->Add( bSizer121115, 0, wxEXPAND, 5 );

	OptionScrollWindow->SetSizer( bSizer1211 );
	OptionScrollWindow->Layout();
	bSizer1211->Fit( OptionScrollWindow );
	bSizer121->Add( OptionScrollWindow, 1, wxEXPAND | wxALL, 5 );

	bSizer12->Add( bSizer121, 1, wxEXPAND, 5 );

	bSizer1->Add( bSizer12, 0, wxEXPAND, 5 );

	this->SetSizer( bSizer1 );
	this->Layout();
	bSizer1->Fit( this );

	// Connect Events
	this->Connect( wxEVT_SIZE, wxSizeEventHandler( CrateApp::OnSizeChange ) );
	ImageField->Connect( wxEVT_LEAVE_WINDOW, wxMouseEventHandler( CrateApp::OnLeaveImageField ), NULL, this );
	ImageField->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateApp::OnLeftMousePressed ), NULL, this );
	ImageField->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( CrateApp::OnLeftMouseRelease ), NULL, this );
	ImageField->Connect( wxEVT_MOTION, wxMouseEventHandler( CrateApp::OnImageMotion ), NULL, this );
	ColorSlider->Connect( wxEVT_SCROLL_TOP, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Connect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Connect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Connect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Connect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Connect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Connect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Connect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Connect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ZoomBox_radioBtn->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateApp::OnZoomRadioButton ), NULL, this );
	ZoomCheckBox->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateApp::OnZoomChange ), NULL, this );
	ZoomButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CrateApp::OnZoom ), NULL, this );
	OriginalImageButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CrateApp::OnOriginal ), NULL, this );
	ObjectTypeCombo->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( CrateApp::OnObjectType ), NULL, this );
	NextObjectButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CrateApp::OnNextObjectButton ), NULL, this );
	SkipButton->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateApp::OnSkip ), NULL, this );
	NextImageButton->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( CrateApp::OnNextImage ), NULL, this );
	ResetButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CrateApp::OnReset ), NULL, this );
	DoneButton->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CrateApp::OnDoneButton ), NULL, this );
}

CrateApp::~CrateApp()
{
	// Disconnect Events
	this->Disconnect( wxEVT_SIZE, wxSizeEventHandler( CrateApp::OnSizeChange ) );
	ImageField->Disconnect( wxEVT_LEAVE_WINDOW, wxMouseEventHandler( CrateApp::OnLeaveImageField ), NULL, this );
	ImageField->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateApp::OnLeftMousePressed ), NULL, this );
	ImageField->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( CrateApp::OnLeftMouseRelease ), NULL, this );
	ImageField->Disconnect( wxEVT_MOTION, wxMouseEventHandler( CrateApp::OnImageMotion ), NULL, this );
	ColorSlider->Disconnect( wxEVT_SCROLL_TOP, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Disconnect( wxEVT_SCROLL_BOTTOM, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Disconnect( wxEVT_SCROLL_LINEUP, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Disconnect( wxEVT_SCROLL_LINEDOWN, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Disconnect( wxEVT_SCROLL_PAGEUP, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Disconnect( wxEVT_SCROLL_PAGEDOWN, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Disconnect( wxEVT_SCROLL_THUMBTRACK, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Disconnect( wxEVT_SCROLL_THUMBRELEASE, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ColorSlider->Disconnect( wxEVT_SCROLL_CHANGED, wxScrollEventHandler( CrateApp::OnColorSlider ), NULL, this );
	ZoomBox_radioBtn->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateApp::OnZoomRadioButton ), NULL, this );
	ZoomCheckBox->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateApp::OnZoomChange ), NULL, this );
	ZoomButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CrateApp::OnZoom ), NULL, this );
	OriginalImageButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CrateApp::OnOriginal ), NULL, this );
	ObjectTypeCombo->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( CrateApp::OnObjectType ), NULL, this );
	NextObjectButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CrateApp::OnNextObjectButton ), NULL, this );
	SkipButton->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateApp::OnSkip ), NULL, this );
	NextImageButton->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( CrateApp::OnNextImage ), NULL, this );
	ResetButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CrateApp::OnReset ), NULL, this );
	DoneButton->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( CrateApp::OnDoneButton ), NULL, this );
}
