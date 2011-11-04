/*
 * CrateGUI.cpp
 *
 *  Created on: Nov 1, 2011
 *      Author: glenn
 */

#include "CrateGUI.h"

CrateGUI::CrateGUI( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( -1,-1 ), wxDefaultSize );

	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer( wxVERTICAL );

	wxBoxSizer* bSizer39;
	bSizer39 = new wxBoxSizer( wxHORIZONTAL );

	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer( wxVERTICAL );

	imageField = new wxStaticBitmap( this, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxSize( -1,-1 ), 0 );
	bSizer11->Add( imageField, 0, wxALL|wxEXPAND, 5 );

	bSizer39->Add( bSizer11, 1, wxEXPAND, 5 );

	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxVERTICAL );

	wxBoxSizer* bSizer121;
	bSizer121 = new wxBoxSizer( wxVERTICAL );

	LT_Radio = new wxRadioButton( this, wxID_ANY, wxT("Left top marker"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer121->Add( LT_Radio, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );

	wxBoxSizer* bSizer1211;
	bSizer1211 = new wxBoxSizer( wxHORIZONTAL );

	LTLTCorner_Radio = new wxRadioButton( this, wxID_ANY, wxT("LTCorner"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1211->Add( LTLTCorner_Radio, 0, wxALL|wxALIGN_RIGHT, 5 );

	LTRLCorner_Radio = new wxRadioButton( this, wxID_ANY, wxT("RLCorner"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1211->Add( LTRLCorner_Radio, 0, wxALL|wxALIGN_RIGHT, 5 );

	bSizer121->Add( bSizer1211, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer1212;
	bSizer1212 = new wxBoxSizer( wxHORIZONTAL );

	m_staticText132 = new wxStaticText( this, wxID_ANY, wxT("Left top"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText132->Wrap( -1 );
	bSizer1212->Add( m_staticText132, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	LTLT_TxtField = new wxStaticText( this, wxID_ANY, wxT("(000,000)"), wxDefaultPosition, wxDefaultSize, 0 );
	LTLT_TxtField->Wrap( -1 );
	bSizer1212->Add( LTLT_TxtField, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1212, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer1213;
	bSizer1213 = new wxBoxSizer( wxHORIZONTAL );

	m_staticText1311 = new wxStaticText( this, wxID_ANY, wxT("Right bottom"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText1311->Wrap( -1 );
	bSizer1213->Add( m_staticText1311, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	LTRB_TxtField = new wxStaticText( this, wxID_ANY, wxT("(000,000)"), wxDefaultPosition, wxDefaultSize, 0 );
	LTRB_TxtField->Wrap( -1 );
	bSizer1213->Add( LTRB_TxtField, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer121->Add( bSizer1213, 0, wxEXPAND, 5 );

	bSizer121->Add( new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL ), 0, wxEXPAND | wxALL, 5 );

	bSizer12->Add( bSizer121, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer122;
	bSizer122 = new wxBoxSizer( wxVERTICAL );

	RT_Radio = new wxRadioButton( this, wxID_ANY, wxT("Right top marker"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer122->Add( RT_Radio, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );

	wxBoxSizer* bSizer1221;
	bSizer1221 = new wxBoxSizer( wxHORIZONTAL );

	RTLTCorner_Radio = new wxRadioButton( this, wxID_ANY, wxT("LTCorner"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1221->Add( RTLTCorner_Radio, 0, wxALL|wxALIGN_RIGHT, 5 );

	RTRLCorner_Radio = new wxRadioButton( this, wxID_ANY, wxT("RLCorner"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1221->Add( RTRLCorner_Radio, 0, wxALL|wxALIGN_RIGHT, 5 );

	bSizer122->Add( bSizer1221, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer1222;
	bSizer1222 = new wxBoxSizer( wxHORIZONTAL );

	m_staticText1321 = new wxStaticText( this, wxID_ANY, wxT("Left top"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText1321->Wrap( -1 );
	bSizer1222->Add( m_staticText1321, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	RTLT_TxtField = new wxStaticText( this, wxID_ANY, wxT("(000,000)"), wxDefaultPosition, wxDefaultSize, 0 );
	RTLT_TxtField->Wrap( -1 );
	bSizer1222->Add( RTLT_TxtField, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer122->Add( bSizer1222, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer1223;
	bSizer1223 = new wxBoxSizer( wxHORIZONTAL );

	m_staticText13111 = new wxStaticText( this, wxID_ANY, wxT("Right bottom"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText13111->Wrap( -1 );
	bSizer1223->Add( m_staticText13111, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	RTRB_TxtField = new wxStaticText( this, wxID_ANY, wxT("(000,000)"), wxDefaultPosition, wxDefaultSize, 0 );
	RTRB_TxtField->Wrap( -1 );
	bSizer1223->Add( RTRB_TxtField, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer122->Add( bSizer1223, 0, wxEXPAND, 5 );

	bSizer122->Add( new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL ), 0, wxEXPAND | wxALL, 5 );

	wxBoxSizer* bSizer1224;
	bSizer1224 = new wxBoxSizer( wxVERTICAL );

	LB_Radio = new wxRadioButton( this, wxID_ANY, wxT("Left bottom marker"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer1224->Add( LB_Radio, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );

	wxBoxSizer* bSizer12241;
	bSizer12241 = new wxBoxSizer( wxHORIZONTAL );

	LBLTCorner_Radio = new wxRadioButton( this, wxID_ANY, wxT("LTCorner"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer12241->Add( LBLTCorner_Radio, 0, wxALL|wxALIGN_RIGHT, 5 );

	LBRLCorner_Radio = new wxRadioButton( this, wxID_ANY, wxT("RLCorner"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer12241->Add( LBRLCorner_Radio, 0, wxALL|wxALIGN_RIGHT, 5 );

	bSizer1224->Add( bSizer12241, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer12242;
	bSizer12242 = new wxBoxSizer( wxHORIZONTAL );

	m_staticText1322 = new wxStaticText( this, wxID_ANY, wxT("Left top"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText1322->Wrap( -1 );
	bSizer12242->Add( m_staticText1322, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	LBLT_TxtField = new wxStaticText( this, wxID_ANY, wxT("(000,000)"), wxDefaultPosition, wxDefaultSize, 0 );
	LBLT_TxtField->Wrap( -1 );
	bSizer12242->Add( LBLT_TxtField, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer1224->Add( bSizer12242, 0, wxEXPAND, 5 );

	wxBoxSizer* bSizer12243;
	bSizer12243 = new wxBoxSizer( wxHORIZONTAL );

	m_staticText13112 = new wxStaticText( this, wxID_ANY, wxT("Right bottom"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText13112->Wrap( -1 );
	bSizer12243->Add( m_staticText13112, 1, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	LBRB_TxtField = new wxStaticText( this, wxID_ANY, wxT("(000,000)"), wxDefaultPosition, wxDefaultSize, 0 );
	LBRB_TxtField->Wrap( -1 );
	bSizer12243->Add( LBRB_TxtField, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	bSizer1224->Add( bSizer12243, 0, wxEXPAND, 5 );

	bSizer1224->Add( new wxStaticLine( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLI_HORIZONTAL ), 0, wxEXPAND | wxALL, 5 );

	bSizer122->Add( bSizer1224, 1, wxEXPAND, 5 );

	bSizer12->Add( bSizer122, 1, wxEXPAND, 5 );

	QRCode_TxtField = new wxTextCtrl( this, wxID_ANY, wxT("QR code"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer12->Add( QRCode_TxtField, 0, wxALL|wxEXPAND, 5 );

	wxBoxSizer* bSizer123;
	bSizer123 = new wxBoxSizer( wxVERTICAL );

	DONE_button = new wxButton( this, wxID_ANY, wxT("DONE"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer123->Add( DONE_button, 0, wxALL|wxALIGN_CENTER_HORIZONTAL, 5 );

	bSizer12->Add( bSizer123, 0, wxEXPAND, 5 );

	bSizer39->Add( bSizer12, 0, wxEXPAND, 5 );

	bSizer1->Add( bSizer39, 1, wxEXPAND, 5 );

	pathField = new wxStaticText( this, wxID_ANY, wxT("/Path/..."), wxDefaultPosition, wxDefaultSize, 0 );
	pathField->Wrap( -1 );
	bSizer1->Add( pathField, 0, wxALL, 5 );

	this->SetSizer( bSizer1 );
	this->Layout();

	// Connect Events
	imageField->Connect( wxEVT_LEAVE_WINDOW, wxMouseEventHandler( CrateGUI::OnLeftImageField ), NULL, this );
	imageField->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateGUI::OnLeftDown ), NULL, this );
	imageField->Connect( wxEVT_LEFT_UP, wxMouseEventHandler( CrateGUI::OnLeftUp ), NULL, this );
	imageField->Connect( wxEVT_MOTION, wxMouseEventHandler( CrateGUI::OnLeftMotion ), NULL, this );
	DONE_button->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateGUI::OnDonePressed ), NULL, this );
	QRCode_TxtField->Connect( wxEVT_SET_FOCUS, wxFocusEventHandler( CrateGUI::OnQRFocus ), NULL, this );
}

CrateGUI::~CrateGUI()
{
	this->GetParent()->Show(true);
	// Disconnect Events
	imageField->Disconnect( wxEVT_LEAVE_WINDOW, wxMouseEventHandler( CrateGUI::OnLeftImageField ), NULL, this );
	imageField->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateGUI::OnLeftDown ), NULL, this );
	imageField->Disconnect( wxEVT_LEFT_UP, wxMouseEventHandler( CrateGUI::OnLeftUp ), NULL, this );
	imageField->Disconnect( wxEVT_MOTION, wxMouseEventHandler( CrateGUI::OnLeftMotion ), NULL, this );
	DONE_button->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( CrateGUI::OnDonePressed ), NULL, this );
	QRCode_TxtField->Disconnect( wxEVT_SET_FOCUS, wxFocusEventHandler( CrateGUI::OnQRFocus ), NULL, this );
}
