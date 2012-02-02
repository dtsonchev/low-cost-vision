///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Sep  8 2010)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include <huniplacer_gui/huniplacer_frame.h>

///////////////////////////////////////////////////////////////////////////

huniplacer_frame::huniplacer_frame( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxFrame( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( 500,710 ), wxDefaultSize );

	wxBoxSizer* root_sizer_1;
	root_sizer_1 = new wxBoxSizer( wxVERTICAL );

	wxFlexGridSizer* root_sizer_2;
	root_sizer_2 = new wxFlexGridSizer( 2, 1, 0, 0 );
	root_sizer_2->SetFlexibleDirection( wxBOTH );
	root_sizer_2->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );

	wxFlexGridSizer* graph_pos_sizer;
	graph_pos_sizer = new wxFlexGridSizer( 1, 2, 0, 0 );
	graph_pos_sizer->SetFlexibleDirection( wxBOTH );
	graph_pos_sizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );

	pos_panel = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 480,480 ), wxTAB_TRAVERSAL );
	pos_panel->SetForegroundColour( wxColour( 255, 0, 0 ) );
	pos_panel->SetBackgroundColour( wxColour( 128, 128, 128 ) );

	graph_pos_sizer->Add( pos_panel, 1, wxEXPAND | wxALL, 5 );

	pos_panel_side = new wxPanel( this, wxID_ANY, wxDefaultPosition, wxSize( 480,480 ), wxTAB_TRAVERSAL );
	pos_panel_side->SetForegroundColour( wxColour( 255, 0, 0 ) );
	pos_panel_side->SetBackgroundColour( wxColour( 128, 128, 128 ) );

	graph_pos_sizer->Add( pos_panel_side, 1, wxEXPAND | wxALL, 5 );

	root_sizer_2->Add( graph_pos_sizer, 1, wxEXPAND, 5 );

	wxFlexGridSizer* split_sizer;
	split_sizer = new wxFlexGridSizer( 1, 2, 0, 0 );
	split_sizer->SetFlexibleDirection( wxBOTH );
	split_sizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );

	wxStaticBoxSizer* text_pos_sizer_label;
	text_pos_sizer_label = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxEmptyString ), wxVERTICAL );

	wxFlexGridSizer* text_pos_sizer;
	text_pos_sizer = new wxFlexGridSizer( 1, 2, 0, 0 );
	text_pos_sizer->SetFlexibleDirection( wxBOTH );
	text_pos_sizer->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );

	wxFlexGridSizer* text_pos_sizer_2;
	text_pos_sizer_2 = new wxFlexGridSizer( 3, 2, 0, 0 );
	text_pos_sizer_2->SetFlexibleDirection( wxBOTH );
	text_pos_sizer_2->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );

	lab_x = new wxStaticText( this, wxID_ANY, wxT("X:"), wxDefaultPosition, wxDefaultSize, 0 );
	lab_x->Wrap( -1 );
	text_pos_sizer_2->Add( lab_x, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	txtbox_x = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	text_pos_sizer_2->Add( txtbox_x, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	lab_y = new wxStaticText( this, wxID_ANY, wxT("Y:"), wxDefaultPosition, wxDefaultSize, 0 );
	lab_y->Wrap( -1 );
	text_pos_sizer_2->Add( lab_y, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	txtbox_y = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	text_pos_sizer_2->Add( txtbox_y, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	lab_z = new wxStaticText( this, wxID_ANY, wxT("Z:"), wxDefaultPosition, wxDefaultSize, 0 );
	lab_z->Wrap( -1 );
	text_pos_sizer_2->Add( lab_z, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	txtbox_z = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	text_pos_sizer_2->Add( txtbox_z, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	text_pos_sizer->Add( text_pos_sizer_2, 1, wxEXPAND, 5 );

	wxFlexGridSizer* text_pos_sizer_3;
	text_pos_sizer_3 = new wxFlexGridSizer( 2, 1, 0, 0 );
	text_pos_sizer_3->SetFlexibleDirection( wxBOTH );
	text_pos_sizer_3->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );

	wxFlexGridSizer* text_pos_sizer_4;
	text_pos_sizer_4 = new wxFlexGridSizer( 2, 2, 0, 0 );
	text_pos_sizer_4->SetFlexibleDirection( wxBOTH );
	text_pos_sizer_4->SetNonFlexibleGrowMode( wxFLEX_GROWMODE_SPECIFIED );

	lab_speed = new wxStaticText( this, wxID_ANY, wxT("speed"), wxDefaultPosition, wxDefaultSize, 0 );
	lab_speed->Wrap( -1 );
	text_pos_sizer_4->Add( lab_speed, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	txtbox_speed = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	text_pos_sizer_4->Add( txtbox_speed, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );

	text_pos_sizer_3->Add( text_pos_sizer_4, 1, wxEXPAND, 5 );

	button_move = new wxButton( this, wxID_ANY, wxT("move"), wxDefaultPosition, wxDefaultSize, 0 );
	text_pos_sizer_3->Add( button_move, 0, wxALL|wxALIGN_RIGHT|wxALIGN_BOTTOM, 5 );

	text_pos_sizer->Add( text_pos_sizer_3, 1, wxEXPAND, 5 );

	text_pos_sizer_label->Add( text_pos_sizer, 1, wxEXPAND, 5 );

	split_sizer->Add( text_pos_sizer_label, 1, wxEXPAND, 5 );

	wxStaticBoxSizer* misc_sizer_label;
	misc_sizer_label = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxEmptyString ), wxVERTICAL );

	wxBoxSizer* misc_sizer;
	misc_sizer = new wxBoxSizer( wxVERTICAL );

	button_connect = new wxButton( this, wxID_ANY, wxT("connect"), wxDefaultPosition, wxDefaultSize, 0 );
	misc_sizer->Add( button_connect, 0, wxALL, 5 );

	button_disconnect = new wxButton( this, wxID_ANY, wxT("disconnect"), wxDefaultPosition, wxDefaultSize, 0 );
	misc_sizer->Add( button_disconnect, 0, wxALL, 5 );

	button_on = new wxButton( this, wxID_ANY, wxT("on"), wxDefaultPosition, wxDefaultSize, 0 );
	misc_sizer->Add( button_on, 0, wxALL, 5 );

	button_off = new wxButton( this, wxID_ANY, wxT("off"), wxDefaultPosition, wxDefaultSize, 0 );
	misc_sizer->Add( button_off, 0, wxALL, 5 );

	misc_sizer_label->Add( misc_sizer, 1, wxEXPAND, 5 );

	split_sizer->Add( misc_sizer_label, 1, wxEXPAND|wxALIGN_RIGHT, 5 );

	root_sizer_2->Add( split_sizer, 1, wxEXPAND, 5 );

	lab_status = new wxStaticText( this, wxID_ANY, wxT("Status: unknown"), wxDefaultPosition, wxDefaultSize, 0 );
	lab_status->Wrap( -1 );
	root_sizer_2->Add( lab_status, 0, wxALL|wxEXPAND, 5 );

	root_sizer_1->Add( root_sizer_2, 1, wxEXPAND, 5 );

	this->SetSizer( root_sizer_1 );
	this->Layout();
	root_sizer_1->Fit( this );

	this->Centre( wxBOTH );

	// Connect Events
	pos_panel->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( huniplacer_frame::pos_panelOnLeftDown ), NULL, this );
	pos_panel->Connect( wxEVT_PAINT, wxPaintEventHandler( huniplacer_frame::pos_panelOnPaint ), NULL, this );
	pos_panel_side->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( huniplacer_frame::pos_panel_sideOnLeftDown ), NULL, this );
	pos_panel_side->Connect( wxEVT_PAINT, wxPaintEventHandler( huniplacer_frame::pos_panel_sideOnPaint ), NULL, this );
	button_move->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( huniplacer_frame::button_moveOnButtonClick ), NULL, this );
	button_connect->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( huniplacer_frame::button_connectOnButtonClick ), NULL, this );
	button_disconnect->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( huniplacer_frame::button_disconnectOnButtonClick ), NULL, this );
	button_on->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( huniplacer_frame::button_onOnButtonClick ), NULL, this );
	button_off->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( huniplacer_frame::button_offOnButtonClick ), NULL, this );
}

huniplacer_frame::~huniplacer_frame()
{
	// Disconnect Events
	pos_panel->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( huniplacer_frame::pos_panelOnLeftDown ), NULL, this );
	pos_panel->Disconnect( wxEVT_PAINT, wxPaintEventHandler( huniplacer_frame::pos_panelOnPaint ), NULL, this );
	pos_panel_side->Disconnect( wxEVT_LEFT_DOWN, wxMouseEventHandler( huniplacer_frame::pos_panel_sideOnLeftDown ), NULL, this );
	pos_panel_side->Disconnect( wxEVT_PAINT, wxPaintEventHandler( huniplacer_frame::pos_panel_sideOnPaint ), NULL, this );
	button_move->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( huniplacer_frame::button_moveOnButtonClick ), NULL, this );
	button_connect->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( huniplacer_frame::button_connectOnButtonClick ), NULL, this );
	button_disconnect->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( huniplacer_frame::button_disconnectOnButtonClick ), NULL, this );
	button_on->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( huniplacer_frame::button_onOnButtonClick ), NULL, this );
	button_off->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( huniplacer_frame::button_offOnButtonClick ), NULL, this );

}
