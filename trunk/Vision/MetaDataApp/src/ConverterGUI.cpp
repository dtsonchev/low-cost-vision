#include "ConverterGUI.h"
//wxImage::AddHandler(new wxJPEGHandler);

GUIFrame::GUIFrame(wxWindow* parent, wxWindowID id, const wxString& title,
		const wxPoint& pos, const wxSize& size, long style) :
		wxFrame(parent, id, title, pos, size, style) {
	this->SetSizeHints(wxDefaultSize, wxDefaultSize);

	wxBoxSizer* bSizer1;
	bSizer1 = new wxBoxSizer(wxHORIZONTAL);

	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer(wxVERTICAL);

	wxBoxSizer* bSizer111;
	bSizer111 = new wxBoxSizer(wxVERTICAL);

	wxImage::AddHandler(new wxJPEGHandler);
	ImageField = new wxStaticBitmap(this, wxID_ANY, wxNullBitmap,
			wxDefaultPosition, wxSize(-1, -1), 0);
	bSizer111->Add(ImageField, 0, wxTOP | wxRIGHT | wxLEFT, 5);

	bSizer11->Add(bSizer111, 0, wxALIGN_CENTER_VERTICAL | wxEXPAND, 5);

	wxBoxSizer* bSizer112;
	bSizer112 = new wxBoxSizer(wxVERTICAL);

	MessageLabel = new wxStaticText(this, wxID_ANY, wxT("Message: press start"),
			wxDefaultPosition, wxDefaultSize, 0);
	MessageLabel->Wrap(-1);
	bSizer112->Add(MessageLabel, 0, wxALIGN_CENTER_VERTICAL | wxEXPAND | wxALL,
			5);

	bSizer11->Add(bSizer112, 1, wxEXPAND, 5);

	bSizer1->Add(bSizer11, 1, wxEXPAND, 5);

	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer(wxVERTICAL);

	bSizer121 = new wxBoxSizer(wxVERTICAL);

	wxScrolledWindow* OptionScrollWindow = new wxScrolledWindow(this, wxID_ANY,
			wxDefaultPosition, wxDefaultSize, wxHSCROLL | wxVSCROLL);
	OptionScrollWindow->SetScrollRate(0, 30);
	wxBoxSizer* bSizer1211;
	bSizer1211 = new wxBoxSizer(wxVERTICAL);

	wxBoxSizer* bSizer121101;
	bSizer121101 = new wxBoxSizer(wxHORIZONTAL);

	AmountOfObjectsLabel = new wxStaticText(OptionScrollWindow, wxID_ANY,
			wxT("Amount Of Object"), wxDefaultPosition, wxDefaultSize, 0);
	AmountOfObjectsLabel->Wrap(-1);
	bSizer121101->Add(AmountOfObjectsLabel, 0, wxALL | wxALIGN_CENTER_VERTICAL,
			5);

	AmountOfObjectsTxtField = new wxTextCtrl(OptionScrollWindow, wxID_ANY,
			wxEmptyString, wxDefaultPosition, wxDefaultSize, 0);
	bSizer121101->Add(AmountOfObjectsTxtField, 0,
			wxALL | wxALIGN_CENTER_VERTICAL, 5);

	bSizer1211->Add(bSizer121101, 0, wxEXPAND, 5);

	wxBoxSizer* bSizer121102;
	bSizer121102 = new wxBoxSizer(wxHORIZONTAL);

	wxStaticText* m_staticText29 = new wxStaticText(OptionScrollWindow,
			wxID_ANY, wxT("Box Color:"), wxDefaultPosition, wxDefaultSize, 0);
	m_staticText29->Wrap(-1);
	bSizer121102->Add(m_staticText29, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	ColorSlider = new wxSlider(OptionScrollWindow, wxID_ANY, 50, 0, 255,
			wxDefaultPosition, wxDefaultSize, wxSL_HORIZONTAL);
	bSizer121102->Add(ColorSlider, 1, wxALL, 5);

	bSizer1211->Add(bSizer121102, 0, wxEXPAND, 5);

	bSizer1211->Add(
			new wxStaticLine(OptionScrollWindow, wxID_ANY, wxDefaultPosition,
					wxSize(-1, 1), wxLI_HORIZONTAL), 0, wxALL | wxEXPAND, 5);

	wxBoxSizer* bSizer121103;
	bSizer121103 = new wxBoxSizer(wxHORIZONTAL);

	BackgroundTextField = new wxStaticText(OptionScrollWindow, wxID_ANY,
			wxT("Path to seperated\nfore- background:"), wxDefaultPosition,
			wxDefaultSize, 0);
	BackgroundTextField->Wrap(-1);
	bSizer121103->Add(BackgroundTextField, 0, wxALL | wxALIGN_CENTER_VERTICAL,
			5);

	filePicker = new wxFilePickerCtrl(OptionScrollWindow, wxID_ANY,
			wxEmptyString, wxT("Select a file"), wxT("*.PNG"),
			wxDefaultPosition, wxDefaultSize, wxFLP_DEFAULT_STYLE);
	bSizer121103->Add(filePicker, 1, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	bSizer1211->Add(bSizer121103, 0, wxEXPAND, 5);

	BackgroundComboBox = new wxComboBox(OptionScrollWindow, wxID_ANY,
			wxT("Background"), wxDefaultPosition, wxSize(-1, -1), 0, NULL,
			wxCB_READONLY);
	bSizer1211->Add(BackgroundComboBox, 0, wxALL | wxEXPAND, 5);

	LightingComboBox = new wxComboBox(OptionScrollWindow, wxID_ANY,
			wxT("Lighting"), wxDefaultPosition, wxSize(-1, -1), 0, NULL, 0);
	bSizer1211->Add(LightingComboBox, 0, wxALL | wxEXPAND, 5);

	PerspectiveComboBox = new wxComboBox(OptionScrollWindow, wxID_ANY,
			wxT("Perspective"), wxDefaultPosition, wxSize(-1, -1), 0, NULL,
			0);
	bSizer1211->Add(PerspectiveComboBox, 0, wxALL | wxEXPAND, 5);

	bSizer1211->Add(
			new wxStaticLine(OptionScrollWindow, wxID_ANY, wxDefaultPosition,
					wxSize(-1, 1), wxLI_HORIZONTAL), 0, wxEXPAND | wxALL, 5);

	wxBoxSizer* bSizer121104;
	bSizer121104 = new wxBoxSizer(wxHORIZONTAL);

	ZoomBox_radioBtn = new wxRadioButton(OptionScrollWindow, wxID_ANY,
			wxT("Zoom box"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer121104->Add(ZoomBox_radioBtn, 1, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	ZoomCheckBox = new wxCheckBox(OptionScrollWindow, wxID_ANY, wxT("On"),
			wxDefaultPosition, wxDefaultSize, 0);
	bSizer121104->Add(ZoomCheckBox, 0, wxALL, 5);

	bSizer1211->Add(bSizer121104, 0, wxEXPAND, 5);

	wxBoxSizer* bSizer121105;
	bSizer121105 = new wxBoxSizer(wxHORIZONTAL);

	ZoomButton = new wxButton(OptionScrollWindow, wxID_ANY, wxT("Zoom"),
			wxDefaultPosition, wxDefaultSize, 0);
	bSizer121105->Add(ZoomButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	wxBoxSizer* bSizer1211051;
	bSizer1211051 = new wxBoxSizer(wxVERTICAL);

	bSizer121105->Add(bSizer1211051, 1, wxEXPAND, 5);

	OriginalImageButton = new wxButton(OptionScrollWindow, wxID_ANY,
			wxT("Original"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer121105->Add(OriginalImageButton, 0, wxALL | wxALIGN_CENTER_VERTICAL,
			5);

	bSizer1211->Add(bSizer121105, 0, wxEXPAND, 5);

	bSizer1211->Add(
			new wxStaticLine(OptionScrollWindow, wxID_ANY, wxDefaultPosition,
					wxSize(-1, 1), wxLI_HORIZONTAL), 0, wxEXPAND | wxALL, 5);

	ObjectComboBox = new wxComboBox(OptionScrollWindow, wxID_ANY, wxT("Object"),
			wxDefaultPosition, wxSize(-1, -1), 0, NULL, 0);
	bSizer1211->Add(ObjectComboBox, 0,
			wxALL | wxALIGN_CENTER_VERTICAL | wxEXPAND, 5);

	wxBoxSizer* bSizer121106;
	bSizer121106 = new wxBoxSizer(wxHORIZONTAL);

	SurroundBox_radioBtn = new wxRadioButton(OptionScrollWindow, wxID_ANY,
			wxT("Surround box"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer121106->Add(SurroundBox_radioBtn, 1, wxALL | wxALIGN_CENTER_VERTICAL,
			5);

	CrateButton = new wxButton(OptionScrollWindow, wxID_ANY, wxT("Crate"),
			wxDefaultPosition, wxDefaultSize, 0);
	bSizer121106->Add(CrateButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	bSizer1211->Add(bSizer121106, 0, wxEXPAND, 5);

	wxBoxSizer* bSizer121107;
	bSizer121107 = new wxBoxSizer(wxHORIZONTAL);

	LUC_RadioBtn = new wxRadioButton(OptionScrollWindow, wxID_ANY,
			wxT("Left Upper Corner"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer121107->Add(LUC_RadioBtn, 1, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	LUC_Label = new wxStaticText(OptionScrollWindow, wxID_ANY,
			wxT("(000, 000)"), wxDefaultPosition, wxDefaultSize, 0);
	LUC_Label->Wrap(-1);
	bSizer121107->Add(LUC_Label, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	bSizer1211->Add(bSizer121107, 0, wxEXPAND, 5);

	wxBoxSizer* bSizer121108;
	bSizer121108 = new wxBoxSizer(wxHORIZONTAL);

	RLC_RadioBtn = new wxRadioButton(OptionScrollWindow, wxID_ANY,
			wxT("Right Lower Corner"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer121108->Add(RLC_RadioBtn, 1, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	RLC_Label = new wxStaticText(OptionScrollWindow, wxID_ANY,
			wxT("(000, 000)"), wxDefaultPosition, wxDefaultSize, 0);
	RLC_Label->Wrap(-1);
	bSizer121108->Add(RLC_Label, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	bSizer1211->Add(bSizer121108, 0, wxEXPAND, 5);

	wxBoxSizer* bSizer121109;
	bSizer121109 = new wxBoxSizer(wxHORIZONTAL);

	CenterLine_radioBtn = new wxRadioButton(OptionScrollWindow, wxID_ANY,
			wxT("Center line"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer121109->Add(CenterLine_radioBtn, 1, wxALL, 5);

	NoRotationCheckBox = new wxCheckBox(OptionScrollWindow, wxID_ANY,
			wxT("No Rotation"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer121109->Add(NoRotationCheckBox, 0, wxALL, 5);

	bSizer1211->Add(bSizer121109, 0, wxEXPAND, 5);

	wxBoxSizer* bSizer121110;
	bSizer121110 = new wxBoxSizer(wxHORIZONTAL);

	CenterTop_RadioBtn = new wxRadioButton(OptionScrollWindow, wxID_ANY,
			wxT("Center Top"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer121110->Add(CenterTop_RadioBtn, 1, wxALL | wxALIGN_CENTER_VERTICAL,
			5);

	CenterTop_Label = new wxStaticText(OptionScrollWindow, wxID_ANY,
			wxT("(000, 000)"), wxDefaultPosition, wxDefaultSize, 0);
	CenterTop_Label->Wrap(-1);
	bSizer121110->Add(CenterTop_Label, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	bSizer1211->Add(bSizer121110, 0, wxEXPAND, 5);

	wxBoxSizer* bSizer121111;
	bSizer121111 = new wxBoxSizer(wxHORIZONTAL);

	CenterBottom_RadioBtn = new wxRadioButton(OptionScrollWindow, wxID_ANY,
			wxT("Center Bottom"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer121111->Add(CenterBottom_RadioBtn, 1, wxALL | wxALIGN_CENTER_VERTICAL,
			5);

	CenterBottom_Label = new wxStaticText(OptionScrollWindow, wxID_ANY,
			wxT("(000, 000)"), wxDefaultPosition, wxDefaultSize, 0);
	CenterBottom_Label->Wrap(-1);
	bSizer121111->Add(CenterBottom_Label, 0, wxALL | wxALIGN_CENTER_VERTICAL,
			5);

	bSizer1211->Add(bSizer121111, 0, wxEXPAND, 5);

	bSizer1211->Add(
			new wxStaticLine(OptionScrollWindow, wxID_ANY, wxDefaultPosition,
					wxSize(-1, 1), wxLI_HORIZONTAL), 0, wxEXPAND | wxALL, 5);

	wxBoxSizer* bSizer121112;
	bSizer121112 = new wxBoxSizer(wxHORIZONTAL);

	Rotation_Label = new wxStaticText(OptionScrollWindow, wxID_ANY,
			wxT("Rotation (degrees)"), wxDefaultPosition, wxDefaultSize, 0);
	Rotation_Label->Wrap(-1);
	bSizer121112->Add(Rotation_Label, 1, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	RotationValue_label = new wxStaticText(OptionScrollWindow, wxID_ANY,
			wxT("360"), wxDefaultPosition, wxDefaultSize, 0);
	RotationValue_label->Wrap(-1);
	bSizer121112->Add(RotationValue_label, 0, wxALL | wxALIGN_CENTER_VERTICAL,
			5);

	bSizer1211->Add(bSizer121112, 0, wxEXPAND, 5);

	wxBoxSizer* bSizer121113;
	bSizer121113 = new wxBoxSizer(wxHORIZONTAL);

	CenterPoint_label = new wxStaticText(OptionScrollWindow, wxID_ANY,
			wxT("Center point"), wxDefaultPosition, wxDefaultSize, 0);
	CenterPoint_label->Wrap(-1);
	bSizer121113->Add(CenterPoint_label, 1, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	CenterPointValue_label = new wxStaticText(OptionScrollWindow, wxID_ANY,
			wxT("(000,000)"), wxDefaultPosition, wxDefaultSize, 0);
	CenterPointValue_label->Wrap(-1);
	bSizer121113->Add(CenterPointValue_label, 0,
			wxALL | wxALIGN_CENTER_VERTICAL, 5);

	bSizer1211->Add(bSizer121113, 0, wxEXPAND, 5);

	bSizer1211->Add(
			new wxStaticLine(OptionScrollWindow, wxID_ANY, wxDefaultPosition,
					wxSize(-1, 1), wxLI_HORIZONTAL), 0, wxEXPAND | wxALL, 5);

	wxBoxSizer* bSizer121114;
	bSizer121114 = new wxBoxSizer(wxHORIZONTAL);
	bSizer121114->SetMinSize( -1, 30);

	NextObjectButton = new wxButton(OptionScrollWindow, wxID_ANY,
			wxT("Next Object"), wxDefaultPosition, wxDefaultSize, 0);
	bSizer121114->Add(NextObjectButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);


	wxBoxSizer* bSizer1211141;
	bSizer1211141 = new wxBoxSizer(wxVERTICAL);

	bSizer121114->Add(bSizer1211141, 1, wxEXPAND, 5);

	SkipButton = new wxButton(OptionScrollWindow, wxID_ANY, wxT("Skip"),
			wxDefaultPosition, wxDefaultSize, 0);
	bSizer121114->Add(SkipButton, 0, wxALL | wxALIGN_CENTER_VERTICAL, 5);

	bSizer1211->Add(bSizer121114, 0, wxEXPAND, 5);

	wxBoxSizer* bSizer121115;
	bSizer121115 = new wxBoxSizer(wxHORIZONTAL);

	ResetButton = new wxButton(OptionScrollWindow, wxID_ANY, wxT("Reset image"),
			wxDefaultPosition, wxDefaultSize, 0);
	bSizer121115->Add(ResetButton, 0, wxALL, 5);

	wxBoxSizer* bSizer1211151;
	bSizer1211151 = new wxBoxSizer(wxVERTICAL);

	bSizer121115->Add(bSizer1211151, 1, wxEXPAND, 5);

	DoneButton = new wxButton(OptionScrollWindow, wxID_ANY, wxT("Done"),
			wxDefaultPosition, wxDefaultSize, 0);
	bSizer121115->Add(DoneButton, 0, wxALL, 5);

	bSizer1211->Add(bSizer121115, 0, wxEXPAND, 5);

	OptionScrollWindow->SetSizer(bSizer1211);
	OptionScrollWindow->Layout();
	bSizer1211->Fit(OptionScrollWindow);
	bSizer121->Add(OptionScrollWindow, 1, wxEXPAND | wxALL, 5);

	bSizer12->Add(bSizer121, 1, wxEXPAND, 5);

	bSizer1->Add(bSizer12, 0, wxEXPAND, 5);

	this->SetSizer(bSizer1);
	this->Layout();
	bSizer1->Fit(this);

	// Connect Events
	this->Connect(wxEVT_SIZE, wxSizeEventHandler( GUIFrame::OnSizeChange ));
	ImageField->Connect(wxEVT_LEAVE_WINDOW,
			wxMouseEventHandler( GUIFrame::OnLeftImageField ), NULL, this);
	ImageField->Connect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( GUIFrame::OnLeftMousePressed ), NULL, this);
	ImageField->Connect(wxEVT_LEFT_UP,
			wxMouseEventHandler( GUIFrame::OnLeftMouseRelease ), NULL, this);
	ImageField->Connect(wxEVT_MOTION,
			wxMouseEventHandler( GUIFrame::OnImageMotion ), NULL, this);
	ColorSlider->Connect(wxEVT_SCROLL_TOP,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Connect(wxEVT_SCROLL_BOTTOM,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Connect(wxEVT_SCROLL_LINEUP,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Connect(wxEVT_SCROLL_LINEDOWN,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Connect(wxEVT_SCROLL_PAGEUP,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Connect(wxEVT_SCROLL_PAGEDOWN,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Connect(wxEVT_SCROLL_THUMBTRACK,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Connect(wxEVT_SCROLL_THUMBRELEASE,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Connect(wxEVT_SCROLL_CHANGED,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Connect(wxEVT_SCROLL_CHANGED,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ZoomButton->Connect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnZoom ), NULL, this);
	OriginalImageButton->Connect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnOriginal ), NULL, this);
	ObjectComboBox->Connect(wxEVT_COMMAND_TEXT_UPDATED,
			wxCommandEventHandler( GUIFrame::OnComboSelect ), NULL, this);
	CrateButton->Connect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnCrateButton ), NULL, this);
	NoRotationCheckBox->Connect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( GUIFrame::NoRotation ), NULL, this);
	NextObjectButton->Connect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnNextObjectButton ), NULL, this);
	SkipButton->Connect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( GUIFrame::OnSkip ), NULL, this);
	ResetButton->Connect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnReset ), NULL, this);
	DoneButton->Connect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnDoneButton ), NULL, this);
	ZoomBox_radioBtn->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( GUIFrame::OnZoomChange ), NULL, this );
	ZoomCheckBox->Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( GUIFrame::OnZoomChange ), NULL, this );
}

GUIFrame::~GUIFrame() {

	this->GetParent()->Show(true);
	this->GetParent()->SetFocus();

	// Disconnect Events
	this->Disconnect(wxEVT_SIZE, wxSizeEventHandler( GUIFrame::OnSizeChange ));
	ImageField->Disconnect(wxEVT_LEAVE_WINDOW,
			wxMouseEventHandler( GUIFrame::OnLeftImageField ), NULL, this);
	ImageField->Disconnect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( GUIFrame::OnLeftMousePressed ), NULL, this);
	ImageField->Disconnect(wxEVT_LEFT_UP,
			wxMouseEventHandler( GUIFrame::OnLeftMouseRelease ), NULL, this);
	ImageField->Disconnect(wxEVT_MOTION,
			wxMouseEventHandler( GUIFrame::OnImageMotion ), NULL, this);
	ColorSlider->Disconnect(wxEVT_SCROLL_TOP,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Disconnect(wxEVT_SCROLL_BOTTOM,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Disconnect(wxEVT_SCROLL_LINEUP,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Disconnect(wxEVT_SCROLL_LINEDOWN,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Disconnect(wxEVT_SCROLL_PAGEUP,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Disconnect(wxEVT_SCROLL_PAGEDOWN,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Disconnect(wxEVT_SCROLL_THUMBTRACK,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Disconnect(wxEVT_SCROLL_THUMBRELEASE,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Disconnect(wxEVT_SCROLL_CHANGED,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ColorSlider->Disconnect(wxEVT_SCROLL_CHANGED,
			wxScrollEventHandler( GUIFrame::OnColorSlider ), NULL, this);
	ZoomButton->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnZoom ), NULL, this);
	OriginalImageButton->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnOriginal ), NULL, this);
	ObjectComboBox->Disconnect(wxEVT_COMMAND_TEXT_UPDATED,
			wxCommandEventHandler( GUIFrame::OnComboSelect ), NULL, this);
	CrateButton->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnCrateButton ), NULL, this);
	NoRotationCheckBox->Disconnect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( GUIFrame::NoRotation ), NULL, this);
	NextObjectButton->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnNextObjectButton ), NULL, this);
	SkipButton->Disconnect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( GUIFrame::OnSkip ), NULL, this);
	ResetButton->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnReset ), NULL, this);
	DoneButton->Disconnect(wxEVT_COMMAND_BUTTON_CLICKED,
			wxCommandEventHandler( GUIFrame::OnDoneButton ), NULL, this);
	ZoomBox_radioBtn->Disconnect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( GUIFrame::OnZoomChange ), NULL, this);
	ZoomCheckBox->Disconnect(wxEVT_LEFT_DOWN,
			wxMouseEventHandler( GUIFrame::OnZoomChange ), NULL, this);
}
