#include "CrateGUI/CrateAppImpl.h"
#include "CrateGUI/CrateApp.h"

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

void CrateAppImpl::OnSizeChange(wxSizeEvent& event){

	MessageLabel->SetLabel(
			wxT("The values of the box and rotation line are reset"));

	wxSize imageMaxSize = this->GetSize();
	imageMaxSize.SetWidth(
			imageMaxSize.GetWidth() - bSizer121->GetSize().GetWidth());
	imageMaxSize.SetHeight(
			imageMaxSize.GetHeight() - MessageLabel->GetSize().GetHeight()
					- 20);

	double heightScale, widthScale;
	if (!zoom) {
		heightScale = (double) image.GetHeight()
				/ (double) imageMaxSize.GetHeight();
		widthScale = (double) image.GetWidth()
				/ (double) imageMaxSize.GetWidth();
	} else {
		heightScale = (double) zoomImage.GetHeight()
				/ (double) imageMaxSize.GetHeight();
		widthScale = (double) zoomImage.GetWidth()
				/ (double) imageMaxSize.GetWidth();
	}

	if(widthScale < heightScale){
		Scale = heightScale;
	}else{
		Scale = widthScale;
	}

	QRCorner = wxPoint(0, 0);
	OppositeCorner = wxPoint(0, 0);

	zoomX = 0;
	zoomY = 0;
	zoomWidth = 0;
	zoomHeight = 0;

	LineRDB->SetValue(true);
	UpdateImageField();

	this->Layout();
}

void CrateAppImpl::OnLeaveImageField(wxMouseEvent& event){
	mousePressedInImageField = false;
	drawCrateAttributes();
}

void CrateAppImpl::OnLeftMousePressed(wxMouseEvent& event){
	mousePressedInImageField = true;

	if(LineRDB->GetValue() || QRCodeCornerRDB->GetValue()){
		QRCorner = event.GetPosition() + wxPoint(coordinateOffset, coordinateOffset);
		std::stringstream coordinate;
		if(!zoom){
			coordinate << "(" << (int)(QRCorner.x * Scale) << ", " << (int)(QRCorner.y * Scale) << ")";
		}else{
			coordinate << "(" << (int)(QRCorner.x * Scale + zoomX) << ", " << (int)(QRCorner.y * Scale + zoomY) << ")";
		}
		QRCodeCornerLabel->SetLabel(wxString(coordinate.str().c_str(), wxConvLocal));
	}else if(OppositeCornerRDB->GetValue()){
		OppositeCorner = event.GetPosition() + wxPoint(coordinateOffset, coordinateOffset);
		std::stringstream coordinate;
		if(!zoom){
			coordinate << "(" << (int)(OppositeCorner.x * Scale) << ", " << (int)(OppositeCorner.y * Scale) << ")";
		}else{
			coordinate << "(" << (int)(OppositeCorner.x * Scale + zoomX) << ", " << (int)(OppositeCorner.y * Scale + zoomY) << ")";
		}
		OppositeCornerLabel->SetLabel(wxString(coordinate.str().c_str(), wxConvLocal));
	}else if(ZoomBox_radioBtn->GetValue()){
		zoomX = event.GetX() + coordinateOffset;
		zoomY = event.GetY() + coordinateOffset;
		zoomWidth = 0;
		zoomHeight = 0;
	}

	calculateFiducialPoints();
	drawCrateAttributes();
}

void CrateAppImpl::OnLeftMouseRelease(wxMouseEvent& event){
	mousePressedInImageField = false;

	if(QRCodeCornerRDB->GetValue()){
		QRCorner = event.GetPosition() + wxPoint(coordinateOffset, coordinateOffset);
		std::stringstream coordinate;
		if(!zoom){
			coordinate << "(" << (int)(QRCorner.x * Scale) << ", " << (int)(QRCorner.y * Scale) << ")";
		}else{
			coordinate << "(" << (int)(QRCorner.x * Scale + zoomX) << ", " << (int)(QRCorner.y * Scale + zoomY) << ")";
		}
		QRCodeCornerLabel->SetLabel(wxString(coordinate.str().c_str(), wxConvLocal));
	}else if(LineRDB->GetValue() || OppositeCornerRDB->GetValue()){
		OppositeCorner = event.GetPosition() + wxPoint(coordinateOffset, coordinateOffset);
		std::stringstream coordinate;
		if(!zoom){
			coordinate << "(" << (int)(OppositeCorner.x * Scale) << ", " << (int)(OppositeCorner.y * Scale) << ")";
		}else{
			coordinate << "(" << (int)(OppositeCorner.x * Scale + zoomX) << ", " << (int)(OppositeCorner.y * Scale + zoomY) << ")";
		}
		OppositeCornerLabel->SetLabel(wxString(coordinate.str().c_str(), wxConvLocal));
	}else if(ZoomBox_radioBtn->GetValue()){
		zoomWidth = event.GetX() + coordinateOffset - zoomX;
		zoomHeight = event.GetY() + coordinateOffset - zoomY;
	}

	calculateFiducialPoints();
	drawCrateAttributes();
}

void CrateAppImpl::OnImageMotion(wxMouseEvent& event){
	if(mousePressedInImageField){

		if(QRCodeCornerRDB->GetValue()){
			QRCorner = event.GetPosition() + wxPoint(coordinateOffset, coordinateOffset);
		}else if(LineRDB->GetValue() || OppositeCornerRDB->GetValue()){
			OppositeCorner = event.GetPosition() + wxPoint(coordinateOffset, coordinateOffset);
		}else if(ZoomBox_radioBtn->GetValue()){
			zoomWidth = event.GetX() + coordinateOffset - zoomX;
			zoomHeight = event.GetY() + coordinateOffset - zoomY;
		}

		calculateFiducialPoints();
		drawCrateAttributes();
	}
}

void CrateAppImpl::OnColorSlider(wxScrollEvent& event){
	drawCrateAttributes();
}

void CrateAppImpl::OnZoomRadioButton(wxMouseEvent& event){
	if(!ZoomBox_radioBtn->GetValue()){
		ZoomCheckBox->SetValue(true);
		ZoomBox_radioBtn->SetValue(true);
		drawCrateAttributes();
	}
}

void CrateAppImpl::OnZoomChange(wxMouseEvent& event) {
	if (!ZoomCheckBox->GetValue()) {
		ZoomBox_radioBtn->SetValue(true);
		ZoomCheckBox->SetValue(true);
	} else {
		LineRDB->SetValue(true);
		ZoomCheckBox->SetValue(false);
	}
	drawCrateAttributes();
}

void CrateAppImpl::OnZoom(wxCommandEvent& event) {
	if (zoomX > 0 && zoomY > 0 && zoomWidth > 0 && zoomHeight > 0) {
		zoomImage = image.Size(
				wxSize(zoomX * Scale + zoomWidth * Scale,
						zoomY * Scale + zoomHeight * Scale),
				wxPoint(zoomX * Scale, zoomY * Scale));

		zoomImage = (zoomImage.Rotate90(true)).Rotate90(true);
		zoomImage = zoomImage.Size(
				wxSize(zoomWidth * Scale, zoomHeight * Scale),
				wxPoint(0, 0));
		zoomImage = (zoomImage.Rotate90(true)).Rotate90(true);

		MessageLabel->SetLabel(
				wxT("The values of the box and rotation line are reset"));
		wxSize imageMaxSize = this->GetSize();
		imageMaxSize.SetWidth(
				imageMaxSize.GetWidth() - bSizer121->GetSize().GetWidth());
		imageMaxSize.SetHeight(
				imageMaxSize.GetHeight() - MessageLabel->GetSize().GetHeight()
						- 20);

		double heightScale = (double) zoomImage.GetHeight()
				/ (double) imageMaxSize.GetHeight();
		double widthScale = (double) zoomImage.GetWidth()
				/ (double) imageMaxSize.GetWidth();

		if(widthScale < heightScale){
			Scale = heightScale;
		}else{
			Scale = widthScale;
		}

		QRCodeCornerLabel->SetLabel(wxString(("(000,000)"), wxConvLocal));
		OppositeCornerLabel->SetLabel(wxString(("(000,000)"), wxConvLocal));

		zoom = true;

		ZoomButton->Enable(false);
		OriginalImageButton->Enable(true);
		ZoomBox_radioBtn->Enable(false);
		LineRDB->SetValue(true);

		UpdateImageField();

		this->Layout();
	} else {
		MessageLabel->SetLabel(wxT("Draw box to zoom into"));
	}
}

void CrateAppImpl::OnOriginal(wxCommandEvent& event) {
	MessageLabel->SetLabel(
			wxT("The values of the box and rotation line are reset"));
	wxSize imageMaxSize = this->GetSize();
	imageMaxSize.SetWidth(
			imageMaxSize.GetWidth() - bSizer121->GetSize().GetWidth());
	imageMaxSize.SetHeight(
			imageMaxSize.GetHeight() - MessageLabel->GetSize().GetHeight()
					- 20);

	double heightScale = (double) image.GetHeight()
			/ (double) imageMaxSize.GetHeight();
	double widthScale = (double) image.GetWidth()
			/ (double) imageMaxSize.GetWidth();

	if(widthScale < heightScale){
		Scale = heightScale;
	}else{
		Scale = widthScale;
	}

	QRCorner = wxPoint(0, 0);
	OppositeCorner = wxPoint(0, 0);

	zoom = false;

	ZoomButton->Enable(true);
	ZoomBox_radioBtn->Enable(true);
	OriginalImageButton->Enable(false);

	drawCrateAttributes();
	this->Layout();
}

void CrateAppImpl::OnNextObjectButton(wxCommandEvent& event){
	double temp;

	SkipButton->Enable(false);

	if(!AmountOfObjectsTxtField->GetValue().ToDouble(&temp) || temp < 1){
		MessageLabel->SetLabel(wxT("Give a amount of crate number"));
		return;
	}

	amountOfCrates = (int)temp;

	if(QRCorner == wxPoint(0, 0) &&
			OppositeCorner == wxPoint(0, 0)){
		MessageLabel->SetLabel(wxT("Crate corners are not defined"));
		LineRDB->SetValue(true);
		LineRDB->SetFocus();
		return;

	}else if(QRCorner == wxPoint(0, 0)){
		MessageLabel->SetLabel(wxT("QR code crate corner is not defined"));
		QRCodeCornerRDB->SetValue(true);
		QRCodeCornerRDB->SetFocus();
		return;

	}else if(OppositeCorner == wxPoint(0, 0)){
		MessageLabel->SetLabel(wxT("Crate corner opposite of the QR code is not defined"));
		OppositeCornerRDB->SetValue(true);
		OppositeCornerRDB->SetFocus();
		return;

	}


	if(QRCodeTextBox->GetValue() == wxT("")){
		QRCodeTextBox->SetFocus();
		MessageLabel->SetLabel(wxT("QR code value is not given"));
		return;
	}

	if (XMLOption == Edit && currentCrateNumber == 1) {
		//if images available within the xml need to overwritten
		foreach(boost::property_tree::ptree::value_type & imageValue,
				pt.get_child("Test_set"))
		{
			//<xmlattr>.path is <image path="/home/...." />
			boost::filesystem::path temp = imageValue.second.get("<xmlattr>.path", "");
			if (temp.leaf() == (*imagePathsIt).leaf()) {
				tempValue = &(imageValue.second);
				//if the image name is found within the xml
				AlreadyInXML = true;

				imageValue.second.erase("property");
				imageValue.second.erase("category");
				imageValue.second.erase("object");
				break;

			}
		}
	}

	if (!AlreadyInXML && currentCrateNumber == 0) {
		tempValue = &(pt.add("Test_set.image", ""));

		boost::filesystem::path p = *imagePathsIt;
		getPathFromChosenDir(p);
		tempValue->put("<xmlattr>.path", p);
	}

	if(currentCrateNumber == 0){
		boost::property_tree::ptree* property = &(tempValue->add("category", ""));
		property->put("<xmlattr>.name", "background");
		property->put("<xmlattr>.value", BackgroundComboBox->GetValue().ToAscii());
		property = &(tempValue->add("category", ""));
		property->put("<xmlattr>.name", "light");
		property->put("<xmlattr>.value", LightingComboBox->GetValue().ToAscii());
		property = &(tempValue->add("category", ""));
		property->put("<xmlattr>.name", "perspective");
		property->put("<xmlattr>.value", PerspectiveComboBox->GetValue().ToAscii());

	}

	QRCode = (std::string)QRCodeTextBox->GetValue().ToAscii();

	if(zoom){
		center.m_x += zoomX;
		center.m_y += zoomY;
		fid1.m_x += zoomX;
		fid1.m_y += zoomY;
		fid2.m_x += zoomX;
		fid2.m_y += zoomY;
		fid3.m_x += zoomX;
		fid3.m_y += zoomY;
	}

	boost::property_tree::ptree& object = tempValue->add("object", "");
	boost::property_tree::ptree* property = &(object.add("property", ""));
	property->put("<xmlattr>.name", "x");
	property->put("<xmlattr>.value", center.m_x * Scale);
	property = &(object.add("property", ""));
	property->put("<xmlattr>.name", "y");
	property->put("<xmlattr>.value", center.m_y * Scale);
	property = &(object.add("property", ""));
	property->put("<xmlattr>.name", "fid1.x");
	property->put("<xmlattr>.value", fid1.m_x * Scale);
	property = &(object.add("property", ""));
	property->put("<xmlattr>.name", "fid1.y");
	property->put("<xmlattr>.value", fid1.m_y * Scale);
	property = &(object.add("property", ""));
	property->put("<xmlattr>.name", "fid2.x");
	property->put("<xmlattr>.value", fid2.m_x * Scale);
	property = &(object.add("property", ""));
	property->put("<xmlattr>.name", "fid2.y");
	property->put("<xmlattr>.value", fid2.m_y * Scale);
	property = &(object.add("property", ""));
	property->put("<xmlattr>.name", "fid3.x");
	property->put("<xmlattr>.value", fid3.m_x * Scale);
	property = &(object.add("property", ""));
	property->put("<xmlattr>.name", "fid3.y");
	property->put("<xmlattr>.value", fid3.m_y * Scale);
	property = &(object.add("property", ""));
	property->put("<xmlattr>.name", "qrcode");
	property->put("<xmlattr>.value", QRCode);

	boost::property_tree::xml_writer_settings<char> w('\t', 1);
	boost::property_tree::write_xml(xmlPath.string().c_str(), pt, std::locale(), w);

	QRCorner = wxPoint(0, 0);
	OppositeCorner = wxPoint(0, 0);

	zoomX = 0;
	zoomY = 0;
	zoomWidth = 0;
	zoomHeight = 0;

	currentCrateNumber++;

	QRCodeCornerLabel->SetLabel(wxT("(000,000)"));
	OppositeCornerLabel->SetLabel(wxT("(000,000)"));

	if(currentCrateNumber == amountOfCrates){
		zoom = false;

		OriginalImageButton->Enable(false);
		ZoomButton->Enable(true);
		ZoomBox_radioBtn->Enable(true);
		NextImage();
	}else{
		UpdateImageField();
		std::stringstream s;
		s << "Next image, current object number: " << currentCrateNumber + 1;
		MessageLabel->SetLabel(wxString(s.str().c_str(), wxConvLocal));
	}
}

void CrateAppImpl::OnSkip(wxMouseEvent& event){

	QRCorner = wxPoint(0, 0);
	OppositeCorner = wxPoint(0, 0);

	zoomX = 0;
	zoomY = 0;
	zoomWidth = 0;
	zoomHeight = 0;

	zoom = false;

	QRCodeCornerLabel->SetLabel(wxT("(000,000)"));
	OppositeCornerLabel->SetLabel(wxT("(000,000)"));

	NextImage();
}

void CrateAppImpl::OnReset(wxCommandEvent& event){

	QRCorner = wxPoint(0, 0);
	OppositeCorner = wxPoint(0, 0);

	zoomX = 0;
	zoomY = 0;
	zoomWidth = 0;
	zoomHeight = 0;

	zoom = false;

	QRCodeCornerLabel->SetLabel(wxT("(000,000)"));
	OppositeCornerLabel->SetLabel(wxT("(000,000)"));

	if(currentCrateNumber > 0){
		foreach(boost::property_tree::ptree::value_type & imageValue,
						pt.get_child("Test_set"))
		{
			boost::filesystem::path temp = imageValue.second.get("<xmlattr>.path", "");
			if (temp.leaf() == (*imagePathsIt).leaf()) {
				tempValue = &(imageValue.second);
				//if the image name is found within the xml
				AlreadyInXML = true;

				imageValue.second.erase("property");
				imageValue.second.erase("category");
				imageValue.second.erase("object");
				break;

			}
		}
		AlreadyInXML = true;
	}
	currentCrateNumber = 0;

	std::stringstream s;
	s << "Image reset, current object number: " << currentCrateNumber + 1;
	MessageLabel->SetLabel(wxString(s.str().c_str(), wxConvLocal));

	UpdateImageField();
}

void CrateAppImpl::OnDoneButton(wxCommandEvent& event){
	this->Close(true);
}
