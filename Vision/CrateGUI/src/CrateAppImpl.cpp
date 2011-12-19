#include "CrateGUI/CrateAppImpl.h"
#include "CrateGUI/CrateApp.h"

#include "DetectQRCode/BarcodeDetector.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <math.h>

#include <wx/pen.h>
#include <wx/colour.h>
#include <wx/dcclient.h>

#ifdef __CDT_PARSER__
#define foreach(a, b) for(a : b)
#else
#define foreach(a, b) BOOST_FOREACH(a, b)
#endif

void CrateAppImpl::setCurrentImagePath(const wxString& path) {
	currentImagePath = path;
	image = wxImage(path);

	wxSize imageMaxSize = this->GetSize();
	imageMaxSize.SetWidth(
			imageMaxSize.GetWidth() - bSizer121->GetSize().GetWidth());
	imageMaxSize.SetHeight(
			imageMaxSize.GetHeight() - MessageLabel->GetSize().GetHeight()
					- 20);

	double heightScale = (double) image.GetHeight()
			/ (double) imageMaxSize.GetHeight();
	double widthScale = (double) image.GetWidth() / (double) imageMaxSize.GetWidth();

	if(widthScale < heightScale){
		Scale = heightScale;
	}else{
		Scale = widthScale;
	}

	UpdateImageField();
	this->Layout();
}

void CrateAppImpl::getPathFromChosenDir(boost::filesystem::path& path) {

	std::stringstream s;
	while (path.leaf() != dirPath.leaf()) {
		std::stringstream ss(s.str().c_str());
		s.str("");
		s << "/" << path.leaf() << ss.str().c_str();
		path = path.parent_path();
	}
	path = s.str().c_str();

}

void CrateAppImpl::UpdateImageField() {
	if (!zoom) {
		wxImage copy = image.Copy();
		copy.Rescale(image.GetWidth() / Scale, image.GetHeight() / Scale);
		ImageField->SetBitmap(copy);
	} else {
		wxImage copy = zoomImage.Copy();
		copy.Rescale(zoomImage.GetWidth() / Scale, zoomImage.GetHeight() / Scale);
		ImageField->SetBitmap(copy);
	}
	ImageField->Refresh();
}

void CrateAppImpl::calculateFiducialPoints(){

	double distance = sqrt(pow(double(QRCorner.x - OppositeCorner.x), 2) + pow(double(QRCorner.y - OppositeCorner.y), 2));
	double angle = atan2(double(QRCorner.y - OppositeCorner.y), double(OppositeCorner.x - QRCorner.x));
	center = wxPoint2DDouble(QRCorner.x + (distance / 2.0) * cos(-angle),
			QRCorner.y + (distance / 2.0) * sin(-angle));

	fid1 = wxPoint2DDouble(center.m_x + (FID_OFFSET*(distance/LINE_LENGTH)) * cos(-angle-M_PI/2.0),
					center.m_y + (FID_OFFSET*(distance/LINE_LENGTH)) * sin(-angle-M_PI/2.0));
	fid2 = wxPoint2DDouble(center.m_x + (FID_OFFSET*(distance/LINE_LENGTH)) * cos(-angle),
			center.m_y + (FID_OFFSET*(distance/LINE_LENGTH)) * sin(-angle));
	fid3 = wxPoint2DDouble(center.m_x + (FID_OFFSET*(distance/LINE_LENGTH)) * cos(-angle+M_PI/2.0),
			center.m_y + (FID_OFFSET*(distance/LINE_LENGTH)) * sin(-angle+M_PI/2.0));

	if(!mousePressedInImageField){
		std::string code;
		if((code = getBarcode(distance, angle)) != ""){
			QRCodeTextBox->SetValue(wxString(code.c_str(), wxConvLocal));
		}
	}
}

std::string CrateAppImpl::getBarcode(int distance, double angle){
	std::string result = "";

	if(distance != 0){
		int crateSideLenght = sin(45) * abs(distance);
		cv::Rect barcodeBox(center.m_x * Scale - (crateSideLenght/2.0) * Scale, center.m_y * Scale - (crateSideLenght/2.0) * Scale, crateSideLenght * Scale, crateSideLenght * Scale);

		if(barcodeBox.width > 0 && barcodeBox.height > 0 &&
				barcodeBox.x > 0 && barcodeBox.y > 0){

			cv::Mat barcode = (cv::imread((std::string)currentImagePath.ToAscii()))(barcodeBox);

			DetectBarcode detector;
			detector.detect(barcode, result);

			if(result != ""){
				return result;
			}

			cv::Mat rotationMatrix = cv::getRotationMatrix2D( cv::Point(barcode.cols/2, barcode.rows/2), (angle / (double)M_PI) * 180, 1.0 );

			cv::Mat temp = barcode.clone();
			cv::warpAffine( temp, barcode, rotationMatrix, barcode.size() );
			temp.release();

			detector.detect(barcode, result);
		}
	}

	return result;
}

void CrateAppImpl::drawCrateAttributes(){
	UpdateImageField();
	ImageField->Update();

	int color = ColorSlider->GetValue();

	if(QRCorner != wxPoint(0, 0) &&
			OppositeCorner != wxPoint(0, 0)){

		wxPaintDC dc(ImageField);
		dc.SetPen(wxPen(wxColour(color, color, color), 2, wxSOLID));
		dc.SetBrush(wxBrush(wxColour(0, 0, 0), wxTRANSPARENT));
		dc.DrawLine(QRCorner, OppositeCorner);

		double distance = sqrt(pow(double(QRCorner.x - OppositeCorner.x), 2) + pow(double(QRCorner.y - OppositeCorner.y), 2));
		double angle = atan2(double(QRCorner.y - OppositeCorner.y), double(OppositeCorner.x - QRCorner.x));
		double fidRadius = FID_RADIUS*(distance/LINE_LENGTH);

		dc.SetPen(wxPen(wxColour(255-color, color, color), 2, wxSOLID));
		dc.DrawCircle(fid1.m_x, fid1.m_y, fidRadius);
		dc.DrawLine(fid1.m_x + fidRadius * cos(-angle-M_PI/4.0),
				fid1.m_y + fidRadius * sin(-angle-M_PI/4.0),
				fid1.m_x - fidRadius * cos(-angle-M_PI/4.0),
				fid1.m_y - fidRadius * sin(-angle-M_PI/4.0));
		dc.DrawLine(fid1.m_x + fidRadius * cos(-angle+M_PI/4.0),
				fid1.m_y + fidRadius * sin(-angle+M_PI/4.0),
				fid1.m_x - fidRadius * cos(-angle+M_PI/4.0),
				fid1.m_y - fidRadius * sin(-angle+M_PI/4.0));

		dc.SetPen(wxPen(wxColour(color, 255-color, color), 2, wxSOLID));
		dc.DrawCircle(fid2.m_x, fid2.m_y, fidRadius);
		dc.DrawLine(fid2.m_x + fidRadius * cos(-angle-M_PI/4.0),
				fid2.m_y + fidRadius * sin(-angle-M_PI/4.0),
				fid2.m_x - fidRadius * cos(-angle-M_PI/4.0),
				fid2.m_y - fidRadius * sin(-angle-M_PI/4.0));
		dc.DrawLine(fid2.m_x + fidRadius * cos(-angle+M_PI/4.0),
				fid2.m_y + fidRadius * sin(-angle+M_PI/4.0),
				fid2.m_x - fidRadius * cos(-angle+M_PI/4.0),
				fid2.m_y - fidRadius * sin(-angle+M_PI/4.0));

		dc.SetPen(wxPen(wxColour(color, color, 255-color), 2, wxSOLID));
		dc.DrawCircle(fid3.m_x, fid3.m_y, fidRadius);
		dc.DrawLine(fid3.m_x + fidRadius * cos(-angle-M_PI/4.0),
				fid3.m_y + fidRadius * sin(-angle-M_PI/4.0),
				fid3.m_x - fidRadius * cos(-angle-M_PI/4.0),
				fid3.m_y - fidRadius * sin(-angle-M_PI/4.0));
		dc.DrawLine(fid3.m_x + fidRadius * cos(-angle+M_PI/4.0),
				fid3.m_y + fidRadius * sin(-angle+M_PI/4.0),
				fid3.m_x - fidRadius * cos(-angle+M_PI/4.0),
				fid3.m_y - fidRadius * sin(-angle+M_PI/4.0));
	}

	if(zoomWidth != 0 && zoomHeight != 0 &&
			zoomX != 0 && zoomY != 0 &&
			ZoomCheckBox->GetValue() && !zoom){

		wxPaintDC dc(ImageField);
		dc.SetPen(wxPen(wxColour(color, color, 255-color), 2, wxSHORT_DASH));
		dc.SetBrush(wxBrush(wxColour(0, 0, 0), wxTRANSPARENT));
		dc.DrawRectangle(wxPoint(zoomX, zoomY), wxSize(zoomWidth, zoomHeight));
	}
}

void CrateAppImpl::NextImage() {

	currentCrateNumber = 0;

	imagePathsIt++;
	if (imagePathsIt < imagePaths.end()) {

		MessageLabel->SetLabel(wxT("Next Image"));
		if (XMLOption == Edit) {
			//Disables/enables the skip button
			SkipButton->Enable(false);
			foreach( boost::property_tree::ptree::value_type& tempValue, pt.get_child("Test_set") )
			{

				boost::filesystem::path temp = tempValue.second.get("<xmlattr>.path", "");
				if (temp.leaf() == (*imagePathsIt).leaf()) {

					int objectCount = 0;
					foreach( boost::property_tree::ptree::value_type& imageCategory,
					tempValue.second.get_child(tempValue.second.data()) )
					{
						if (imageCategory.first == "category") {

							const char* s =
									(imageCategory.second.get(
											"<xmlattr>.value",
											"")).c_str();
							if (imageCategory.second.get(
									"<xmlattr>.name", "")
									== "background") {
								BackgroundComboBox->SetValue(
										wxString(s,
												wxConvLocal));
							} else if (imageCategory.second.get(
									"<xmlattr>.name", "")
									== "light") {
								LightingComboBox->SetValue(
										wxString(s,
												wxConvLocal));
							} else if (imageCategory.second.get(
									"<xmlattr>.name", "")
									== "perspective") {
								PerspectiveComboBox->SetValue(
										wxString(s,
												wxConvLocal));
							}

						} else if (imageCategory.first
								== "object") {
							objectCount++;
						}
					}
					std::stringstream s;
					s << objectCount;
					const wxString temp = wxString(s.str().c_str(),
							wxConvLocal);
					AmountOfObjectsTxtField->SetValue(temp);

					MessageLabel->SetLabel(
							wxT("Image is already available within the xml file, press skip button to go to next image."));
					SkipButton->Enable(true);

					break;
				}
			}
		}

		std::stringstream globalfile;
		globalfile << (*imagePathsIt).parent_path() << "/global.xml";
		if(boost::filesystem::exists(globalfile.str().c_str()) && boost::filesystem::is_regular_file(globalfile.str().c_str())){
			boost::property_tree::ptree temp;
			boost::property_tree::read_xml(globalfile.str().c_str(), temp);
			foreach( boost::property_tree::ptree::value_type& values, temp.get_child("global_values") )
			{
				if (values.first == "category") {

					const char* s = (values.second.get("<xmlattr>.value", "")).c_str();

					if (values.second.get("<xmlattr>.name", "")== "background") {
						BackgroundComboBox->SetValue( wxString(s, wxConvLocal));
					} else if (values.second.get( "<xmlattr>.name", "") == "light") {
						LightingComboBox->SetValue( wxString(s, wxConvLocal));
					} else if (values.second.get( "<xmlattr>.name", "")	== "perspective") {
						PerspectiveComboBox->SetValue( wxString(s, wxConvLocal));
					} else if (values.second.get( "<xmlattr>.name", "")	== "object-amount") {
						AmountOfObjectsTxtField->SetValue( wxString(s, wxConvLocal));
					} else if (values.second.get( "<xmlattr>.name", "")	== "qrcode") {
						QRCodeTextBox->SetValue( wxString(s, wxConvLocal));
					}
				}
			}
		}

		std::stringstream s;
		s << *imagePathsIt;
		wxString wxStringTemp((s.str().c_str()), wxConvLocal);
		setCurrentImagePath(wxStringTemp);
	} else {


		LineRDB->Enable(false);
		ZoomButton->Enable(false);
		SkipButton->Enable(false);
		ImageField->Enable(false);
		ResetButton->Enable(false);
		ColorSlider->Enable(false);
		ZoomCheckBox->Enable(false);
		QRCodeTextBox->Enable(false);
		m_staticText11->Enable(false);
		m_staticText29->Enable(false);
		QRCodeCornerRDB->Enable(false);
		LightingComboBox->Enable(false);
		NextObjectButton->Enable(false);
		ZoomBox_radioBtn->Enable(false);
		QRCodeCornerLabel->Enable(false);
		OppositeCornerRDB->Enable(false);
		BackgroundComboBox->Enable(false);
		OppositeCornerLabel->Enable(false);
		PerspectiveComboBox->Enable(false);
		OriginalImageButton->Enable(false);
		AmountOfObjectsLabel->Enable(false);
		AmountOfObjectsTxtField->Enable(false);

		MessageLabel->SetLabel(wxT("All images handled, press done to finish"));
	}
}

void CrateAppImpl::Start(){
	if (XMLOption == NewXML) {
		boost::property_tree::ptree temp;
		temp.add("Test_set", "");

		std::stringstream path;
		path << dirPath << "/crateTestSet.xml";
		xmlPath = path.str().c_str();
	}

	if (!boost::filesystem::is_regular_file(xmlPath.string().c_str())) {
		std::stringstream temp;
		temp << "Couldn't handle: " << xmlPath.string().c_str();
		MessageLabel->SetLabel(wxString(temp.str().c_str(), wxConvLocal));
		this->Close(true);
		return;
	}

	boost::property_tree::read_xml(xmlPath.string().c_str(), pt);
	imagePathsIt = imagePaths.begin() -1;
	NextImage();
}

CrateAppImpl::CrateAppImpl(wxWindow* parent, wxWindowID id,
		const wxString& title,
		const wxPoint& pos,
		const wxSize& size,
		long style) :
		CrateApp(parent, id, title, pos, size, style){

	wxImage::AddHandler(new wxJPEGHandler);
	wxImage::AddHandler(new wxPNGHandler);

	LineRDB->SetValue(true);

	Scale = 0;

	QRCorner = wxPoint(0, 0);
	OppositeCorner = wxPoint(0, 0);

	zoom = false;
	zoomX = 0;
	zoomY = 0;
	zoomWidth = 0;
	zoomHeight = 0;
	coordinateOffset = 4;

	mousePressedInImageField = false;

	OriginalImageButton->Enable(false);
	SkipButton->Enable(false);
}

CrateAppImpl::~CrateAppImpl(){
	this->GetParent()->Show(true);
}
