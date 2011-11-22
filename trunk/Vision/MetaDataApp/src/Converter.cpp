#include <MetaDataApp/ConverterGUI.h>

using namespace std;
using namespace boost::filesystem;

void GUIFrame::AddBackground(const wxString &backgroundName) {
	BackgroundComboBox->Append(backgroundName);
}
void GUIFrame::AddLight(const wxString &lightName) {
	LightingComboBox->Append(lightName);
}
void GUIFrame::AddPerspective(const wxString &perspectiveName) {
	PerspectiveComboBox->Append(perspectiveName);
}
void GUIFrame::AddObject(const wxString &objectName) {
	ObjectComboBox->Append(objectName);
}
void GUIFrame::ChangeImagePaths(const vector<path> &newImagePaths) {
	imagePaths = newImagePaths;
}
void GUIFrame::SetXMLPath(const path &XMLPath) {
	this->XMLPath = XMLPath;
}

void GUIFrame::ChangeImagePath(const wxString& path) {
	currentImagePath = path;
	image = wxImage(path);

	wxSize imageMaxSize = this->GetSize();
	imageMaxSize.SetWidth(
			imageMaxSize.GetWidth() - bSizer121->GetSize().GetWidth());
	imageMaxSize.SetHeight(
			imageMaxSize.GetHeight() - MessageLabel->GetSize().GetHeight()
					- 20);

	heightScale = (double) image.GetHeight()
			/ (double) imageMaxSize.GetHeight();
	widthScale = (double) image.GetWidth() / (double) imageMaxSize.GetWidth();

	UpdateImageField();
	this->Layout();
}

void GUIFrame::UpdateImageField() {
	if (!zoom) {
		wxImage copy = image.Copy();
		if (widthScale < heightScale) {
			copy.Rescale(image.GetWidth() / heightScale,
					image.GetHeight() / heightScale);
			widthScale = heightScale;
		} else {
			copy.Rescale(image.GetWidth() / widthScale,
					image.GetHeight() / widthScale);
			heightScale = widthScale;
		}
		ImageField->SetBitmap(copy);
	} else {
		wxImage copy = zoomImage.Copy();
		if (widthScale < heightScale) {
			copy.Rescale(zoomImage.GetWidth() / heightScale,
					zoomImage.GetHeight() / heightScale);
			widthScale = heightScale;
		} else {
			copy.Rescale(zoomImage.GetWidth() / widthScale,
					zoomImage.GetHeight() / widthScale);
			heightScale = widthScale;
		}
		ImageField->SetBitmap(copy);
	}
}

double GUIFrame::GetRotation() {
	if (!NoRotationCheckBox->GetValue()) {
		if (TopX > BottomX && TopY < BottomY) { //Right top -> Left under
			return (360.0
					- (atan(
							(double) (abs(TopX - BottomX))
									/ (double) (abs(TopY - BottomY))) / M_PI)
							* 180.0);

		} else if (TopX > BottomX && TopY > BottomY) { //Right under -> Left top
			return (180.0
					+ (atan(
							(double) (abs(TopX - BottomX))
									/ (double) (abs(TopY - BottomY))) / M_PI)
							* 180.0);

		} else if (TopX < BottomX && TopY > BottomY) { //Left under -> Right top
			return (180.0
					- (atan(
							(double) (abs(TopX - BottomX))
									/ (double) (abs(TopY - BottomY))) / M_PI)
							* 180.0);

		} else if (TopX < BottomX && TopY < BottomY) { //Left top -> Right under
			return ((atan(
					(double) (abs(TopX - BottomX))
							/ (double) (abs(TopY - BottomY))) / M_PI) * 180.0);
		}
	}
	return 0.0;
}

void GUIFrame::Start() {

	x = 0;
	y = 0;
	width = 0;
	height = 0;
	zoomX = 0;
	zoomY = 0;
	zoomWidth = 0;
	zoomHeight = 0;
	TopX = 0;
	TopY = 0;
	BottomX = 0;
	BottomY = 0;

	currentObjectNr = 1;
	mousePressedInImage = false;
	AlreadyInXML = false;
	noMarkerValues = true;
	zoom = false;

	SkipButton->Show(false);
	CrateButton->Show(false);
	OriginalImageButton->Show(false);

	stringstream s;
	s << XMLPath;

	if (editXML == NewXML) {
		boost::property_tree::ptree temp;
		temp.add("Test_set", "");

		boost::property_tree::write_xml(s.str().c_str(), temp);
	}

	if (!is_regular_file(s.str().c_str())) {
		stringstream temp;
		temp << "Couldn't handle: " << s.str().c_str();
		MessageLabel->SetLabel(wxString(temp.str().c_str(), wxConvLocal));
		return;
	}

	boost::property_tree::read_xml(s.str().c_str(), pt);
	imagePathsIt = imagePaths.begin();
	imagePathsIt--;
	this->Show(true);
	NextImage();

}

void GUIFrame::NextImage() {

	imagePathsIt++;
	if (imagePathsIt < imagePaths.end()) {

		MessageLabel->SetLabel(wxT("Next Image"));
		if (editXML == Edit) {
			//Disables/enables the skip button
			SkipButton->Show(false);
			BOOST_FOREACH( boost::property_tree::ptree::value_type& tempValue, pt.get_child("Test_set") )
			{

				path temp = tempValue.second.get("<xmlattr>.path", "");
				if (temp.leaf() == (*imagePathsIt).leaf()) {

					int objectCount = 0;
					BOOST_FOREACH( boost::property_tree::ptree::value_type& imageCategory,
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
								== "property"
								&& imageCategory.second.get(
										"<xmlattr>.nsme", "")
										== "FB") {
							const char* s =
									(imageCategory.second.get(
											"<xmlattr>.path",
											"")).c_str();
							filePicker->SetPath(
									wxString(s, wxConvLocal));
						} else if (imageCategory.first
								== "object") {
							objectCount++;
						}
					}
					stringstream s;
					s << objectCount;
					const wxString temp = wxString(s.str().c_str(),
							wxConvLocal);
					AmountOfObjectsTxtField->SetValue(temp);

					MessageLabel->SetLabel(
							wxT("Image is already available within the xml file, press skip button to go to next image."));
					SkipButton->Show(true);

					break;
				}
			}
		}

		if(AmountOfObjectsTxtField->GetValue().size() == 0){
			stringstream globalfile;
			globalfile << (*imagePathsIt).parent_path() << "/global.xml";
			if(exists(globalfile.str().c_str()) && is_regular_file(globalfile.str().c_str())){
				boost::property_tree::ptree temp;
				boost::property_tree::read_xml(globalfile.str().c_str(), temp);
				BOOST_FOREACH( boost::property_tree::ptree::value_type& values, temp.get_child("global_values") )
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
						} else if (values.second.get( "<xmlattr>.name", "")	== "object-type") {
							ObjectComboBox->SetValue( wxString(s, wxConvLocal));
						}
					}
				}
			}
			if(ObjectComboBox->GetValue() == wxT("Crate")){
				NextObjectButton->Show(false);
				CrateButton->Show(true);
			}
		}

		stringstream s;
		s << *imagePathsIt;
		wxString wxStringTemp((s.str().c_str()), wxConvLocal);
		ChangeImagePath(wxStringTemp);
	} else {
		NextObjectButton->Show(false);
		ResetButton->Show(false);
		SkipButton->Show(false);
		ImageField->Show(false);
		MessageLabel->SetLabel(wxT("All images handled, press done to finish"));
	}
}

void GUIFrame::DrawBoxAndRotationLineOnImage() {
	int color = ColorSlider->GetValue();

	UpdateImageField();
	ImageField->Update();
	if (x >= 0 && y >= 0 && width > 0 && height > 0
			&& (x + width) < ImageField->GetSize().GetWidth()
			&& (y + height) < ImageField->GetSize().GetHeight()) {

		wxPaintDC dc(ImageField);
		dc.SetPen(wxPen(wxColour(color, color, color), 2, wxSOLID));
		int tempX = x + 3;
		int tempY = y + 3;
		dc.DrawLine(tempX, tempY, tempX + width, tempY);
		dc.DrawLine(tempX, tempY, tempX, tempY + height);

		dc.DrawLine(tempX + width, tempY, tempX + width, tempY + height);
		dc.DrawLine(tempX, tempY + height, tempX + width, tempY + height);

		int CenterX = tempX + (width / 2);
		int CenterY = tempY + (height / 2);
		dc.DrawLine(CenterX, CenterY, CenterX + ((TopX - BottomX) / 2),
				CenterY + ((TopY - BottomY) / 2));

		dc.DrawRectangle(
				wxPoint((tempX + (width / 2)) - 1, (tempY + (height / 2)) - 1),
				wxSize(3, 3));
	}

	if (zoomX >= 0 && zoomY >= 0 && zoomWidth > 0 && zoomHeight > 0
			&& (zoomX + zoomWidth) < ImageField->GetSize().GetWidth()
			&& (zoomY + zoomHeight) < ImageField->GetSize().GetHeight()) {
		if (ZoomCheckBox->GetValue()) {
			wxPaintDC dc(ImageField);
			dc.SetPen(
					wxPen(wxColour(color, color, 255 - color), 2, wxDOT_DASH));
			int tempX = zoomX + 3;
			int tempY = zoomY + 3;
			dc.DrawLine(tempX, tempY, tempX + zoomWidth, tempY);
			dc.DrawLine(tempX, tempY, tempX, tempY + zoomHeight);

			dc.DrawLine(tempX + zoomWidth, tempY, tempX + zoomWidth,
					tempY + zoomHeight);
			dc.DrawLine(tempX, tempY + zoomHeight, tempX + zoomWidth,
					tempY + zoomHeight);
		}
	}

	if (!NoRotationCheckBox->GetValue()) {
		if (TopX > 0 && TopY > 0 && BottomX > 0 && BottomY > 0
				&& TopX < ImageField->GetSize().GetWidth()
				&& TopY < ImageField->GetSize().GetHeight()
				&& BottomX < ImageField->GetSize().GetWidth()
				&& BottomY < ImageField->GetSize().GetHeight()) {

			wxPaintDC dc(this);
			dc.SetPen(wxPen(wxColour(color, 255 - color, color), 2, wxSOLID));
			dc.DrawLine(TopX + 3, TopY + 3, BottomX + 3, BottomY + 3);
		}
	}
}

void GUIFrame::DrawOnImageAccordingToRadioButtonOption(int eventX, int eventY) {
	int color = ColorSlider->GetValue();

	UpdateImageField();
	ImageField->Update();
	stringstream s;
	s << "(" << eventX << ", " << eventY << ")";

	if (SurroundBox_radioBtn->GetValue() || RLC_RadioBtn->GetValue()) {
		RLC_Label->SetLabel(wxString(s.str().c_str(), wxConvLocal));

		wxPaintDC dc(ImageField);
		dc.SetPen(wxPen(wxColour(color, color, color), 2, wxSOLID));
		int tempX = x + 3;
		int tempY = y + 3;
		dc.DrawLine(tempX, tempY, tempX + eventX - x, tempY);
		dc.DrawLine(tempX, tempY, tempX, tempY + eventY - y);

		dc.DrawLine(tempX + eventX - x, tempY, tempX + eventX - x,
				tempY + eventY - y);
		dc.DrawLine(tempX, tempY + eventY - y, tempX + eventX - x,
				tempY + eventY - y);

	} else if (CenterLine_radioBtn->GetValue()
			|| CenterBottom_RadioBtn->GetValue()) {
		s.str("");
		s << "(" << (int) (eventX * widthScale) << ", "
				<< (int) (eventY * heightScale) << ")";
		CenterBottom_Label->SetLabel(wxString(s.str().c_str(), wxConvLocal));

		wxPaintDC dc(this);
		dc.SetPen(wxPen(wxColour(color, 255 - color, color), 2, wxSOLID));
		dc.DrawLine(TopX + 3, TopY + 3, eventX + 3, eventY + 3);

	} else if (LUC_RadioBtn->GetValue()) {
		LUC_Label->SetLabel(wxString(s.str().c_str(), wxConvLocal));

		wxPaintDC dc(ImageField);
		dc.SetPen(wxPen(wxColour(color, color, color), 2, wxSOLID));
		int tempwidth = width + x - eventX;
		int tempheight = height + y - eventY;
		int tempX = eventX + 3;
		int tempY = eventY + 3;
		dc.DrawLine(tempX, tempY, tempX + tempwidth, tempY);
		dc.DrawLine(tempX, tempY, tempX, tempY + tempheight);

		dc.DrawLine(tempX + tempwidth, tempY, tempX + tempwidth,
				tempY + tempheight);
		dc.DrawLine(tempX, tempY + tempheight, tempX + tempwidth,
				tempY + tempheight);
	} else if (CenterTop_RadioBtn->GetValue()) {
		CenterTop_Label->SetLabel(wxString(s.str().c_str(), wxConvLocal));

		wxPaintDC dc(ImageField);
		dc.SetPen(wxPen(wxColour(color, 255 - color, color), 2, wxSOLID));
		dc.DrawLine(eventX + 3, eventY + 3, BottomX + 3, BottomY + 3);

	} else if (ZoomBox_radioBtn->GetValue()) {
		if (ZoomCheckBox->GetValue()) {
			wxPaintDC dc(ImageField);
			dc.SetPen(
					wxPen(wxColour(color, color, 255 - color), 2, wxDOT_DASH));
			int tempX = zoomX + 3;
			int tempY = zoomY + 3;
			dc.DrawLine(tempX, tempY, tempX + eventX - zoomX, tempY);
			dc.DrawLine(tempX, tempY, tempX, tempY + eventY - zoomY);

			dc.DrawLine(tempX + eventX - zoomX, tempY, tempX + eventX - zoomX,
					tempY + eventY - zoomY);
			dc.DrawLine(tempX, tempY + eventY - zoomY, tempX + eventX - zoomX,
					tempY + eventY - zoomY);
		}
	}
}

void GUIFrame::EditXML(int edit) {
	editXML = edit;
}

void GUIFrame::SetDirPath(const boost::filesystem::path path) {
	dirPath = path;
}

void GUIFrame::GetPathFromChosenDir(boost::filesystem::path &path) {
	stringstream s;
	while (path.leaf() != dirPath.parent_path().leaf()) {
		stringstream ss(s.str().c_str());
		s.str("");
		s << "/" << path.leaf() << ss.str().c_str();
		path = path.parent_path();
	}
	path = s.str().c_str();
}
