#include "ConverterGUI.h"

using namespace std;
using namespace boost::filesystem;

void GUIFrame::AddBackground(const wxString &backgroundName){
	BackgroundComboBox->Append(backgroundName);
}
void GUIFrame::AddLight(const wxString &lightName){
	LightingComboBox->Append(lightName);
}
void GUIFrame::AddPerspective(const wxString &perspectiveName){
	PerspectiveComboBox->Append(perspectiveName);
}
void GUIFrame::AddObject(const wxString &objectName){
	ObjectComboBox->Append(objectName);
}
void GUIFrame::ChangeImagePaths(const vector<path> &newImagePaths){
	imagePaths = newImagePaths;
}

void GUIFrame::ChangeImagePath(const wxString& path){
	image = wxImage(path);
	heightScale = image.GetHeight()/imageHeight;
	widthScale = image.GetWidth()/imageWidth;
	image.Rescale(imageWidth, imageHeight);
	UpdateImageField();
}

void GUIFrame::UpdateImageField(){
	ImageField->SetBitmap(image);
	ImageField->SetSize(imageWidth, imageHeight);
}

double GUIFrame::GetRotation(){
	if(TopX > BottomX && TopY < BottomY){		//Right top -> Left under
		return ((tan((double)(abs(TopX - BottomX)) / (double)(abs(TopY - BottomY)))/M_PI)*180.0);

	}else if(TopX > BottomX && TopY > BottomY){	//Right under -> Left top
		return (180.0 - (tan((double)(abs(TopX - BottomX)) / (double)(abs(TopY - BottomY)))/M_PI)*180.0);

	}else if(TopX < BottomX && TopY > BottomY){	//Left under -> Right top
		return (180.0 + (tan((double)(abs(TopX - BottomX)) / (double)(abs(TopY - BottomY)))/M_PI)*180.0);

	}else if(TopX < BottomX && TopY < BottomY){	//Left top -> Right under
		return (360.0 - (tan((double)(abs(TopX - BottomX)) / (double)(abs(TopY - BottomY)))/M_PI)*180.0);
	}else{
		return -1.0;
	}
}

void GUIFrame::Start(){

	x = 0;			y = 0;
	width = 0;		height = 0;
	TopX = 0;		TopY = 0;
	BottomX = 0;	BottomY = 0;

	currentObjectNr = 1;
	mousePressedInImage = false;
	AlreadyInXML = false;
	SkipButton->Show(false);

	stringstream s;
	s << dirPath << "/TestSet.xml";
	if(editXML == NewXML){
		boost::property_tree::ptree temp;
		temp.add("Test_set", "");

		boost::property_tree::write_xml(s.str().c_str(), temp);
	}


	if(!is_regular_file(s.str().c_str())){
		stringstream temp;
		temp << "Couldn't handle: " << s.str().c_str();
		MessageLabel->SetLabel( wxString(temp.str().c_str(), wxConvLocal));
		return;
	}


	boost::property_tree::read_xml(s.str().c_str(), pt);
	imagePathsIt = imagePaths.begin();
	imagePathsIt--;
    this->Show(true);
    NextImage();

}

void GUIFrame::NextImage(){

	imagePathsIt++;
	if(imagePathsIt < imagePaths.end()){

		MessageLabel->SetLabel( wxT("Next Image") );
		if(editXML == Edit){
			//Disables/enables the skip button
			SkipButton->Show(false);
			BOOST_FOREACH( boost::property_tree::ptree::value_type& tempValue, pt.get_child("Test_set") ) {

				path temp = tempValue.second.get("<xmlattr>.path", "");
				if(temp.leaf() == (*imagePathsIt).leaf()){

					int objectCount = 0;
					BOOST_FOREACH( boost::property_tree::ptree::value_type& imageCategory, tempValue.second.get_child(tempValue.second.data()) ) {
						if(imageCategory.first == "category"){

							const char* s = (imageCategory.second.get("<xmlattr>.value", "")).c_str();
							if(imageCategory.second.get("<xmlattr>.name", "") == "background"){
								BackgroundComboBox->SetValue( wxString( s , wxConvLocal));
							}else if(imageCategory.second.get("<xmlattr>.name", "") == "light"){
								LightingComboBox->SetValue( wxString( s , wxConvLocal));
							}else if(imageCategory.second.get("<xmlattr>.name", "") == "perspective"){
								PerspectiveComboBox->SetValue( wxString( s , wxConvLocal));
							}

						}else if(imageCategory.first == "property" && imageCategory.second.get("<xmlattr>.nsme", "") == "FB"){
							const char* s = (imageCategory.second.get("<xmlattr>.path", "")).c_str();
							filePicker->SetPath(wxString( s , wxConvLocal));
						}else if(imageCategory.first == "object"){
							objectCount++;
						}
					}
					stringstream s;
					s << objectCount;
					const wxString temp = wxString( s.str().c_str() , wxConvLocal);
					AmountOfObjectsTxtField->SetValue(temp);

					MessageLabel->SetLabel( wxT("Image is already available within the xml file, press skip button to go to next image."));
					SkipButton->Show(true);

					break;
				}
			}
		}

		stringstream s;
		s << *imagePathsIt;
		wxString wxStringTemp((s.str().c_str()), wxConvLocal);
		ChangeImagePath(wxStringTemp);
	}else{
		this->Close(true);
	}

}

void GUIFrame::DrawBoxAndRotationLineOnImage(){
	UpdateImageField();
	ImageField->Update();
	if(x >= 0 && y >= 0 && width > 0 && height > 0 &&
			(x + width) < imageWidth && (y + height) < imageHeight){

		wxPaintDC dc(this);
		int tempX = x + 3;
		int tempY = y + 3;
		dc.DrawLine(tempX, tempY, tempX + width, tempY);
		dc.DrawLine(tempX, tempY, tempX, tempY + height);

		dc.DrawLine(tempX + width, tempY, tempX + width, tempY + height);
		dc.DrawLine(tempX, tempY + height, tempX + width, tempY + height);
	}

	if(!NoRotationCheckBox->GetValue()){
		if(TopX > 0 && TopY > 0 && BottomX > 0 && BottomY > 0 &&
				TopX < imageWidth && TopY < imageHeight &&
				BottomX < imageWidth && BottomY < imageHeight){

			wxPaintDC dc(this);
			dc.DrawLine(TopX + 3, TopY + 3, BottomX + 3, BottomY + 3);
		}
	}
}

void GUIFrame::DrawOnImageAccordingToRadioButtonOption(int eventX, int eventY){
	UpdateImageField();
	ImageField->Update();
	stringstream s;
	s << "(" << eventX << ", " << eventY << ")";

	if(SurroundBox_radioBtn->GetValue() || RLC_RadioBtn->GetValue()){
		RLC_Label->SetLabel(wxString( s.str().c_str(), wxConvLocal));

		wxPaintDC dc(this);
		int tempX = x + 3;
		int tempY = y + 3;
		dc.DrawLine(tempX, tempY, tempX + eventX - x, tempY);
		dc.DrawLine(tempX, tempY, tempX, tempY + eventY - y);

		dc.DrawLine(tempX + eventX - x, tempY, tempX + eventX - x, tempY + eventY - y);
		dc.DrawLine(tempX, tempY + eventY - y, tempX + eventX - x, tempY + eventY - y);

	}else if(CenterLine_radioBtn->GetValue() || CenterBottom_RadioBtn->GetValue()){
		CenterBottom_Label->SetLabel(wxString( s.str().c_str(), wxConvLocal));

		wxPaintDC dc(this);
		dc.DrawLine(TopX + 3, TopY + 3, eventX + 3, eventY + 3);

	}else if(LUC_RadioBtn->GetValue()){
		LUC_Label->SetLabel(wxString( s.str().c_str(), wxConvLocal));

		wxPaintDC dc(this);
		int tempwidth = width + x - eventX;
		int tempheight = height + y - eventY;
		int tempX = eventX + 3;
		int tempY = eventY + 3;
		dc.DrawLine(tempX, tempY, tempX + tempwidth, tempY);
		dc.DrawLine(tempX, tempY, tempX, tempY + tempheight);

		dc.DrawLine(tempX + tempwidth, tempY, tempX + tempwidth, tempY + tempheight);
		dc.DrawLine(tempX, tempY + tempheight, tempX + tempwidth, tempY + tempheight);
	}else if(CenterTop_RadioBtn->GetValue()){
		CenterTop_Label->SetLabel(wxString( s.str().c_str(), wxConvLocal));

		wxPaintDC dc(this);
		dc.DrawLine(eventX + 3, eventY + 3, BottomX + 3, BottomY + 3);

	}
}

void GUIFrame::EditXML(int edit){
	editXML = edit;
}

void GUIFrame::SetDirPath(boost::filesystem::path path){
	dirPath = path;
}

void GUIFrame::GetPathFromChosenDir(boost::filesystem::path &path){
	stringstream s;
	while(path.leaf() != dirPath.parent_path().leaf() ){
		stringstream ss(s.str().c_str());
		s.str("");
		s  << "/" << path.leaf() << ss.str().c_str();
		path = path.parent_path();
	}
	path = s.str().c_str();
}