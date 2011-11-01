#include "ConverterGUI.h"
#include <boost/foreach.hpp>

using namespace std;
using namespace boost::filesystem;

void GUIFrame::OnAmountOfObjects(wxFocusEvent& event){

	AmountOfObjectsTxtField->GetValue().ToULong(&AmountOfObjects ,10);

	if(AmountOfObjectsTxtField->GetValue().size() > 0 && AmountOfObjects > 0 && currentObjectNr == 1){

		if(currentObjectNr == AmountOfObjects - 1 || currentObjectNr == AmountOfObjects){
			NextObjectButton->SetLabel( wxT("Next Image"));
		}else{
			NextObjectButton->SetLabel( wxT("Next Object"));
		}

	}else{
		MessageLabel->SetLabel(wxString(("Fill the box with a value"), wxConvLocal));
	}
}

void GUIFrame::NoRotation(wxMouseEvent& event){
	NoRotationCheckBox->SetValue(!NoRotationCheckBox->GetValue());

	if(NoRotationCheckBox->GetValue()){
		CenterLine_radioBtn->Show(false);
		CenterBottom_Label->Show(false);
		CenterBottom_RadioBtn->Show(false);
		CenterTop_Label->Show(false);
		CenterTop_RadioBtn->Show(false);
		if(CenterLine_radioBtn->GetValue() || CenterBottom_RadioBtn->GetValue() || CenterTop_RadioBtn->GetValue()){
			LUC_RadioBtn->SetValue(true);
		}
	}else{
		CenterLine_radioBtn->Show(true);
		CenterBottom_Label->Show(true);
		CenterBottom_RadioBtn->Show(true);
		CenterTop_Label->Show(true);
		CenterTop_RadioBtn->Show(true);
	}
}

void GUIFrame::OnNextObjectButton(wxCommandEvent& event){
	AmountOfObjectsTxtField->GetValue().ToULong(&AmountOfObjects ,10);

	if(currentObjectNr == AmountOfObjects - 1 || currentObjectNr == AmountOfObjects){
		NextObjectButton->SetLabel( wxT("Next Image"));
	}

	if(NoRotationCheckBox->GetValue()){
		TopX = TopY = BottomX = BottomY = 1;
	}

	if(x > 0 && y > 0 && width > 0 && height > 0
			&& TopX > 0 && TopY > 0 && BottomX > 0 && BottomY > 0
			&& ObjectComboBox->GetValue() != wxString(("Object"), wxConvLocal)
			&& AmountOfObjectsTxtField->GetValue().size() > 0){

		boost::property_tree::ptree *tempValue;
		if(editXML == Edit && currentObjectNr == 1){
			//if images available within the xml need to overwritten
			BOOST_FOREACH( boost::property_tree::ptree::value_type& imageValue, pt.get_child("Test_set") ) {
				//<xmlattr>.path is <image path="/home/...." />
				path temp = imageValue.second.get("<xmlattr>.path", "");
				if(temp.leaf() == (*imagePathsIt).leaf()){
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

		if(!AlreadyInXML){
			//If the image is not already available in the xml file
			tempValue = &(pt.add("Test_set.image", ""));

			path p = *imagePathsIt;
			GetPathFromChosenDir(p);
			tempValue->put("<xmlattr>.path", p);
		}

		if(currentObjectNr == 1){
			//We want to write certain values only one time for each image
			if(filePicker->GetPath() != wxT("")){
				stringstream s;
				s << filePicker->GetPath().ToAscii();
				path p = s.str().c_str();
				GetPathFromChosenDir(p);

				path temp = (string)filePicker->GetPath().ToAscii();

				boost::property_tree::ptree &property = tempValue->add("property", "");
				property.put("<xmlattr>.name", "FB");
				property.put("<xmlattr>.path", p);
			}if(BackgroundComboBox->GetValue() != wxT("Background")){
				boost::property_tree::ptree &category = tempValue->add("category", "");
				category.put("<xmlattr>.name", "background");
				category.put("<xmlattr>.value", BackgroundComboBox->GetValue().ToAscii());
			}if(LightingComboBox->GetValue() != wxT("Light")){
				boost::property_tree::ptree &category = tempValue->add("category", "");
				category.put("<xmlattr>.name", "light");
				category.put("<xmlattr>.value", LightingComboBox->GetValue().ToAscii());
			}if(PerspectiveComboBox->GetValue() != wxT("Perspective")){
				boost::property_tree::ptree &category = tempValue->add("category", "");
				category.put("<xmlattr>.name", "perspective");
				category.put("<xmlattr>.value", PerspectiveComboBox->GetValue().ToAscii());
			}
		}
		boost::property_tree::ptree &object = tempValue->add("object", "");
		boost::property_tree::ptree *property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "x");
		property->put("<xmlattr>.value", (int)(x * widthScale));
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "y");
		property->put("<xmlattr>.value", (int)(y * widthScale));
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "width");
		property->put("<xmlattr>.value", (int)(width * widthScale));
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "height");
		property->put("<xmlattr>.value", (int)(height * heightScale));

		if(NoRotationCheckBox->GetValue()){
			property = &(object.add("property", ""));
			property->put("<xmlattr>.name", "rotation");
			property->put("<xmlattr>.value", "0");
		}else{
			property = &(object.add("property", ""));
			property->put("<xmlattr>.name", "rotation");
			property->put("<xmlattr>.value", GetRotation());
		}
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "object-type");
		property->put("<xmlattr>.value", ObjectComboBox->GetValue().ToAscii());

		x = 0;			y = 0;
		width = 0;		height = 0;
		TopX = 0;		TopY = 0;
		BottomX = 0;	BottomY = 0;

		LUC_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
		RLC_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
		CenterTop_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
		CenterBottom_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
		ObjectComboBox->Show(true);
		ObjectComboBox->SetValue( wxT("Object") );
		SurroundBox_radioBtn->SetValue(true);

		if(NoRotationCheckBox->GetValue()){
			NoRotationCheckBox->SetValue(false);
			CenterLine_radioBtn->Show(true);
			CenterBottom_Label->Show(true);
			CenterBottom_RadioBtn->Show(true);
			CenterTop_Label->Show(true);
			CenterTop_RadioBtn->Show(true);
		}

		UpdateImageField();

		if(currentObjectNr >= AmountOfObjects){
			BackgroundComboBox->SetValue(wxT("Background"));
			BackgroundComboBox->Show(true);
			PerspectiveComboBox->Show(true);
			PerspectiveComboBox->SetValue(wxT("Perspective"));
			LightingComboBox->Show(true);
			LightingComboBox->SetValue(wxT("Light"));
			AmountOfObjectsTxtField->SetValue( wxT(""));
			currentObjectNr = 1;
			NextImage();
			AlreadyInXML = false;
		}else{
			currentObjectNr++;
			MessageLabel->SetLabel( wxT("Next Object") );
		}
	}else{
		MessageLabel->SetLabel(wxString(("Give amount of objects,\nBackground color,\nObject type,\nDraw an rectangle around the object,\nDraw an center line"), wxConvLocal));
	}
}

void GUIFrame::OnImageMotion(wxMouseEvent& event){
	if(mousePressedInImage){
		DrawOnImageAccordingToRadioButtonOption(event.GetX(), event.GetY());
	}
}


void GUIFrame::OnLeftMousePressed(wxMouseEvent& event){
	mousePressedInImage = true;
	stringstream s;
	s << "(" << event.GetX() << ", " << event.GetY() << ")";

	if(SurroundBox_radioBtn->GetValue()){
		LUC_Label->SetLabel(wxString( s.str().c_str(), wxConvLocal));
		width += x - event.GetX();
		height += y - event.GetY();
		x = event.GetX();
		y = event.GetY();

	}else if(CenterLine_radioBtn->GetValue()){
		CenterTop_Label->SetLabel(wxString( s.str().c_str(), wxConvLocal));
		TopX = event.GetX();
		TopY = event.GetY();
	}

	DrawOnImageAccordingToRadioButtonOption(event.GetX(), event.GetY());
}

void GUIFrame::OnLeftMouseRelease(wxMouseEvent& event){
	mousePressedInImage = false;

	if(LUC_RadioBtn->GetValue()){
		width += x - event.GetX();
		height += y - event.GetY();
		x = event.GetX();
		y = event.GetY();

		RLC_RadioBtn->SetValue(true);

	}else if(RLC_RadioBtn->GetValue() || SurroundBox_radioBtn->GetValue()){

		//Checks if these values truly are the right lower corner
		if(x > event.GetX()){
			width = x - event.GetX();

			stringstream s;
			s << "(" << x << ", " << y + width << ")";
			RLC_Label->SetLabel(wxString( s.str().c_str(), wxConvLocal));

			x = event.GetX();

			s.str("");
			s << "(" << x << ", " << y << ")";
			LUC_Label->SetLabel(wxString( s.str().c_str(), wxConvLocal));

		}else{
			width = event.GetX() - x;
		}

		if(y > event.GetY()){
			height = y - event.GetY();

			stringstream s;
			s << "(" << x + height << ", " << y << ")";
			RLC_Label->SetLabel(wxString( s.str().c_str(), wxConvLocal));

			y = event.GetY();

			s.str("");
			s << "(" << x << ", " << y << ")";
			LUC_Label->SetLabel(wxString( s.str().c_str(), wxConvLocal));

		}else{
			height = event.GetY() - y;
		}

		if(!NoRotationCheckBox->GetValue()){
			if(SurroundBox_radioBtn->GetValue()){
				CenterLine_radioBtn->SetValue(true);
			}else{
				LUC_RadioBtn->SetValue(true);
			}
		}else{
			LUC_RadioBtn->SetValue(true);
		}

	}else if(CenterTop_RadioBtn->GetValue()){
		TopX = event.GetX();
		TopY = event.GetY();

		CenterBottom_RadioBtn->SetValue(true);

	}else if(CenterBottom_RadioBtn->GetValue() || CenterLine_radioBtn->GetValue()){
		BottomX = event.GetX();
		BottomY = event.GetY();

		CenterTop_RadioBtn->SetValue(true);
	}

	DrawBoxAndRotationLineOnImage();
}

void GUIFrame::OnSkip( wxMouseEvent& event ){
	x = 0;			y = 0;
	width = 0;		height = 0;
	TopX = 0;		TopY = 0;
	BottomX = 0;	BottomY = 0;

	LUC_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
	RLC_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
	CenterTop_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
	CenterBottom_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
	ObjectComboBox->Show(true);
	ObjectComboBox->SetValue( wxT("Object") );
	SurroundBox_radioBtn->SetValue(true);

	if(NoRotationCheckBox->GetValue()){
		NoRotationCheckBox->SetValue(false);
		CenterLine_radioBtn->Show(true);
		CenterBottom_Label->Show(true);
		CenterBottom_RadioBtn->Show(true);
		CenterTop_Label->Show(true);
		CenterTop_RadioBtn->Show(true);
	}

	BackgroundComboBox->SetValue(wxT("Background"));
	BackgroundComboBox->Show(true);
	PerspectiveComboBox->Show(true);
	PerspectiveComboBox->SetValue(wxT("Perspective"));
	LightingComboBox->Show(true);
	LightingComboBox->SetValue(wxT("Light"));
	AmountOfObjectsTxtField->SetValue( wxT(""));
	currentObjectNr = 1;
	AlreadyInXML = false;

	NextImage();
}
