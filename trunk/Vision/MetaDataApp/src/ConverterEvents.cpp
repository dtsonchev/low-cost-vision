//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        MetaDataApp
// File:           ConverterEvents.cpp
// Description:    The implementation of the events generated in the GUI
// Author:         Glenn Meerstra
// Notes:          ...
//
// License: newBSD 
//  
// Copyright © 2012, HU University of Applied Sciences Utrecht. 
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//******************************************************************************
#include <MetaDataApp/ConverterGUI.h>
#include <boost/foreach.hpp>

using namespace std;
using namespace boost::filesystem;

void GUIFrame::OnAmountOfObjects(wxFocusEvent& event) {

	AmountOfObjectsTxtField->GetValue().ToULong(&AmountOfObjects, 10);

	if (AmountOfObjectsTxtField->GetValue().size() > 0 && AmountOfObjects > 0
			&& currentObjectNr == 1) {

		if (currentObjectNr == AmountOfObjects - 1
				|| currentObjectNr == AmountOfObjects) {
			NextObjectButton->SetLabel(wxT("Next Image"));
		} else {
			NextObjectButton->SetLabel(wxT("Next Object"));
		}

	} else {
		MessageLabel->SetLabel(
				wxString(("Fill the box with a value"), wxConvLocal));
	}
}

void GUIFrame::NoRotation(wxMouseEvent& event) {
	NoRotationCheckBox->SetValue(!NoRotationCheckBox->GetValue());

	if (NoRotationCheckBox->GetValue()) {
		CenterLine_radioBtn->Show(false);
		CenterBottom_Label->Show(false);
		CenterBottom_RadioBtn->Show(false);
		CenterTop_Label->Show(false);
		CenterTop_RadioBtn->Show(false);
		if (CenterLine_radioBtn->GetValue() || CenterBottom_RadioBtn->GetValue()
				|| CenterTop_RadioBtn->GetValue()) {
			LUC_RadioBtn->SetValue(true);
		}

		TopX = TopY = BottomX = BottomY = 0;

	} else {
		CenterLine_radioBtn->Show(true);
		CenterBottom_Label->Show(true);
		CenterBottom_RadioBtn->Show(true);
		CenterTop_Label->Show(true);
		CenterTop_RadioBtn->Show(true);
	}
}

void GUIFrame::OnNextObjectButton(wxCommandEvent& event) {

	AmountOfObjectsTxtField->GetValue().ToULong(&AmountOfObjects, 10);

	if (currentObjectNr == AmountOfObjects - 1
			|| currentObjectNr == AmountOfObjects) {
		NextObjectButton->SetLabel(wxT("Next Image"));
	}

	if (NoRotationCheckBox->GetValue()) {
		TopX = TopY = BottomX = BottomY = 1;
	}

	long int amount;
	if (x > 0 && y > 0 && width > 0 && height > 0 && TopX > 0 && TopY > 0
			&& BottomX > 0 && BottomY > 0
			&& ObjectComboBox->GetValue() != wxT("Object")
			&& AmountOfObjectsTxtField->GetValue().size() > 0
			&& AmountOfObjectsTxtField->GetValue().ToLong(&amount, 10)) {

		int values[3 * 2];wxString barcode;
		if(ObjectComboBox->GetValue() == wxT("Crate") && !(crateGUI->getValues(values, barcode))) {
			MessageLabel->SetLabel(wxT("Set all crate values by pressing the crate button"));
			return;
		}

		if(editXML == Edit && currentObjectNr == 1) {
			//if images available within the xml need to overwritten
			BOOST_FOREACH( boost::property_tree::ptree::value_type& imageValue,
					pt.get_child("Test_set") ) {
				//<xmlattr>.path is <image path="/home/...." />
				path temp = imageValue.second.get("<xmlattr>.path", "");
				if(temp.leaf() == (*imagePathsIt).leaf()) {
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

		if(!AlreadyInXML && currentObjectNr == 1) {
			//If the image is not already available in the xml file
			tempValue = &(pt.add("Test_set.image", ""));

			path p = *imagePathsIt;
			GetPathFromChosenDir(p);
			tempValue->put("<xmlattr>.path", p);
		}

		if(currentObjectNr == 1) {
			//We want to write certain values only one time for each image
			if(filePicker->GetPath() != wxT("")) {
				stringstream s;
				s << filePicker->GetPath().ToAscii();
				path p = s.str().c_str();
				GetPathFromChosenDir(p);

				path temp = (string)filePicker->GetPath().ToAscii();

				boost::property_tree::ptree &property = tempValue->add("property", "");
				property.put("<xmlattr>.name", "FB");
				property.put("<xmlattr>.path", p);
			}if(BackgroundComboBox->GetValue() != wxT("Background")) {
				boost::property_tree::ptree &category = tempValue->add("category", "");
				category.put("<xmlattr>.name", "background");
				category.put("<xmlattr>.value", BackgroundComboBox->GetValue().ToAscii());
			}if(LightingComboBox->GetValue() != wxT("Light")) {
				boost::property_tree::ptree &category = tempValue->add("category", "");
				category.put("<xmlattr>.name", "light");
				category.put("<xmlattr>.value", LightingComboBox->GetValue().ToAscii());
			}if(PerspectiveComboBox->GetValue() != wxT("Perspective")) {
				boost::property_tree::ptree &category = tempValue->add("category", "");
				category.put("<xmlattr>.name", "perspective");
				category.put("<xmlattr>.value", PerspectiveComboBox->GetValue().ToAscii());
			}
		}
		boost::property_tree::ptree &object = tempValue->add("object", "");
		boost::property_tree::ptree *property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "x");
		property->put("<xmlattr>.value", (int)(x * widthScale) + zoomX);
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "y");
		property->put("<xmlattr>.value", (int)(y * widthScale) + zoomY);
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "width");
		property->put("<xmlattr>.name", "center_x");
		property->put("<xmlattr>.value", (x * widthScale) + zoomX + ((width * widthScale) / 2.0));
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "center_y");
		property->put("<xmlattr>.value", (y * widthScale) + zoomY + ((height * heightScale) / 2.0));
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "width");
		property->put("<xmlattr>.value", (int)(width * widthScale));
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "height");
		property->put("<xmlattr>.value", (int)(height * heightScale));
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "rotation");
		property->put("<xmlattr>.value", GetRotation());
		property = &(object.add("property", ""));
		property->put("<xmlattr>.name", "object-type");
		property->put("<xmlattr>.value", ObjectComboBox->GetValue().ToAscii());

		if(ObjectComboBox->GetValue() == wxT("Crate")) {

			string temp[3] = {"markerLT", "markerRT", "markerLB"};
			stringstream name;
			for(int i = 0; i < 3; i++) {
				 name.str("");
				 name << temp[i] << "_center_x";
				 property = &(object.add("property", ""));
				 property->put("<xmlattr>.name", name.str().c_str());
				 property->put("<xmlattr>.value", values[(i*2)+0]);
				 name.str("");
				 name << temp[i] << "_center_y";
				 property = &(object.add("property", ""));
				 property->put("<xmlattr>.name", name.str().c_str());
				 property->put("<xmlattr>.value", values[(i*2)+1]);
			}
			property = &(object.add("property", ""));
			property->add("<xmlattr>.name", "barcode");
			property->add("<xmlattr>.value", barcode.ToAscii());
		}

		std::stringstream s;
		s << XMLPath;
		boost::property_tree::xml_writer_settings<char> w('\t', 1);
		boost::property_tree::write_xml(s.str().c_str(), pt, std::locale(), w);

		x = 0; y = 0;
		width = 0; height = 0;
		TopX = 0; TopY = 0;
		BottomX = 0; BottomY = 0;
		zoom = false;
		zoomX = 0; zoomY = 0;
		zoomWidth = 0; zoomHeight = 0;

		ZoomBox_radioBtn->Show(true);
		ZoomCheckBox->Show(true);
		ZoomButton->Show(true);
		OriginalImageButton->Show(false);

		LUC_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
		RLC_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
		CenterTop_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
		CenterBottom_Label->SetLabel(wxString( ("(000,000)"), wxConvLocal));
		ObjectComboBox->SetValue( wxT("Object") );
		CrateButton->Show(false);
		SurroundBox_radioBtn->SetValue(true);

		if(NoRotationCheckBox->GetValue()) {
			NoRotationCheckBox->SetValue(false);
			CenterLine_radioBtn->Show(true);
			CenterBottom_Label->Show(true);
			CenterBottom_RadioBtn->Show(true);
			CenterTop_Label->Show(true);
			CenterTop_RadioBtn->Show(true);
		}

		UpdateImageField();

		if(currentObjectNr >= AmountOfObjects) {
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
		} else {
			currentObjectNr++;
			MessageLabel->SetLabel( wxT("Next Object") );
		}
	} else {
		stringstream s;
		s << "Still need: \t";
		if(AmountOfObjectsTxtField->GetValue().size() == 0
				|| !AmountOfObjectsTxtField->GetValue().ToLong(&amount, 10)) {
			AmountOfObjectsTxtField->SetFocus();
			s << "Amount of objects\t";

		}
		if(ObjectComboBox->GetValue() == wxT("Object")) {
			ObjectComboBox->SetFocus();
			s << "Object type\t";

		}
		if (x == 0 || y == 0 || width == 0 || height == 0) {
			SurroundBox_radioBtn->SetValue(true);
			SurroundBox_radioBtn->SetFocus();
			s << "Object box\t";

		}
		if(BottomX == 0 || BottomY == 0 || TopX == 0 || TopY == 0) {
			CenterLine_radioBtn->SetValue(true);
			CenterLine_radioBtn->SetFocus();
			s << "Center line";

		}
		MessageLabel->SetLabel(wxString(s.str().c_str(), wxConvLocal));
	}
}

void GUIFrame::OnImageMotion(wxMouseEvent& event) {
	if (mousePressedInImage) {
		DrawOnImageAccordingToRadioButtonOption(event.GetX(), event.GetY());
	}
}

void GUIFrame::OnLeftMousePressed(wxMouseEvent& event) {
	mousePressedInImage = true;
	stringstream s;
	s << "(" << event.GetX() << ", " << event.GetY() << ")";

	if (SurroundBox_radioBtn->GetValue()) {
		LUC_Label->SetLabel(wxString(s.str().c_str(), wxConvLocal));
		width += x - event.GetX();
		height += y - event.GetY();
		x = event.GetX();
		y = event.GetY();

	} else if (CenterLine_radioBtn->GetValue()) {
		CenterTop_Label->SetLabel(wxString(s.str().c_str(), wxConvLocal));
		TopX = event.GetX();
		TopY = event.GetY();

	} else if (ZoomBox_radioBtn->GetValue()) {
		zoomX = event.GetX();
		zoomY = event.GetY();

	}

	DrawOnImageAccordingToRadioButtonOption(event.GetX(), event.GetY());
}

void GUIFrame::OnLeftMouseRelease(wxMouseEvent& event) {
	mousePressedInImage = false;

	if (LUC_RadioBtn->GetValue()) {
		width += x - event.GetX();
		height += y - event.GetY();
		x = event.GetX();
		y = event.GetY();

		RLC_RadioBtn->SetValue(true);

		stringstream s;
		s << "(" << x + (width / 2) << ", " << y + (height / 2) << ")";
		CenterPointValue_label->SetLabel(
				wxString(s.str().c_str(), wxConvLocal));

	} else if (RLC_RadioBtn->GetValue() || SurroundBox_radioBtn->GetValue()) {

		//Checks if these values truly are the right lower corner
		if (x > event.GetX()) {
			width = x - event.GetX();

			stringstream s;
			s << "(" << x << ", " << y + width << ")";
			RLC_Label->SetLabel(wxString(s.str().c_str(), wxConvLocal));

			x = event.GetX();

			s.str("");
			s << "(" << x << ", " << y << ")";
			LUC_Label->SetLabel(wxString(s.str().c_str(), wxConvLocal));

		} else {
			width = event.GetX() - x;
		}

		if (y > event.GetY()) {
			height = y - event.GetY();

			stringstream s;
			s << "(" << x + height << ", " << y << ")";
			RLC_Label->SetLabel(wxString(s.str().c_str(), wxConvLocal));

			y = event.GetY();

			s.str("");
			s << "(" << x << ", " << y << ")";
			LUC_Label->SetLabel(wxString(s.str().c_str(), wxConvLocal));

		} else {
			height = event.GetY() - y;
		}

		stringstream s;
		s << "(" << x + (width / 2) << ", " << y + (height / 2) << ")";
		CenterPointValue_label->SetLabel(
				wxString(s.str().c_str(), wxConvLocal));

		if (!NoRotationCheckBox->GetValue()) {
			if (SurroundBox_radioBtn->GetValue()) {
				CenterLine_radioBtn->SetValue(true);
			} else {
				LUC_RadioBtn->SetValue(true);
			}
		} else {
			LUC_RadioBtn->SetValue(true);
		}

	} else if (CenterTop_RadioBtn->GetValue()) {
		TopX = event.GetX();
		TopY = event.GetY();

		stringstream s;
		int rotation = (GetRotation() * 10);
		s << rotation / 10 << "." << rotation % 10;
		RotationValue_label->SetLabel(wxString(s.str().c_str(), wxConvLocal));

	} else if (CenterBottom_RadioBtn->GetValue()
			|| CenterLine_radioBtn->GetValue()) {
		BottomX = event.GetX();
		BottomY = event.GetY();

		CenterTop_RadioBtn->SetValue(true);

		stringstream s;
		int rotation = (GetRotation() * 10);
		s << rotation / 10 << "." << rotation % 10;
		RotationValue_label->SetLabel(wxString(s.str().c_str(), wxConvLocal));

	} else if (ZoomBox_radioBtn->GetValue()) {
		if (zoomX > event.GetX()) {
			zoomWidth = zoomX - event.GetX();
			zoomX = event.GetX();

		} else {
			zoomWidth = event.GetX() - zoomX;
		}

		if (zoomY > event.GetY()) {
			zoomHeight = zoomY - event.GetY();
			zoomY = event.GetY();

		} else {
			zoomHeight = event.GetY() - zoomY;
		}
	}

	DrawBoxAndRotationLineOnImage();
}

void GUIFrame::OnSkip(wxMouseEvent& event) {
	x = 0;
	y = 0;
	width = 0;
	height = 0;
	TopX = 0;
	TopY = 0;
	BottomX = 0;
	BottomY = 0;

	LUC_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	RLC_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	CenterTop_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	CenterBottom_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	ObjectComboBox->Show(true);
	ObjectComboBox->SetValue(wxT("Object"));
	SurroundBox_radioBtn->SetValue(true);

	if (NoRotationCheckBox->GetValue()) {
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
	AmountOfObjectsTxtField->SetValue(wxT(""));
	currentObjectNr = 1;
	AlreadyInXML = false;

	NextImage();
}

void GUIFrame::OnComboSelect(wxCommandEvent& event) {
	if (event.GetId() == ObjectComboBox->GetId()) {
		if (ObjectComboBox->GetValue() == wxT("Crate")) {
			CrateButton->Show(true);
			NextObjectButton->Show(false);
		} else {
			CrateButton->Show(false);
			NextObjectButton->Show(true);
		}
	}
}

void GUIFrame::OnCrateButton(wxCommandEvent& event) {
	ObjectComboBox->SetValue(wxT("Crate"));
	if (x > 0 && y > 0 && width > 0 && height > 0 && TopX > 0 && TopY > 0
			&& BottomX > 0 && BottomY > 0) {

		double rotation = GetRotation();

		wxImage temp;
		if (!zoom) {
			temp = image.Size(
					wxSize(x * widthScale + width * widthScale,
							y * heightScale + height * heightScale),
					wxPoint(x * widthScale, y * heightScale));

		} else {
			temp = zoomImage.Size(
					wxSize(x * widthScale + width * widthScale,
							y * heightScale + height * heightScale),
					wxPoint(x * widthScale, y * heightScale));
		}

		temp = (temp.Rotate90(true)).Rotate90(true);

		temp = temp.Size(wxSize(width * widthScale, height * heightScale),
				wxPoint(0, 0));

		temp = temp.Rotate(
				((180 - rotation) * (M_PI / 180.0)), wxPoint(0,0), false);

		crateGUI =
				new CrateGUI(
						this,
						wxID_ANY,
						wxT("Surround markers with boxes and give the barcode value"),
						wxDefaultPosition, wxSize(700, 510),
						wxDEFAULT_FRAME_STYLE | wxTAB_TRAVERSAL);
		crateGUI->Start(temp, ((x + (width / 2)) * widthScale),
		((y + (height / 2)) * heightScale), currentImagePath,
		rotation);

		NextObjectButton->Show(true);
	} else {
		MessageLabel->SetLabel(
				wxT("First surround the crate with a box and draw a rotation line"));
	}
}

void GUIFrame::OnSizeChange(wxSizeEvent& event) {
	MessageLabel->SetLabel(
			wxT("The values of the box and rotation line are reset"));
	wxSize imageMaxSize = this->GetSize();
	imageMaxSize.SetWidth(
			imageMaxSize.GetWidth() - bSizer121->GetSize().GetWidth());
	imageMaxSize.SetHeight(
			imageMaxSize.GetHeight() - MessageLabel->GetSize().GetHeight()
					- 20);

	if (!ZoomCheckBox->GetValue()) {
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

	x = 0;
	y = 0;
	width = 0;
	height = 0;
	TopX = 0;
	TopY = 0;
	BottomX = 0;
	BottomY = 0;

	zoom = false;
	zoomX = 0;
	zoomY = 0;
	zoomWidth = 0;
	zoomHeight = 0;

	LUC_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	RLC_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	CenterTop_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	CenterBottom_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));

	SurroundBox_radioBtn->SetValue(true);

	UpdateImageField();
	this->Layout();
}

void GUIFrame::OnDoneButton(wxCommandEvent& event) {
	this->Close(true);
}

void GUIFrame::OnReset(wxCommandEvent& event) {
	x = 0;
	y = 0;
	width = 0;
	height = 0;
	TopX = 0;
	TopY = 0;
	BottomX = 0;
	BottomY = 0;
	zoom = false;
	zoomX = 0;
	zoomY = 0;
	zoomWidth = 0;
	zoomHeight = 0;

	LUC_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	RLC_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	CenterTop_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	CenterBottom_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	ObjectComboBox->SetValue(wxT("Object"));
	CrateButton->Show(false);
	SurroundBox_radioBtn->SetValue(true);

	BackgroundComboBox->SetValue(wxT("Background"));
	BackgroundComboBox->Show(true);
	PerspectiveComboBox->Show(true);
	PerspectiveComboBox->SetValue(wxT("Perspective"));
	LightingComboBox->Show(true);
	LightingComboBox->SetValue(wxT("Light"));
	AmountOfObjectsTxtField->SetValue(wxT(""));
	currentObjectNr = 1;

	if (NoRotationCheckBox->GetValue()) {
		NoRotationCheckBox->SetValue(false);
		CenterLine_radioBtn->Show(true);
		CenterBottom_Label->Show(true);
		CenterBottom_RadioBtn->Show(true);
		CenterTop_Label->Show(true);
		CenterTop_RadioBtn->Show(true);
	}

	UpdateImageField();
}

void GUIFrame::OnColorSlider(wxScrollEvent& event) {
	DrawBoxAndRotationLineOnImage();
}

void GUIFrame::OnZoom(wxCommandEvent& event) {
	if (zoomX > 0 && zoomY > 0 && zoomWidth > 0 && zoomHeight > 0) {
		zoomImage = image.Size(
				wxSize(zoomX * widthScale + zoomWidth * widthScale,
						zoomY * heightScale + zoomHeight * heightScale),
				wxPoint(zoomX * widthScale, zoomY * heightScale));

		zoomImage = (zoomImage.Rotate90(true)).Rotate90(true);
		zoomImage = zoomImage.Size(
				wxSize(zoomWidth * widthScale, zoomHeight * heightScale),
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

		zoomX = zoomX * widthScale;
		zoomY = zoomY * heightScale;
		zoomWidth = zoomWidth * widthScale;
		zoomHeight = zoomHeight * heightScale;

		heightScale = (double) zoomImage.GetHeight()
				/ (double) imageMaxSize.GetHeight();
		widthScale = (double) zoomImage.GetWidth()
				/ (double) imageMaxSize.GetWidth();

		x = 0;
		y = 0;
		width = 0;
		height = 0;
		TopX = 0;
		TopY = 0;
		BottomX = 0;
		BottomY = 0;

		LUC_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
		RLC_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
		CenterTop_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
		CenterBottom_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));

		zoom = true;
		SurroundBox_radioBtn->SetValue(true);
		ZoomButton->Show(false);
		OriginalImageButton->Show(true);
		ZoomCheckBox->Show(false);
		ZoomCheckBox->SetValue(false);
		ZoomBox_radioBtn->Show(false);

		UpdateImageField();
	} else {
		MessageLabel->SetLabel(wxT("Draw box to zoom into"));
	}
}

void GUIFrame::OnOriginal(wxCommandEvent& event) {
	MessageLabel->SetLabel(
			wxT("The values of the box and rotation line are reset"));
	wxSize imageMaxSize = this->GetSize();
	imageMaxSize.SetWidth(
			imageMaxSize.GetWidth() - bSizer121->GetSize().GetWidth());
	imageMaxSize.SetHeight(
			imageMaxSize.GetHeight() - MessageLabel->GetSize().GetHeight()
					- 20);

	heightScale = (double) image.GetHeight()
			/ (double) imageMaxSize.GetHeight();
	widthScale = (double) image.GetWidth() / (double) imageMaxSize.GetWidth();

	x = 0;
	y = 0;
	width = 0;
	height = 0;
	TopX = 0;
	TopY = 0;
	BottomX = 0;
	BottomY = 0;
	zoomX = 0;
	zoomY = 0;
	zoomWidth = 0;
	zoomHeight = 0;

	LUC_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	RLC_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	CenterTop_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));
	CenterBottom_Label->SetLabel(wxString(("(000,000)"), wxConvLocal));

	zoom = false;
	ZoomButton->Show(true);
	OriginalImageButton->Show(false);
	ZoomCheckBox->Show(true);
	ZoomBox_radioBtn->Show(true);

	UpdateImageField();
}

void GUIFrame::OnZoomChange(wxMouseEvent& event) {
	if (ZoomCheckBox->GetId() == event.GetId()) {
		if (!ZoomCheckBox->GetValue()) {
			ZoomBox_radioBtn->SetValue(true);
			ZoomCheckBox->SetValue(true);
		} else {
			SurroundBox_radioBtn->SetValue(true);
			ZoomCheckBox->SetValue(false);
		}
	}

	if (ZoomBox_radioBtn->GetId() == event.GetId()) {
		if (ZoomBox_radioBtn->GetValue()) {
			SurroundBox_radioBtn->SetValue(true);
			ZoomCheckBox->SetValue(false);
		} else {
			ZoomBox_radioBtn->SetValue(true);
			ZoomCheckBox->SetValue(true);
		}
	}
}
