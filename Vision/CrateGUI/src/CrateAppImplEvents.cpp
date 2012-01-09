
//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CrateGUI
// File:           CrateAppImplEvents.cpp
// Description:    The implementation of the events generated in the GUI
// Author:         Glenn Meerstra
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of CrateGUI.
//
// CrateGUI is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// CrateGUI is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with CrateGUI.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#include "CrateGUI/CrateAppImpl.h"
#include "CrateGUI/CrateApp.h"

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <iostream>
#include <string>

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

        if(!zoom){
                if(widthScale < heightScale){
                        Scale = heightScale;
                }else{
                        Scale = widthScale;
                }
        }else{
                if(widthScale < heightScale){
                        LargestScale = heightScale;
                }else{
                        LargestScale = widthScale;
                }
        }

        QRCorner = wxPoint(0, 0);
        OppositeCorner = wxPoint(0, 0);

       //zoomX = 0;
       // zoomY = 0;
        //zoomWidth = 0;
       // zoomHeight = 0;

        CrateLineRDB->SetValue(true);
        UpdateImageField();

        this->Layout();
}

void CrateAppImpl::OnObjectType( wxCommandEvent& event ){
        CrateLineRDB->SetValue(true);
        if(ObjectTypeCombo->GetValue() == wxT("QR code")){
        	QRCodeCornerRDB->SetLabel(wxT("Bottom right"));
        	OppositeCornerRDB->SetLabel(wxT("Top Left"));

        }else if(ObjectTypeCombo->GetValue() == wxT("Crate")){
        	QRCodeCornerRDB->SetLabel(wxT("QR code corner"));
        	OppositeCornerRDB->SetLabel(wxT("Opposite corner"));

        }else if(ObjectTypeCombo->GetValue() == wxT("Marker")){
        	QRCodeCornerRDB->SetLabel(wxT("Center point"));
        	OppositeCornerRDB->SetLabel(wxT("Marker edge"));

        }
}

void CrateAppImpl::OnNextImage( wxMouseEvent& event ){

        zoomX = 0;
        zoomY = 0;
        zoomWidth = 0;
        zoomHeight = 0;

        QRCorner = wxPoint(0, 0);
        OppositeCorner = wxPoint(0, 0);

        zoom = false;

        ZoomButton->Enable(true);
        ZoomBox_radioBtn->Enable(true);
        OriginalImageButton->Enable(false);

        NextImage();
}

void CrateAppImpl::OnLeaveImageField(wxMouseEvent& event){
        mousePressedInImageField = false;
        drawCrateAttributes();
}

void CrateAppImpl::OnLeftMousePressed(wxMouseEvent& event){
        mousePressedInImageField = true;

        if(CrateLineRDB->GetValue() || QRCodeCornerRDB->GetValue()){
                QRCorner = event.GetPosition();
                std::stringstream coordinate;
                if(!zoom){
                        coordinate << "(" << (int)(QRCorner.x * Scale) << ", " << (int)(QRCorner.y * Scale) << ")";
                }else{
                        coordinate << "(" << (int)(QRCorner.x * Scale + zoomX) << ", " << (int)(QRCorner.y * Scale + zoomY) << ")";
                }
                QRCodeCornerLabel->SetLabel(wxString(coordinate.str().c_str(), wxConvLocal));
        }else if(OppositeCornerRDB->GetValue()){
                OppositeCorner = event.GetPosition();
                std::stringstream coordinate;
                if(!zoom){
                        coordinate << "(" << (int)(OppositeCorner.x * Scale) << ", " << (int)(OppositeCorner.y * Scale) << ")";
                }else{
                        coordinate << "(" << (int)(OppositeCorner.x * Scale + zoomX) << ", " << (int)(OppositeCorner.y * Scale + zoomY) << ")";
                }
                OppositeCornerLabel->SetLabel(wxString(coordinate.str().c_str(), wxConvLocal));
        }else if(ZoomBox_radioBtn->GetValue()){
                zoomX = event.GetX();
                zoomY = event.GetY();
                zoomWidth = 0;
                zoomHeight = 0;
        }

        calculateFiducialPoints();
        drawCrateAttributes();
}

void CrateAppImpl::OnLeftMouseRelease(wxMouseEvent& event){
        mousePressedInImageField = false;

        if(QRCodeCornerRDB->GetValue()){
                QRCorner = event.GetPosition();
                std::stringstream coordinate;
                if(!zoom){
                        coordinate << "(" << (int)(QRCorner.x * Scale) << ", " << (int)(QRCorner.y * Scale) << ")";
                }else{
                        coordinate << "(" << (int)(QRCorner.x * Scale + zoomX) << ", " << (int)(QRCorner.y * Scale + zoomY) << ")";
                }
                QRCodeCornerLabel->SetLabel(wxString(coordinate.str().c_str(), wxConvLocal));
        }else if(CrateLineRDB->GetValue() || OppositeCornerRDB->GetValue()){
                OppositeCorner = event.GetPosition();
                std::stringstream coordinate;
                if(!zoom){
                        coordinate << "(" << (int)(OppositeCorner.x * Scale) << ", " << (int)(OppositeCorner.y * Scale) << ")";
                }else{
                        coordinate << "(" << (int)(OppositeCorner.x * Scale + zoomX) << ", " << (int)(OppositeCorner.y * Scale + zoomY) << ")";
                }
                OppositeCornerLabel->SetLabel(wxString(coordinate.str().c_str(), wxConvLocal));
        }else if(ZoomBox_radioBtn->GetValue()){
                zoomWidth = event.GetX() - zoomX;
                zoomHeight = event.GetY() - zoomY;

                if(zoomWidth < 0) {
                        zoomX += zoomWidth;
                        zoomWidth = abs(zoomWidth);
                }

                if(zoomHeight < 0) {
                        zoomY += zoomHeight;
                        zoomHeight = abs(zoomHeight);
                }
        }

        calculateFiducialPoints();
        drawCrateAttributes();
}

void CrateAppImpl::OnImageMotion(wxMouseEvent& event){
        if(mousePressedInImageField){
                if(QRCodeCornerRDB->GetValue()){
                        QRCorner = event.GetPosition();
                }else if(CrateLineRDB->GetValue() || OppositeCornerRDB->GetValue()){
                        OppositeCorner = event.GetPosition();
                }else if(ZoomBox_radioBtn->GetValue()){

                        zoomWidth = event.GetX() - zoomX;
                        zoomHeight = event.GetY() - zoomY;
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
                CrateLineRDB->SetValue(true);
                ZoomCheckBox->SetValue(false);
        }
        drawCrateAttributes();
}

void CrateAppImpl::OnZoom(wxCommandEvent& event) {
        if (zoomWidth != 0 && zoomHeight != 0) {
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
                        LargestScale = heightScale;
                }else{
                        LargestScale = widthScale;
                }

                QRCodeCornerLabel->SetLabel(wxString(("(000,000)"), wxConvLocal));
                OppositeCornerLabel->SetLabel(wxString(("(000,000)"), wxConvLocal));

                zoom = true;

                ZoomButton->Enable(false);
                OriginalImageButton->Enable(true);
                ZoomBox_radioBtn->Enable(false);
                CrateLineRDB->SetValue(true);

                UpdateImageField();

                this->Layout();
        } else {
                MessageLabel->SetLabel(wxT("Draw box to zoom into"));
        }
}

void CrateAppImpl::OnOriginal(wxCommandEvent& event) {
        MessageLabel->SetLabel(
                        wxT("The values of the box and rotation line are reset"));

        QRCorner = wxPoint(0, 0);
        OppositeCorner = wxPoint(0, 0);

        zoom = false;

        zoomX = 0;
        zoomY = 0;
        zoomWidth = 0;
        zoomHeight = 0;

        ZoomButton->Enable(true);
        ZoomBox_radioBtn->Enable(true);
        OriginalImageButton->Enable(false);

        drawCrateAttributes();
        this->Layout();
}
std::string CrateAppImpl::DoubleToStringFormatted(double d){
	std::stringstream ss;
	ss << std::fixed;
	ss << std::setprecision (2) << d;
	return ss.str();
}

void CrateAppImpl::OnNextObjectButton(wxCommandEvent& event){

        SkipButton->Enable(false);
        if(ObjectTypeCombo->GetValue() == wxT("None")){
                MessageLabel->SetLabel(wxT("No object type selected"));
                return;
        }

        if(QRCorner == wxPoint(0, 0) ||
                        OppositeCorner == wxPoint(0, 0)){
                MessageLabel->SetLabel(wxT("Corners are not defined"));
                CrateLineRDB->SetValue(true);
                CrateLineRDB->SetFocus();
                return;

        }

        if(QRCodeTextBox->GetValue() == wxT("") && ObjectTypeCombo->GetValue() != wxT("Marker")){
                QRCodeTextBox->SetFocus();
                MessageLabel->SetLabel(wxT("QR code value is not given"));
                return;
        }

        if (XMLOption == Edit && currentCrateNumber == 0) {

                SkipButton->Enable(false);
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
        if(ObjectTypeCombo->GetValue() == wxT("Marker")){
                center.m_x = QRCorner.x;
                center.m_y = QRCorner.y;
        }

        if(zoom){
			center.m_x = center.m_x * LargestScale;
			fid1.m_x = fid1.m_x * LargestScale;
			fid2.m_x = fid2.m_x * LargestScale;
			fid3.m_x = fid3.m_x * LargestScale;

			center.m_y = center.m_y * LargestScale;
			fid1.m_y = fid1.m_y * LargestScale;
			fid2.m_y = fid2.m_y * LargestScale;
			fid3.m_y = fid3.m_y * LargestScale;

			center.m_x += zoomX* Scale;
			fid1.m_x += zoomX* Scale;
			fid2.m_x += zoomX* Scale;
			fid3.m_x += zoomX* Scale;

			center.m_y += zoomY* Scale;
			fid1.m_y += zoomY* Scale;
			fid2.m_y += zoomY* Scale;
			fid3.m_y += zoomY* Scale;
        }else{
			center.m_x = center.m_x * Scale;
			fid1.m_x = fid1.m_x * Scale;
			fid2.m_x = fid2.m_x * Scale;
			fid3.m_x = fid3.m_x * Scale;

			center.m_y = center.m_y * Scale;
			fid1.m_y = fid1.m_y * Scale;
			fid2.m_y = fid2.m_y * Scale;
			fid3.m_y = fid3.m_y * Scale;

        }



        boost::property_tree::ptree& object = tempValue->add("object", "");
        boost::property_tree::ptree* property = &(object.add("property", ""));
        property->put("<xmlattr>.name", "type");
        property->put("<xmlattr>.value", ObjectTypeCombo->GetValue().ToAscii());
        property = &(object.add("property", ""));
        property->put("<xmlattr>.name", "x");
        property->put("<xmlattr>.value", DoubleToStringFormatted(center.m_x));
        property = &(object.add("property", ""));
        property->put("<xmlattr>.name", "y");
        property->put("<xmlattr>.value", DoubleToStringFormatted(center.m_y));


        if(ObjectTypeCombo->GetValue() == wxT("Crate")){
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "fid1.x");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid1.m_x));
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "fid1.y");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid1.m_y));
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "fid2.x");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid2.m_x));
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "fid2.y");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid2.m_y));
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "fid3.x");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid3.m_x));
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "fid3.y");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid3.m_y));

        }else if(ObjectTypeCombo->GetValue() == wxT("QR code")){
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "marker1.x");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid1.m_x));
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "marker1.y");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid1.m_y));
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "marker2.x");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid2.m_x));
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "marker2.y");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid2.m_y));
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "marker3.x");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid3.m_x));
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "marker3.y");
                property->put("<xmlattr>.value", DoubleToStringFormatted(fid3.m_y));
        }

        if(ObjectTypeCombo->GetValue() != wxT("Marker")){
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "qrcode");
                property->put("<xmlattr>.value", QRCode);
        }else{
                property = &(object.add("property", ""));
                property->put("<xmlattr>.name", "radius");
                property->put("<xmlattr>.value", DoubleToStringFormatted(sqrt(pow(double(QRCorner.x - OppositeCorner.x), 2) + pow(double(QRCorner.y - OppositeCorner.y), 2)) * Scale));
        }

        boost::property_tree::xml_writer_settings<char> w('\t', 1);
        boost::property_tree::write_xml(xmlPath.string().c_str(), pt, std::locale(), w);
        MessageLabel->SetLabel(wxString(xmlPath.string().c_str(), wxConvLocal));

        QRCorner = wxPoint(0, 0);
        OppositeCorner = wxPoint(0, 0);

        currentCrateNumber++;

        QRCodeCornerLabel->SetLabel(wxT("(000,000)"));
        OppositeCornerLabel->SetLabel(wxT("(000,000)"));

        UpdateImageField();
        std::stringstream s;
        s << "Next image, current object number: " << currentCrateNumber + 1;
        MessageLabel->SetLabel(wxString(s.str().c_str(), wxConvLocal));
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
                foreach(boost::property_tree::ptree::value_type & imageValue, pt.get_child("Test_set"))
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

        setCurrentImagePath(currentImagePath);
}

void CrateAppImpl::OnDoneButton(wxCommandEvent& event){
        this->Close(true);
}
